/* ===========================================================================
   PRESENTATION LAYER  (v2 · "night drive")
   Reads the simulation, never writes physics. Everything here is drawing,
   camera, and choreography. The engine in sim.js / network.js is unchanged.

   Views
     map     real DVP alignment over an OSM basemap; cars as light points
     chase   lane-level close-up that follows one car
     scrub   24 h time control drawn over the MEASURED speed curves
     story   the brake-event narrative and its numbers
     st      time x distance strip, where jam waves show up as diagonal streaks
   =========================================================================== */

const $ = (id) => document.getElementById(id);
const clamp = (v,a,b) => v < a ? a : v > b ? b : v;
const lerp = (a,b,t) => a + (b-a)*t;
const DAY_ORDER = DAYS;   // defined in network.js
const DAYNAME = {Mon:'Monday',Tue:'Tuesday',Wed:'Wednesday',Thu:'Thursday',Fri:'Friday',Sat:'Saturday',Sun:'Sunday'};
const fmtClock = (h) => { h = ((h%24)+24)%24; const hh=Math.floor(h), mm=Math.floor((h%1)*60);
  return String(hh).padStart(2,'0')+':'+String(mm).padStart(2,'0'); };
const fmtClock12 = (h) => { const hh=Math.floor(h)%24, mm=Math.floor((h%1)*60);
  const ap = hh<12?'a.m.':'p.m.'; const h12 = hh%12===0?12:hh%12;
  return h12+':'+String(mm).padStart(2,'0')+' '+ap; };

const sim = new Sim();
const state = {
  running: true, rate: 4,
  selected: null,          // explicitly chosen vehicle
  chaseVeh: null,          // vehicle the close-up is following
  stDir: 'SB', chaseDir: 'SB', track: true, lastSel: null,
  severity: 4.0, duration: 1.2,
  story: null,             // active guided story controller
  busy: false,             // settling traffic / loading
  lastDemandT: 0,
  eventSeries: [],         // drivers-caught over time, for the sparkline
  tween: { caught:0, reach:0, delay:0 }
};

/* ===================================================== render smoothing ==== */
/* The engine advances in fixed 0.1 s ticks. At 4x that is 0 or 1 ticks per screen
   frame, so drawing raw positions makes every car hop ~2.5 m and then freeze.
   Instead each car is drawn between its last two tick positions (classic
   fixed-timestep interpolation), and lane changes, which are instantaneous in
   the model, glide over about half a second. These underscore fields live only
   in the UI. The physics never reads them. */
let RENDER_ALPHA = 1;
function snapshotPrev(){
  for (const d of ['NB','SB']) for (let l=0;l<MAX_LANES;l++) for (const v of sim.lanes[d][l]) v._ps = v.s;
}
/** interpolated longitudinal position for drawing */
const rs = (v) => (v._ps == null || v.s - v._ps > 60 || v.s < v._ps) ? v.s : v._ps + (v.s - v._ps)*RENDER_ALPHA;
/** visual lane (float), eased toward the model lane */
function easeLanes(dtSim){
  const k = 1 - Math.exp(-dtSim*5);
  for (const d of ['NB','SB']) for (let l=0;l<MAX_LANES;l++) for (const v of sim.lanes[d][l]){
    if (v._vl == null || v._vd !== d) { v._vl = v.lane; v._vd = d; }
    else v._vl += (v.lane - v._vl)*k;
  }
}
const rl = (v) => v._vl == null ? v.lane : v._vl;

/* =========================================================== geometry ==== */
const LINE = GEO.line, NL = LINE.length - 1, DX = CORRIDOR_LENGTH_KM / NL;
const TAN = LINE.map((_,i) => {
  const a = LINE[Math.max(0,i-2)], b = LINE[Math.min(NL,i+2)];
  const dx=b[0]-a[0], dy=b[1]-a[1], L=Math.hypot(dx,dy)||1; return [dx/L, dy/L];   // points north (+km)
});
function geoAt(xkm){
  const f = clamp(xkm/DX, 0, NL), i = Math.min(NL-1, Math.floor(f)), t = f-i;
  const a = LINE[i], b = LINE[i+1], ta = TAN[i], tb = TAN[i+1];
  return { x:lerp(a[0],b[0],t), y:lerp(a[1],b[1],t), tx:lerp(ta[0],tb[0],t), ty:lerp(ta[1],tb[1],t) };
}
const MEDIAN_M = 5, LANE_M = 3.7;
/** world position of a vehicle, with lateral exaggeration so lanes read at any zoom */
function vehWorld(dir, s, lane, exag){
  const g = geoAt(NET[dir].xOf(s));
  const hx = dir==='NB' ? g.tx : -g.tx, hy = dir==='NB' ? g.ty : -g.ty;
  const off = (MEDIAN_M + (lane+0.5)*LANE_M) * exag;          // right-hand drive: right of heading
  return { x: g.x - hy*off, y: g.y + hx*off, hx, hy };
}

/* ============================================================== colours ==== */
// Road / heat palette: quiet and cool when free, hot when jammed.
const HEAT = [ [0,[255,40,60]], [15,[255,64,56]], [30,[255,128,58]], [48,[255,190,90]], [66,[96,178,200]], [85,[34,112,150]], [110,[20,64,96]] ];
function heat(v){
  for (let i=1;i<HEAT.length;i++) if (v <= HEAT[i][0]){
    const [v0,c0]=HEAT[i-1],[v1,c1]=HEAT[i], t=clamp((v-v0)/(v1-v0),0,1);
    return [lerp(c0[0],c1[0],t)|0, lerp(c0[1],c1[1],t)|0, lerp(c0[2],c1[2],t)|0];
  }
  return HEAT[HEAT.length-1][1];
}
const rgb = (c,a=1) => `rgba(${c[0]},${c[1]},${c[2]},${a})`;
// Space-time LUT, indexed by speed*2 (the engine stores speed*2 in a byte)
const ST_LUT = new Uint8ClampedArray(256*4);
for (let i=0;i<256;i++){
  const v=i/2, c=heat(v), fast=clamp((v-60)/40,0,1);
  const k = lerp(1, .42, fast);   // free flow recedes into the background
  ST_LUT[i*4]=c[0]*k; ST_LUT[i*4+1]=c[1]*k; ST_LUT[i*4+2]=c[2]*k; ST_LUT[i*4+3]=255;
}

/* glow sprites: one per vehicle-light bucket, drawn with additive blending */
function makeSprite(r,g,b,size=64){
  const c=document.createElement('canvas'); c.width=c.height=size; const x=c.getContext('2d');
  const grd=x.createRadialGradient(size/2,size/2,0,size/2,size/2,size/2);
  grd.addColorStop(0,`rgba(255,255,255,1)`);
  grd.addColorStop(.12,`rgba(${r},${g},${b},.95)`);
  grd.addColorStop(.35,`rgba(${r},${g},${b},.28)`);
  grd.addColorStop(1,`rgba(${r},${g},${b},0)`);
  x.fillStyle=grd; x.fillRect(0,0,size,size); return c;
}
const SPR = {
  fast: makeSprite(255,226,184), mid: makeSprite(255,196,112), slow: makeSprite(255,140,64),
  crawl: makeSprite(255,84,58), brake: makeSprite(255,40,58), sel: makeSprite(255,214,120)
};
function sprFor(veh){
  if (sim.t < veh.brakeUntil || veh.a < -2.2) return SPR.brake;
  const k = veh.v*KMH;
  if (k < 14) return SPR.crawl; if (k < 38) return SPR.slow; if (k < 66) return SPR.mid; return SPR.fast;
}

/* ================================================================ MAP ==== */
const mapC = $('map'), mapG = mapC.getContext('2d');
const cam = { x:0, y:0, z:0.06, tx:0, ty:0, tz:0.06, W:0, H:0, dpr:1 };
let baseCache = null, baseKey = '';
const PATHS = {};
(function buildPaths(){
  const mk = (lines) => { const p = new Path2D(); for (const l of lines){ p.moveTo(l[0][0],l[0][1]); for (let i=1;i<l.length;i++) p.lineTo(l[i][0],l[i][1]); } return p; };
  PATHS.sec = mk(GEO.sec);
  PATHS.art0 = mk(GEO.art.filter(a=>a.c===0).map(a=>a.p));
  PATHS.art1 = mk(GEO.art.filter(a=>a.c===1).map(a=>a.p));
  PATHS.art2 = mk(GEO.art.filter(a=>a.c===2).map(a=>a.p));
  PATHS.rail = mk(GEO.rail);
  PATHS.river = mk(GEO.river);
  PATHS.roads = mk(GEO.roads);
  const g = new Path2D(); for (const poly of GEO.green){ g.moveTo(poly[0][0],poly[0][1]); for (let i=1;i<poly.length;i++) g.lineTo(poly[i][0],poly[i][1]); g.closePath(); }
  PATHS.green = g;
})();

function fitCam(instant){
  const [x0,y0,x1,y1] = GEO.bbox, labelRoom = 150, padT = 70, padB = 120;
  const z = Math.min((cam.W - labelRoom - 40)/(x1-x0), (cam.H - padT - padB)/(y1-y0));
  // centre the corridor in the space left after the junction labels (which sit to the west)
  cam.tx = (x0+x1)/2 - (labelRoom-40)/2/z; cam.ty = (y0+y1)/2 + (padB-padT)/2/z; cam.tz = z;
  if (instant){ cam.x=cam.tx; cam.y=cam.ty; cam.z=cam.tz; }
}
const toScr = (wx,wy) => [ (wx-cam.x)*cam.z + cam.W/2, (wy-cam.y)*cam.z + cam.H/2 ];
const toWorld = (sx,sy) => [ (sx-cam.W/2)/cam.z + cam.x, (sy-cam.H/2)/cam.z + cam.y ];

function resizeCanvas(c){
  const dpr = Math.min(2, window.devicePixelRatio||1);
  const w = c.clientWidth, h = c.clientHeight;
  if (c.width !== Math.round(w*dpr) || c.height !== Math.round(h*dpr)){ c.width=Math.round(w*dpr); c.height=Math.round(h*dpr); return [w,h,dpr,true]; }
  return [w,h,dpr,false];
}

function renderBase(){
  const key = [cam.x|0, cam.y|0, cam.z.toFixed(5), cam.W, cam.H].join(',');
  if (key === baseKey && baseCache) return;
  baseKey = key;
  if (!baseCache) baseCache = document.createElement('canvas');
  baseCache.width = mapC.width; baseCache.height = mapC.height;
  const g = baseCache.getContext('2d'), d = cam.dpr;
  g.setTransform(d,0,0,d,0,0);
  // deep night background with a faint warm city haze toward downtown
  const bg = g.createRadialGradient(cam.W*.45, cam.H*1.05, 10, cam.W*.45, cam.H*.9, cam.H*1.1);
  bg.addColorStop(0,'#0d0f14'); bg.addColorStop(1,'#05070b');
  g.fillStyle = bg; g.fillRect(0,0,cam.W,cam.H);
  // world transform
  g.setTransform(d*cam.z,0,0,d*cam.z, d*(cam.W/2 - cam.x*cam.z), d*(cam.H/2 - cam.y*cam.z));
  const px = 1/cam.z;                                   // one screen pixel, in metres
  g.fillStyle = 'rgba(28,66,46,.30)'; g.fill(PATHS.green);
  g.lineCap = 'round'; g.lineJoin = 'round';
  g.strokeStyle = 'rgba(150,165,200,.075)'; g.lineWidth = .7*px; g.stroke(PATHS.sec);
  g.strokeStyle = 'rgba(160,175,210,.13)';  g.lineWidth = 1*px;  g.stroke(PATHS.art0);
  g.strokeStyle = 'rgba(180,190,220,.17)';  g.lineWidth = 1.3*px; g.stroke(PATHS.art1);
  g.strokeStyle = 'rgba(214,190,150,.22)';  g.lineWidth = 1.8*px; g.stroke(PATHS.art2);
  g.setLineDash([3*px,3*px]); g.strokeStyle = 'rgba(190,160,220,.12)'; g.lineWidth = .9*px; g.stroke(PATHS.rail); g.setLineDash([]);
  g.strokeStyle = 'rgba(60,140,200,.10)'; g.lineWidth = 7*px; g.stroke(PATHS.river);
  g.strokeStyle = 'rgba(80,160,220,.42)'; g.lineWidth = 1.3*px; g.stroke(PATHS.river);
  g.strokeStyle = 'rgba(255,255,255,.10)'; g.lineWidth = Math.max(1.4*px, 22); g.stroke(PATHS.roads);
  g.setTransform(d,0,0,d,0,0);
  // vignette
  const vg = g.createRadialGradient(cam.W/2,cam.H/2,Math.min(cam.W,cam.H)*.35,cam.W/2,cam.H/2,Math.max(cam.W,cam.H)*.75);
  vg.addColorStop(0,'rgba(0,0,0,0)'); vg.addColorStop(1,'rgba(0,0,0,.55)');
  g.fillStyle = vg; g.fillRect(0,0,cam.W,cam.H);
  // faint street labels
  g.font = '500 10px "Inter Tight", system-ui'; g.fillStyle = 'rgba(190,198,215,.28)'; g.textAlign = 'center';
  for (const l of GEO.labels){
    if (l.far === undefined){ let dmin=1e9; for (let i=0;i<LINE.length;i+=6) dmin=Math.min(dmin, Math.hypot(LINE[i][0]-l.p[0], LINE[i][1]-l.p[1])); l.far = dmin; }
    if (l.far < 1400 || /Gardiner|Lake Shore/.test(l.t)) continue;
    const [sx,sy] = toScr(l.p[0], l.p[1]);
    if (sx<-80||sx>cam.W+80||sy<0||sy>cam.H) continue;
    g.fillText(l.t.toUpperCase(), sx+ (sx < cam.W/2 ? -46 : 46), sy-6);
  }
}

function exagNow(){ return Math.max(1, 1.15/(LANE_M*cam.z)); }

function drawMap(){
  const [W,H,dpr] = resizeCanvas(mapC);
  if (W !== cam.W || H !== cam.H){ const first = !cam.W; cam.W=W; cam.H=H; cam.dpr=dpr; if (first) fitCam(true); else fitCam(false); baseKey=''; }
  cam.dpr = dpr;
  // keep the followed car in view while zoomed in
  const tv = state.selected;
  if (state.track && tv && vehicleAlive(tv) && cam.tz > 0.15 && !drag.on && !state.story){
    const p = vehWorld(tv.dir, rs(tv), rl(tv), exagNow()); cam.tx = p.x; cam.ty = p.y;
  }
  // ease camera (log-space zoom)
  const k = 0.11;
  cam.x += (cam.tx-cam.x)*k; cam.y += (cam.ty-cam.y)*k;
  cam.z = Math.exp(lerp(Math.log(cam.z), Math.log(cam.tz), k));
  renderBase();
  const g = mapG;
  g.setTransform(1,0,0,1,0,0); g.drawImage(baseCache,0,0);
  g.setTransform(dpr,0,0,dpr,0,0);
  const exag = exagNow();

  // --- corridor glow, coloured by the smoothed speed of each 100 m cell ------
  // Batched: segments are bucketed by speed so each pass is ~12 strokes, not 300.
  g.lineCap = 'round';
  const NB_ = 12, buckets = [];
  for (let b=0;b<NB_;b++) buckets.push(new Path2D());
  const bucketV = (b) => (b+0.5)*(110/NB_);
  for (const dir of ['NB','SB']){
    const c = sim.cells[dir];
    let prev = vehWorld(dir, 0, 1, exag), [px0,py0] = toScr(prev.x, prev.y);
    for (let i=0;i<NCELLS;i++){
      const b = vehWorld(dir, (i+1)*CELL_M, 1, exag), [bx,by] = toScr(b.x,b.y);
      if (!((px0<-20&&bx<-20)||(px0>W+20&&bx>W+20)||(py0<-20&&by<-20)||(py0>H+20&&by>H+20))){
        const k = clamp(Math.floor(c.vs[i]/(110/NB_)), 0, NB_-1);
        buckets[k].moveTo(px0,py0); buckets[k].lineTo(bx,by);
      }
      px0=bx; py0=by;
    }
  }
  for (let pass=0; pass<2; pass++){
    g.globalCompositeOperation = pass===0 ? 'lighter' : 'source-over';
    g.lineWidth = pass===0 ? Math.min(26, Math.max(5, 3.4*LANE_M*cam.z*exag + 4)) : Math.min(8, Math.max(1.1, 1.3*LANE_M*cam.z*exag));
    for (let b=0;b<NB_;b++){
      const v = bucketV(b), col = heat(v), slow = clamp((70-v)/60,0,1);
      const zf = clamp(0.12/cam.z, 0.18, 1);       // let the cars, not the glow, carry close zooms
      g.strokeStyle = rgb(col, (pass===0 ? 0.05+0.22*slow : 0.32+0.5*slow) * zf);
      g.stroke(buckets[b]);
    }
  }
  g.globalCompositeOperation = 'source-over';

  // --- event trail: the stretch of road the jam currently occupies ----------
  const e = sim.event;
  if (e && e.front.length){
    const last = e.front[e.front.length-1];
    if ((sim.t - e.t0) - last.t < 1.5){
      g.globalCompositeOperation = 'lighter';
      g.lineWidth = Math.max(6, 3.6*LANE_M*cam.z*exag + 6);
      const pulse = 0.55 + 0.25*Math.sin(performance.now()/180);
      g.strokeStyle = `rgba(255,40,60,${0.22*pulse})`;
      g.beginPath();
      for (let s = last.s; s <= e.s0+50; s += 25){
        const p = vehWorld(e.dir, s, 1, exag), [sx,sy] = toScr(p.x,p.y);
        s===last.s ? g.moveTo(sx,sy) : g.lineTo(sx,sy);
      }
      g.stroke();
      g.globalCompositeOperation = 'source-over';
    }
  }

  // --- vehicles as light ----------------------------------------------------
  g.globalCompositeOperation = 'lighter';
  const base = clamp(1.3 + cam.z*16, 1.5, 9);
  g.globalAlpha = clamp(0.38 + cam.z*2.2, 0.38, 1);
  let hovered = null, hd = 1e9;
  for (const dir of ['NB','SB']){
    for (let lane=0; lane<MAX_LANES; lane++){
      for (const veh of sim.lanes[dir][lane]){
        const p = vehWorld(dir, rs(veh), rl(veh), exag);
        const sx = (p.x-cam.x)*cam.z + W/2, sy = (p.y-cam.y)*cam.z + H/2;
        if (sx<-10||sx>W+10||sy<-10||sy>H+10) continue;
        const spr = sprFor(veh);
        const r = spr===SPR.brake ? base*2.6 : spr===SPR.crawl ? base*1.5 : base;
        if (spr===SPR.brake){ const ga=g.globalAlpha; g.globalAlpha=1; g.drawImage(spr, sx-r, sy-r, r*2, r*2); g.globalAlpha=ga; }
        else g.drawImage(spr, sx-r, sy-r, r*2, r*2);
        if (mouse.in){ const d = (sx-mouse.x)**2 + (sy-mouse.y)**2; if (d < hd){ hd=d; hovered={veh,sx,sy}; } }
      }
    }
  }
  g.globalAlpha = 1;
  g.globalCompositeOperation = 'source-over';

  // --- event marker + the travelling front -----------------------------------
  if (e){
    const p = vehWorld(e.dir, e.s0, e.lane0 ?? 1, exag), [sx,sy] = toScr(p.x,p.y);
    const age = sim.t - e.t0;
    for (let k2=0;k2<3;k2++){
      const ph = ((age*0.9 + k2/3) % 1);
      if (age > 6 && k2>0) continue;
      g.strokeStyle = `rgba(255,70,80,${(1-ph)*0.7})`; g.lineWidth = 1.5;
      g.beginPath(); g.arc(sx,sy, 6 + ph*34, 0, 7); g.stroke();
    }
    g.fillStyle = '#ff3b4a'; g.beginPath(); g.arc(sx,sy,3.2,0,7); g.fill();
    if (e.front.length){
      const last = e.front[e.front.length-1];
      if (age - last.t < 1.5 && e.s0 - last.s > 120){
        const q = vehWorld(e.dir, last.s, 1, exag), [fx,fy] = toScr(q.x,q.y);
        drawTag(g, fx, fy, 'jam front', ((e.s0-last.s)/1000).toFixed(2)+' km back', '#ff5b66');
      }
    }
    drawTag(g, sx, sy, 'brake tap', NET[e.dir].xOf(e.s0).toFixed(1)+' km', '#ffd1d5', true);
  }

  // --- junction labels -------------------------------------------------------
  g.font = '500 10.5px "Inter Tight", system-ui'; g.textBaseline = 'middle';
  for (const j of JUNCTIONS){
    if (j.terminal) continue;
    if (j.id==='lakeshore' && cam.z < 0.25) continue;
    if ((j.id==='queen'||j.id==='dundas') && cam.z < 0.09) continue;
    const gp = geoAt(j.x), [sx,sy] = toScr(gp.x, gp.y);
    if (sy<20||sy>H-20) continue;
    const nx = -gp.ty, ny = gp.tx;                       // left normal of north = west
    const off = 16 + (MEDIAN_M+4*LANE_M)*exag*cam.z;
    const lx = sx - off - 6, ly = sy;
    g.strokeStyle = 'rgba(255,255,255,.22)'; g.lineWidth = 1;
    g.beginPath(); g.moveTo(sx - (MEDIAN_M+4*LANE_M)*exag*cam.z - 3, sy); g.lineTo(lx+3, ly); g.stroke();
    g.textAlign = 'right';
    g.fillStyle = 'rgba(232,236,244,.78)'; g.fillText(j.short, lx - (j.exitNum?22:0), ly);
    if (j.exitNum){
      g.fillStyle = 'rgba(255,255,255,.09)'; roundRect(g, lx-18, ly-7.5, 18, 15, 4); g.fill();
      g.fillStyle = 'rgba(232,236,244,.75)'; g.font = '600 9.5px "JetBrains Mono", monospace'; g.textAlign = 'center';
      g.fillText(String(j.exitNum), lx-9, ly+.5); g.font = '500 10.5px "Inter Tight", system-ui';
    }
  }
  // terminals
  for (const [x,t] of [[15,'Hwy 401'],[0,'Gardiner']]){
    const gp = geoAt(x), [sx,sy] = toScr(gp.x,gp.y);
    g.textAlign = 'center'; g.fillStyle = 'rgba(255,226,184,.85)'; g.font = '600 11px "Inter Tight", system-ui';
    g.fillText(t.toUpperCase(), sx + (x===15?0:-10), sy + (x===15?-16:18));
  }
  g.textBaseline = 'alphabetic';

  // --- the car the close-up follows -----------------------------------------
  const cv = state.chaseVeh;
  if (cv && vehicleAlive(cv)){
    const p = vehWorld(cv.dir, rs(cv), rl(cv), exag), [sx,sy] = toScr(p.x,p.y);
    const t = performance.now()/600;
    g.strokeStyle = state.selected===cv ? 'rgba(255,214,120,.95)' : 'rgba(255,255,255,.45)'; g.lineWidth = 1.4;
    g.beginPath(); g.arc(sx,sy, 9 + Math.sin(t)*1.2, 0, 7); g.stroke();
    g.beginPath(); for (let a=0;a<4;a++){ const an=a*Math.PI/2+.0; g.moveTo(sx+Math.cos(an)*12, sy+Math.sin(an)*12); g.lineTo(sx+Math.cos(an)*16, sy+Math.sin(an)*16);} g.stroke();
  }

  // hover tooltip
  const tip = $('tip');
  if (hovered && hd < 140 && !drag.on){
    const v = hovered.veh;
    tip.style.left = hovered.sx+'px'; tip.style.top = hovered.sy+'px'; tip.style.opacity = 1;
    tip.innerHTML = `<span class="mono">${Math.round(v.v*KMH)} km/h</span> · ${v.dir==='NB'?'northbound':'southbound'} · ${v.truck?'truck':'car'}${v.exit?' → '+v.exit.short:''}`;
    mapC.style.cursor = 'pointer'; state.hover = v;
  } else { tip.style.opacity = 0; mapC.style.cursor = drag.on ? 'grabbing' : 'grab'; state.hover = null; }
}

function drawTag(g, x, y, k, v, col, below){
  g.font = '600 9.5px "Inter Tight", system-ui';
  const t1 = k.toUpperCase(), w = Math.max(g.measureText(t1).width, (g.font='500 11px "JetBrains Mono", monospace', g.measureText(v).width)) + 16;
  const flip = x + 14 + w > cam.W - 8;
  const bx = flip ? x - 14 - w : x + 14, by = below ? y + 8 : y - 34;
  g.fillStyle = 'rgba(8,10,16,.86)'; roundRect(g, bx, by, w, 28, 6); g.fill();
  g.strokeStyle = col; g.globalAlpha = .55; g.lineWidth = 1; roundRect(g, bx, by, w, 28, 6); g.stroke(); g.globalAlpha = 1;
  g.beginPath(); g.moveTo(x + (flip?-4:4), y); g.lineTo(flip ? bx+w : bx, by+14); g.stroke();
  g.textAlign = 'left';
  g.fillStyle = col; g.font = '600 9.5px "Inter Tight", system-ui'; g.fillText(t1, bx+8, by+11);
  g.fillStyle = '#eef1f7'; g.font = '500 11px "JetBrains Mono", monospace'; g.fillText(v, bx+8, by+23);
}
function roundRect(g,x,y,w,h,r){ g.beginPath(); g.moveTo(x+r,y); g.arcTo(x+w,y,x+w,y+h,r); g.arcTo(x+w,y+h,x,y+h,r); g.arcTo(x,y+h,x,y,r); g.arcTo(x,y,x+w,y,r); g.closePath(); }

/* map interaction */
const mouse = { x:0, y:0, in:false };
const drag = { on:false, moved:false, sx:0, sy:0, cx:0, cy:0 };
mapC.addEventListener('mousemove', e => { const r=mapC.getBoundingClientRect(); mouse.x=e.clientX-r.left; mouse.y=e.clientY-r.top; mouse.in=true;
  if (drag.on){ const dx=mouse.x-drag.sx, dy=mouse.y-drag.sy; if (Math.abs(dx)+Math.abs(dy)>4) drag.moved=true;
    cam.tx = cam.x = drag.cx - dx/cam.z; cam.ty = cam.y = drag.cy - dy/cam.z;
    if (drag.moved && state.track && state.selected){ state.track = false; syncTrack(); } } });
mapC.addEventListener('mouseleave', () => { mouse.in=false; });
mapC.addEventListener('mousedown', e => { if (state.story) return; drag.on=true; drag.moved=false; drag.sx=mouse.x; drag.sy=mouse.y; drag.cx=cam.x; drag.cy=cam.y; });
window.addEventListener('mouseup', () => {
  if (drag.on && !drag.moved && state.hover){ selectVehicle(state.hover); }
  drag.on=false; });
mapC.addEventListener('wheel', e => { if (state.story) return; e.preventDefault();
  const f = Math.exp(-e.deltaY*0.0015), [wx,wy] = toWorld(mouse.x,mouse.y);
  const nz = clamp(cam.tz*f, 0.03, 2.5);
  cam.tx = wx - (mouse.x-cam.W/2)/nz; cam.ty = wy - (mouse.y-cam.H/2)/nz; cam.tz = nz;
  cam.x = cam.tx; cam.y = cam.ty; cam.z = nz; }, { passive:false });
mapC.addEventListener('dblclick', () => fitCam(false));
$('zin').onclick = () => { cam.tz = clamp(cam.tz*1.6, .03, 2.5); };
$('zout').onclick = () => { cam.tz = clamp(cam.tz/1.6, .03, 2.5); };
$('zfit').onclick = () => fitCam(false);
function flyTo(veh, z){ const p = vehWorld(veh.dir, veh.s, veh.lane, 1); cam.tx = p.x; cam.ty = p.y; cam.tz = z; }

function vehicleAlive(v){ return v && sim.lanes[v.dir][v.lane] && sim.lanes[v.dir][v.lane].includes(v); }
function selectVehicle(v){ state.selected = v; state.chaseVeh = v; state.chaseDir = v.dir; state.track = true; syncTrack(); syncChaseDir(); renderStory(true); }
function syncTrack(){ const b = $('trackBtn'); if (b) b.classList.toggle('on', state.track); }

/* ============================================================== CHASE ==== */
const chC = $('chase'), chG = chC.getContext('2d');
const chase = { s:null, dir:null, span:118 };
const PAINT = ['#d8dde6','#9aa3b2','#3a4252','#1b2230','#7e2a2a','#2c4a6b','#4d5a3a','#b8a07a','#5b5f69','#e8e6df','#24303f','#6a1f2b'];
// pre-rendered asphalt noise
const ASPH = (() => { const c=document.createElement('canvas'); c.width=c.height=128; const x=c.getContext('2d');
  const im=x.createImageData(128,128); for (let i=0;i<im.data.length;i+=4){ const n=14+Math.random()*10; im.data[i]=n; im.data[i+1]=n+1; im.data[i+2]=n+4; im.data[i+3]=255; }
  x.putImageData(im,0,0); return c; })();
let asphPat = null;

function nearestCarTo(dir, sTarget, maxD=1e9){
  let best=null, bd=maxD;
  for (let lane=0; lane<BASE_LANES; lane++) for (const v of sim.lanes[dir][lane]){
    const d = Math.abs(v.s - sTarget); if (d < bd){ bd=d; best=v; }
  }
  return best;
}
function pickChaseVehicle(){
  // 1. the car you picked, for as long as it is on the road
  if (state.selected){
    if (vehicleAlive(state.selected)){ state.lastSel = { dir: state.selected.dir, s: rs(state.selected) }; return state.selected; }
    // it took an exit or reached the end: hand off to the car now nearest that spot, keep following
    const ls = state.lastSel, h = ls && ls.s < CORRIDOR_LENGTH_M-300 ? nearestCarTo(ls.dir, ls.s, 600) : null;
    state.selected = h; if (h){ state.handoff = performance.now(); return h; }
  }
  // 2. nothing picked: follow the car nearest the map centre, and re-target when the map is panned away
  const cv = state.chaseVeh;
  if (cv && vehicleAlive(cv)){
    const p = vehWorld(cv.dir, rs(cv), rl(cv), 1);
    const far = Math.hypot(p.x-cam.tx, p.y-cam.ty) > Math.max(500, 90/cam.z);
    if (!far) return cv;
  }
  let best=null, bd=1e18;
  for (const dir of [state.chaseDir, state.chaseDir==='SB'?'NB':'SB']){
    for (let lane=0; lane<BASE_LANES; lane++) for (const v of sim.lanes[dir][lane]){
      if (v.s < 300 || v.s > CORRIDOR_LENGTH_M-500) continue;
      const p = vehWorld(dir, v.s, lane, 1), d=(p.x-cam.tx)**2+(p.y-cam.ty)**2;
      if (d<bd){ bd=d; best=v; }
    }
    if (best) break;
  }
  return best;
}

/** Jump the close-up (and the map) to a kilometre post in a direction. */
function jumpTo(dir, km, opts={}){
  km = clamp(km, 0.3, CORRIDOR_LENGTH_KM-0.4);
  const v = nearestCarTo(dir, NET[dir].sOf(km));
  if (!v) return;
  state.chaseDir = dir; state.selected = v; state.chaseVeh = v;
  if (opts.cut !== false){ chase.s = null; chase.flash = 1; }
  const p = vehWorld(dir, rs(v), rl(v), 1);
  cam.tx = p.x; cam.ty = p.y; if (opts.zoom !== false) cam.tz = Math.max(cam.tz, 0.32);
  syncChaseDir(); renderStory(true);
}
function syncChaseDir(){ $('chDir').querySelectorAll('button').forEach(b => b.classList.toggle('on', b.dataset.d === state.chaseDir)); }

function drawChase(){
  const [W,H,dpr] = resizeCanvas(chC);
  const g = chG; g.setTransform(dpr,0,0,dpr,0,0);
  const veh = state.chaseVeh = pickChaseVehicle();
  if (veh) state.chaseDir = veh.dir;
  chase.hits = [];
  // background: dark verge
  const bg = g.createLinearGradient(0,0,0,H); bg.addColorStop(0,'#05070a'); bg.addColorStop(.5,'#080b0e'); bg.addColorStop(1,'#05070a');
  g.fillStyle = bg; g.fillRect(0,0,W,H);
  if (!veh){ return; }
  const vs = rs(veh);
  if (chase.dir !== veh.dir || chase.s == null || Math.abs(chase.s - vs) > 400){ chase.dir = veh.dir; chase.s = vs; chase.off = 0; chase.target = veh; }
  if (chase.target !== veh){ chase.off = chase.s - vs; chase.target = veh; }   // new car: glide over, don't cut
  chase.off *= Math.exp(-state.wallDt*4);
  if (Math.abs(chase.off) < 0.01) chase.off = 0;
  chase.s = vs + chase.off;
  const dir = chase.dir, opp = dir==='NB'?'SB':'NB';
  const topPad = 46, botPad = 84, avail = H - topPad - botPad;
  const ppm = Math.min(W / chase.span, avail / (6*LANE_M + 3.2));   // fit 6 main lanes + median
  const cx = NET[dir].xOf(chase.s);                              // km under the camera
  const anchor = W*0.40;
  const dsign = dir==='NB' ? 1 : -1;
  const X = (xkm) => anchor + (xkm - cx)*1000*ppm*dsign;
  const laneH = LANE_M*ppm, medH = 3.2*ppm;
  const mid = topPad + avail/2;
  const yMain = (lane) => mid + medH/2 + lane*laneH;             // top edge of a main-dir lane
  const yOpp  = (lane) => mid - medH/2 - (lane+1)*laneH;         // top edge of an opposite-dir lane
  const sAt = (d, px) => { const xkm = cx + (px - anchor)/(1000*ppm*dsign); return NET[d].sOf(xkm); };

  // asphalt
  if (!asphPat) asphPat = g.createPattern(ASPH,'repeat');
  g.save(); g.fillStyle = asphPat;
  const scroll = ((cx*1000*ppm*dsign) % 128 + 128) % 128;
  g.translate(-scroll, 0);
  for (const [d, yfn] of [[dir,yMain],[opp,yOpp]]){
    for (let lane=0; lane<MAX_LANES; lane++){
      const y = yfn(lane);
      if (lane < BASE_LANES){ g.fillRect(scroll-2, y, W+4, laneH); continue; }
      for (let px=0; px<W; px+=6){ if (sim.laneExists(d, lane, sAt(d,px))) g.fillRect(px+scroll, y, 6.5, laneH); }
    }
  }
  g.restore();
  // median barrier
  const mg = g.createLinearGradient(0,mid-medH/2,0,mid+medH/2);
  mg.addColorStop(0,'#1b1f26'); mg.addColorStop(.5,'#3a3f48'); mg.addColorStop(1,'#1b1f26');
  g.fillStyle = mg; g.fillRect(0, mid-medH*0.18, W, medH*0.36);
  // lane markings (3 m dash / 6 m gap, positioned in world metres so they scroll)
  const dashPx = 3*ppm, gapPx = 6*ppm, per = dashPx+gapPx;
  const off = ((cx*1000*ppm*dsign) % per + per) % per;
  g.fillStyle = 'rgba(235,238,245,.55)';
  for (const yfn of [yMain, yOpp]){
    for (let lane=1; lane<BASE_LANES; lane++){
      const y = (yfn===yMain ? yMain(lane) : yOpp(lane-1) + laneH) - 0.6;
      for (let x=-off; x<W; x+=per) g.fillRect(x, y, dashPx, 1.3);
    }
  }
  // edge lines + aux-lane lines
  g.fillStyle = 'rgba(255,214,120,.55)'; g.fillRect(0, yMain(0)-1, W, 1.4); g.fillRect(0, yOpp(0)+laneH-.4, W, 1.4);
  for (const [d, yfn, top] of [[dir,yMain,false],[opp,yOpp,true]]){
    for (let px=0; px<W; px+=3){
      const aux = sim.laneExists(d, AUX, sAt(d,px));
      const yEdge = top ? (aux ? yfn(AUX) : yfn(BASE_LANES-1)) : (aux ? yfn(AUX)+laneH : yfn(BASE_LANES-1)+laneH);
      g.fillStyle = 'rgba(235,238,245,.6)'; g.fillRect(px, yEdge - (top?0:1.2), 3, 1.2);
      if (aux && ((px/3)|0)%3===0){ const yl = top ? yfn(AUX)+laneH : yfn(AUX); g.fillStyle='rgba(235,238,245,.4)'; g.fillRect(px, yl-.6, 4, 1.2); }
    }
  }

  // vehicles: opposite direction first, then ours
  const halfSpanKm = (W/ppm)/1000*0.6 + 0.02;
  for (const [d, yfn, flip] of [[opp,yOpp,true],[dir,yMain,false]]){
    const sC = NET[d].sOf(cx), sLo = Math.max(0, sC - halfSpanKm*1000), sHi = Math.min(CORRIDOR_LENGTH_M, sC + halfSpanKm*1000);
    const list = sim.window(d, sLo, sHi);
    // headlight cones under the bodies (additive)
    g.globalCompositeOperation = 'lighter';
    for (const v of list){
      const x = X(NET[d].xOf(rs(v))), y = yfn(rl(v)) + laneH/2, hdg = flip ? -1 : 1;
      const L = 26*ppm, w = 1.2*ppm;
      const grd = g.createLinearGradient(x, y, x + hdg*L, y);
      grd.addColorStop(0,'rgba(255,236,200,.16)'); grd.addColorStop(1,'rgba(255,236,200,0)');
      g.fillStyle = grd; g.beginPath(); g.moveTo(x, y-w*.6); g.lineTo(x+hdg*L, y-w*2.2); g.lineTo(x+hdg*L, y+w*2.2); g.lineTo(x, y+w*.6); g.closePath(); g.fill();
    }
    g.globalCompositeOperation = 'source-over';
    for (const v of list){
      const xf = X(NET[d].xOf(rs(v))), yc = yfn(rl(v)) + laneH/2, hd = flip ? -1 : 1;
      drawCar(g, v, xf, yc, hd, ppm, v === veh);
      const L = v.len*ppm; chase.hits.push({ v, x0: hd>0 ? xf-L : xf, x1: hd>0 ? xf : xf+L, y0: yc-laneH/2, y1: yc+laneH/2 });
    }
  }

  // speed tag above the followed car
  const vx = X(NET[dir].xOf(rs(veh))), vy = yMain(rl(veh));
  g.font = '500 11px "JetBrains Mono", monospace'; g.textAlign = 'center';
  const tag = Math.round(veh.v*KMH)+' km/h';
  const tw = g.measureText(tag).width + 14;
  g.fillStyle = 'rgba(8,10,16,.82)'; roundRect(g, vx - veh.len*ppm/2 - tw/2, vy - 24, tw, 18, 9); g.fill();
  g.fillStyle = sim.t < veh.brakeUntil ? '#ff8b8b' : '#ffe2b8'; g.fillText(tag, vx - veh.len*ppm/2, vy - 11);
  // edge fades
  const fl = g.createLinearGradient(0,0,W,0); fl.addColorStop(0,'rgba(5,7,10,.9)'); fl.addColorStop(.08,'rgba(5,7,10,0)'); fl.addColorStop(.92,'rgba(5,7,10,0)'); fl.addColorStop(1,'rgba(5,7,10,.9)');
  g.fillStyle = fl; g.fillRect(0,0,W,H);
  if (chase.flash > 0){ g.fillStyle = `rgba(5,7,10,${chase.flash})`; g.fillRect(0,0,W,H); chase.flash = Math.max(0, chase.flash - state.wallDt*3.5); }

  // HUD text
  const near = nearestJunction(NET[dir].xOf(veh.s));
  $('chaseTitle').innerHTML = `<b>Close-up</b> · ${dir==='NB'?'northbound':'southbound'} · km ${NET[dir].xOf(veh.s).toFixed(1)} near ${near.short}`;
  $('carinfo').innerHTML = `<b>${veh.truck?'Truck':'Car'} #${veh.id}</b> · <span class="mono">${Math.round(veh.v*KMH)} km/h</span> · lane ${veh.lane===AUX?'ramp':veh.lane+1} · ${veh.exit?'exits at '+veh.exit.short:'through traffic'}`;
  $('chaseHint').textContent = state.selected===veh
    ? (state.handoff && performance.now()-state.handoff < 4000 ? 'Your car exited. Following the one behind it.' : 'Following your car')
    : 'Following the car nearest the map centre. Click one to lock on';
  drawLocator(veh);
}

function drawCar(g, v, xFront, yC, hdg, ppm, focus){
  const L = v.len*ppm, Wd = (v.truck?2.5:1.85)*ppm;
  const x0 = hdg>0 ? xFront - L : xFront;                  // rear-left corner in screen x
  const y0 = yC - Wd/2;
  const braking = sim.t < v.brakeUntil || v.a < -1.4;
  // shadow
  g.fillStyle = 'rgba(0,0,0,.45)'; roundRect(g, x0+1.5, y0+2, L, Wd, Math.min(4, Wd*.35)); g.fill();
  // body
  const col = PAINT[(v.id*7) % PAINT.length];
  const bd = g.createLinearGradient(0, y0, 0, y0+Wd);
  bd.addColorStop(0, shade(col, 22)); bd.addColorStop(.5, col); bd.addColorStop(1, shade(col, -30));
  g.fillStyle = v.truck ? '#cfd3da' : bd; roundRect(g, x0, y0, L, Wd, Math.min(4, Wd*.35)); g.fill();
  if (v.truck){
    g.fillStyle = shade(col,-10); const cab = Math.min(L*.22, 3.2*ppm);
    roundRect(g, hdg>0 ? x0+L-cab : x0, y0, cab, Wd, 3); g.fill();
    g.fillStyle = 'rgba(20,26,36,.85)'; g.fillRect(hdg>0 ? x0+L-cab*0.55 : x0+cab*0.25, y0+Wd*.14, cab*0.3, Wd*.72);
  } else {
    // glasshouse
    g.fillStyle = 'rgba(14,18,26,.86)';
    const wsx = hdg>0 ? x0 + L*.58 : x0 + L*.18, wsw = L*.24;
    roundRect(g, wsx, y0 + Wd*.14, wsw, Wd*.72, 2.5); g.fill();
    g.fillStyle = 'rgba(255,255,255,.08)'; g.fillRect(wsx, y0 + Wd*.14, wsw, Wd*.12);
  }
  // headlights
  const fx = hdg>0 ? x0+L-1.2 : x0, rx = hdg>0 ? x0 : x0+L-2.2;
  g.fillStyle = '#fff4dc'; g.fillRect(fx, y0+Wd*.12, 1.4, Wd*.2); g.fillRect(fx, y0+Wd*.68, 1.4, Wd*.2);
  // tail lights (+ bloom when braking)
  g.fillStyle = braking ? '#ff2a3d' : '#7a1520';
  g.fillRect(rx, y0+Wd*.08, 2.2, Wd*.24); g.fillRect(rx, y0+Wd*.68, 2.2, Wd*.24);
  if (braking){
    g.globalCompositeOperation = 'lighter';
    const r = (sim.t < v.brakeUntil ? 7.5 : 4.5)*ppm*0.7;
    const bx = hdg>0 ? x0 : x0+L;
    const gr = g.createRadialGradient(bx, yC, 0, bx, yC, r);
    gr.addColorStop(0,'rgba(255,50,70,.75)'); gr.addColorStop(1,'rgba(255,40,60,0)');
    g.fillStyle = gr; g.fillRect(bx-r, yC-r, r*2, r*2);
    // reflection streak on the road behind
    const sl = 9*ppm, sg = g.createLinearGradient(bx, 0, bx - hdg*sl, 0);
    sg.addColorStop(0,'rgba(255,40,60,.22)'); sg.addColorStop(1,'rgba(255,40,60,0)');
    g.fillStyle = sg; g.fillRect(Math.min(bx, bx-hdg*sl), yC-Wd*.45, sl, Wd*.9);
    g.globalCompositeOperation = 'source-over';
  }
  if (focus){
    g.strokeStyle = 'rgba(255,214,120,.95)'; g.lineWidth = 1.4;
    roundRect(g, x0-3, y0-3, L+6, Wd+6, 5); g.stroke();
  }
}
function shade(hex, amt){ const n=parseInt(hex.slice(1),16); const r=clamp((n>>16)+amt,0,255), g=clamp(((n>>8)&255)+amt,0,255), b=clamp((n&255)+amt,0,255); return `rgb(${r},${g},${b})`; }
function nearestJunction(x){ let b=JUNCTIONS[0], d=1e9; for (const j of JUNCTIONS){ const dd=Math.abs(j.x-x); if (dd<d){d=dd;b=j;} } return b; }
chC.addEventListener('click', (e) => {
  if (state.story) return;
  const r = chC.getBoundingClientRect(), x = e.clientX - r.left, y = e.clientY - r.top;
  let hit = null, bd = 1e9;
  for (const h of (chase.hits||[])){
    const cx = (h.x0+h.x1)/2, cy = (h.y0+h.y1)/2;
    if (x >= h.x0-6 && x <= h.x1+6 && y >= h.y0-4 && y <= h.y1+4){ const d=(x-cx)**2+(y-cy)**2; if (d<bd){bd=d;hit=h.v;} }
  }
  if (hit) selectVehicle(hit);
});
chC.style.cursor = 'pointer';

/* ------------------------------------------------- locator: the whole 15 km */
const locC = $('loc'), locG = locC.getContext('2d');
const LOC = { L:70, R:70 };
function drawLocator(veh){
  const [W,H,dpr] = resizeCanvas(locC);
  const g = locG; g.setTransform(dpr,0,0,dpr,0,0); g.clearRect(0,0,W,H);
  const L = LOC.L, R = LOC.R, pw = W-L-R, y = 15, bh = 6;
  const xk = (km) => L + (km/CORRIDOR_LENGTH_KM)*pw;
  const dir = state.chaseDir, c = sim.cells[dir];
  // speed along the carriageway, so you can see where the jams are and jump straight to one
  for (let i=0;i<NCELLS;i++){
    const a = NET[dir].xOf(i*CELL_M), b = NET[dir].xOf((i+1)*CELL_M);
    const v = c.vs[i], col = heat(v), slow = clamp((75-v)/60,0,1);
    g.fillStyle = rgb(col, 0.35 + 0.65*slow);
    g.fillRect(xk(Math.min(a,b)), y, Math.abs(xk(b)-xk(a))+0.6, bh);
  }
  // junction ticks
  g.font = '500 9.5px "Inter Tight", system-ui'; g.textAlign = 'center';
  for (const j of JUNCTIONS){
    if (['lakeshore','queen','dundas','danforth','wynford'].includes(j.id) || j.terminal) continue;
    const x = xk(j.x);
    g.fillStyle = 'rgba(255,255,255,.35)'; g.fillRect(x, y-3, 1, bh+6);
    g.fillStyle = 'rgba(196,202,214,.6)'; g.fillText(j.short.replace('/Bloor',''), x, y+bh+13);
  }
  g.textAlign = 'right'; g.fillStyle = 'rgba(255,226,184,.8)'; g.font = '600 9.5px "Inter Tight", system-ui';
  g.fillText('GARDINER', L-8, y+bh-0.5);
  g.textAlign = 'left'; g.fillText('HWY 401', W-R+8, y+bh-0.5);
  // brake event
  const e = sim.event;
  if (e && e.dir === dir){ const x = xk(NET[dir].xOf(e.s0)); g.fillStyle = '#ff3b4a'; g.beginPath(); g.arc(x, y+bh/2, 3.2, 0, 7); g.fill(); }
  // where the close-up is
  if (veh){
    const x = xk(NET[veh.dir].xOf(rs(veh)));
    g.fillStyle = 'rgba(255,226,184,.16)'; g.fillRect(x-8, y-6, 16, bh+12);
    g.strokeStyle = '#ffe2b8'; g.lineWidth = 1.5; g.strokeRect(x-8, y-6, 16, bh+12);
    // direction of travel arrow
    g.fillStyle = '#ffe2b8'; g.beginPath();
    const ax = x + (veh.dir==='NB' ? 12 : -12);
    g.moveTo(ax, y+bh/2); g.lineTo(ax + (veh.dir==='NB'?-5:5), y-1); g.lineTo(ax + (veh.dir==='NB'?-5:5), y+bh+1); g.closePath(); g.fill();
  }
  if (locDrag.hover != null){ g.fillStyle='rgba(255,255,255,.5)'; g.fillRect(locDrag.hover-0.5, y-7, 1, bh+14); }
}
const locDrag = { on:false, hover:null };
function locKm(e){ const r = locC.getBoundingClientRect(); return ((e.clientX - r.left - LOC.L)/(r.width - LOC.L - LOC.R))*CORRIDOR_LENGTH_KM; }
locC.addEventListener('pointerdown', e => { if (state.story) return; locDrag.on = true; locC.setPointerCapture(e.pointerId); jumpTo(state.chaseDir, locKm(e)); });
locC.addEventListener('pointermove', e => { const r = locC.getBoundingClientRect(); locDrag.hover = e.clientX - r.left;
  if (locDrag.on) jumpTo(state.chaseDir, locKm(e), { cut:false }); });
locC.addEventListener('pointerup', () => locDrag.on = false);
locC.addEventListener('pointerleave', () => locDrag.hover = null);


/* ============================================================ SCRUBBER ==== */
const scC = $('scrub'), scG = scC.getContext('2d');
const PRESETS = [
  { id:'sun',  day:'Sun', t:10.5, dir:'SB', label:'Sun 10:30', note:'Light traffic' },
  { id:'am',   day:'Thu', t:9.0,  dir:'SB', label:'Thu 09:00', note:'Southbound rush' },
  { id:'mid',  day:'Thu', t:13.0, dir:'NB', label:'Thu 13:00', note:'Knife edge' },
  { id:'pm',   day:'Fri', t:17.5, dir:'NB', label:'Fri 17:30', note:'Worst hour' }
];
let scrubDrag = false;
function drawScrub(){
  const [W,H,dpr] = resizeCanvas(scC);
  const g = scG; g.setTransform(dpr,0,0,dpr,0,0); g.clearRect(0,0,W,H);
  const L=6, R=6, T=8, B=18, pw=W-L-R, ph=H-T-B;
  const xh = (h) => L + (h/24)*pw, yv = (v) => T + (1 - v/110)*ph;
  const obs = cal().observedSpeeds;
  // hour grid
  g.fillStyle = 'rgba(255,255,255,.035)';
  for (let h=0; h<24; h+=2) g.fillRect(xh(h), T, pw/24, ph);
  g.fillStyle = 'rgba(132,142,162,.7)'; g.font = '10px "JetBrains Mono", monospace'; g.textAlign = 'center';
  for (const h of [0,3,6,9,12,15,18,21]) g.fillText(String(h).padStart(2,'0'), xh(h)+ (h===0?8:0), H-4);
  // 60 km/h guide
  g.strokeStyle = 'rgba(255,255,255,.08)'; g.setLineDash([2,4]); g.beginPath(); g.moveTo(L, yv(60)); g.lineTo(L+pw, yv(60)); g.stroke(); g.setLineDash([]);
  g.fillStyle = 'rgba(132,142,162,.55)'; g.textAlign='left'; g.fillText('60', L+2, yv(60)-3); g.fillText('100 km/h', L+2, yv(100)-3);
  if (obs){
    const weekend = sim.day==='Sat'||sim.day==='Sun';
    for (const [dir,col] of [['SB',[255,181,71]],['NB',[111,211,255]]]){
      const d = obs[dir], pts = [];
      for (let i=0;i<=48;i++){ const h=i/2, a=d[Math.floor(h)%24], b=d[(Math.floor(h)+1)%24], f=h%1, w=.5-.5*Math.cos(Math.PI*f); pts.push([xh(h), yv(a*(1-w)+b*w)]); }
      const fill = g.createLinearGradient(0,T,0,T+ph); fill.addColorStop(0,rgb(col,0)); fill.addColorStop(1,rgb(col,weekend?.04:.12));
      g.beginPath(); g.moveTo(pts[0][0], T); for (const p of pts) g.lineTo(p[0],p[1]); g.lineTo(pts[pts.length-1][0], T); g.closePath();
      // shade ABOVE the curve = speed lost
      const lost = g.createLinearGradient(0,T,0,T+ph); lost.addColorStop(0,rgb(col,0)); lost.addColorStop(1,rgb(col,weekend?.03:.10));
      g.fillStyle = lost; g.fill();
      g.strokeStyle = rgb(col, weekend?.35:.9); g.lineWidth = 1.6; g.beginPath(); pts.forEach((p,i)=> i?g.lineTo(p[0],p[1]):g.moveTo(p[0],p[1])); g.stroke();
    }
    if (weekend){ g.fillStyle='rgba(196,202,214,.6)'; g.textAlign='right'; g.font='11px "Inter Tight", system-ui'; g.fillText('weekday curves shown · weekend demand is scaled down', L+pw-4, T+12); }
  }
  // preset pins
  for (const p of PRESETS){
    if (p.day !== sim.day) continue;
    const x = xh(p.t); g.fillStyle='rgba(255,255,255,.35)'; g.beginPath(); g.arc(x, T+ph+1, 2.2, 0, 7); g.fill();
  }
  // playhead
  const x = xh(sim.clockHours);
  const glow = g.createLinearGradient(x-26,0,x+26,0); glow.addColorStop(0,'rgba(255,226,184,0)'); glow.addColorStop(.5,'rgba(255,226,184,.10)'); glow.addColorStop(1,'rgba(255,226,184,0)');
  g.fillStyle = glow; g.fillRect(x-26, T, 52, ph);
  g.strokeStyle = '#ffe2b8'; g.lineWidth = 1.5; g.beginPath(); g.moveTo(x, T-2); g.lineTo(x, T+ph); g.stroke();
  g.fillStyle = '#ffe2b8'; g.beginPath(); g.arc(x, T-1, 4, 0, 7); g.fill();
  if (obs){
    for (const [dir,col] of [['SB','#ffb547'],['NB','#6fd3ff']]){
      const hh=sim.clockHours, a=obs[dir][Math.floor(hh)%24], b=obs[dir][(Math.floor(hh)+1)%24], f=hh%1, w=.5-.5*Math.cos(Math.PI*f), v=a*(1-w)+b*w;
      g.fillStyle = col; g.beginPath(); g.arc(x, yv(v), 3.2, 0, 7); g.fill();
    }
  }
  g.font='600 11px "JetBrains Mono", monospace'; g.textAlign = x > W-60 ? 'right' : 'left';
  g.fillStyle = '#ffe2b8'; g.fillText(fmtClock(sim.clockHours), x + (x > W-60 ? -8 : 8), T+10);
}
function scrubTo(ev){
  const r = scC.getBoundingClientRect(), L=6, pw=r.width-12;
  const h = clamp(((ev.clientX - r.left - L)/pw)*24, 0, 23.98);
  sim.clockHours = Math.round(h*12)/12; sim.refreshDemand(); state.lastDemandT = sim.t;
  setPresetChip(null);
}
scC.addEventListener('pointerdown', e => { if (state.story) return; scrubDrag = true; scC.setPointerCapture(e.pointerId); scrubTo(e); });
scC.addEventListener('pointermove', e => { if (scrubDrag) scrubTo(e); });
scC.addEventListener('pointerup', () => scrubDrag = false);

function buildDayAndChips(){
  $('days').innerHTML = DAY_ORDER.map(d=>`<button data-d="${d}">${d}</button>`).join('');
  $('days').querySelectorAll('button').forEach(b => b.onclick = () => { if (state.story) return; sim.day = b.dataset.d; sim.refreshDemand(); setPresetChip(null); syncDays(); });
  $('chips').innerHTML = PRESETS.map(p=>`<button class="chip" data-p="${p.id}" title="${p.note}">${p.label}</button>`).join('');
  $('chips').querySelectorAll('button').forEach(b => b.onclick = () => { if (state.story) return; const p = PRESETS.find(q=>q.id===b.dataset.p); loadScenario(p.day, p.t, p.dir, 11, p.id); });
}
function syncDays(){ $('days').querySelectorAll('button').forEach(b => b.classList.toggle('on', b.dataset.d === sim.day)); }
function setPresetChip(id){ $('chips').querySelectorAll('button').forEach(b => b.classList.toggle('on', b.dataset.p === id)); }

/* ========================================================== SPACE-TIME ==== */
const stC = $('st'), stG = stC.getContext('2d');
const ST = { NB:null, SB:null, lastT:{NB:-1,SB:-1}, cols:sim.stMaxRows };
function stBuf(dir){
  if (!ST[dir]){ const c=document.createElement('canvas'); c.width=ST.cols; c.height=NCELLS; const x=c.getContext('2d'); x.fillStyle='#070a10'; x.fillRect(0,0,c.width,c.height); ST[dir]={c,x,col:x.createImageData(1,NCELLS)}; }
  return ST[dir];
}
function stIngest(){
  for (const dir of ['NB','SB']){
    const b = stBuf(dir), H = sim.stHistory[dir];
    if (!H.length) continue;
    let start = H.length;
    while (start > 0 && H[start-1].t > ST.lastT[dir]) start--;
    for (let k=start; k<H.length; k++){
      const row = H[k].row;
      // shift left one column, then paint the newest column on the right
      b.x.globalCompositeOperation = 'copy'; b.x.drawImage(b.c, -1, 0); b.x.globalCompositeOperation = 'source-over';
      const d = b.col.data;
      for (let i=0;i<NCELLS;i++){
        const x = NET[dir].xOf(i*CELL_M + CELL_M/2);            // km, north positive
        const yi = Math.min(NCELLS-1, Math.max(0, Math.floor((CORRIDOR_LENGTH_KM - x)/CORRIDOR_LENGTH_KM*NCELLS)));
        const v = row[i], o = yi*4;
        d[o]=ST_LUT[v*4]; d[o+1]=ST_LUT[v*4+1]; d[o+2]=ST_LUT[v*4+2]; d[o+3]=255;
      }
      b.x.putImageData(b.col, ST.cols-1, 0);
      ST.lastT[dir] = H[k].t;
    }
  }
}
function stReset(){ for (const d of ['NB','SB']){ ST[d]=null; ST.lastT[d]=-1; } }
function drawST(){
  const [W,H,dpr] = resizeCanvas(stC);
  const g = stG; g.setTransform(dpr,0,0,dpr,0,0); g.clearRect(0,0,W,H);
  const L=96, R=16, T=4, B=24, pw=W-L-R, ph=H-T-B;
  const b = stBuf(state.stDir);
  g.imageSmoothingEnabled = true; g.drawImage(b.c, 0, 0, ST.cols, NCELLS, L, T, pw, ph);
  // soft top/bottom fade
  // junction axis
  g.font = '500 10.5px "Inter Tight", system-ui'; g.textAlign = 'right'; g.textBaseline = 'middle';
  for (const j of JUNCTIONS){
    if (j.terminal && j.id!=='hwy401' && j.id!=='gardiner') continue;
    if (['queen','dundas','danforth','lakeshore','wynford'].includes(j.id)) continue;
    const y = T + (1 - j.x/CORRIDOR_LENGTH_KM)*ph;
    g.fillStyle = 'rgba(196,202,214,.62)'; g.fillText(j.short, L-10, y);
    g.fillStyle = 'rgba(255,255,255,.07)'; g.fillRect(L, y, pw, 1);
  }
  g.textBaseline = 'alphabetic';
  // flow arrow
  g.fillStyle = 'rgba(132,142,162,.7)'; g.textAlign='left'; g.font = '10px "Inter Tight", system-ui';
  g.fillText(state.stDir==='SB' ? '↓ traffic flows toward the Gardiner' : '↑ traffic flows toward the 401', L+6, T+ph+16);
  g.textAlign='right'; g.font = '10px "JetBrains Mono", monospace';
  const mins = (ST.cols*0.5/60);
  g.textAlign='left'; g.fillText(`← ${mins.toFixed(1)} min ago`, L+pw*0.55, T+ph+16); g.textAlign='right'; g.fillText('now', L+pw, T+ph+16);
  // event overlay
  const e = sim.event;
  if (e && e.dir === state.stDir){
    const age = sim.t - e.t0, colsAgo = age/0.5;
    const ex = L + pw - colsAgo*(pw/ST.cols);
    if (ex > L){
      g.strokeStyle = 'rgba(255,255,255,.55)'; g.setLineDash([2,3]); g.beginPath(); g.moveTo(ex, T); g.lineTo(ex, T+ph); g.stroke(); g.setLineDash([]);
      const ey = T + (1 - NET[e.dir].xOf(e.s0)/CORRIDOR_LENGTH_KM)*ph;
      g.fillStyle = '#fff'; g.beginPath(); g.arc(ex, ey, 3.5, 0, 7); g.fill();
      g.strokeStyle = 'rgba(255,255,255,.9)'; g.lineWidth=1; g.beginPath(); g.arc(ex, ey, 7, 0, 7); g.stroke();
      g.fillStyle = '#fff'; g.textAlign='left'; g.font='600 10px "Inter Tight", system-ui'; g.fillText('BRAKE TAP', ex+10, ey-8);
      // front trajectory
      if (e.front.length > 2){
        g.strokeStyle = 'rgba(255,255,255,.8)'; g.lineWidth = 1.2; g.setLineDash([1,3]); g.beginPath();
        let started=false;
        for (const p of e.front){
          const px = ex + (p.t/0.5)*(pw/ST.cols), py = T + (1 - NET[e.dir].xOf(p.s)/CORRIDOR_LENGTH_KM)*ph;
          if (px > L+pw) break;
          started ? g.lineTo(px,py) : (g.moveTo(px,py), started=true);
        }
        g.stroke(); g.setLineDash([]);
      }
    }
  }
}
$('stDir').querySelectorAll('button').forEach(b => b.onclick = () => { state.stDir = b.dataset.d; $('stDir').querySelectorAll('button').forEach(x=>x.classList.toggle('on', x===b)); });

/* =============================================================== STORY ==== */
function renderStory(force){
  const box = $('story'), e = sim.event;
  const mode = !e ? 'idle' : 'live';
  if (!force && box.dataset.mode === mode) return;
  box.dataset.mode = mode;
  if (mode === 'idle'){
    const v = state.chaseVeh;
    box.innerHTML = `
      <span class="eyebrow"><b>The experiment</b></span>
      <h2>Tap the <i>brakes</i>.</h2>
      <p class="lede">One driver brakes hard for just over a second, the way everyone does a dozen times on a commute. Nothing else changes. Watch whether the traffic behind them shrugs it off or turns it into a jam.</p>
      <button class="brake" id="brakeBtn"><span class="lamp"></span><span id="brakeLbl">Tap the brakes on ${state.selected ? 'your car' : 'the followed car'}</span></button>
      <div class="tune">
        <label>Force <input type="range" id="sev" min="1.5" max="8" step="0.5" value="${state.severity}"><span class="v" id="sevV">${state.severity.toFixed(1)} m/s²</span></label>
        <label>Length <input type="range" id="dur" min="0.4" max="4" step="0.1" value="${state.duration}"><span class="v" id="durV">${state.duration.toFixed(1)} s</span></label>
      </div>
      <div class="howto">
        <div class="step"><span class="i">1</span><div><b>Pick a time.</b> Drag along the speed curve above. The dips are when the real DVP slows down.</div></div>
        <div class="step"><span class="i">2</span><div><b>Pick a car.</b> Click any light on the map, or use the one in the close-up.</div></div>
        <div class="step"><span class="i">3</span><div><b>Watch the strip.</b> A jam shows up as a red streak climbing back up the valley.</div></div>
      </div>
      <p class="fine"><b>B</b> taps the brakes · <b>Space</b> pauses · scroll to zoom the map</p>`;
    $('brakeBtn').onclick = () => tapBrakes();
    $('sev').oninput = (ev) => { state.severity = +ev.target.value; $('sevV').textContent = state.severity.toFixed(1)+' m/s²'; };
    $('dur').oninput = (ev) => { state.duration = +ev.target.value; $('durV').textContent = state.duration.toFixed(1)+' s'; };
  } else {
    box.innerHTML = `
      <div class="status"><span class="eyebrow"><b>One brake light</b></span><span id="evPill"></span></div>
      <div class="where" id="evWhere"></div>
      <div class="verdict" id="verdict"></div>
      <div class="metrics">
        <div class="m hot"><div class="n" id="mCaught">0</div><div class="l">drivers forced to slow down</div></div>
        <div class="m"><div class="n" id="mReach">0.00<small>km</small></div><div class="l">furthest the slowdown reached back</div></div>
        <div class="m"><div class="n" id="mDelay">0<small>veh·min</small></div><div class="l">time lost, all drivers combined</div></div>
        <div class="m"><div class="n" id="mWave">—</div><div class="l">speed the jam front travels upstream</div></div>
      </div>
      <canvas id="spark"></canvas>
      <p class="fine" id="evFine"></p>
      <div class="again"><button class="btn" id="clearBtn">Clear</button><button class="btn" id="againBtn">Tap another car</button></div>`;
    $('clearBtn').onclick = () => { sim.clearEvent(); state.eventSeries=[]; renderStory(true); };
    $('againBtn').onclick = () => { sim.clearEvent(); state.eventSeries=[]; state.selected=null; renderStory(true); tapBrakes(); };
  }
}

function updateStory(){
  const e = sim.event;
  if (!e){
    const lbl = $('brakeLbl'); if (lbl) lbl.textContent = `Tap the brakes on ${state.selected && vehicleAlive(state.selected) ? 'your car' : 'the followed car'}`;
    return;
  }
  if ($('story').dataset.mode !== 'live') renderStory(true);
  const age = sim.t - e.t0;
  const n = e.affected.size, reach = e.maxUpstream, delayMin = e.delayVehHours*60;
  const tw = (k, target) => { const d = target - state.tween[k]; state.tween[k] = Math.abs(d) < (k==='reach'?0.005:0.6) ? target : state.tween[k] + d*0.22; };
  tw('caught', n); tw('reach', reach); tw('delay', delayMin);
  $('mCaught').textContent = Math.round(state.tween.caught).toLocaleString();
  $('mReach').innerHTML = state.tween.reach.toFixed(2)+'<small>km</small>';
  $('mDelay').innerHTML = (state.tween.delay < 10 ? state.tween.delay.toFixed(1) : Math.round(state.tween.delay).toLocaleString())+'<small>veh·min</small>';
  const mw = $('mWave');
  if (e.waveSpeed != null){ mw.className='n'; mw.innerHTML = Math.abs(e.waveSpeed).toFixed(1)+'<small>km/h</small>'; }
  else { mw.className='n note'; mw.textContent = e.waveNote ? cap(e.waveNote) : (age < 10 ? 'measuring…' : 'not measurable'); }
  $('evWhere').textContent = `${e.dir==='NB'?'northbound':'southbound'} · km ${NET[e.dir].xOf(e.s0).toFixed(1)} · ${age.toFixed(0)} s ago`;
  const pill = $('evPill');
  if (e.neverFormed) pill.innerHTML = '<span class="pill ok">Absorbed</span>';
  else if (e.resolved) pill.innerHTML = `<span class="pill ok">Gone after ${Math.round(e.resolvedAt)} s</span>`;
  else pill.innerHTML = '<span class="pill live">Spreading</span>';
  let v;
  if (e.neverFormed) v = `The gap behind them <em>absorbed it</em>. ${n ? n+' driver'+(n>1?'s':'')+' eased off and it was over.' : 'Nobody behind had to react.'}`;
  else if (e.resolved) v = `It rippled back ${reach.toFixed(1)} km, caught <em>${n.toLocaleString()} drivers</em>, then faded out.`;
  else if (age < 6) v = `Brake lights on…`;
  else v = `One brake light has now caught <em>${n.toLocaleString()} drivers</em> across ${reach.toFixed(1)} km of road.`;
  $('verdict').innerHTML = v;
  const amb = e.ambient[(e.s0/CELL_M)|0];
  let fine = `Time lost is measured against traffic as it was the instant before the tap. At ~$22 per vehicle-hour that's about <b>$${(e.delayVehHours*22).toFixed(e.delayVehHours*22<10?2:0)}</b>.`;
  if (e.v0kmh != null && e.v0kmh < 12) fine += ` This car was almost stopped already (${Math.round(e.v0kmh)} km/h), so the tap added little. A moving car makes a cleaner test.`;
  if (amb < 45) fine += ` Traffic here was already at ${Math.round(amb)} km/h, a density where jams also form without any trigger, so crediting this one to the tap is generous.`;
  $('evFine').innerHTML = fine;
  // sparkline
  if (!state.eventSeries.length || age - state.eventSeries[state.eventSeries.length-1][0] >= 0.5) state.eventSeries.push([age, n]);
  drawSpark();
}
const cap = (s) => s.charAt(0).toUpperCase()+s.slice(1);
function drawSpark(){
  const c = $('spark'); if (!c) return;
  const [W,H,dpr] = resizeCanvas(c); const g = c.getContext('2d'); g.setTransform(dpr,0,0,dpr,0,0); g.clearRect(0,0,W,H);
  const S = state.eventSeries; if (S.length < 2) return;
  const tMax = Math.max(60, S[S.length-1][0]), nMax = Math.max(10, ...S.map(s=>s[1]));
  const px = (t) => (t/tMax)*W, py = (n) => H-2 - (n/nMax)*(H-6);
  const gr = g.createLinearGradient(0,0,0,H); gr.addColorStop(0,'rgba(255,70,80,.35)'); gr.addColorStop(1,'rgba(255,70,80,0)');
  g.beginPath(); g.moveTo(0,H); for (const s of S) g.lineTo(px(s[0]), py(s[1])); g.lineTo(px(S[S.length-1][0]), H); g.closePath(); g.fillStyle = gr; g.fill();
  g.beginPath(); S.forEach((s,i)=> i?g.lineTo(px(s[0]),py(s[1])):g.moveTo(px(s[0]),py(s[1]))); g.strokeStyle='#ff5b66'; g.lineWidth=1.5; g.stroke();
  g.fillStyle='rgba(132,142,162,.8)'; g.font='10px "JetBrains Mono", monospace'; g.textAlign='right'; g.fillText(`${Math.round(tMax)} s after the tap`, W-2, H-2);
}

function tapBrakes(){
  const v = (state.selected && vehicleAlive(state.selected)) ? state.selected : state.chaseVeh;
  if (!v) return;
  state.selected = v; state.chaseVeh = v; state.chaseDir = v.dir; state.eventSeries = []; state.tween = {caught:0,reach:0,delay:0};
  sim.triggerBrake(v, state.severity, state.duration);
  state.stDir = v.dir; $('stDir').querySelectorAll('button').forEach(x=>x.classList.toggle('on', x.dataset.d===v.dir));
  renderStory(true);
}

/* ========================================================= SCENARIOS ==== */
/** Same selection rule the headless tests use, so the guided story is reproducible. */
function demoPick(dir, sLo, sHi){
  const amb = sim.ambient[dir].vm; let best=null, score=-1;
  for (const v of sim.lanes[dir][1]){
    if (v.s<sLo || v.s>sHi) continue;
    const a = amb[(v.s/CELL_M)|0], kmh = v.v*KMH;
    if (kmh < 35 || kmh < 0.85*a) continue;
    let n = 0; for (let l=0;l<3;l++) for (const u of sim.lanes[dir][l]) if (u.s < v.s && u.s > v.s-400) n++;
    if (n > score){ score=n; best=v; }
  }
  return best;
}

/** Reset and settle traffic off-screen, in chunks, behind a veil. */
function loadScenario(day, t, dir, seed, chipId, settleSteps=400, title){
  return new Promise(resolve => {
    state.busy = true; const wasRunning = state.running; state.running = false;
    sim.clearEvent(); state.selected = null; state.chaseVeh = null; state.eventSeries = [];
    sim.reset(day, t, seed); stReset(); state.lastDemandT = 0;
    state.stDir = dir; $('stDir').querySelectorAll('button').forEach(x=>x.classList.toggle('on', x.dataset.d===dir));
    syncDays(); setPresetChip(chipId); renderStory(true);
    const veil = $('veil'); $('veilT').textContent = title || `${DAYNAME[day]}, ${fmtClock12(t)}`;
    $('veilS').textContent = 'letting traffic settle'; veil.classList.add('on');
    let done = 0;
    const tick = () => {
      const t0 = performance.now();
      while (done < settleSteps && performance.now() - t0 < 24){ sim.step(0.1); done++; }
      $('veilP').style.width = (done/settleSteps*100)+'%';
      if (done < settleSteps) requestAnimationFrame(tick);
      else { stIngest(); veil.classList.remove('on'); state.busy = false; state.running = wasRunning || !!state.story; resolve(); }
    };
    requestAnimationFrame(tick);
  });
}

/* ========================================================= GUIDED STORY ==== */
function caption(a, b){
  const c = $('caption');
  if (!a){ c.classList.remove('on'); return; }
  c.classList.remove('on');
  setTimeout(() => { $('cap1').innerHTML = a; $('cap2').innerHTML = b||''; c.classList.add('on'); }, 220);
}
async function runStory(){
  if (state.story) return stopStory();
  const S = state.story = { cancelled:false };
  $('storyBtn').textContent = '■ Stop the story'; $('storybar').classList.add('on'); document.body.classList.add('storymode');
  const bar = $('storybar').querySelector('i');
  const prog = (f) => bar.style.width = (f*100)+'%';
  const wallWait = (ms) => new Promise(r => { const t=setTimeout(r, ms); S.timer=t; });
  const simWait = (sec) => new Promise(r => { const t0 = sim.t; const f = () => { if (S.cancelled) return r(); if (sim.t - t0 >= sec) r(); else requestAnimationFrame(f); }; f(); });
  const ok = () => !S.cancelled;
  const prevRate = state.rate; setRate(4);
  try {
    prog(.02); caption('');
    await loadScenario('Sun', 10.5, 'SB', 11, 'sun', 400, 'Sunday, 10:30 a.m.'); if (!ok()) return;
    fitCam(false); state.running = true;
    caption('This is the Don Valley Parkway.', 'Fifteen kilometres from the 401 to the Gardiner. Every point of light is one simulated driver.');
    await wallWait(6000); if (!ok()) return; prog(.12);
    caption('Sunday morning. Traffic is light.', `Southbound is moving at about ${Math.round(sim.stats.SB.meanSpeed)} km/h.`);
    await wallWait(4200); if (!ok()) return; prog(.2);
    state.running = false;
    let v = demoPick('SB', 5000, 9500);
    if (v){ state.selected = v; state.chaseVeh = v; flyTo(v, 0.55); }
    caption('We pick one driver…', '…and have them brake hard for 1.2 seconds.');
    await wallWait(3400); if (!ok()) return;
    state.severity = 4.0; state.duration = 1.2;
    if (v && vehicleAlive(v)) tapBrakes();
    state.running = true; prog(.28);
    await simWait(8); if (!ok()) return;
    cam.tz = 0.22;
    caption('The driver behind eases off. So does the next.', 'Then the gap closes and it\'s over.');
    await simWait(48); if (!ok()) return; prog(.42);
    const e1 = sim.event ? { n: sim.event.affected.size } : { n:0 };
    caption(`${e1.n} driver${e1.n===1?'':'s'} touched the brakes. Nobody else noticed.`, 'Now the same tap, on a weekday morning.');
    await wallWait(5600); if (!ok()) return; prog(.5);

    await loadScenario('Thu', 9.0, 'SB', 11, 'am', 400, 'Thursday, 9:00 a.m.'); if (!ok()) return;
    fitCam(false); state.running = true;
    caption('Thursday, 9:00 a.m. Southbound rush.', `Same road, same drivers, more of them. It's moving at about ${Math.round(sim.stats.SB.meanSpeed)} km/h.`);
    await wallWait(5600); if (!ok()) return; prog(.58);
    state.running = false;
    v = demoPick('SB', 5000, 9500);
    if (v){ state.selected = v; state.chaseVeh = v; flyTo(v, 0.55); }
    caption('Same tap. Same 1.2 seconds.', '');
    await wallWait(3000); if (!ok()) return;
    if (v && vehicleAlive(v)) tapBrakes();
    state.running = true; prog(.64);
    await simWait(10); if (!ok()) return;
    caption('This time each driver brakes a little harder than the one ahead.', 'The gaps are too short to soak it up.');
    await simWait(28); if (!ok()) return; prog(.74);
    cam.tz = 0.11;
    caption('The red band is the jam travelling <i>backwards</i>, up the valley.', 'Against the traffic, long after the first car has driven away.');
    await simWait(52); if (!ok()) return; prog(.86);
    state.stDir = 'SB';
    const e = sim.event;
    if (e){
      caption(`One brake light. <i>${e.affected.size.toLocaleString()} drivers.</i>`,
        `${e.maxUpstream.toFixed(1)} km of slowdown and ${Math.round(e.delayVehHours*60)} vehicle-minutes lost, measured against the traffic the instant before the tap. At this density, jams also start on their own.`);
    }
    await wallWait(10000); if (!ok()) return; prog(1);
    caption('Your turn.', 'Click any car on the map, pick a time of day, and tap the brakes.');
    await wallWait(6000);
  } finally {
    if (state.story === S) endStory(prevRate);
  }
}
function stopStory(){ if (!state.story) return; state.story.cancelled = true; clearTimeout(state.story.timer); endStory(state.rate); }
function endStory(rate){
  state.story = null; caption(''); document.body.classList.remove('storymode'); $('storybar').classList.remove('on'); $('storybar').querySelector('i').style.width='0';
  $('storyBtn').textContent = '▶ Watch the story'; state.running = true; setRate(rate||4);
}
$('storyBtn').onclick = () => runStory();

/* ========================================================= TOP CONTROLS ==== */
function setRate(r){ state.rate = r; $('rate').querySelectorAll('button').forEach(b => b.classList.toggle('on', +b.dataset.r === r)); }
$('rate').querySelectorAll('button').forEach(b => b.onclick = () => setRate(+b.dataset.r));
$('playBtn').onclick = () => { state.running = !state.running; $('playBtn').textContent = state.running ? '❚❚' : '▶'; };
function updateTop(){
  $('clock').textContent = fmtClock(sim.clockHours); $('clockDay').textContent = DAYNAME[sim.day];
  for (const d of ['NB','SB']){ const s = sim.stats[d]; if (!s || s.meanSpeed==null) continue;
    $(d.toLowerCase()+'V').textContent = Math.round(s.meanSpeed);
    const l = $(d.toLowerCase()+'L'); l.textContent = s.los; l.className = 'los los-'+s.los; }
}
window.addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT') return;
  if (e.key === ' '){ e.preventDefault(); $('playBtn').click(); }
  else if (e.key === 'b' || e.key === 'B'){ if (!state.story){ if (sim.event){ sim.clearEvent(); } tapBrakes(); } }
  else if (e.key === 'Escape'){ if ($('drawer').classList.contains('on')) toggleHood(false); else stopStory(); }
  else if (e.key === '0') fitCam(false);
  else if ((e.key === 'ArrowUp' || e.key === 'ArrowDown') && !state.story){
    e.preventDefault(); const v = state.chaseVeh; const km = v ? NET[v.dir].xOf(rs(v)) : 7.5;
    jumpTo(state.chaseDir, km + (e.key === 'ArrowUp' ? 1 : -1), { zoom:false });
  }
  else if ((e.key === 'n' || e.key === 'N') && !state.story){ const v = state.chaseVeh; jumpTo('NB', v ? NET[v.dir].xOf(rs(v)) : 7.5); }
  else if ((e.key === 's' || e.key === 'S') && !state.story){ const v = state.chaseVeh; jumpTo('SB', v ? NET[v.dir].xOf(rs(v)) : 7.5); }
  else if (e.key === '+' || e.key === '=') $('zin').click();
  else if (e.key === '-') $('zout').click();
});

/* ================================================================ HOOD ==== */
function toggleHood(on){ $('drawer').classList.toggle('on', on); $('scrim').classList.toggle('on', on); $('drawer').setAttribute('aria-hidden', !on); if (on){ drawValidation(); drawRamps(); } }
$('hoodBtn').onclick = () => toggleHood(true); $('hoodClose').onclick = () => toggleHood(false); $('scrim').onclick = () => toggleHood(false);
$('tabs').querySelectorAll('button').forEach(b => b.onclick = () => {
  $('tabs').querySelectorAll('button').forEach(x => x.classList.toggle('on', x===b));
  document.querySelectorAll('.pane').forEach(p => p.classList.toggle('on', p.id === 'p-'+b.dataset.p));
  if (b.dataset.p === 'valid') drawValidation(); if (b.dataset.p === 'ramps') drawRamps();
});
function bindPhys(id, key, f){
  const el = $(id);
  el.oninput = () => { P[key] = +el.value; $(id+'L').textContent = f(P[key]);
    if (key === 'reactionTime') for (const d of ['NB','SB']) for (let l=0;l<MAX_LANES;l++) for (const v of sim.lanes[d][l])
      v.tau = Math.max(0.05, P.reactionTime*(0.75+0.5*((v.id*2654435761)%1000)/1000)); };
}
bindPhys('pReaction','reactionTime', v=>v.toFixed(2)+' s');
bindPhys('pHetero','heterogeneity', v=>v.toFixed(2)+'×');
bindPhys('pPolite','politeness', v=>v.toFixed(2));
bindPhys('pAggr','aggression', v=>v.toFixed(2)+'×');
$('reseed').onclick = () => { toggleHood(false); loadScenario(sim.day, sim.clockHours, state.stDir, Math.floor(Math.random()*1e9), null, 300, 'Rebuilding traffic'); };

function drawRamps(){
  const dir = state.stDir, dem = sim.demand[dir]; if (!dem) return;
  $('rampDir').textContent = dir==='NB' ? 'northbound' : 'southbound';
  const rows = [...dem.ramps].sort((a,b)=> NET[dir].xOf(b.s) - NET[dir].xOf(a.s));
  $('rampBody').innerHTML = rows.map(r => `<tr><td>${r.short}</td><td class="num">${NET[dir].xOf(r.s).toFixed(1)}</td><td><span class="tag ${r.t}">${r.t==='on'?'ON':'OFF'}</span></td><td class="num">${Math.round(r.flow).toLocaleString()}</td></tr>`).join('');
}
function drawValidation(){
  const c = cal(), cv = $('vchart'); if (!cv || !c.observedSpeeds || !cv.clientWidth) return;
  const [W,H,dpr] = resizeCanvas(cv); const g = cv.getContext('2d'); g.setTransform(dpr,0,0,dpr,0,0); g.clearRect(0,0,W,H);
  const L=30,R=8,T=10,B=20, pw=W-L-R, ph=H-T-B, xh=(h)=>L+(h/23)*pw, yv=(v)=>T+(1-v/110)*ph;
  g.font='10px "JetBrains Mono", monospace'; g.fillStyle='rgba(132,142,162,.8)';
  for (const v of [0,25,50,75,100]){ g.fillStyle='rgba(255,255,255,.06)'; g.fillRect(L,yv(v),pw,1); g.fillStyle='rgba(132,142,162,.8)'; g.textAlign='right'; g.fillText(v, L-6, yv(v)+3); }
  g.textAlign='center'; for (const h of [0,6,12,18,23]) g.fillText(String(h).padStart(2,'0'), xh(h), H-5);
  for (const [dir,col] of [['SB','#ffb547'],['NB','#6fd3ff']]){
    for (const [data, dash] of [[c.observedSpeeds[dir],[]],[c.simulatedSpeeds && c.simulatedSpeeds[dir],[4,4]]]){
      if (!data) continue; g.strokeStyle=col; g.lineWidth = dash.length?1.3:2; g.setLineDash(dash);
      g.beginPath(); for (let h=0;h<24;h++){ const x=xh(h),y=yv(data[h]); h?g.lineTo(x,y):g.moveTo(x,y); } g.stroke(); g.setLineDash([]);
    }
  }
  if (c.simulatedSpeeds){
    let eN=0,eS=0; for (let h=0;h<24;h++){ eN+=Math.abs(c.simulatedSpeeds.NB[h]-c.observedSpeeds.NB[h]); eS+=Math.abs(c.simulatedSpeeds.SB[h]-c.observedSpeeds.SB[h]); }
    $('vsum').innerHTML = `Mean absolute error over all 24 hours: <b>northbound ${(eN/24).toFixed(1)} km/h</b>, <b>southbound ${(eS/24).toFixed(1)} km/h</b>. Hours above about 92 km/h were fitted to the measured volume shape instead of speed, because there speed stops responding to volume and inverting it produces nonsense.`;
  }
}
function renderProvenance(){
  const c = cal(), rows = [];
  const add = (q, tag, note) => rows.push(`<tr><td>${q}</td><td><span class="prov prov-${tag}">${tag}</span></td><td class="note">${note}</td></tr>`);
  add('Corridor length, lanes, interchange positions', 'published', 'City of Toronto; DVP exit list. Ramp directions are exact.');
  add('Road alignment on the map', 'measured', 'OpenStreetMap geometry. Model kilometres are anchored to OSM exit nodes and land within about 0.3 km of them.');
  add('Weekday volume (135,000/day)', 'published', 'City of Toronto DVP page.');
  add('Corridor speed by hour, both directions', 'measured', 'Toronto Open Data Bluetooth travel times, 2017 weekday averages. <b>The backbone of the calibration.</b>');
  add('Hourly demand profile', 'measured', 'Fitted so the model reproduces the measured speeds. See Validation.');
  add('Day-of-week factors', 'literature', 'FHWA Traffic Monitoring Guide. No measured DVP weekend data exists.');
  add('Ramp entry and exit shares', 'estimated', '<b>The weakest part of the model.</b> No public per-ramp DVP counts exist.');
  add('Driver behaviour (IDM / MOBIL)', 'literature', 'Treiber, Hennecke & Helbing (2000); Kesting, Treiber & Helbing (2007).');
  add('Lateral spacing of lanes on the map', 'estimated', 'Exaggerated at low zoom so lanes stay visible. Close-up and simulation use true widths.');
  $('provBody').innerHTML = rows.join('');
  $('gapList').innerHTML = (c.dataGaps||[]).map(g=>`<li>${g}</li>`).join('') || '<li>None recorded.</li>';
  $('srcList').innerHTML = [ ...(c.sources||[]),
    { url:'https://www.openstreetmap.org/copyright', name:'OpenStreetMap contributors', note:'Basemap and road geometry, ODbL' } ]
    .map(s=>`<li><a href="${s.url}" target="_blank" rel="noopener noreferrer">${s.name||s.url}</a>${s.note?' — '+s.note:''}</li>`).join('');
}

/* =========================================================== MAIN LOOP ==== */
let last = performance.now(), simDebt = 0;
function frame(now){
  const wall = Math.min(0.1, (now-last)/1000); last = now;
  if (state.running && !state.busy){
    simDebt += wall*state.rate;
    const t0 = performance.now();
    while (simDebt >= 0.1 && performance.now() - t0 < 14){ snapshotPrev(); sim.step(0.1); simDebt -= 0.1; }
    if (simDebt > 0.1) simDebt = 0.1;              // can't keep up: slow down smoothly rather than lurch
    // keep demand in step with the clock (once per simulated minute)
    if (sim.t - state.lastDemandT > 60){ sim.refreshDemand(); state.lastDemandT = sim.t; }
  }
  RENDER_ALPHA = state.running && !state.busy ? clamp(simDebt/0.1, 0, 1) : 1;
  state.wallDt = wall;
  easeLanes(state.running && !state.busy ? wall*state.rate : 0);
  stIngest();
  drawMap(); drawChase(); drawScrub(); drawST(); updateStory(); updateTop();
  requestAnimationFrame(frame);
}

function wireChaseControls(){
  $('chDir').querySelectorAll('button').forEach(b => b.onclick = () => { if (state.story) return; const v = state.chaseVeh; jumpTo(b.dataset.d, v ? NET[v.dir].xOf(rs(v)) : 7.5); });
  $('trackBtn').onclick = () => { state.track = !state.track; syncTrack(); };
  $('freeBtn').onclick = () => { state.selected = null; state.track = false; syncTrack(); };
  syncChaseDir(); syncTrack();
}
function boot(){
  wireChaseControls();
  setCalibration(window.DVP_CALIBRATION || null);
  buildDayAndChips(); renderProvenance(); renderStory(true);
  cam.W = mapC.clientWidth; cam.H = mapC.clientHeight; fitCam(true);
  loadScenario('Thu', 9.0, 'SB', 11, 'am', 220, 'Thursday, 9:00 a.m.');
  requestAnimationFrame(frame);
}
window.addEventListener('load', boot);
