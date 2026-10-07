/* ===========================================================================
   DVP CORRIDOR NETWORK
   Geometry of the Don Valley Parkway, Hwy 401 (north) <-> Gardiner (south).

   Coordinate convention
   ---------------------
   `x`  = kilometres north of the Gardiner interchange.  x=0 Gardiner, x=15 Hwy401.
   `s`  = metres travelled along a given direction of travel, always increasing.
            NB (northbound):  s = x * 1000
            SB (southbound):  s = (15 - x) * 1000
   All car-following logic works in `s` so both directions share one code path.
   The renderer maps `s` back to `x` for display.

   Interchange geometry and ramp directionality are taken from the Wikipedia
   DVP exit list (which matches MTO/City signage) and the City of Toronto DVP
   page.  Ramp asymmetry is real and load-bearing: everything from Bloor south
   is northbound-entrance-only, which is why SB traffic is fully loaded by the
   time it reaches the Bayview flats.
   =========================================================================== */

const CORRIDOR_LENGTH_KM = 15.0;
const CORRIDOR_LENGTH_M  = 15000;

// --- Interchange inventory -------------------------------------------------
// on / off flags are per direction of travel.
// "nbOn" = there is an on-ramp feeding the northbound carriageway here, etc.
const JUNCTIONS = [
  { id:'gardiner',  x: 0.0,  name:'Gardiner Expwy',        short:'Gardiner',   exitNum:null,
    nbOn:false, nbOff:false, sbOn:false, sbOff:false, terminal:'south',
    note:'Corridor terminus. SB traffic continues onto the Gardiner or exits to Lake Shore.' },

  { id:'lakeshore', x: 0.05, name:'Lake Shore Blvd / Don Roadway', short:'Lake Shore', exitNum:null,
    nbOn:true,  nbOff:false, sbOn:false, sbOff:true,
    note:'SB exit and NB entrance only. Formerly Highway 2.' },

  { id:'richmond',  x: 0.8,  name:'Richmond St / Eastern Ave',  short:'Richmond/Eastern', exitNum:1,
    nbOn:true,  nbOff:false, sbOn:false, sbOff:true,
    note:'SB exit and NB entrance only. EB Eastern feeds NB; SB exits to Richmond/Eastern WB.' },

  { id:'queen',     x: 1.2,  name:'Queen St East',         short:'Queen',      exitNum:null,
    nbOn:true,  nbOff:false, sbOn:false, sbOff:false,
    note:'Northbound entrance only.' },

  { id:'dundas',    x: 1.6,  name:'Dundas St East',        short:'Dundas',     exitNum:null,
    nbOn:true,  nbOff:false, sbOn:false, sbOff:false,
    note:'Northbound entrance only.' },

  { id:'danforth',  x: 2.7,  name:'Danforth Ave',          short:'Danforth',   exitNum:null,
    nbOn:true,  nbOff:false, sbOn:false, sbOff:false,
    note:'Eastbound Danforth to northbound parkway only.' },

  { id:'bayview',   x: 3.8,  name:'Bayview Ave / Bloor St', short:'Bayview/Bloor', exitNum:3,
    nbOn:true,  nbOff:true,  sbOn:true,  sbOff:true,
    note:'No access between Bayview and Bloor/Danforth. Southernmost SB on-ramp.' },

  { id:'donmills',  x: 7.0,  name:'Don Mills Rd',          short:'Don Mills',  exitNum:7,
    nbOn:true,  nbOff:true,  sbOn:true,  sbOff:true,
    note:'Exits 7A/7B. No access to NB parkway from SB Don Mills.' },

  { id:'eglinton',  x:10.0,  name:'Eglinton Ave East',     short:'Eglinton',   exitNum:10,
    nbOn:true,  nbOff:true,  sbOn:true,  sbOff:true,
    note:'Partial cloverleaf, originally a full cloverleaf.' },

  { id:'wynford',   x:10.7,  name:'Wynford Dr',            short:'Wynford',    exitNum:11,
    nbOn:true,  nbOff:false, sbOn:false, sbOff:true,
    note:'SB exit and NB entrance only. Sits 700m from Eglinton - short weave.' },

  { id:'lawrence',  x:11.8,  name:'Lawrence Ave East',     short:'Lawrence',   exitNum:12,
    nbOn:true,  nbOff:true,  sbOn:true,  sbOff:true,
    note:'Full cloverleaf. Exits 12A/12B.' },

  { id:'yorkmills', x:14.0,  name:'York Mills Rd',         short:'York Mills', exitNum:14,
    nbOn:true,  nbOff:true,  sbOn:true,  sbOff:true,
    note:'Partial cloverleaf, no ramps on the northeast side.' },

  { id:'hwy401',    x:15.0,  name:'Highway 401 / 404',     short:'Hwy 401',    exitNum:null,
    nbOn:false, nbOff:false, sbOn:false, sbOff:false, terminal:'north',
    note:'NB splits: 2 left lanes continue as Hwy 404, 3 lanes exit to 401 E/W.' }
];

// --- Mainline cross-section volume shape -----------------------------------
// Relative mainline volume by position, as a fraction of the busiest section.
// The corridor is busiest through the Don Mills - Eglinton - Lawrence middle,
// and lightest at the two ends where traffic has dispersed onto the ramps.
// PROVENANCE: estimated, but anchored on measured evidence. The breakpoints are
// the 2017 Bluetooth segment boundaries (E=0, F=2.4, G=4.8, H=8.5, I=11.5,
// J=12.9, K=15 km), and the peak is placed over the H-I-J stretch because that
// is where both measured peak-hour profiles collapse: SB 08:00 runs 27 km/h
// through I-H and J-I, NB 17:00 runs 28 and 22 km/h through H-I and I-J, while
// the G-H segment between them stays at 78-100 km/h in both. Congestion that
// localised is a demand peak, not a geometry artefact.
const SEGMENT_SHAPE = [
  { toX: 0.8,  f: 0.60 },
  { toX: 2.4,  f: 0.68 },
  { toX: 4.8,  f: 0.78 },
  { toX: 8.5,  f: 0.90 },
  { toX:11.5,  f: 1.00 },
  { toX:12.9,  f: 0.97 },
  { toX:15.0,  f: 0.88 }
];

function segmentFactorAtX(x){
  for (const seg of SEGMENT_SHAPE) if (x <= seg.toX) return seg.f;
  return SEGMENT_SHAPE[SEGMENT_SHAPE.length-1].f;
}

// --- Physical cross-section ------------------------------------------------
const BASE_LANES     = 3;      // "primarily six lanes" per City of Toronto = 3/direction
const LANE_WIDTH_M   = 3.7;
const ACCEL_LANE_M   = 260;    // on-ramp acceleration lane
const DECEL_LANE_M   = 210;    // off-ramp deceleration lane
const DIVERGE_WARN_M = 1100;   // how far ahead a driver starts working right for their exit

/** Build the direction-specific view of the network. dir = 'NB' | 'SB' */
function buildDirection(dir){
  const sOf = (x) => dir === 'NB' ? x*1000 : (CORRIDOR_LENGTH_KM - x)*1000;

  const onRamps = [], offRamps = [];
  for (const j of JUNCTIONS){
    const hasOn  = dir === 'NB' ? j.nbOn  : j.sbOn;
    const hasOff = dir === 'NB' ? j.nbOff : j.sbOff;
    if (hasOn)  onRamps.push({  id:j.id, name:j.name, short:j.short, x:j.x, s:sOf(j.x), kind:'on'  });
    if (hasOff) offRamps.push({ id:j.id, name:j.name, short:j.short, x:j.x, s:sOf(j.x), kind:'off' });
  }
  onRamps.sort((a,b)=>a.s-b.s);
  offRamps.sort((a,b)=>a.s-b.s);

  // Mainline source at s=0 (the big one: Gardiner feeding NB, 401/404 feeding SB)
  const source = dir === 'NB'
    ? { id:'src_gardiner', name:'From Gardiner Expwy', short:'Gardiner', s:0 }
    : { id:'src_401',      name:'From Hwy 401 / 404',  short:'Hwy 401',  s:0 };

  // Terminal sink at s = L
  const sink = dir === 'NB'
    ? { id:'snk_401',      name:'To Hwy 401 / 404',    short:'Hwy 401',  s:CORRIDOR_LENGTH_M }
    : { id:'snk_gardiner', name:'To Gardiner Expwy',   short:'Gardiner', s:CORRIDOR_LENGTH_M };

  return { dir, onRamps, offRamps, source, sink, sOf,
           xOf: (s) => dir === 'NB' ? s/1000 : CORRIDOR_LENGTH_KM - s/1000 };
}

const NET = { NB: buildDirection('NB'), SB: buildDirection('SB') };

/** Lanes available at longitudinal position s (mainline lanes only). */
function mainlineLanesAt(dir, s){
  // NB widens on the approach to the 401/404 split.
  if (dir === 'NB' && s > 13800) return 4;
  return BASE_LANES;
}

/** Is there an acceleration lane (extra lane on the right) at s? Returns ramp or null. */
function accelLaneAt(net, s){
  for (const r of net.onRamps) if (s >= r.s && s < r.s + ACCEL_LANE_M) return r;
  return null;
}
function decelLaneAt(net, s){
  for (const r of net.offRamps) if (s > r.s - DECEL_LANE_M && s <= r.s) return r;
  return null;
}

/* ===========================================================================
   DEMAND MODEL
   Turns (day, hour) into: a mainline source flow, a flow for every on-ramp,
   and an exit probability for every off-ramp -- all mass-balanced so that
   what enters the corridor equals what leaves it.

   Construction
   ------------
   1. V(x,h) = target mainline cross-section volume  = peakDaily * shape(x)
                 * hourlyFraction(h, dir) * dowFactor(day)
   2. Every off-ramp j pulls a fraction beta_j of the mainline flow reaching it.
   3. On-ramp flow is then whatever is needed to satisfy conservation:
         onFlow_i = V_downstream - V_upstream + offFlow
      clamped at >= 0.
   This guarantees the simulated cross-section volumes reproduce the target
   profile while keeping every ramp flow non-negative and physically sensible.
   =========================================================================== */

// Filled from data/calibration.json at load; these are the fallbacks.
let CAL = null;

const FALLBACK_CAL = {
  peakSectionDailyTwoWay: 135000,
  provenance: { peakSectionDailyTwoWay: 'published' },
  // Fractions of the direction's daily volume occurring in each hour.
  // Classic urban radial commuter shape: sharp inbound AM, broader outbound PM.
  hourlyProfile: {
    SB: [.009,.005,.004,.004,.007,.021,.052,.081,.089,.071,.052,.046,
         .046,.047,.050,.058,.065,.068,.056,.040,.031,.025,.019,.014],
    NB: [.012,.007,.005,.004,.006,.015,.031,.043,.048,.043,.041,.044,
         .047,.050,.058,.073,.086,.089,.072,.052,.039,.031,.025,.017]
  },
  dowFactors: { Mon:0.99, Tue:1.02, Wed:1.04, Thu:1.05, Fri:1.09, Sat:0.83, Sun:0.70 },
  // Fraction of arriving mainline flow that leaves at each off-ramp.
  exitShares: {
    NB: { bayview:0.06, donmills:0.14, eglinton:0.13, lawrence:0.12, yorkmills:0.11 },
    SB: { yorkmills:0.05, lawrence:0.07, wynford:0.06, eglinton:0.10,
          donmills:0.11, bayview:0.13, richmond:0.16, lakeshore:0.14 }
  },
  observedSpeeds: null,
  sources: [],
  dataGaps: ['Loaded fallback values; calibration.json not applied.']
};

function setCalibration(obj){ CAL = obj || FALLBACK_CAL; }
function cal(){ return CAL || FALLBACK_CAL; }

const DAYS = ['Mon','Tue','Wed','Thu','Fri','Sat','Sun'];

/** Smoothly interpolate the hourly profile so the time slider is continuous. */
function hourlyFraction(dir, tHours){
  const p = cal().hourlyProfile[dir];
  const h0 = Math.floor(tHours) % 24;
  const h1 = (h0 + 1) % 24;
  const f  = tHours - Math.floor(tHours);
  // Cosine blend avoids the sawtooth artefacts of linear interpolation on the
  // steep shoulders of the peak.
  const w = 0.5 - 0.5*Math.cos(Math.PI*f);
  return p[h0]*(1-w) + p[h1]*w;
}

/**
 * Full demand state for a direction at a given day + time.
 * Returns flows in vehicles/hour.
 */
// Global multiplier used by the offline calibration sweep. Stays at 1 in the app.
let DEMAND_SCALE = 1;
function setDemandScale(k){ DEMAND_SCALE = k; }
// Absolute entry-flow override, used only by the offline calibration sweep.
let FLOW_OVERRIDE = null;
function setFlowOverride(o){ FLOW_OVERRIDE = o; }

/** Entry (corridor-mouth) flow in veh/h for a direction at a day+time. */
function entryFlowFor(dir, day, tHours){
  if (FLOW_OVERRIDE && FLOW_OVERRIDE[dir] != null) return FLOW_OVERRIDE[dir];
  const c = cal();
  const dow = (c.dowFactors && c.dowFactors[day]) ?? 1;
  // Preferred path: a flow table fitted so the simulation reproduces the
  // measured Bluetooth corridor speeds hour by hour.
  if (c.fittedSourceFlow && c.fittedSourceFlow[dir]){
    const tbl = c.fittedSourceFlow[dir];
    const h0 = Math.floor(tHours) % 24, h1 = (h0+1)%24;
    const f  = tHours - Math.floor(tHours);
    const w  = 0.5 - 0.5*Math.cos(Math.PI*f);
    return (tbl[h0]*(1-w) + tbl[h1]*w) * dow * DEMAND_SCALE;
  }
  // Fallback: daily volume x hourly fraction.
  const daily = c.peakSectionDailyTwoWay * 0.5 * dow;
  return daily * hourlyFraction(dir, tHours) * segmentFactorAtX(NET[dir].xOf(0)) * DEMAND_SCALE;
}

/* Two-sided ramp shares.
   An earlier version specified only the EXIT shares and made on-ramp flow the
   residual needed to hit a target volume profile. That is mass-balanced but
   physically wrong: because the southbound profile declines monotonically from
   Lawrence to the Gardiner, every southbound on-ramp came out at exactly zero
   vehicles per hour. Real ramps carry traffic in both directions at once; the
   volume profile is the RESULT of that, not a constraint on it.

   So both sides are now specified as a share of the mainline flow arriving at
   the junction, and the cross-section profile is whatever they produce.
   PROVENANCE: estimated. Cross-street ATR proxies exist for Richmond/Eastern,
   Queen and Dundas only; the rest are apportioned by interchange type, the
   commuter direction, and the arterials each ramp feeds. */
const RAMP_SHARES = {
  SB: {   // travelling 15 km -> 0, inbound to downtown
    on:  { yorkmills:0.065, lawrence:0.095, eglinton:0.090, donmills:0.080, bayview:0.055 },
    off: { yorkmills:0.045, lawrence:0.065, wynford:0.045,  eglinton:0.075,
           donmills:0.095,  bayview:0.110,  richmond:0.150, lakeshore:0.145 }
  },
  NB: {   // travelling 0 -> 15 km, outbound from downtown
    on:  { lakeshore:0.120, richmond:0.140, queen:0.070, dundas:0.060, danforth:0.060,
           bayview:0.090,   donmills:0.075, eglinton:0.065, wynford:0.040,
           lawrence:0.050,  yorkmills:0.040 },
    off: { bayview:0.050, donmills:0.120, eglinton:0.120, lawrence:0.130, yorkmills:0.110 }
  }
};

/** Ordered on/off events along a direction of travel, exits before entries. */
function corridorEvents(dir){
  const net = NET[dir];
  const events = [];
  for (const r of net.offRamps) events.push({ ...r, t:'off' });
  for (const r of net.onRamps)  events.push({ ...r, t:'on'  });
  // At a junction with both, drivers leave before the on-ramp merges in.
  events.sort((a,b)=> a.s - b.s || (a.t === 'off' ? -1 : 1));
  return events;
}

/** Cache of the unit-entry flow profile: multipliers relative to entry flow. */
const PROFILE = {};
function corridorProfile(dir){
  if (PROFILE[dir]) return PROFILE[dir];
  const events = corridorEvents(dir);
  const sh = RAMP_SHARES[dir];
  let m = 1, peak = 1;
  const steps = [];
  for (const e of events){
    if (e.t === 'off'){
      const share = sh.off[e.id] ?? 0.05;
      const f = m*share;
      steps.push({ ...e, mult:f, share });
      m -= f;
    } else {
      const share = sh.on[e.id] ?? 0.05;
      const f = m*share;
      steps.push({ ...e, mult:f, share });
      m += f;
    }
    peak = Math.max(peak, m);
  }
  return PROFILE[dir] = { steps, exitMult:m, peakMult:peak };
}

/** Peak-section flow produced by one unit of entry flow. Used by calibration. */
function peakMultiplier(dir){ return corridorProfile(dir).peakMult; }

/** Mainline flow multiplier (relative to entry flow) at longitudinal position s. */
function flowMultAt(dir, s){
  const prof = corridorProfile(dir);
  let m = 1;
  for (const st of prof.steps){
    if (st.s > s) break;
    m += (st.t === 'on' ? st.mult : -st.mult);
  }
  return m;
}

function demandFor(dir, day, tHours){
  const prof  = corridorProfile(dir);
  const entry = entryFlowFor(dir, day, tHours);
  const ramps = prof.steps.map(s => ({ ...s, flow: s.mult*entry }));
  return {
    dir, day, tHours,
    sourceFlow: entry,
    exitFlow: prof.exitMult*entry,   // what reaches the far terminal
    ramps,
    onRamps:  ramps.filter(r=>r.t==='on'),
    offRamps: ramps.filter(r=>r.t==='off'),
    peakSection: prof.peakMult*entry
  };
}

/** Exit-choice draw for a vehicle entering at longitudinal position sEntry. */
function drawExit(dir, sEntry, rng){
  const net = NET[dir];
  const shares = RAMP_SHARES[dir].off;
  for (const r of net.offRamps){
    if (r.s <= sEntry + 300) continue;              // can't exit right where you got on
    if (rng() < (shares[r.id] ?? 0.05)) return r;
  }
  return null;                                       // rides through to the terminal
}
