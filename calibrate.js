/* ===========================================================================
   OFFLINE CALIBRATION
   Fits the corridor entry-flow table so that the simulation reproduces the
   measured 2017 Bluetooth corridor speeds, hour by hour, in each direction.

   Method
   ------
   1. Sweep entry flow over a wide range; for each level, run the simulation to
      a settled state and record the resulting corridor mean speed.  This gives
      an empirical speed(flow) curve for THIS model, which is the model's own
      speed-flow relationship including its ramp merge bottlenecks.
   2. Invert that curve: for each hour's measured speed, read off the entry flow
      the model needs in order to produce it.
   3. Emit the fitted table into calibration.built.json.

   Why fit flow to speed rather than the other way round: the only *measured*
   mainline quantity available for the DVP is speed (Bluetooth travel times).
   The one hourly volume series in open data is a single ramp-to-arterial count
   at Eastern Ave, which the research flagged as unrepresentative of mainline
   flow -- and indeed its shape contradicts the speed data, putting the SB peak
   in the evening when the measured SB speed minimum is clearly at 08:00.
   Speed is the trustworthy signal here, so speed is what we fit to.
   =========================================================================== */

const fs = require('fs'), path = require('path'), vm = require('vm');
const D = __dirname;

const src = fs.readFileSync(path.join(D,'src/network.js'),'utf8')
          + '\n' + fs.readFileSync(path.join(D,'src/sim.js'),'utf8');
const ctx = { console, Math, performance:{now:()=>Date.now()}, Float32Array, Uint8Array, Set, window:{} };
vm.createContext(ctx); vm.runInContext(src, ctx);
// class declarations are lexical, so surface the ones we need on the context
vm.runInContext('globalThis.Sim = Sim; globalThis.NCELLS = NCELLS;', ctx);

const raw = JSON.parse(fs.readFileSync(path.join(D,'data/calibration.json'),'utf8'));

// ---- pull the research JSON into the flat shape the engine wants -----------
const obsN = raw.observedSpeeds.corridorAverageSpeed_kmh.northbound;
const obsS = raw.observedSpeeds.corridorAverageSpeed_kmh.southbound;
const observed = {
  NB: Array.from({length:24}, (_,h)=> obsN[h] ?? obsN[String(h)]),
  SB: Array.from({length:24}, (_,h)=> obsS[h] ?? obsS[String(h)])
};

const dowSrc = raw.dowFactors.factors;
const dowFactors = { Mon:dowSrc.Monday, Tue:dowSrc.Tuesday, Wed:dowSrc.Wednesday,
                     Thu:dowSrc.Thursday, Fri:dowSrc.Friday, Sat:dowSrc.Saturday, Sun:dowSrc.Sunday };

// ---- exit shares ----------------------------------------------------------
// Cross-street ATR proxies exist for three southern junctions only; the rest
// are apportioned by interchange type and the corridor's commuter geometry.
const proxy = {};
for (const i of raw.rampShares.interchanges)
  if (i.rampProxyVolume_AADT) proxy[i.km] = i.rampProxyVolume_AADT;

const exitShares = {
  NB: { bayview:0.055, donmills:0.135, eglinton:0.125, lawrence:0.135, yorkmills:0.115 },
  SB: { yorkmills:0.045, lawrence:0.075, wynford:0.055, eglinton:0.105,
        donmills:0.105, bayview:0.115, richmond:0.15,  lakeshore:0.135 }
};

const baseCal = {
  peakSectionDailyTwoWay: raw.aadt.corridorWideWeekdayAverage.value,
  dowFactors,
  exitShares,
  observedSpeeds: observed,
  hourlyProfile: {   // retained only as a display fallback
    NB: Array.from({length:24},(_,h)=> raw.hourlyProfile.northbound.fractionOfDailyTotal[h] ?? 0.04),
    SB: Array.from({length:24},(_,h)=> raw.hourlyProfile.southbound.fractionOfDailyTotal[h] ?? 0.04)
  }
};

ctx.setCalibration(baseCal);

// ---- 1. sweep -------------------------------------------------------------
const SETTLE = 150, MEASURE = 40, DT = 0.1;
// Parameterised by PEAK-SECTION flow, not entry flow. The two carriageways
// enter the corridor at opposite ends and therefore at different points on the
// volume shape (NB enters where the shape is 0.60, SB where it is 0.88), so
// equal entry flows are not equal loading. Peak-section flow is comparable.
const levels = [600, 1200, 1800, 2400, 3000, 3600, 4200, 4800, 5400, 6000, 6500, 7000, 7600, 8200, 8800];
// entry flow that yields one unit of peak-section flow, from the ramp-share profile
const entryFactor = { NB: 1/ctx.peakMultiplier('NB'), SB: 1/ctx.peakMultiplier('SB') };
const curve = { NB:[], SB:[] };

console.log('Sweeping model speed-flow response...');
console.log('entry factors: NB '+entryFactor.NB+'  SB '+entryFactor.SB);
console.log('  peak veh/h | /lane |   NB km/h |   SB km/h');
for (const pf of levels){
  const f = pf;   // label the curve by peak-section flow
  ctx.setFlowOverride({ NB: pf*entryFactor.NB, SB: pf*entryFactor.SB });
  const sim = new ctx.Sim();
  sim.reset('Wed', 12, 20250805);
  for (let i=0;i<SETTLE/DT;i++) sim.step(DT);
  let accN=0, accS=0, n=0;
  for (let i=0;i<MEASURE/DT;i++){ sim.step(DT);
    if (i%5===0){ accN+=sim.stats.NB.meanSpeed; accS+=sim.stats.SB.meanSpeed; n++; } }
  const vN = accN/n, vS = accS/n;
  curve.NB.push({ f, v:vN }); curve.SB.push({ f, v:vS });
  console.log('  '+String(f).padStart(10)+' | '+(pf/3).toFixed(0).padStart(5)+' | '
    +vN.toFixed(1).padStart(9)+' | '+vS.toFixed(1).padStart(9));
}
ctx.setFlowOverride(null);

// ---- 2. invert ------------------------------------------------------------
/** Given a target speed, find the entry flow that produces it. */
function flowForSpeed(pts, target){
  // pts are ordered by increasing flow; speed decreases monotonically (mostly).
  const mono = [];
  let vPrev = Infinity;
  for (const p of pts){ const v = Math.min(p.v, vPrev - 0.01); mono.push({f:p.f, v}); vPrev = v; }
  if (target >= mono[0].v) return mono[0].f;
  const last = mono[mono.length-1];
  if (target <= last.v){
    // Extrapolate beyond the sweep: speed keeps falling as demand overshoots
    // capacity, so push flow up proportionally rather than clipping.
    const prev = mono[mono.length-2];
    const slope = (last.f - prev.f) / (prev.v - last.v);        // veh/h per km/h lost
    return last.f + (last.v - target)*slope;
  }
  for (let i=1;i<mono.length;i++){
    if (target >= mono[i].v){
      const a = mono[i-1], b = mono[i];
      const w = (a.v - target)/(a.v - b.v);
      return a.f + w*(b.f - a.f);
    }
  }
  return last.f;
}

/* Speed only carries volume information on the congested branch. Above roughly
   92 km/h this corridor's speed-flow curve is nearly flat -- 600 and 2400 veh/h
   both return ~103 km/h -- so inverting it there is ill-conditioned and turns a
   few km/h of measurement noise into thousands of vehicles per hour. The first
   attempt did exactly that, and confidently claimed 1,400 veh/h/lane on the
   southbound DVP at 3 a.m.

   So: use the measured speed where the corridor is actually loaded, and fall
   back to the measured volume SHAPE (Eastern Ave ramp ATR) for the free-flow
   hours, scaled to join continuously onto the speed-fitted hours. Each hour is
   fitted by whichever measurement is informative at that hour. */
const SPEED_TRUST_MAX = 92;
const volFrac = baseCal.hourlyProfile;

const fittedPeak = { NB:[], SB:[] }, method = { NB:[], SB:[] };
for (const dir of ['NB','SB']){
  const reliable = [];
  for (let h=0;h<24;h++){
    if (observed[dir][h] <= SPEED_TRUST_MAX){
      const pf = flowForSpeed(curve[dir], observed[dir][h]);
      fittedPeak[dir][h] = pf; method[dir][h] = 'speed';
      reliable.push(pf / volFrac[dir][h]);
    } else { fittedPeak[dir][h] = null; method[dir][h] = 'volume'; }
  }
  // scale factor joining the volume shape onto the speed-fitted hours
  reliable.sort((a,b)=>a-b);
  const K = reliable.length ? reliable[Math.floor(reliable.length/2)] : 130000;  // median, robust
  for (let h=0;h<24;h++)
    if (fittedPeak[dir][h] == null) fittedPeak[dir][h] = K * volFrac[dir][h];
  console.log(`  ${dir}: ${method[dir].filter(m=>m==='speed').length} h fitted to measured speed, `
    + `${method[dir].filter(m=>m==='volume').length} h to volume shape (K=${Math.round(K).toLocaleString()})`);

  // gentle smoothing across the method boundaries
  const sm = fittedPeak[dir].map((_,h)=>{
    const a=fittedPeak[dir][(h+23)%24], b=fittedPeak[dir][h], c=fittedPeak[dir][(h+1)%24];
    return 0.15*a + 0.70*b + 0.15*c;
  });
  fittedPeak[dir] = sm.map(v=>Math.round(v));
}

// convert peak-section flow back to entry flow
const fitted = { NB:[], SB:[] };
for (const dir of ['NB','SB'])
  fitted[dir] = fittedPeak[dir].map(pf => Math.round(pf * entryFactor[dir]));

// The measured speeds are weekday (Mon-Fri) averages, so the fitted flows
// already embody an average weekday. Divide out the mean weekday DOW factor so
// that applying dowFactors on top does not double-count it.
const wkMean = (dowFactors.Mon+dowFactors.Tue+dowFactors.Wed+dowFactors.Thu+dowFactors.Fri)/5;
for (const dir of ['NB','SB'])
  fitted[dir] = fitted[dir].map(v => Math.round(v/wkMean));

console.log('\nFitted demand, average weekday. entry = corridor mouth, peak = busiest section:');
console.log('  h  |  NB entry   peak  /lane   obs |  SB entry   peak  /lane   obs');
for (let h=0;h<24;h++){
  const r = (dir)=> String(fitted[dir][h]).padStart(6)+String(fittedPeak[dir][h]).padStart(7)
    +(fittedPeak[dir][h]/3).toFixed(0).padStart(7)+String(observed[dir][h]).padStart(6)
    +' '+method[dir][h].padEnd(6);
  console.log('  '+String(h).padStart(2)+' |'+r('NB')+' |'+r('SB'));
}

// ---- 2b. validation: does the fitted table actually reproduce the speeds? --
console.log('\nValidating fitted table against measured speeds...');
const finalCal = { ...baseCal, fittedSourceFlow: fitted };
ctx.setCalibration(finalCal);
ctx.setFlowOverride(null);
const validation = { NB:[], SB:[] };
let errN=0, errS=0, nComp=0;
console.log('  h  |  NB sim   obs   err |  SB sim   obs   err');
for (let h=0; h<24; h++){
  const sim = new ctx.Sim();
  sim.reset('Wed', h, 555000+h);
  for (let i=0;i<120/DT;i++) sim.step(DT);
  let aN=0,aS=0,n=0;
  for (let i=0;i<30/DT;i++){ sim.step(DT); if(i%5===0){ aN+=sim.stats.NB.meanSpeed; aS+=sim.stats.SB.meanSpeed; n++; } }
  const vN=aN/n, vS=aS/n;
  validation.NB.push(+vN.toFixed(1)); validation.SB.push(+vS.toFixed(1));
  const eN = vN-observed.NB[h], eS = vS-observed.SB[h];
  errN += Math.abs(eN); errS += Math.abs(eS); nComp++;
  const f=(x)=>(x>=0?'+':'')+x.toFixed(0);
  console.log('  '+String(h).padStart(2)+' | '+vN.toFixed(0).padStart(5)+String(observed.NB[h]).padStart(6)
    +f(eN).padStart(6)+' | '+vS.toFixed(0).padStart(5)+String(observed.SB[h]).padStart(6)+f(eS).padStart(6));
}
console.log(`\n  mean absolute error:  NB ${(errN/nComp).toFixed(1)} km/h   SB ${(errS/nComp).toFixed(1)} km/h`);

// ---- 3. emit --------------------------------------------------------------
const out = {
  ...baseCal,
  fittedSourceFlow: fitted,
  fittedPeakFlow: fittedPeak,
  fitMethod: method,
  simulatedSpeeds: validation,
  speedFlowCurve: curve,
  hourlyProfileNote: 'Fitted so the simulation reproduces measured 2017 Bluetooth corridor speeds',
  provenance: {
    peakSectionDailyTwoWay: 'published',
    hourlyProfile: 'measured',
    dowFactors: 'literature',
    observedSpeeds: 'measured',
    exitShares: 'estimated'
  },
  sources: [
    { url:'https://www.toronto.ca/services-payments/streets-parking-transportation/road-maintenance/bridges-and-expressways/expressways/don-valley-parkway/',
      name:'City of Toronto — Don Valley Parkway', note:'15 km, ~6 lanes, 12 exits, ~135,000 veh/weekday' },
    { url:'https://open.toronto.ca/dataset/travel-times-bluetooth/',
      name:'Toronto Open Data — Travel Times (Bluetooth)', note:'MEASURED corridor speed by hour, both directions, 2017 weekday averages. The backbone of this calibration.' },
    { url:'https://open.toronto.ca/dataset/traffic-volumes-midblock-vehicle-speed-volume-and-classification-counts/',
      name:'Toronto Open Data — midblock volume counts', note:'Eastern Ave / DVP ramp ATR counts 2021–2023; ramp cross-street AADT proxies' },
    { url:'https://github.com/CityofToronto/bdit_data-sources',
      name:'City of Toronto Transportation Data & Analytics', note:'Confirms the RESCU loop-detector open dataset is deprecated' },
    { url:'https://en.wikipedia.org/wiki/Don_Valley_Parkway',
      name:'DVP exit list', note:'Interchange positions, exit numbers and ramp directionality' },
    { url:'https://trid.trb.org/view/1188773',
      name:'Treiber, Hennecke & Helbing (2000), "Congested traffic states in empirical observations and microscopic simulations"',
      note:'The Intelligent Driver Model and its calibrated parameter ranges' },
    { url:'https://mtreiber.de/publications/MOBIL_TRB.pdf',
      name:'Kesting, Treiber & Helbing (2007), "General lane-changing model MOBIL"',
      note:'Lane-change decision rule and politeness factor' }
  ],
  dataGaps: [
    'No DVP mainline volume counts exist in Toronto Open Data — every count in the midblock dataset is on a ramp or a cross-street approach, never the mainline itself.',
    'The RESCU loop-detector network covers the DVP but its open dataset is deprecated and inactive; it has no public replacement.',
    'HERE probe speed data, which the City itself uses for congestion analysis, is licensed and not public.',
    'Per-ramp throughput is not measured anywhere public. Ramp flows here are mass-balanced from the corridor volume shape, not observed.',
    'Bluetooth travel-time coverage ends in 2017. Volumes are taken to have recovered to pre-COVID levels by 2024 (Parsons, Downtown Toronto Congestion Study 2024), so 2017 is used as a modern baseline.',
    'No weekend speed or volume data for the DVP was found; Saturday and Sunday rely on literature day-of-week factors.',
    'Off-ramp queue spillback from the signalised intersections the ramps feed into is outside the model boundary.'
  ]
};

fs.writeFileSync(path.join(D,'data/calibration.built.json'), JSON.stringify(out));
console.log('\nwrote data/calibration.built.json');
