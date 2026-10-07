/* ===========================================================================
   MICROSIMULATION ENGINE
   IDM car-following + MOBIL lane changing, with finite reaction time and
   heterogeneous driver parameters.

   Why those two ingredients matter
   --------------------------------
   A platoon of *identical* drivers with *instant* reactions is string-stable
   at almost any density: tap the brakes and the disturbance dies out behind
   you.  Add a reaction delay and a spread of desired headways, and above a
   critical density the same tap amplifies as it passes backwards through the
   platoon until vehicles come to a full stop several kilometres upstream, with
   no obstacle anywhere in sight.  That is the phantom jam / "jamiton", and it
   is the thing this model exists to reproduce.  Both parameters are exposed as
   sliders rather than buried, because the result genuinely depends on them.

   Lane indexing (per direction of travel)
   ---------------------------------------
     0,1,2  mainline lanes, 0 = leftmost / median
     3      auxiliary right lane: on-ramp acceleration lane, off-ramp
            deceleration lane, or the NB widening approaching Hwy 401
   =========================================================================== */

const MAX_LANES  = 4;
const AUX        = 3;
const HIST       = 26;     // perception history slots (2.6 s at dt = 0.1 s)
const HIST_DT    = 0.1;
const CELL_M     = 100;                       // aggregation cell size
const NCELLS     = CORRIDOR_LENGTH_M / CELL_M;
const KMH        = 3.6;                        // m/s -> km/h

// ---- deterministic RNG so a given day+time reproduces exactly --------------
function mulberry32(seed){
  let a = seed >>> 0;
  return function(){
    a |= 0; a = a + 0x6D2B79F5 | 0;
    let t = Math.imul(a ^ a >>> 15, 1 | a);
    t = t + Math.imul(t ^ t >>> 7, 61 | t) ^ t;
    return ((t ^ t >>> 14) >>> 0) / 4294967296;
  };
}
function gauss(rng){
  let u=0,v=0; while(!u) u=rng(); while(!v) v=rng();
  return Math.sqrt(-2*Math.log(u))*Math.cos(2*Math.PI*v);
}

// ---- global tunables (bound to UI sliders) --------------------------------
const P = {
  reactionTime : 0.55,   // s   -- perception-reaction lag
  heterogeneity: 1.0,    // multiplier on the spread of driver parameters
  politeness   : 0.25,   // MOBIL p: how much you care about the driver you cut off
  aggression   : 1.0,    // scales desired speed and shrinks desired headway
  truckShare   : 0.06,
  speedLimit   : 90/KMH  // m/s (posted 90 km/h)
};

let VID = 1;

class Vehicle {
  constructor(dir, s, v, lane, rng){
    this.id   = VID++;
    this.dir  = dir;
    this.s    = s;
    this.v    = v;
    this.lane = lane;
    this.a    = 0;

    const het = P.heterogeneity;
    const truck = rng() < P.truckShare;
    this.truck = truck;
    this.len  = truck ? 16 + rng()*4 : 4.4 + rng()*0.9;

    // Desired speed: people drive above the 90 limit on the DVP when it's open.
    // Tuned so the model's free-flow corridor speed lands on the measured
    // overnight Bluetooth average of ~97 km/h rather than overshooting it.
    const base = truck ? 90 : 99;
    this.v0   = Math.max(58/KMH, (base + gauss(rng)*11*het) / KMH) * P.aggression;
    // Desired time headway - the main driver of capacity.
    this.T    = Math.max(0.45, (truck ? 1.55 : 1.05) + gauss(rng)*0.28*het) / P.aggression;
    // Acceleration and comfortable deceleration in Treiber's calibrated range
    // for congested freeway traffic. These are deliberately modest: a high aMax
    // lets drivers recover from a disturbance faster than it propagates, which
    // suppresses the very instability we are here to observe.
    this.aMax = truck ? 0.7 + rng()*0.3 : 1.0 + rng()*0.6*het;
    this.bCmf = truck ? 1.5 : 1.7 + rng()*0.45;
    this.s0   = truck ? 4.0 : 2.2 + rng()*0.9;
    this.pol  = Math.max(0, P.politeness + gauss(rng)*0.12*het);
    this.tau  = Math.max(0.05, P.reactionTime * (0.75 + rng()*0.5));

    // Perception history: a follower sees where its leader WAS one reaction
    // time ago, not where it is now. This lag is the engine of string
    // instability -- without it a platoon damps almost any disturbance.
    this.hist = new Float32Array(HIST*2);
    for (let i=0;i<HIST;i++){ this.hist[i*2] = s; this.hist[i*2+1] = v; }
    this.hIdx = 0;

    this.exit        = null;   // off-ramp object, or null = rides to terminal
    this.enteredAt   = 0;
    this.mergeBy     = null;   // s by which an on-ramp vehicle must be in lane 2
    this.brakeUntil  = -1;
    this.brakeDecel  = 0;
    this.isTagged    = false;  // the user-selected vehicle
    this.inWave      = false;
    this.minVSince   = 999;
    this.lcCooldown  = 0;
  }

  pushHistory(){
    this.hIdx = (this.hIdx + 1) % HIST;
    this.hist[this.hIdx*2]   = this.s;
    this.hist[this.hIdx*2+1] = this.v;
  }
  /** this vehicle's state as perceived `lag` seconds later by a follower */
  delayedS(lag){
    const back = Math.min(HIST-1, Math.round(lag/HIST_DT));
    return this.hist[(((this.hIdx - back) % HIST) + HIST) % HIST * 2];
  }
  delayedV(lag){
    const back = Math.min(HIST-1, Math.round(lag/HIST_DT));
    return this.hist[((((this.hIdx - back) % HIST) + HIST) % HIST) * 2 + 1];
  }
}

class Sim {
  constructor(){
    this.t = 0;                       // sim seconds since start of run
    this.clockHours = 8.0;            // time of day
    this.day = 'Thu';
    this.rng = mulberry32(12345);
    this.lanes = { NB:[], SB:[] };
    for (const d of ['NB','SB'])
      for (let i=0;i<MAX_LANES;i++) this.lanes[d][i] = [];   // sorted ascending by s
    this.rampQueue = { NB:{}, SB:{} };
    this.emit      = { NB:{}, SB:{} };   // fractional-vehicle accumulators
    this.demand    = { NB:null, SB:null };

    // measurement
    this.cells = { NB:this._blankCells(), SB:this._blankCells() };
    this.ambient = { NB:this._blankAmbient(), SB:this._blankAmbient() };
    this.stHistory = { NB:[], SB:[] };    // space-time ring buffer
    this.stMaxRows = 420;
    this.lastAgg = 0;

    this.event = null;    // active brake-event measurement
    this.stats = { NB:{}, SB:{} };
    this.vehCount = 0;
  }

  _blankCells(){
    const c = { v:new Float32Array(NCELLS).fill(90), n:new Float32Array(NCELLS),
                vm:new Float32Array(NCELLS).fill(90), nm:new Float32Array(NCELLS),
                vs:new Float32Array(NCELLS).fill(90),
                laneV:[], laneN:[] };
    for (let l=0;l<MAX_LANES;l++){
      c.laneV[l] = new Float32Array(NCELLS).fill(90);
      c.laneN[l] = new Float32Array(NCELLS);
    }
    return c;
  }
  _blankAmbient(){
    const a = { v:new Float32Array(NCELLS).fill(90), vm:new Float32Array(NCELLS).fill(90), laneV:[] };
    for (let l=0;l<MAX_LANES;l++) a.laneV[l] = new Float32Array(NCELLS).fill(90);
    return a;
  }

  reset(day, hours, seed){
    this.day = day; this.clockHours = hours; this.t = 0;
    this.lastAgg = 0;              // must follow t, or aggregation silently stops
    this.rng = mulberry32(seed ?? 12345);
    for (const d of ['NB','SB']){
      for (let i=0;i<MAX_LANES;i++) this.lanes[d][i] = [];
      this.rampQueue[d] = {}; this.emit[d] = {};
      this.stHistory[d] = [];
      this.ambient[d] = this._blankAmbient();
    }
    this.event = null;
    this.refreshDemand();
    this.warmStart();
  }

  refreshDemand(){
    for (const d of ['NB','SB']) this.demand[d] = demandFor(d, this.day, this.clockHours);
  }

  /* --- seed the corridor so you don't watch an empty road fill up --------- */
  warmStart(){
    for (const dir of ['NB','SB']){
      const dem = this.demand[dir];
      const net = NET[dir];
      const nl = BASE_LANES;
      for (let cell = 0; cell < NCELLS; cell++){
        const s = cell*CELL_M;
        const flowHere = dem.sourceFlow * flowMultAt(dir, s);
        const perLane  = flowHere / nl;                       // veh/h/lane
        // Crude speed-flow curve just to pick a plausible starting state; the
        // simulation relaxes away from it within a few seconds anyway.
        const capacity = 2050;
        const ratio = Math.min(1.25, perLane/capacity);
        let vGuess = 101;
        if (ratio > 0.80) vGuess = 101 - (ratio - 0.80)*175;
        vGuess = Math.max(16, vGuess);
        const density = perLane / vGuess;                     // veh/km/lane
        const nInCell = density * (CELL_M/1000);              // can exceed 1
        for (let lane = 0; lane < nl; lane++){
          let k = Math.floor(nInCell) + (this.rng() < (nInCell % 1) ? 1 : 0);
          for (let m = 0; m < k; m++){
            const sv = s + (m + 0.5 + (this.rng()-0.5)*0.6) * (CELL_M/Math.max(1,k));
            const veh = new Vehicle(dir, sv, (vGuess/KMH)*(0.88+this.rng()*0.24), lane, this.rng);
            veh.exit = drawExit(dir, veh.s, this.rng);
            veh.enteredAt = 0;
            this.lanes[dir][lane].push(veh);
          }
        }
      }
      for (let i=0;i<MAX_LANES;i++) this.lanes[dir][i].sort((a,b)=>a.s-b.s);
    }
    this.aggregate(true);
    for (const d of ['NB','SB']){
      this.ambient[d].v.set(this.cells[d].v);
      this.ambient[d].vm.set(this.cells[d].vm);
      for (let l=0;l<MAX_LANES;l++) this.ambient[d].laneV[l].set(this.cells[d].laneV[l]);
    }
  }

  /* ---------------------------------------------------------------- lanes */
  laneExists(dir, lane, s){
    if (lane < 0 || lane >= MAX_LANES) return false;
    if (lane < BASE_LANES) return true;
    if (lane === AUX){
      if (dir === 'NB' && s > 13800) return true;            // 401 approach widening
      const net = NET[dir];
      return !!(accelLaneAt(net, s) || decelLaneAt(net, s));
    }
    return false;
  }

  /** binary search for insertion index in a lane array */
  _idx(arr, s){
    let lo=0, hi=arr.length;
    while(lo<hi){ const m=(lo+hi)>>1; if(arr[m].s < s) lo=m+1; else hi=m; }
    return lo;
  }
  _insert(dir, lane, veh){
    const arr = this.lanes[dir][lane];
    arr.splice(this._idx(arr, veh.s), 0, veh);
    veh.lane = lane;
  }
  _remove(dir, lane, veh){
    const arr = this.lanes[dir][lane];
    const i = arr.indexOf(veh);
    if (i >= 0) arr.splice(i,1);
  }

  /** leader/follower in a given lane at position s (excluding `self`) */
  neighbours(dir, lane, s, self){
    const arr = this.lanes[dir][lane];
    let i = this._idx(arr, s);
    let lead = null, foll = null;
    for (let k=i; k<arr.length; k++){ if(arr[k]!==self){ lead=arr[k]; break; } }
    for (let k=i-1; k>=0; k--){ if(arr[k]!==self){ foll=arr[k]; break; } }
    return { lead, foll };
  }

  /* ------------------------------------------------------------------ IDM */
  /**
   * @param delayed when true, the leader is perceived as it was one reaction
   *   time ago. Used for the actual driving decision. Lane-change evaluation
   *   uses the undelayed view, since a driver checking a gap looks at it now.
   */
  idm(veh, lead, sOverride, delayed){
    const s = sOverride ?? veh.s;
    const v = veh.v;
    let gap, dv;
    if (lead){
      const lS = delayed ? lead.delayedS(veh.tau) : lead.s;
      const lV = delayed ? lead.delayedV(veh.tau) : lead.v;
      gap = lS - lead.len - s;
      dv  = v - lV;
    } else {
      gap = 1e5; dv = 0;
    }
    // Virtual obstacle: end of an acceleration lane you haven't merged out of.
    if (veh.lane === AUX && veh.mergeBy != null){
      const tap = veh.mergeBy - s;
      if (tap < gap){ gap = Math.max(0.5, tap); dv = v; }
    }
    if (gap < 0.1) gap = 0.1;
    const sStar = veh.s0 + Math.max(0, v*veh.T + (v*dv)/(2*Math.sqrt(veh.aMax*veh.bCmf)));
    const free  = 1 - Math.pow(v/veh.v0, 4);
    const inter = (sStar/gap)*(sStar/gap);
    let a = veh.aMax * (free - inter);
    return Math.max(-9.5, Math.min(3.0, a));
  }

  /* ---------------------------------------------------------------- MOBIL */
  considerLaneChange(veh, dt){
    if (veh.lcCooldown > 0){ veh.lcCooldown -= dt; return; }
    const dir = veh.dir, s = veh.s;
    const cur = veh.lane;

    // --- mandatory pressures -------------------------------------------
    let wantRight = 0, wantLeft = 0;
    if (veh.lane === AUX && veh.mergeBy != null){
      // on an acceleration lane: must get left into the mainline
      const remain = Math.max(1, veh.mergeBy - s);
      wantLeft = 6 * (1 - remain/ACCEL_LANE_M) + 1.2;
    }
    if (veh.exit){
      const toGo = veh.exit.s - s;
      if (toGo < DIVERGE_WARN_M && toGo > -50){
        // needs to reach the rightmost available lane
        const urgency = 1 - Math.max(0, toGo)/DIVERGE_WARN_M;
        wantRight = 0.6 + 6.5*urgency*urgency;
      }
    } else {
      wantLeft += 0.12;                       // mild keep-left-to-cruise for through traffic
    }
    // Ontario keep-right convention: weak rightward bias when not passing
    const keepRight = 0.14;

    const opts = [];
    if (this.laneExists(dir, cur-1, s)) opts.push(cur-1);
    if (this.laneExists(dir, cur+1, s)) opts.push(cur+1);
    if (!opts.length) return;

    const meLead = this.neighbours(dir, cur, s, veh).lead;
    const aSelfOld = this.idm(veh, meLead);

    let best = null, bestScore = 0;
    for (const tgt of opts){
      // an exiting vehicle must not leave the aux/right lane once committed
      if (veh.exit && tgt < cur && wantRight > 2) continue;

      const { lead, foll } = this.neighbours(dir, tgt, s, veh);
      // hard geometric safety
      if (lead && (lead.s - lead.len - s) < veh.s0*0.6) continue;
      if (foll && (s - veh.len - foll.s) < foll.s0*0.6) continue;

      const aSelfNew = this.idm(veh, lead);

      let aFollNew = 0, aFollOld = 0;
      if (foll){
        aFollOld = this.idm(foll, this.neighbours(dir, tgt, foll.s, foll).lead);
        aFollNew = this.idm(foll, veh);
        const bSafe = (veh.lane === AUX && wantLeft > 3) ? -6.5 : -4.0;  // merging drivers push in harder
        if (aFollNew < bSafe) continue;
      }
      // the follower we're leaving behind benefits
      const myFoll = this.neighbours(dir, cur, s, veh).foll;
      let aOldFollNew = 0, aOldFollOld = 0;
      if (myFoll){
        aOldFollOld = this.idm(myFoll, veh);
        aOldFollNew = this.idm(myFoll, meLead);
      }

      let incentive = (aSelfNew - aSelfOld)
                    + veh.pol * ((aFollNew - aFollOld) + (aOldFollNew - aOldFollOld));

      if (tgt > cur) incentive += wantRight + keepRight;
      if (tgt < cur) incentive += wantLeft;

      // don't drift into an aux lane you have no business in
      if (tgt === AUX && !veh.exit && !(dir==='NB' && s>13800)) incentive -= 3.0;

      const threshold = 0.16;
      if (incentive > threshold && incentive > bestScore){ bestScore = incentive; best = tgt; }
    }

    if (best != null){
      this._remove(dir, cur, veh);
      this._insert(dir, best, veh);
      veh.lcCooldown = 1.2 + this.rng()*0.8;
      if (best < AUX) veh.mergeBy = null;      // merge complete
    }
  }

  /* ------------------------------------------------------------- stepping */
  step(dt){
    this.t += dt;
    this.clockHours += dt/3600;
    if (this.clockHours >= 24) this.clockHours -= 24;

    for (const dir of ['NB','SB']){
      this.inject(dir, dt);

      // 1. decide accelerations, perceiving each leader with a reaction lag
      for (let lane=0; lane<MAX_LANES; lane++){
        const arr = this.lanes[dir][lane];
        for (let i=0; i<arr.length; i++){
          const veh = arr[i];
          const lead = arr[i+1] || null;
          let a = this.idm(veh, lead, null, true);
          // Emergency override: perception lag explains ordinary stop-and-go,
          // but a driver about to hit the car in front reacts to what is
          // actually there. Without this, delayed perception drives vehicles
          // straight through each other at high density.
          if (lead){
            const trueGap = lead.s - lead.len - veh.s;
            if (trueGap < Math.max(7, 0.9*veh.v))
              a = Math.min(a, this.idm(veh, lead, null, false));
          }
          veh.a = a;
          // an active brake event overrides everything
          if (this.t < veh.brakeUntil) veh.a = Math.min(veh.a, -veh.brakeDecel);
        }
      }

      // 2. integrate (ballistic), then record perception history
      for (let lane=0; lane<MAX_LANES; lane++){
        const arr = this.lanes[dir][lane];
        for (const veh of arr){
          const vNew = Math.max(0, veh.v + veh.a*dt);
          veh.s += 0.5*(veh.v + vNew)*dt;
          veh.v  = vNew;
          if (veh.v < veh.minVSince) veh.minVSince = veh.v;
          veh.pushHistory();
        }
      }

      // 3. lane changes
      for (let lane=0; lane<MAX_LANES; lane++){
        const arr = this.lanes[dir][lane].slice();
        for (const veh of arr) this.considerLaneChange(veh, dt);
      }

      // 4. exits, terminal, and vanishing aux lanes
      this.handleExits(dir);
    }

    if (this.t - this.lastAgg >= 0.5){ this.aggregate(); this.lastAgg = this.t; }
    if (this.event) this.trackEvent();
  }

  handleExits(dir){
    const net = NET[dir];
    for (let lane=0; lane<MAX_LANES; lane++){
      const arr = this.lanes[dir][lane];
      for (let i=arr.length-1; i>=0; i--){
        const veh = arr[i];

        // left the far end of the corridor
        if (veh.s >= CORRIDOR_LENGTH_M){ arr.splice(i,1); continue; }

        // took their off-ramp
        if (veh.exit && veh.s >= veh.exit.s){
          if (lane === AUX){ arr.splice(i,1); continue; }       // clean exit
          // missed it (wasn't in the right lane) - carry on to the next one
          veh.exit = drawExit(dir, veh.s, this.rng);
          veh.missedExit = true;
        }

        // stranded in an aux lane that no longer exists
        if (lane === AUX && !this.laneExists(dir, AUX, veh.s)){
          const { lead, foll } = this.neighbours(dir, BASE_LANES-1, veh.s, veh);
          const gapOK = (!lead || lead.s - lead.len - veh.s > 3) &&
                        (!foll || veh.s - veh.len - foll.s > 3);
          if (gapOK){ arr.splice(i,1); this._insert(dir, BASE_LANES-1, veh); }
          else { veh.s = Math.min(veh.s, (veh.mergeBy ?? veh.s)); veh.v = Math.min(veh.v, 3); }
        }
      }
    }
  }

  /* ------------------------------------------------------------ injection */
  inject(dir, dt){
    const dem = this.demand[dir];
    if (!dem) return;

    // mainline source at s = 0
    this._emitAt(dir, 'SOURCE', dem.sourceFlow, dt, 0, null);

    // on-ramps
    for (const r of dem.onRamps) this._emitAt(dir, r.id, r.flow, dt, r.s, r);
  }

  _emitAt(dir, key, flowPerHour, dt, s, ramp){
    const acc = this.emit[dir];
    acc[key] = (acc[key] ?? this.rng()) + (flowPerHour/3600)*dt;
    let guard = 0;
    while (acc[key] >= 1 && guard++ < 6){
      if (this._tryPlace(dir, s, ramp)) acc[key] -= 1;
      else break;                                   // no gap: demand is held back (a real queue)
    }
    if (acc[key] > 4) acc[key] = 4;                 // cap the backlog so it can recover
  }

  _tryPlace(dir, s, ramp){
    if (ramp){
      // on-ramp: enters the acceleration lane
      const lane = AUX;
      const { lead } = this.neighbours(dir, lane, s, null);
      if (lead && lead.s - lead.len - s < 9) return false;
      const v = (lead ? Math.min(lead.v, 70/KMH) : 68/KMH) * (0.9 + this.rng()*0.15);
      const veh = new Vehicle(dir, s, Math.max(6, v), lane, this.rng);
      veh.mergeBy = s + ACCEL_LANE_M - 12;
      veh.exit = drawExit(dir, s, this.rng);
      veh.enteredAt = this.t;
      this._insert(dir, lane, veh);
      return true;
    }
    // mainline source: pick the emptiest lane with room
    const nl = BASE_LANES;
    let bestLane = -1, bestGap = 12;
    for (let l=0; l<nl; l++){
      const arr = this.lanes[dir][l];
      const gap = arr.length ? arr[0].s : 1e5;
      if (gap > bestGap){ bestGap = gap; bestLane = l; }
    }
    if (bestLane < 0) return false;
    const lead = this.lanes[dir][bestLane][0];
    const v = lead ? Math.min(lead.v, 108/KMH) : (105/KMH);
    const veh = new Vehicle(dir, 0, v*(0.93+this.rng()*0.12), bestLane, this.rng);
    veh.exit = drawExit(dir, 0, this.rng);
    veh.enteredAt = this.t;
    this._insert(dir, bestLane, veh);
    return true;
  }

  /* ---------------------------------------------------------- measurement */
  aggregate(initial){
    for (const dir of ['NB','SB']){
      const c = this.cells[dir];
      c.v.fill(0); c.n.fill(0); c.vm.fill(0); c.nm.fill(0);
      for (let l=0;l<MAX_LANES;l++){ c.laneV[l].fill(0); c.laneN[l].fill(0); }
      let total = 0;
      for (let lane=0; lane<MAX_LANES; lane++){
        const lv = c.laneV[lane], ln = c.laneN[lane];
        for (const veh of this.lanes[dir][lane]){
          const idx = Math.min(NCELLS-1, Math.max(0, (veh.s/CELL_M)|0));
          const kmh = veh.v*KMH;
          c.v[idx] += kmh; c.n[idx] += 1; lv[idx] += kmh; ln[idx] += 1; total++;
          if (lane < BASE_LANES){ c.vm[idx] += kmh; c.nm[idx] += 1; }
        }
      }
      for (let i=0;i<NCELLS;i++) c.v[i] = c.n[i] ? c.v[i]/c.n[i] : NaN;
      for (let i=0;i<NCELLS;i++) c.vm[i] = c.nm[i] ? c.vm[i]/c.nm[i] : NaN;
      { let lm = 95;
        for (let i=0;i<NCELLS;i++){ if (isNaN(c.vm[i])) c.vm[i] = lm; else lm = c.vm[i]; } }
      // fill gaps so the heatmap doesn't strobe
      let last = 95;
      for (let i=0;i<NCELLS;i++){ if (isNaN(c.v[i])) c.v[i] = last; else last = c.v[i]; }
      /* Spatially smoothed copy for DISPLAY only. A 100 m cell often holds two
         or three vehicles, so the raw cell mean is extremely noisy and the
         corridor heatmap renders as confetti rather than as traffic. A 5-cell
         (500 m) box filter is short enough to preserve stop-and-go structure
         and long enough to kill the sampling noise. Detection still uses the
         raw values. */
      for (let i=0;i<NCELLS;i++){
        let acc=0, cnt=0;
        for (let k=-2;k<=2;k++){ const j=i+k; if(j<0||j>=NCELLS) continue; acc+=c.v[j]; cnt++; }
        c.vs[i] = acc/cnt;
      }

      // per-lane means; empty lane-cells inherit the cell mean so they never
      // read as a phantom jam
      for (let l=0;l<MAX_LANES;l++){
        const lv = c.laneV[l], ln = c.laneN[l];
        for (let i=0;i<NCELLS;i++) lv[i] = ln[i] ? lv[i]/ln[i] : c.v[i];
      }

      // ambient = slow EMA, frozen while an event is being measured
      if (!this.event){
        const A = this.ambient[dir];
        const k = initial ? 1 : 0.02;
        for (let i=0;i<NCELLS;i++) A.v[i] = A.v[i]*(1-k) + c.v[i]*k;
        for (let i=0;i<NCELLS;i++) A.vm[i] = A.vm[i]*(1-k) + c.vm[i]*k;
        for (let l=0;l<MAX_LANES;l++){
          const av = A.laneV[l], lv = c.laneV[l];
          for (let i=0;i<NCELLS;i++) av[i] = av[i]*(1-k) + lv[i]*k;
        }
      }

      // space-time ring buffer
      const row = new Uint8Array(NCELLS);
      for (let i=0;i<NCELLS;i++) row[i] = Math.max(0, Math.min(255, c.vs[i]*2));
      const H = this.stHistory[dir];
      H.push({ t:this.t, row });
      if (H.length > this.stMaxRows) H.shift();

      // headline stats
      let sumV=0, cnt=0;
      for (let i=0;i<NCELLS;i++){ sumV += c.v[i]; cnt++; }
      const meanV = sumV/cnt;
      let mainTotal = 0;
      for (let l=0;l<BASE_LANES;l++) mainTotal += this.lanes[dir][l].length;
      const densPerLane = mainTotal / (CORRIDOR_LENGTH_M/1000) / BASE_LANES;
      this.stats[dir] = {
        vehicles: total,
        meanSpeed: meanV,
        density: densPerLane,
        flow: densPerLane*meanV,
        los: losLetter(densPerLane)
      };
    }
    this.vehCount = this.stats.NB.vehicles + this.stats.SB.vehicles;
  }

  /* --------------------------------------------------------- brake events */
  triggerBrake(veh, decel, duration){
    veh.brakeUntil = this.t + duration;
    veh.brakeDecel = decel;
    const dir = veh.dir;
    const A = this.ambient[dir];
    this.event = {
      dir, s0: veh.s, t0: this.t, vehId: veh.id, lane0: veh.lane, v0kmh: veh.v*KMH,
      decel, duration,
      ambient: Float32Array.from(A.v),
      ambientVm: Float32Array.from(A.vm),
      ambientLane: A.laneV.map(a => Float32Array.from(a)),
      front: [],            // {t, s} samples of the upstream jam front
      hits:  [],            // {t, s} where the wave reached each vehicle
      affected: new Set(),
      delayVehHours: 0,
      maxUpstream: 0,
      waveSpeed: null,
      everFormed: false,
      resolved: false,
      lastTrack: this.t
    };
    // reset the per-vehicle minima in the neighbourhood so "affected" is clean
    for (let l=0;l<MAX_LANES;l++)
      for (const v2 of this.lanes[dir][l])
        if (Math.abs(v2.s - veh.s) < 8000){ v2.minVSince = v2.v; v2.inWave = false; }
  }

  trackEvent(){
    const e = this.event;
    if (e.resolved) return;                      // freeze the result once it's over
    const dt = this.t - e.lastTrack;
    if (dt < 0.25) return;
    e.lastTrack = this.t;

    const c = this.cells[e.dir];
    const cell0 = (e.s0/CELL_M)|0;
    const lo = Math.max(0, cell0 - 110);         // look up to 11 km upstream
    const hi = Math.min(NCELLS-1, cell0 + 3);

    /* Scan UPSTREAM from the event, staying contiguous. Two details matter:
       - Contiguity. Without it, any unrelated slow patch elsewhere on the
         corridor gets mistaken for our wave and the fitted speed is nonsense.
       - Per-lane detection. A brake tap starts in ONE lane. Averaged across
         three lanes it barely registers, so a cell counts as congested when
         its slowest lane is depressed against that lane's own ambient. */
    /* A cell is "in the jam" when its MAINLINE MEAN speed is well below the
       frozen ambient mean for that cell. Using individual slow vehicles instead
       fails badly: in stop-and-go traffic some car is below ambient in almost
       every cell at any instant, so a contiguity test on individual vehicles
       links up the entire corridor and reports a 6 km wave the moment you press
       the button. The cell mean only drops when a whole platoon is affected,
       which is what a jam actually is. */
    const THRESH = 0.78, MARGIN = 7;
    const qualifies = (i) => {
      if (!c.nm[i]) return false;
      const amb = Math.max(12, e.ambientVm[i]);
      return c.vm[i] < THRESH*amb && c.vm[i] < amb - MARGIN;
    };

    /* Seed the search AT the event, then walk upstream. Scanning from the top
       of the window with a shared gap budget doesn't work: the few free-flowing
       cells downstream of the tap exhaust the budget before the scan ever
       reaches the jam sitting one cell behind the braking car. */
    let seed = null;
    for (let i=Math.min(hi, cell0+2); i>=Math.max(lo, cell0-8); i--)
      if (qualifies(i)){ seed = i; break; }

    let frontCell = null, delay = 0, nInWave = 0;
    if (seed != null){
      let gapRun = 0;
      for (let i=seed; i>=lo; i--){
        if (qualifies(i)){
          frontCell = i; gapRun = 0; nInWave++;
          const amb = Math.max(12, e.ambientVm[i]);
          // Total-travel-time delay: N x (1 - v/vAmbient) x dt, in veh-hours
          delay += c.nm[i] * Math.max(0, 1 - c.vm[i]/amb) * (dt/3600);
        } else {
          gapRun++;
          if (gapRun > 3) break;                 // tolerate 300 m of recovery within a wave train
        }
      }
    }
    e.delayVehHours += delay;
    e.cellsInWave = nInWave;

    if (frontCell != null){
      const sFront = frontCell*CELL_M;
      if (nInWave >= 2) e.everFormed = true;
      e.front.push({ t: this.t - e.t0, s: sFront });
      if (e.front.length > 900) e.front.shift();
      // Record-setting upstream extents only. The instantaneous front rattles
      // back and forth; the leading edge of the jam does not, so fitting the
      // running maximum gives a far steadier propagation speed than fitting
      // every sample.
      if (sFront < (e.frontBest ?? Infinity) - 1){
        e.frontBest = sFront;
        (e.frontRecord = e.frontRecord || []).push({ t: this.t - e.t0, s: sFront });
      }
      e.maxUpstream = Math.max(e.maxUpstream, (e.s0 - sFront)/1000);

      // vehicles inside the contiguous wave region
      for (let l=0;l<BASE_LANES;l++){
        const arr = this.lanes[e.dir][l];
        let k = this._idx(arr, sFront - 50);
        for (; k<arr.length && arr[k].s <= (cell0+1)*CELL_M; k++){
          const v2 = arr[k];
          if (v2.inWave) continue;
          const amb = Math.max(12, e.ambientVm[(v2.s/CELL_M)|0]);
          const vkk = v2.v*KMH;
          if (vkk < THRESH*amb){
            v2.inWave = true; e.affected.add(v2.id);
            /* Where and when the JAM reached this particular driver. Note the
               stricter test: the canonical 15-20 km/h figure describes the
               upstream edge of a near-stopped region, not the edge of any
               slowdown at all. In light traffic a mild disturbance propagates
               back much faster (roughly spacing / reaction time, which is tens
               of km/h at motorway spacings), so mixing the two measures the
               wrong thing and reports 80 km/h "jam waves". */
            if (v2.s <= e.s0 + 40 && l === e.lane0 && (vkk < 0.55*amb || vkk < 25))
              e.hits.push({ t: this.t - e.t0, s: v2.s });
          }
        }
      }
    }

    /* Propagation speed: regress the position of each newly-caught vehicle on
       the time it was caught. Robustified by trimming the residual outliers,
       because merge disturbances near a ramp can catch vehicles out of
       sequence and drag a plain least-squares fit badly off. */
    const span = e.hits.length ? e.hits[e.hits.length-1].t - e.hits[0].t : 0;
    e.waveSpeed = null; e.waveNote = null; e.waveR2 = null;
    if (e.hits.length < 12 || span < 10){
      e.waveNote = 'not enough of a jam to measure';
    } else {
      /* Reduce the hit cloud to a front trajectory before fitting. Hits are
         scattered across three lanes and a wide region, so a regression through
         all of them mostly measures the scatter. Binning by time and taking the
         furthest-upstream hit in each bin recovers the actual leading edge,
         which is the thing that has a propagation speed. */
      /* Single lane only. The disturbance passes from each driver to the one
         directly behind, so within one lane the onset times form a clean
         monotone chain and its slope is the propagation speed. Pooling three
         lanes mostly measures the scatter between them. */
      const pts = e.hits.slice().sort((a,b)=>a.t-b.t);
      const fit = (arr) => {
        let n=0,st=0,ss=0,stt=0,sts=0;
        for (const p of arr){ n++; st+=p.t; ss+=p.s; stt+=p.t*p.t; sts+=p.t*p.s; }
        const den = n*stt - st*st;
        if (Math.abs(den) < 1e-6) return null;
        const slope = (n*sts - st*ss)/den;
        return { slope, icept:(ss - slope*st)/n };
      };
      let f = fit(pts);
      if (!f){ e.waveNote = 'degenerate fit'; }
      else {
        const resid = pts.map(p => Math.abs(p.s - (f.icept + f.slope*p.t)));
        const cut = [...resid].sort((a,b)=>a-b)[Math.floor(resid.length*0.75)] ?? Infinity;
        const keep = pts.filter((p,i)=> resid[i] <= cut);
        if (keep.length >= 8){ const g = fit(keep); if (g) f = g; }
        const used = keep.length >= 8 ? keep : pts;
        let sy=0,n=0; for (const p of used){ sy+=p.s; n++; }
        const mean = sy/n;
        let ssTot=0, ssRes=0;
        for (const p of used){
          ssTot += (p.s-mean)*(p.s-mean);
          const r = p.s - (f.icept + f.slope*p.t); ssRes += r*r;
        }
        const r2 = ssTot > 1 ? 1 - ssRes/ssTot : 0;
        e.waveR2 = r2;
        const kmh = f.slope*KMH;                   // negative = upstream
        /* Report only a well-conditioned, physically meaningful front.
           Otherwise say WHY, which is more useful than a confident number. */
        if (f.slope >= 0)            e.waveNote = 'no upstream front';
        else if (r2 < 0.45)          e.waveNote = 'front too ragged to fit';
        else if (Math.abs(kmh) > 50) e.waveNote = 'region stalling all at once';
        else if (Math.abs(kmh) < 3)  e.waveNote = 'queue pinned, not travelling';
        else e.waveSpeed = kmh;
      }
    }

    // has it dissipated?  (distinguish "died out" from "never formed")
    if (frontCell == null){
      e.quietSince = e.quietSince ?? this.t;
      const quiet = this.t - e.quietSince;
      if (e.everFormed && quiet > 3){
        e.resolved = true; e.resolvedAt = this.t - e.t0;
      } else if (!e.everFormed && this.t - e.t0 > 10){
        e.resolved = true; e.neverFormed = true; e.resolvedAt = this.t - e.t0;
      }
    } else e.quietSince = null;
  }

  clearEvent(){ this.event = null; }

  /** all vehicles within a longitudinal window, for the zoom renderer */
  window(dir, sLo, sHi){
    const out = [];
    for (let l=0;l<MAX_LANES;l++){
      const arr = this.lanes[dir][l];
      let i = this._idx(arr, sLo);
      for (; i<arr.length && arr[i].s <= sHi; i++) out.push(arr[i]);
    }
    return out;
  }
}

/** HCM level-of-service by density (passenger cars / km / lane) */
function losLetter(d){
  if (d <  7) return 'A';
  if (d < 11) return 'B';
  if (d < 16) return 'C';
  if (d < 22) return 'D';
  if (d < 28) return 'E';
  return 'F';
}
