/* Headless engine test: no DOM, just network.js + sim.js. */
const fs = require('fs'), path = require('path'), vm = require('vm');
const D = __dirname;
const src = fs.readFileSync(path.join(D,'src/network.js'),'utf8')
          + '\n' + fs.readFileSync(path.join(D,'src/sim.js'),'utf8');

const ctx = { console, Math, performance:{now:()=>Date.now()}, Float32Array, Uint8Array, Set, window:{} };
vm.createContext(ctx);
vm.runInContext(src, ctx);
const cp = path.join(D,'data/calibration.built.json');
if (fs.existsSync(cp)){ ctx.setCalibration(JSON.parse(fs.readFileSync(cp,'utf8'))); console.log('[using fitted calibration]'); }
vm.runInContext(`
  var sim = new Sim();
  function run(day, hours, seconds, label){
    sim.reset(day, hours, 4242);
    const t0 = Date.now();
    const n = Math.round(seconds/0.1);
    for (let i=0;i<n;i++) sim.step(0.1);
    const ms = Date.now()-t0;
    const S = sim.stats.SB, N = sim.stats.NB;
    console.log(
      label.padEnd(22),
      'SB', String(Math.round(S.meanSpeed)).padStart(3)+'km/h',
      String(S.density.toFixed(1)).padStart(5)+'v/km/ln', 'LOS'+S.los,
      '| NB', String(Math.round(N.meanSpeed)).padStart(3)+'km/h',
      String(N.density.toFixed(1)).padStart(5)+'v/km/ln', 'LOS'+N.los,
      '| n='+String(sim.vehCount).padStart(5),
      '| '+ (seconds/(ms/1000)).toFixed(0)+'x realtime'
    );
    return sim;
  }

  console.log('--- demand sanity ---');
  for (const h of [3, 8.25, 12, 17.5]){
    const d = demandFor('SB','Thu',h);
    const onTot = d.onRamps.reduce((a,r)=>a+r.flow,0);
    const offTot = d.offRamps.reduce((a,r)=>a+r.flow,0);
    console.log('SB '+String(h).padStart(5)+'h  src='+Math.round(d.sourceFlow).toString().padStart(5)+
      '  +on='+Math.round(onTot).toString().padStart(5)+'  -off='+Math.round(offTot).toString().padStart(5)+
      '  =out='+Math.round(d.exitFlow).toString().padStart(5)+
      '  balance err='+Math.round(d.sourceFlow+onTot-offTot-d.exitFlow));
  }

  console.log('\\n--- steady-state runs (180 s each) ---');
  run('Sun', 11.0,  180, 'Sun 11:00 free flow');
  run('Thu', 15.75, 180, 'Thu 15:45 near-crit');
  run('Thu', 8.25,  180, 'Thu 08:15 AM peak');
  run('Fri', 17.5,  180, 'Fri 17:30 PM peak');

  console.log('\\n--- brake-event response ---');
  function brakeTest(day, hours, dir, label){
    sim.reset(day, hours, 777);
    for (let i=0;i<1500;i++) sim.step(0.1);          // 150 s settle
    // grab a vehicle mid-corridor in a mainline lane
    let target=null;
    for (const v of sim.lanes[dir][1]) if (v.s>6000 && v.s<9000){ target=v; break; }
    if (!target){ console.log(label, 'no target found'); return; }
    const vBefore = target.v*3.6;
    sim.triggerBrake(target, 4.0, 1.2);   // the UI defaults
    for (let i=0;i<2400;i++) sim.step(0.1);          // 240 s observe
    const e = sim.event;
    console.log(label.padEnd(22),
      'trigger@'+Math.round(vBefore)+'km/h',
      '| wave='+(e.waveSpeed==null?'  none':(Math.abs(e.waveSpeed).toFixed(1)+'km/h').padStart(9)),
      '| upstream='+e.maxUpstream.toFixed(2)+'km',
      '| affected='+String(e.affected.size).padStart(4),
      '| R2='+(e.waveR2==null?'  - ':e.waveR2.toFixed(2)),
      '| delay='+e.delayVehHours.toFixed(2)+'vh',
      e.resolved?('| dissipated '+e.resolvedAt.toFixed(0)+'s'):'| ongoing');
  }
  brakeTest('Sun', 10.5, 'SB', 'preset: free flow');
  brakeTest('Thu', 13.0, 'NB', 'preset: near critical');
  brakeTest('Thu', 8.25, 'SB', 'preset: AM peak SB');
  brakeTest('Fri', 17.5, 'NB', 'preset: PM peak NB');
`, ctx);
