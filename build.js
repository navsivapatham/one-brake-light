const fs = require('fs');
const path = require('path');
const D = __dirname;

const read = (p) => fs.readFileSync(path.join(D, p), 'utf8');

// Prefer the fitted output of calibrate.js; fall back to raw research JSON.
let calJson = null, calFrom = null;
for (const f of ['data/calibration.built.json']) {
  const p = path.join(D, f);
  if (fs.existsSync(p)) {
    try { calJson = JSON.parse(read(f)); calFrom = f; break; }
    catch (e) { console.error('!! ' + f + ' is not valid JSON:', e.message); }
  }
}
if (calJson) {
  // the speed-flow sweep is diagnostic only; keep it out of the shipped file
  delete calJson.speedFlowCurve;
}

const calScript = calJson
  ? 'window.DVP_CALIBRATION = ' + JSON.stringify(calJson) + ';'
  : '/* no calibration.json found - engine falls back to built-in defaults */';

let html = read('src/shell.html');
html = html.replace('/*__CALIBRATION__*/', () => calScript);
html = html.replace('/*__GEO__*/',         () => read('src/geo.js'));
html = html.replace('/*__NETWORK__*/',     () => read('src/network.js'));
html = html.replace('/*__SIM__*/',         () => read('src/sim.js'));
html = html.replace('/*__UI__*/',          () => read('src/ui.js'));

fs.writeFileSync(path.join(D, 'dvp-simulation.html'), html);
fs.writeFileSync(path.join(D, 'index.html'), html);          // GitHub Pages entry point
console.log('built dvp-simulation.html  (' + (html.length/1024).toFixed(1) + ' KB)' +
            (calJson ? '  [calibration: ' + calFrom + ']' : '  [built-in fallback calibration]'));
