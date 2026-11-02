// Node check for src/live_demo_core.js: plan registration vs plain ICP from far random starts
// (any heading, up to 30 m), on the stage clouds the live demo makes, clean and busy site.
// Needs the data block of results/live_demo.html.
// Run: node src/live_demo_check.js [stage ...]   (after .venv/bin/python src/make_live_demo.py)
const fs = require('fs'), path = require('path');
const Core = require('./live_demo_core.js');
const html = fs.readFileSync(path.join(__dirname, '../results/live_demo.html'), 'utf8');
const D = JSON.parse(html.match(/window\.DEMO_DATA = (\{.*?\});<\/script>/s)[1]);
const bim = Core.loadBim(D), model = Core.bimModel(bim);
const stages = process.argv.slice(2).map(Number);
for (const site of ['clean', 'busy']) for (const stage of stages.length ? stages : [2, 7, 11, 15]) {
  const sc = new Core.Scanner(bim, stage, 1, { site }); while (!sc.done()) sc.step(8);
  let ok = 0, silent = 0, flagged = 0, icpOk = 0; const n = 6;
  for (let k = 0; k < n; k++) {
    const off = Core.surveyOffset(bim, 'local', 500 + 13 * k + stage), { cloud, eval: ev } = sc.finish(off.T);
    const r = Core.runPlan(model, cloud.xyz), err = Core.regError(r.T, ev);
    if (Core.regError(Core.plainICP(model, cloud.xyz).T, ev) < 0.05) icpOk++;
    if (err < 0.05) ok++; else if (r.ok) silent++; else flagged++;
  }
  console.log(`${site} ${bim.stages[stage]}: search ${ok}/${n} (silent fails ${silent}, flagged ${flagged}) | plain ICP ${icpOk}/${n}`);
}
