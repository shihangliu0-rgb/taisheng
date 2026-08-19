const PathSim = require("./path_sim.js");

const results = PathSim.verifyAll();
let failed = 0;

results.forEach(function (r) {
    const mark = r.ok ? "PASS" : "FAIL";
    if (!r.ok) {
        failed += 1;
    }
    console.log(
        "[" + mark + "] " + r.name +
        "  t=" + r.timeMs + "ms" +
        "  pos=(" + r.x.toFixed(1) + "," + r.y.toFixed(1) + ")" +
        "  left=" + r.leftCm + " front=" + r.frontCm +
        "  usedSeg2=" + r.usedSeg2 +
        (r.reason ? "  " + r.reason : "")
    );
});

console.log(failed === 0
    ? "\n全部 " + results.length + " 组跑通"
    : "\n有 " + failed + "/" + results.length + " 组失败");
process.exit(failed === 0 ? 0 : 1);
