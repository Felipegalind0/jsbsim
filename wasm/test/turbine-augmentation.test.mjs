// Actual native afterburner state, through the public real-WASM property API.
import assert from "node:assert/strict";
import { readdirSync, readFileSync } from "node:fs";
import path from "node:path";
import { it } from "node:test";
import { JSBSimSdk } from "../dist/index.js";
import { wasmBinaryUrl, wasmModuleUrl } from "../dist/wasm.js";

const nativeRoot = process.env.JSBSIM_SOURCE_ROOT;
assert.ok(nativeRoot, "The verified build must supply native fixtures.");
const engine = "propulsion/engine[0]/";

function copyXml(sdk, from, to) {
  for (const entry of readdirSync(from, { withFileTypes: true })) {
    const source = path.join(from, entry.name);
    const target = `${to}/${entry.name}`;
    if (entry.isDirectory()) copyXml(sdk, source, target);
    else if (entry.name.endsWith(".xml")) sdk.writeDataFile(target, readFileSync(source));
  }
}

it("reads live boolean augmentation without writable commands, including spool lag, cutoff, reset and reload", async () => {
  const sdk = await JSBSimSdk.create({ moduleUrl: wasmModuleUrl, wasmUrl: wasmBinaryUrl, log: { console: false } });
  try {
    copyXml(sdk, path.join(nativeRoot, "aircraft/f16"), "aircraft/f16");
    copyXml(sdk, path.join(nativeRoot, "engine"), "engine");
    copyXml(sdk, path.join(nativeRoot, "systems"), "systems");
    // An ordinary throttle activates native method 1 only after N2 exceeds 97%.
    const turbine = readFileSync(path.join(nativeRoot, "engine/F100-PW-229.xml"), "utf8");
    sdk.writeDataFile("engine/F100-PW-229.xml", turbine.replace(/<augmethod>\s*2\s*<\/augmethod>/, "<augmethod>1</augmethod>"));
    const aircraft = readFileSync(path.join(nativeRoot, "aircraft/f16/f16.xml"), "utf8");
    const ordinaryThrottle = aircraft.replace(/(<pure_gain name="fcs\/throttle1">[\s\S]*?<gain>)2(<\/gain>)/,
      (_match, prefix, suffix) => `${prefix}1${suffix}`);
    assert.notEqual(ordinaryThrottle, aircraft, "Fixture must have its native expanded throttle replaced");
    sdk.writeDataFile("aircraft/f16/f16.xml", ordinaryThrottle);
    assert.equal(sdk.loadModel("f16"), true);
    assert.ok(sdk.getPropertyCatalog().some(line => /^propulsion\/engine(?:\[0\])?\/augmentation \(R\)$/.test(line)));
    sdk.setPropertyValue("ic/h-sl-ft", 5000);
    sdk.setPropertyValue("ic/vc-kts", 200);
    sdk.setPropertyValue("fcs/throttle-cmd-norm", 0);
    assert.equal(sdk.runIc(), true);
    sdk.setPropertyValue("propulsion/set-running", -1);
    assert.equal(sdk.runIc(), true);
    assert.equal(sdk.run(), true);
    const batch = sdk.createPropertyBatch([engine + "augmentation", engine + "n2"]);
    assert.deepEqual(batch.missing, []);
    sdk.setPropertyValue("fcs/throttle-cmd-norm", 1);
    assert.equal(sdk.run(), true);
    assert.equal(batch.read()[0], 0);
    assert.ok(batch.read()[1] < 97);
    for (let step = 0; step < 1200 && batch.read()[0] === 0; step++) assert.equal(sdk.run(), true);
    assert.equal(batch.read()[0], 1);
    assert.ok(batch.read()[1] > 97);
    sdk.setPropertyValue(engine + "augmentation", 0);
    assert.equal(batch.read()[0], 1, "Read-only observation cannot disable a real afterburner");
    sdk.setPropertyValue("propulsion/cutoff_cmd", 1);
    assert.equal(sdk.run(), true);
    assert.equal(batch.read()[0], 0);
    sdk.resetToInitialConditions(0);
    assert.equal(batch.read()[0], 0);
    assert.equal(sdk.loadModel("f16"), true);
    assert.throws(() => batch.read(), /disposed/);
    assert.equal(sdk.getPropertyValue(engine + "augmentation"), 0);
  } finally { sdk.destroy(); }
});
