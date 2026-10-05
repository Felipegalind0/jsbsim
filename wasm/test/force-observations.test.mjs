// Actual cached body-force vectors through the public real-WASM property API.
import assert from "node:assert/strict";
import { readdirSync, readFileSync } from "node:fs";
import path from "node:path";
import { it } from "node:test";
import { JSBSimSdk } from "../dist/index.js";
import { wasmBinaryUrl, wasmModuleUrl } from "../dist/wasm.js";

const nativeRoot = process.env.JSBSIM_SOURCE_ROOT;
assert.ok(nativeRoot, "The verified build must supply native fixtures.");

function copyXml(sdk, from, to) {
  for (const entry of readdirSync(from, { withFileTypes: true })) {
    const source = path.join(from, entry.name);
    const target = `${to}/${entry.name}`;
    if (entry.isDirectory()) copyXml(sdk, source, target);
    else if (entry.name.endsWith(".xml")) sdk.writeDataFile(target, readFileSync(source));
  }
}

it("reads indexed transformed thruster forces and separate resolved aerodynamic forces without mutation", async () => {
  const sdk = await JSBSimSdk.create({ moduleUrl: wasmModuleUrl, wasmUrl: wasmBinaryUrl, log: { console: false } });
  try {
    copyXml(sdk, path.join(nativeRoot, "aircraft/f16"), "aircraft/f16");
    copyXml(sdk, path.join(nativeRoot, "engine"), "engine");
    copyXml(sdk, path.join(nativeRoot, "systems"), "systems");
    const xml = readFileSync(path.join(nativeRoot, "aircraft/f16/f16.xml"), "utf8");
    const engine = xml.match(/<engine\b[\s\S]*?<\/engine>/)?.[0];
    assert.ok(engine, "Fixture engine exists");
    sdk.writeDataFile("aircraft/f16/f16.xml", xml.replace("</propulsion>", engine + "</propulsion>"));
    assert.equal(sdk.loadModel("f16"), true);
    sdk.setDt(1 / 120);
    sdk.setPropertyValue("ic/h-sl-ft", 5000);
    sdk.setPropertyValue("ic/terrain-elevation-ft", -100000);
    sdk.setPropertyValue("ic/vc-kts", 200);
    assert.equal(sdk.runIc(), true);
    sdk.setPropertyValue("propulsion/set-running", -1);
    sdk.setPropertyValue("fcs/throttle-cmd-norm", 0.35);
    sdk.setPropertyValue("fcs/throttle-pos-norm[1]", 0.2);
    sdk.setPropertyValue("propulsion/engine[1]/pitch-angle-rad", 0.3);
    sdk.setPropertyValue("propulsion/engine[1]/yaw-angle-rad", -0.4);
    assert.equal(sdk.run(), true);
    const paths = [...[0, 1].flatMap(index => [..."xyz"].map(axis => `propulsion/engine[${index}]/body-force-${axis}-lbs`)),
      ...[..."xyz"].flatMap(axis => [`forces/fb${axis}-aero-rp-lbs`, `forces/fb${axis}-aero-cg-lbs`, `aero/rp-body-${axis}-ft`])];
    const catalog = sdk.getPropertyCatalog();
    for (const property of paths) {
      const canonical = property.replace("[0]", "");
      assert.ok(catalog.includes(property + " (R)") || catalog.includes(canonical + " (R)"), property);
    }
    const batch = sdk.createPropertyBatch(paths);
    assert.deepEqual(batch.missing, []);
    const values = [...batch.read()];
    const time = sdk.getPropertyValue("simulation/sim-time-sec");
    for (const [index, axis] of [..."xyz"].entries()) {
      assert.ok(Math.abs(values[index] + values[index + 3] - sdk.getPropertyValue(`forces/fb${axis}-prop-lbs`)) < 1e-8);
      const atRP = values[6 + index * 3];
      const atCG = values[7 + index * 3];
      assert.ok(Math.abs(atRP + atCG - sdk.getPropertyValue(`forces/fb${axis}-aero-lbs`)) < 1e-8);
    }
    assert.ok(Math.abs(values[4]) > 100, "Nonzero-index engine supplies resolved lateral force");
    assert.ok(values.every(Number.isFinite));
    sdk.setPropertyValue(paths[4], 0);
    assert.deepEqual([...batch.read()], values, "Observer reads/writes do not recalculate or alter cached state");
    assert.equal(sdk.getPropertyValue("simulation/sim-time-sec"), time);
    sdk.resetToInitialConditions(0);
    assert.ok([...batch.read()].every(Number.isFinite));
    assert.equal(sdk.loadModel("f16"), true);
    assert.throws(() => batch.read(), /disposed/);
    assert.equal(sdk.getPropertyValue(paths[4]), 0, "New engine starts with an empty cached vector");
  } finally { sdk.destroy(); }
});
