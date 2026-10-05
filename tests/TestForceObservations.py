"""Read-only cached force vectors and resolved application points."""

import copy
import math
import os
import shutil
import xml.etree.ElementTree as et

from JSBSim_utils import JSBSimTestCase, RunTest
import jsbsim


class TestForceObservations(JSBSimTestCase):
    def read_only(self, fdm, path):
        node = fdm.get_property_manager().get_node(path, False)
        self.assertIsNotNone(node, path)
        self.assertTrue(node.get_attribute(jsbsim.Attribute.READ), path)
        self.assertFalse(node.get_attribute(jsbsim.Attribute.WRITE), path)
        value = fdm[path]
        fdm[path] = value + 123.0
        self.assertEqual(fdm[path], value, path)
        return value

    def initialize(self, fdm):
        fdm.set_dt(1.0 / 120.0)
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/terrain-elevation-ft"] = -100000
        fdm["ic/vc-kts"] = 200
        self.assertTrue(fdm.run_ic())

    def test_indexed_thruster_vectors_are_cached_and_sum_to_propulsion(self):
        model = "force-twin"
        shutil.copytree(self.sandbox.path_to_jsbsim_file("aircraft", "f16"), model)
        tree = et.parse(os.path.join(model, "f16.xml"))
        propulsion = tree.getroot().find("propulsion")
        propulsion.append(copy.deepcopy(propulsion.find("engine")))
        tree.write(os.path.join(model, model + ".xml"))
        fdm = self.create_fdm()
        fdm.set_aircraft_path(".")
        self.assertTrue(fdm.load_model(model))
        self.initialize(fdm)
        fdm["propulsion/set-running"] = -1
        fdm["fcs/throttle-cmd-norm"] = 0.35
        fdm["fcs/throttle-pos-norm[1]"] = 0.2
        fdm["propulsion/engine[1]/pitch-angle-rad"] = 0.3
        fdm["propulsion/engine[1]/yaw-angle-rad"] = -0.4
        self.assertTrue(fdm.run())
        before_time = fdm.get_sim_time()
        before_moments = [fdm["moments/%s-prop-lbsft" % axis] for axis in "lmn"]
        for component, axis in enumerate("xyz"):
            values = [self.read_only(fdm, "propulsion/engine[%d]/body-force-%s-lbs" % (i, axis))
                      for i in (0, 1)]
            self.assertAlmostEqual(sum(values), fdm["forces/fb%s-prop-lbs" % axis])
            thrust = fdm["propulsion/engine[1]/thrust-lbs"]
            direction = (math.cos(0.3) * math.cos(-0.4),
                         math.cos(0.3) * math.sin(-0.4), -math.sin(0.3))
            self.assertAlmostEqual(values[1], thrust * direction[component])
        self.assertGreater(abs(fdm["propulsion/engine[1]/body-force-y-lbs"]), 100)
        self.assertEqual(fdm.get_sim_time(), before_time)
        self.assertEqual([fdm["moments/%s-prop-lbsft" % axis] for axis in "lmn"], before_moments)

    def test_propeller_uses_the_same_native_vector_observers(self):
        fdm = self.create_fdm()
        self.assertTrue(fdm.load_model("c172p"))
        self.initialize(fdm)
        fdm["propulsion/set-running"] = -1
        fdm["fcs/throttle-cmd-norm"] = 0.8
        for _ in range(12):
            self.assertTrue(fdm.run())
        for axis in "xyz":
            value = self.read_only(fdm, "propulsion/engine[0]/body-force-%s-lbs" % axis)
            self.assertAlmostEqual(value, fdm["forces/fb%s-prop-lbs" % axis])
        self.assertGreater(abs(fdm["propulsion/engine[0]/body-force-x-lbs"]), 10)

    def test_aero_cg_split_and_resolved_shifted_reference_point(self):
        tree = et.parse(self.sandbox.path_to_jsbsim_file("tests", "tripod.xml"))
        root = tree.getroot()
        metrics = root.find("metrics")
        et.SubElement(metrics, "chord", unit="FT").text = "4"
        rp = et.SubElement(metrics, "location", name="AERORP", unit="IN")
        cg = et.SubElement(root.find("mass_balance"), "location", name="CG", unit="IN")
        for axis, rpv, cgv in zip("xyz", (24, 36, 48), (12, 12, 12)):
            et.SubElement(rp, axis).text = str(rpv)
            et.SubElement(cg, axis).text = str(cgv)
        aero = root.find("aerodynamics")
        shift = et.SubElement(aero, "aero_ref_pt_shift_x")
        et.SubElement(et.SubElement(shift, "function"), "value").text = "0.25"
        for axis, rp_force, cg_force in zip("XYZ", (100, 200, -300), (11, -22, 33)):
            element = et.SubElement(aero, "axis", name=axis)
            et.SubElement(et.SubElement(element, "function"), "value").text = str(rp_force)
            at_cg = et.SubElement(element, "function", apply_at_cg="true")
            et.SubElement(at_cg, "value").text = str(cg_force)
        tree.write("force-aero.xml")
        fdm = self.create_fdm()
        fdm.set_aircraft_path(".")
        self.assertTrue(fdm.load_model("force-aero", False))
        self.initialize(fdm)
        arm = []
        for axis, rp_force, cg_force in zip("xyz", (100, 200, -300), (11, -22, 33)):
            self.assertEqual(self.read_only(fdm, "forces/fb%s-aero-rp-lbs" % axis), rp_force)
            self.assertEqual(self.read_only(fdm, "forces/fb%s-aero-cg-lbs" % axis), cg_force)
            self.assertEqual(fdm["forces/fb%s-aero-lbs" % axis], rp_force + cg_force)
            arm.append(self.read_only(fdm, "aero/rp-body-%s-ft" % axis))
        # Structural x/z are reversed into body axes; shift is one foot aft.
        self.assertEqual(arm, [-2.0, 2.0, -3.0])
        expected_moment = (2 * -300 - -3 * 200, -3 * 100 - -2 * -300,
                           -2 * 200 - 2 * 100)
        for axis, value in zip("lmn", expected_moment):
            self.assertAlmostEqual(fdm["moments/%s-aero-lbsft" % axis], value)
        # Existing applied-total observers exclude gravity; weight is distinct.
        self.assertGreater(fdm["forces/fbz-weight-lbs"], 2900)
        self.assertAlmostEqual(fdm["forces/fbz-total-lbs"], -300 + 33)


RunTest(TestForceObservations)
