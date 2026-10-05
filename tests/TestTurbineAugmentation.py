"""Read-only actual afterburner and nozzle state across turbine operating points."""

import copy
import os
import shutil
import xml.etree.ElementTree as et

from JSBSim_utils import JSBSimTestCase, RunTest, append_xml
import jsbsim

ENGINE = "propulsion/engine[0]/"


class TestTurbineAugmentation(JSBSimTestCase):
    def turbine(self, method=1, installed=True, twin=False):
        # Adapt the maintained F16 fixture only to select the native
        # augmentation method. No aircraft-specific afterburner inference.
        script = et.parse(self.sandbox.path_to_jsbsim_file(
            "scripts", "f16_test.xml"))
        use = script.getroot().find("use")
        name = use.attrib["aircraft"]
        aircraft = et.parse(self.sandbox.path_to_jsbsim_file(
            "aircraft", name, name + ".xml"))
        engine = aircraft.getroot().find("propulsion/engine")
        engine_file = append_xml(engine.attrib["file"])
        turbine = et.parse(self.sandbox.path_to_jsbsim_file(
            "engine", engine_file))
        turbine.getroot().find("augmethod").text = str(method)
        turbine.getroot().find("augmented").text = "1" if installed else "0"
        turbine.write(engine_file)
        thruster = append_xml(engine.find("thruster").attrib["file"])
        shutil.copy(self.sandbox.path_to_jsbsim_file("engine", thruster), ".")
        # Method 2's expanded throttle range is part of the native fixture.
        # Method 1 needs an ordinary 0..1 throttle instead.
        gain = aircraft.getroot().find(
            "flight_control/channel/pure_gain[@name='fcs/throttle1']/gain")
        gain.text = "2" if method == 2 else "1"
        if twin:
            aircraft.getroot().find("propulsion").append(copy.deepcopy(engine))
        model = "augmentation-fixture"
        shutil.copytree(self.sandbox.path_to_jsbsim_file("aircraft", name),
                        model, dirs_exist_ok=True)
        aircraft.write(os.path.join(model, model + ".xml"))
        fdm = self.create_fdm()
        fdm.set_aircraft_path(".")
        fdm.set_engine_path(".")
        self.assertTrue(fdm.load_model(model))
        fdm.set_dt(1.0 / 120.0)
        fdm["ic/h-sl-ft"] = 5000
        fdm["ic/vc-kts"] = 200
        self.assertTrue(fdm.run_ic())
        return fdm

    def throttle(self, fdm, command):
        fdm["fcs/throttle-cmd-norm"] = command

    def test_property_is_boolean_read_only_and_engine_indexed(self):
        fdm = self.turbine(twin=True)
        for index in (0, 1):
            path = "propulsion/engine[%d]/augmentation" % index
            node = fdm.get_property_manager().get_node(path, False)
            self.assertIsNotNone(node)
            self.assertTrue(node.get_attribute(jsbsim.Attribute.READ))
            self.assertFalse(node.get_attribute(jsbsim.Attribute.WRITE))
            self.assertEqual(fdm[path], 0)
            fdm[path] = 1
            self.assertEqual(fdm[path], 0)
        fdm["propulsion/set-running"] = -1
        self.throttle(fdm, 1)
        fdm["fcs/throttle-pos-norm[1]"] = 0
        self.assertTrue(fdm.run_ic())
        self.assertEqual(fdm[ENGINE + "augmentation"], 1)
        self.assertEqual(fdm["propulsion/engine[1]/augmentation"], 0)
        fdm[ENGINE + "augmentation"] = 0
        self.assertEqual(fdm[ENGINE + "augmentation"], 1)

    def test_nozzle_property_is_read_only_and_engine_indexed(self):
        fdm = self.turbine(twin=True)
        for index in (0, 1):
            path = "propulsion/engine[%d]/nozzle-pos-norm" % index
            node = fdm.get_property_manager().get_node(path, False)
            self.assertIsNotNone(node)
            self.assertTrue(node.get_attribute(jsbsim.Attribute.READ))
            self.assertFalse(node.get_attribute(jsbsim.Attribute.WRITE))
            self.assertEqual(fdm[path], 1)
            fdm[path] = 0
            self.assertEqual(fdm[path], 1)
        fdm["propulsion/set-running"] = -1
        self.throttle(fdm, 0.7)
        fdm["fcs/throttle-cmd-norm[1]"] = 0.2
        self.assertTrue(fdm.run_ic())
        for _ in range(240):
            self.assertTrue(fdm.run())
        nozzle0 = fdm[ENGINE + "nozzle-pos-norm"]
        nozzle1 = fdm["propulsion/engine[1]/nozzle-pos-norm"]
        self.assertGreater(nozzle1, nozzle0)
        for value in (nozzle0, nozzle1):
            self.assertGreaterEqual(value, 0)
            self.assertLessEqual(value, 1)
        fdm[ENGINE + "nozzle-pos-norm"] = 1
        self.assertEqual(fdm[ENGINE + "nozzle-pos-norm"], nozzle0)

    def test_nozzle_tracks_native_dry_afterburning_shutdown_and_reset(self):
        fdm = self.turbine()
        fdm["propulsion/set-running"] = -1
        self.throttle(fdm, 0.7)
        self.assertTrue(fdm.run_ic())
        for _ in range(240):
            self.assertTrue(fdm.run())
        dry_nozzle = fdm[ENGINE + "nozzle-pos-norm"]
        self.assertLess(dry_nozzle, 0.5)
        self.throttle(fdm, 1)
        for _ in range(1200):
            self.assertTrue(fdm.run())
            if fdm[ENGINE + "augmentation"] == 1:
                break
        self.assertEqual(fdm[ENGINE + "augmentation"], 1)
        for _ in range(180):
            self.assertTrue(fdm.run())
        self.assertEqual(fdm[ENGINE + "nozzle-pos-norm"], 1)
        self.throttle(fdm, 0.7)
        for _ in range(600):
            self.assertTrue(fdm.run())
        self.assertLess(fdm[ENGINE + "nozzle-pos-norm"], 0.5)
        fdm["propulsion/cutoff_cmd"] = 1
        for _ in range(180):
            self.assertTrue(fdm.run())
        self.assertEqual(fdm[ENGINE + "augmentation"], 0)
        self.assertEqual(fdm[ENGINE + "nozzle-pos-norm"], 1)
        fdm.reset_to_initial_conditions(0)
        self.assertEqual(fdm[ENGINE + "nozzle-pos-norm"], 1)

    def test_method_one_waits_for_native_spool_threshold(self):
        fdm = self.turbine()
        self.throttle(fdm, 0)
        fdm["propulsion/set-running"] = -1
        self.assertTrue(fdm.run_ic())
        self.assertTrue(fdm.run())  # Leave Trim at idle before opening throttle.
        self.throttle(fdm, 1)
        self.assertTrue(fdm.run())
        self.assertLess(fdm[ENGINE + "n2"], 97)
        self.assertEqual(fdm[ENGINE + "augmentation"], 0)
        for _ in range(1200):
            self.assertTrue(fdm.run())
            if fdm[ENGINE + "augmentation"] == 1:
                break
        self.assertGreater(fdm[ENGINE + "n2"], 97)
        self.assertEqual(fdm[ENGINE + "augmentation"], 1)
        self.throttle(fdm, 0.99)
        self.assertTrue(fdm.run())
        self.assertEqual(fdm[ENGINE + "augmentation"], 0)

    def test_trim_tracks_each_method_and_engine_off(self):
        for method in (1, 2):
            with self.subTest(method=method):
                fdm = self.turbine(method)
                self.throttle(fdm, 1)
                self.assertTrue(fdm.run_ic())
                self.assertEqual(fdm[ENGINE + "augmentation"], 0)
                fdm["propulsion/set-running"] = -1
                for command, active in ((1, 1), (0.49, 0), (1, 1)):
                    self.throttle(fdm, command)
                    self.assertTrue(fdm.run_ic())
                    self.assertEqual(fdm[ENGINE + "augmentation"], active)
                self.assertTrue(fdm.run())  # Complete the native Trim handover.
                fdm["propulsion/cutoff_cmd"] = 1
                self.assertTrue(fdm.run())
                self.assertEqual(fdm[ENGINE + "augmentation"], 0)
                fdm.reset_to_initial_conditions(0)
                self.assertEqual(fdm[ENGINE + "augmentation"], 0)
                del fdm

    def test_turbine_without_afterburner_never_reports_augmentation(self):
        for method in (1, 2):
            with self.subTest(method=method):
                fdm = self.turbine(method, installed=False)
                fdm["propulsion/set-running"] = -1
                if method == 2:
                    self.throttle(fdm, 0.5)  # Full dry position, no augmentation.
                    self.assertTrue(fdm.run_ic())
                    dry_thrust = fdm[ENGINE + "thrust-lbs"]
                self.throttle(fdm, 1)
                self.assertTrue(fdm.run_ic())
                self.assertEqual(fdm[ENGINE + "augmentation"], 0)
                if method == 2:
                    self.assertAlmostEqual(fdm[ENGINE + "thrust-lbs"],
                                           dry_thrust, places=7)
                self.assertTrue(fdm.run())
                self.assertEqual(fdm[ENGINE + "augmentation"], 0)
                del fdm


    def test_stalled_or_seized_engine_does_not_report_afterburning(self):
        fdm = self.turbine()
        fdm["propulsion/set-running"] = -1
        self.throttle(fdm, 1)
        self.assertTrue(fdm.run_ic())
        self.assertEqual(fdm[ENGINE + "augmentation"], 1)
        fdm[ENGINE + "stalled"] = 1
        self.assertTrue(fdm.run())
        self.assertEqual(fdm[ENGINE + "augmentation"], 0)
        fdm[ENGINE + "stalled"] = 0
        fdm[ENGINE + "seized"] = 1
        self.assertTrue(fdm.run())
        self.assertEqual(fdm[ENGINE + "augmentation"], 0)


RunTest(TestTurbineAugmentation)
