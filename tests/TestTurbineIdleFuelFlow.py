"""Regression for a configured turbine idle fuel flow."""

import shutil
import xml.etree.ElementTree as et

from JSBSim_utils import JSBSimTestCase, CreateFDM, RunTest, append_xml

FUEL_FLOW_PPS = "propulsion/engine[0]/fuel-flow-rate-pps"
# At idle the F16 fixture's steady fuel flow is above the estimated idle
# floor, so the floor never engages and a configured floor below the estimate
# could not be told apart from it. Lowering TSFC scales that steady product
# down without changing thrust or the estimate, which depends on milthrust
# alone.
LOW_TSFC = 0.1


class TestTurbineIdleFuelFlow(JSBSimTestCase):
    def setUp(self):
        super().setUp()
        script = et.parse(self.sandbox.path_to_jsbsim_file(
            "scripts", "f16_test.xml"))
        use = script.getroot().find("use")
        self.aircraft_name = use.attrib["aircraft"]
        self.ic_name = use.attrib["initialize"]
        aircraft = et.parse(self.sandbox.path_to_jsbsim_file(
            "aircraft", self.aircraft_name, self.aircraft_name + ".xml"))
        engine = aircraft.getroot().find("propulsion/engine")
        self.engine_file = append_xml(engine.attrib["file"])
        self.engine_path = self.sandbox.path_to_jsbsim_file(
            "engine", self.engine_file)
        self.estimate = float(et.parse(self.engine_path).getroot()
                              .find("milthrust").text) ** 0.2 * 107.0
        # The rewritten engine loads from the sandbox, so its thruster must
        # be found there too.
        thruster = append_xml(engine.find("thruster").attrib["file"])
        shutil.copy(self.sandbox.path_to_jsbsim_file("engine", thruster),
                    self.sandbox())

    def fdm_with(self, idle_fuel_flow=None, tsfc=None):
        tree = et.parse(self.engine_path)
        root = tree.getroot()
        if idle_fuel_flow is not None:
            et.SubElement(root, "idlefuelflow").text = repr(idle_fuel_flow)
        if tsfc is not None:
            root.find("tsfc").text = repr(tsfc)
        tree.write(self.sandbox(self.engine_file))
        fdm = CreateFDM(self.sandbox)
        fdm.set_engine_path(".")
        return fdm

    def running(self, idle_fuel_flow=None, tsfc=None):
        fdm = self.fdm_with(idle_fuel_flow, tsfc)
        self.assertTrue(fdm.load_model(self.aircraft_name))
        self.assertTrue(fdm.load_ic(self.ic_name, True))
        fdm.set_dt(1.0 / 120.0)
        self.assertTrue(fdm.run_ic())
        fdm["propulsion/set-running"] = -1
        return fdm

    def set_throttle(self, fdm, command):
        fdm["fcs/throttle-cmd-norm[0]"] = command
        fdm["fcs/throttle-pos-norm[0]"] = command

    def trim_pph(self, fdm, command):
        self.set_throttle(fdm, command)
        self.assertTrue(fdm.run_ic())
        return self.pph(fdm)

    def pph(self, fdm):
        return fdm[FUEL_FLOW_PPS] * 3600.0

    def steady_product_pph(self, fdm):
        # The quantity the idle flow floors: dry thrust, before bleed, times
        # the corrected TSFC at the current operating point.
        bleed = fdm["propulsion/engine[0]/bleed-factor"]
        thrust = fdm["propulsion/engine[0]/thrust-lbs"] / (1.0 - bleed)
        return thrust * fdm["propulsion/engine[0]/tsfc"]

    def test_estimate_is_unchanged_when_not_configured(self):
        for tsfc in (None, LOW_TSFC):
            with self.subTest(tsfc=tsfc):
                fdm = self.running(tsfc=tsfc)
                pph = self.trim_pph(fdm, 0.0)
                self.assertAlmostEqual(
                    pph / max(self.estimate, self.steady_product_pph(fdm)),
                    1.0, places=9)
                del fdm

    def test_configured_value_above_the_estimate_is_the_floor(self):
        configured = 6.0 * self.estimate
        fdm = self.running(configured)
        pph = self.trim_pph(fdm, 0.0)
        self.assertLess(self.steady_product_pph(fdm), configured)
        self.assertAlmostEqual(pph / configured, 1.0, places=9)

    def test_configured_value_below_the_estimate_is_the_floor(self):
        # A floor of max(estimate, configured) would report the estimate here.
        configured = 0.5 * self.estimate
        fdm = self.running(configured, LOW_TSFC)
        pph = self.trim_pph(fdm, 0.0)
        self.assertLess(self.steady_product_pph(fdm), configured)
        self.assertAlmostEqual(pph / configured, 1.0, places=9)

    def test_running_engine_settles_at_the_configured_idle(self):
        # Trim covers zero-time evaluation; Run() must honour the same floor
        # while time advances and the engine spools down from part power.
        configured = 0.5 * self.estimate
        fdm = self.running(configured, LOW_TSFC)
        self.trim_pph(fdm, 0.49)
        self.assertGreater(self.pph(fdm), configured)
        end = fdm.get_sim_time() + 20.0
        while fdm.get_sim_time() < end:
            self.set_throttle(fdm, 0.0)
            self.assertTrue(fdm.run())
        self.assertLess(self.steady_product_pph(fdm), configured)
        self.assertAlmostEqual(self.pph(fdm) / configured, 1.0, places=9)

    def test_negative_value_is_rejected(self):
        fdm = self.fdm_with(-1.0)
        try:
            loaded = fdm.load_model(self.aircraft_name)
        except Exception:
            loaded = False
        self.assertFalse(loaded)


RunTest(TestTurbineIdleFuelFlow)
