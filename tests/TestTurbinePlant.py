"""Coupled turbine plant behind FGTurbine: properties, fuel debit, causality,
lifecycle and staged state restore, on the F16 fixture with a synthetic
<plant> engine (an openly specified test engine, not the F100)."""

import shutil
import xml.etree.ElementTree as et

from JSBSim_utils import JSBSimTestCase, CreateFDM, RunTest, append_xml
import jsbsim

ENGINE = "propulsion/engine[0]/"
PLANT = ENGINE + "plant/"
KG_PER_LB = 0.45359237

# The synthetic mixed-flow augmented turbofan of the native unit tests.
PLANT_XML = """
<plant name="synthetic-mixed-flow-augmented">
  <design>
    <mass-flow-kg-sec>100</mass-flow-kg-sec>
    <bypass-ratio>0.6</bypass-ratio>
    <overall-pressure-ratio>25</overall-pressure-ratio>
    <turbine-inlet-temperature-k>1800</turbine-inlet-temperature-k>
    <cooling-fraction>0.10</cooling-fraction>
    <accessory-power-w>150000</accessory-power-w>
    <fan><efficiency>0.86</efficiency></fan>
    <compressor><efficiency>0.86</efficiency></compressor>
    <hp-turbine><efficiency>0.89</efficiency></hp-turbine>
    <lp-turbine><efficiency>0.90</efficiency></lp-turbine>
  </design>
  <lp-shaft><inertia-kg-m2>6</inertia-kg-m2><design-speed-rad-sec>1000</design-speed-rad-sec>
    <coulomb-torque-n-m>5</coulomb-torque-n-m><viscous-n-m-s>0.02</viscous-n-m-s></lp-shaft>
  <hp-shaft><inertia-kg-m2>2.5</inertia-kg-m2><design-speed-rad-sec>1500</design-speed-rad-sec>
    <coulomb-torque-n-m>5</coulomb-torque-n-m><viscous-n-m-s>0.02</viscous-n-m-s></hp-shaft>
  <starter><stall-torque-n-m>350</stall-torque-n-m><free-speed-rad-sec>900</free-speed-rad-sec></starter>
  <core-fuel><valve-time-constant-s>0.05</valve-time-constant-s><valve-rate-limit-kg-sec2>5</valve-rate-limit-kg-sec2>
    <max-flow-kg-sec>5</max-flow-kg-sec><manifold-prime-mass-kg>0.2</manifold-prime-mass-kg>
    <manifold-time-constant-s>0.02</manifold-time-constant-s></core-fuel>
  <augmentor-fuel><valve-time-constant-s>0.05</valve-time-constant-s><valve-rate-limit-kg-sec2>10</valve-rate-limit-kg-sec2>
    <max-flow-kg-sec>12</max-flow-kg-sec><manifold-prime-mass-kg>1.0</manifold-prime-mass-kg>
    <manifold-time-constant-s>0.03</manifold-time-constant-s></augmentor-fuel>
  <core-combustion><ignition-delay-s>0.3</ignition-delay-s><lean-ignition-equivalence>0.2</lean-ignition-equivalence>
    <lean-blowout-equivalence>0.06</lean-blowout-equivalence><rich-limit-equivalence>3.0</rich-limit-equivalence>
    <half-efficiency-loading>0.006</half-efficiency-loading></core-combustion>
  <augmentor-combustion><ignition-delay-s>0.15</ignition-delay-s><lean-ignition-equivalence>0.2</lean-ignition-equivalence>
    <lean-blowout-equivalence>0.1</lean-blowout-equivalence><rich-limit-equivalence>1.4</rich-limit-equivalence>
    <minimum-inlet-temperature-k>600</minimum-inlet-temperature-k><constant-efficiency>0.9</constant-efficiency>
  </augmentor-combustion>
  <controller>
    <fan-speed-demand>0 0.55
      1 1.0</fan-speed-demand>
    <acceleration-limit>0 0.8
      0.5 1.0
      0.9 1.4
      1.1 1.6</acceleration-limit>
    <deceleration-limit>0 0.05
      1.1 0.12</deceleration-limit>
    <start-schedule>0 1.0
      0.6 1.0</start-schedule>
    <dry-throat-area>0.5 1.15
      1.0 1.0</dry-throat-area>
    <augmentor-fuel-air-ratio>0 0.02
      1 0.04</augmentor-fuel-air-ratio>
    <exit-area-ratio>0 1.05
      1 1.4</exit-area-ratio>
    <throat-area-min-fraction>0.8</throat-area-min-fraction>
    <throat-area-max-fraction>2.0</throat-area-max-fraction>
    <throat-area-rate-per-sec>1.0</throat-area-rate-per-sec>
    <idle-corrected-n2>0.62</idle-corrected-n2>
    <augmentor-minimum-n2>0.95</augmentor-minimum-n2>
    <augmentor-lightoff-fuel-air-ratio>0.012</augmentor-lightoff-fuel-air-ratio>
    <max-n2>1.03</max-n2>
    <max-turbine-exit-temperature-k>1400</max-turbine-exit-temperature-k>
  </controller>
  <solid name="liner"><heat-capacity-j-k>12000</heat-capacity-j-k><gas-station>7</gas-station>
    <coolant>bypass</coolant><gas-conductance-w-k>600</gas-conductance-w-k>
    <coolant-conductance-w-k>600</coolant-conductance-w-k><radiation-area-sq-m>1.0</radiation-area-sq-m>
    <emissivity>0.8</emissivity></solid>
  <solid name="core"><heat-capacity-j-k>1800</heat-capacity-j-k><gas-station>5</gas-station>
    <coolant>cooling</coolant><gas-conductance-w-k>600</gas-conductance-w-k>
    <coolant-conductance-w-k>40</coolant-conductance-w-k><radiation-area-sq-m>0.2</radiation-area-sq-m>
    <emissivity>0.75</emissivity></solid>
</plant>
"""


class TestTurbinePlant(JSBSimTestCase):
    def setUp(self):
        super().setUp()
        script = et.parse(self.sandbox.path_to_jsbsim_file("scripts", "f16_test.xml"))
        use = script.getroot().find("use")
        self.aircraft_name = use.attrib["aircraft"]
        self.ic_name = use.attrib["initialize"]
        aircraft = et.parse(self.sandbox.path_to_jsbsim_file(
            "aircraft", self.aircraft_name, self.aircraft_name + ".xml"))
        engine = aircraft.getroot().find("propulsion/engine")
        engine_file = append_xml(engine.attrib["file"])
        tree = et.parse(self.sandbox.path_to_jsbsim_file("engine", engine_file))
        tree.getroot().append(et.fromstring(PLANT_XML))
        tree.write(self.sandbox(engine_file))
        thruster = append_xml(engine.find("thruster").attrib["file"])
        shutil.copy(self.sandbox.path_to_jsbsim_file("engine", thruster), self.sandbox())

    def fdm(self, running=True, throttle=0.0):
        fdm = CreateFDM(self.sandbox)
        fdm.set_engine_path(".")
        self.assertTrue(fdm.load_model(self.aircraft_name))
        self.assertTrue(fdm.load_ic(self.ic_name, True))
        fdm.set_dt(1.0 / 120.0)
        fdm["fcs/throttle-cmd-norm"] = throttle
        self.assertTrue(fdm.run_ic())
        if running:
            fdm["propulsion/set-running"] = -1
            self.assertTrue(fdm.run_ic())
        self._fdm = fdm
        return fdm

    def tank_lbs(self, fdm):
        return sum(fdm["propulsion/tank[%d]/contents-lbs" % i] for i in range(4))

    def test_observations_are_read_only_and_legacy_names_map_the_plant(self):
        fdm = self.fdm(throttle=0.5)
        for path in ("nozzle/gross-thrust-lbs", "station/st7/total-temperature-k",
                     "fuel/ab-burned-kg-sec", "ledger/energy-relative", "numerics/iterations"):
            node = fdm.get_property_manager().get_node(PLANT + path, False)
            self.assertIsNotNone(node, path)
            self.assertTrue(node.get_attribute(jsbsim.Attribute.READ))
            self.assertFalse(node.get_attribute(jsbsim.Attribute.WRITE))
        self.assertEqual(fdm[PLANT + "running"], 1)
        self.assertAlmostEqual(fdm[ENGINE + "n1"], fdm[PLANT + "shaft/n1-percent"])
        self.assertAlmostEqual(fdm[ENGINE + "n2"], fdm[PLANT + "shaft/n2-percent"])
        self.assertAlmostEqual(fdm[ENGINE + "thrust-lbs"], fdm[PLANT + "nozzle/gross-thrust-lbs"], 6)
        self.assertAlmostEqual(fdm[ENGINE + "thermal/nozzle-gas-temperature-k"],
                               fdm[PLANT + "station/st7/total-temperature-k"])
        self.assertEqual(fdm[ENGINE + "augmentation"], 0)
        self.assertGreater(fdm[ENGINE + "thermal/metal-temperature-k"], 300)

    def test_tanks_are_debited_exactly_the_metered_fuel(self):
        fdm = self.fdm(throttle=0.5)
        before = self.tank_lbs(fdm)
        metered = 0.0
        for _ in range(600):
            self.assertTrue(fdm.run())
            metered += fdm[PLANT + "ledger/fuel-in-kg"] / KG_PER_LB
        self.assertGreater(metered, 1.0)
        self.assertAlmostEqual(before - self.tank_lbs(fdm), metered, delta=1e-9 * before)

    def test_starvation_never_debits_more_than_the_tanks_hold(self):
        fdm = self.fdm(throttle=0.5)
        for i in range(4):
            fdm["propulsion/tank[%d]/contents-lbs" % i] = 0.0
        fdm["propulsion/tank[0]/contents-lbs"] = 2.0
        for _ in range(240):
            self.assertTrue(fdm.run())
        self.assertGreaterEqual(fdm["propulsion/tank[0]/contents-lbs"], 0.0)
        self.assertEqual(fdm[PLANT + "combustion/core-lit"], 0)
        self.assertEqual(fdm[PLANT + "running"], 0)

    def test_afterburner_thrust_follows_burned_fuel(self):
        fdm = self.fdm(throttle=0.5)
        for _ in range(60):
            self.assertTrue(fdm.run())
        dry = fdm[PLANT + "nozzle/gross-thrust-lbs"]
        fdm["fcs/throttle-cmd-norm"] = 1.0
        burned = False
        for _ in range(600):
            self.assertTrue(fdm.run())
            burning = fdm[PLANT + "fuel/ab-burned-kg-sec"] > 0
            self.assertEqual(fdm[ENGINE + "augmentation"], 1 if burning else 0)
            if not burning:
                self.assertLess(fdm[PLANT + "nozzle/gross-thrust-lbs"], 1.02 * dry)
            burned = burned or burning
        self.assertTrue(burned)
        self.assertGreater(fdm[PLANT + "nozzle/gross-thrust-lbs"], 1.3 * dry)

    def test_staged_state_restore_round_trips(self):
        fdm = self.fdm(throttle=0.3)
        fdm["fcs/throttle-cmd-norm"] = 0.5
        for _ in range(60):
            self.assertTrue(fdm.run())
        # The catalogue spells engine index 0 without brackets.
        names = [line.split()[0] for line in
                 fdm.query_property_catalog("/plant/state/").splitlines()
                 if line.strip().startswith("propulsion/engine")
                 and not line.split()[0].endswith("/commit")]
        self.assertGreater(len(names), 40)
        saved = {name: fdm[name] for name in names}
        n1 = fdm[PLANT + "shaft/n1-percent"]
        for _ in range(240):
            self.assertTrue(fdm.run())
        self.assertNotAlmostEqual(fdm[PLANT + "shaft/n1-percent"], n1, 3)
        for name, value in saved.items():
            fdm[name] = value
        fdm[PLANT + "state/commit"] = 1
        self.assertEqual(fdm[PLANT + "state/commit"], 1)
        self.assertAlmostEqual(fdm[PLANT + "shaft/n1-percent"], n1, 9)
        # A record from another configuration is rejected.
        digest = [name for name in names if name.endswith("state/config-digest")][0]
        fdm[digest] = saved[digest] + 1
        fdm[PLANT + "state/commit"] = 1
        self.assertEqual(fdm[PLANT + "state/commit"], -1)
        self.assertAlmostEqual(fdm[PLANT + "shaft/n1-percent"], n1, 9)

    def test_settings_are_bounded(self):
        fdm = self.fdm(throttle=0.5)
        self.assertEqual(fdm[PLANT + "settings/iteration-cap"], 24)
        fdm[PLANT + "settings/iteration-cap"] = 40
        self.assertEqual(fdm[PLANT + "settings/iteration-cap"], 24)
        fdm[PLANT + "settings/iteration-cap"] = 16
        self.assertEqual(fdm[PLANT + "settings/iteration-cap"], 16)
        fdm[PLANT + "settings/algorithm"] = 0
        self.assertEqual(fdm[PLANT + "settings/algorithm"], 0)
        self.assertTrue(fdm.run())
        self.assertEqual(fdm[PLANT + "numerics/algorithm"], 0)

    def test_set_running_keeps_the_pilots_throttle(self):
        # set-running is a running engine at the FCS throttle, not at the
        # full throttle FGPropulsion writes into its own copy of the inputs.
        # The F16 FCS doubles the command: 0.25 is half the dry range.
        fdm = self.fdm(throttle=0.25)
        n1 = fdm[PLANT + "shaft/n1-percent"]
        self.assertLess(n1, 95)
        for _ in range(1200):
            fdm.run()
        self.assertAlmostEqual(fdm[PLANT + "shaft/n1-percent"], n1, delta=0.5)
        self.assertEqual(fdm[PLANT + "combustion/ab-selected"], 0)

    def test_zero_time_calls_and_reset(self):
        fdm = self.fdm(throttle=0.5)
        n1 = fdm[ENGINE + "n1"]
        self.assertTrue(fdm.run_ic())
        self.assertAlmostEqual(fdm[ENGINE + "n1"], n1, 6)
        fdm.reset_to_initial_conditions(0)
        self.assertEqual(fdm[PLANT + "shaft/hp-rad-sec"], 0)
        self.assertEqual(fdm[PLANT + "running"], 0)

    def test_cold_start_with_starter(self):
        fdm = self.fdm(running=False)
        fdm["propulsion/starter_cmd"] = 1
        fdm["propulsion/cutoff_cmd"] = 1
        for _ in range(int(30 * 120)):
            self.assertTrue(fdm.run())
            if fdm[ENGINE + "n2"] > 20:
                fdm["propulsion/cutoff_cmd"] = 0
            if fdm[PLANT + "running"] == 1:
                break
        self.assertEqual(fdm[PLANT + "running"], 1)
        self.assertLessEqual(fdm[PLANT + "ledger/energy-relative"], 1e-5)


RunTest(TestTurbinePlant)
