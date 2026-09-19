"""Regression for steady turbine fuel flow during zero-time trim."""

import math
import xml.etree.ElementTree as et

from JSBSim_utils import JSBSimTestCase, RunTest

ENGINE = "propulsion/engine[0]/"
FUEL_FLOW = ENGINE + "fuel-flow-rate-gph"
# The F16 FCS maps the command to a 0..2 position range, so commands at or
# below 0.49 stay in the dry range and 1.0 requests augmentation.
DRY_COMMANDS = (0.0, 0.2, 0.35, 0.49)
ALL_COMMANDS = DRY_COMMANDS + (1.0,)
OFF_COMMANDS = (0.35, 1.0)
# A zero-time evaluation must not give an engine that is off a running
# operating point, so its fuel state is expected to be unchanged. Nothing
# should write it at all, so the difference is expected to be exactly zero;
# the tolerance is far below the flow a running engine reports at the same
# commands, which the tests measure rather than assume.
OFF_TOLERANCE = 1e-9
# Enough to separate a preserved zero from any value the steady assignment
# could produce: it is floored at the engine's idle fuel flow.
MINIMUM_RUNNING_FLOW_GPH = 100.0


class TestTurbineTrimFuelFlow(JSBSimTestCase):
    def setUp(self):
        super().setUp()
        # Use the standard turbine fixture without script events that could
        # overwrite the throttle while the trim value is being read back.
        script = et.parse(self.sandbox.path_to_jsbsim_file(
            "scripts", "f16_test.xml"))
        self.use = script.getroot().find("use")
        self.aircraft_name = self.use.attrib["aircraft"]

    def create_turbine_fdm(self):
        fdm = self.create_fdm()
        self.assertTrue(fdm.load_model(self.aircraft_name))
        self.assertTrue(fdm.load_ic(self.use.attrib["initialize"], True))
        fdm.set_dt(1.0 / 120.0)
        return fdm

    def trimmed_fdm(self):
        fdm = self.create_turbine_fdm()
        self.assertTrue(fdm.run_ic())
        fdm["propulsion/set-running"] = -1
        # InitRunning() marks the engine running before its zero-time
        # evaluation, so this public path really trims a running engine.
        self.assertEqual(fdm[ENGINE + "set-running"], 1)
        return fdm

    def trim_at(self, fdm, command):
        fdm["fcs/throttle-cmd-norm[0]"] = command
        fdm["fcs/throttle-pos-norm[0]"] = command
        before_time = fdm.get_sim_time()
        self.assertTrue(fdm.run_ic())
        self.assertEqual(fdm.get_sim_time(), before_time)
        return fdm[FUEL_FLOW]

    def steady_flow_at(self, command):
        """The flow a running engine reports at this command, as a reference."""
        fdm = self.trimmed_fdm()
        flow = self.trim_at(fdm, command)
        self.assertEqual(fdm[ENGINE + "set-running"], 1)
        del fdm
        return flow

    def tank_contents(self, fdm):
        catalog = fdm.query_property_catalog("propulsion/tank")
        names = [line.split()[0] for line in catalog.split("\n")
                 if "/contents-lbs" in line]
        self.assertTrue(names)
        return {name: fdm[name] for name in names}

    def fuel_state(self, fdm):
        state = {name: fdm[ENGINE + name] for name in (
            "fuel-flow-rate-pps", "fuel-flow-rate-gph", "fuel-used-lbs",
            "tsfc", "n1", "n2")}
        state["sim-time-sec"] = fdm.get_sim_time()
        state.update(self.tank_contents(fdm))
        return state

    def assert_run_ic_preserves_off_fuel_state(self, fdm, commands):
        # What the steady assignment would have written, measured on running
        # engines at the same commands, so a check that cannot detect the
        # difference fails here instead of passing quietly.
        for command in commands:
            with self.subTest(reference=command):
                self.assertGreater(self.steady_flow_at(command),
                                   MINIMUM_RUNNING_FLOW_GPH)

        for command in commands:
            with self.subTest(command=command):
                # The engine must be off before the evaluation being tested.
                self.assertEqual(fdm[ENGINE + "set-running"], 0)
                fdm["fcs/throttle-cmd-norm[0]"] = command
                before = self.fuel_state(fdm)
                self.assertTrue(fdm.run_ic())
                self.assertEqual(fdm[ENGINE + "set-running"], 0)
                after = self.fuel_state(fdm)
                for name, value in before.items():
                    with self.subTest(property=name):
                        self.assertAlmostEqual(after[name], value,
                                               delta=OFF_TOLERANCE)

    def assert_burns_no_fuel(self, fdm):
        # This fixture's flow has settled to zero, so the following frames
        # must not create any. An engine handed a running flow while off
        # reports it and debits the tanks as the flow decays.
        self.assertEqual(fdm[FUEL_FLOW], 0.0)
        tanks = self.tank_contents(fdm)
        fuel_used = fdm[ENGINE + "fuel-used-lbs"]
        before_time = fdm.get_sim_time()
        for _ in range(24):
            self.assertTrue(fdm.run())
            self.assertEqual(fdm[ENGINE + "set-running"], 0)
            self.assertEqual(fdm[FUEL_FLOW], 0.0)
            # The spools may windmill, so only their finiteness is checked.
            self.assertTrue(math.isfinite(fdm[ENGINE + "n1"]))
            self.assertTrue(math.isfinite(fdm[ENGINE + "n2"]))
        self.assertGreater(fdm.get_sim_time(), before_time)
        self.assertEqual(fdm[ENGINE + "fuel-used-lbs"], fuel_used)
        self.assertEqual(self.tank_contents(fdm), tanks)

    def test_trim_fuel_flow_needs_no_seeking(self):
        # Fuel flow is rate limited while time advances, so a trim value that
        # already is the steady one cannot be sought away on the next frame.
        # That frame still moves the airplane slightly, which shifts the thrust
        # lookups, so allow a small relative change rather than an exact match.
        for command in ALL_COMMANDS:
            with self.subTest(command=command):
                fdm = self.trimmed_fdm()
                trimmed = self.trim_at(fdm, command)
                self.assertGreater(trimmed, 0.0)
                self.assertTrue(fdm.run())
                self.assertLess(abs(fdm[FUEL_FLOW] / trimmed - 1.0), 2e-3)
                del fdm

    def test_trim_fuel_flow_does_not_retain_the_previous_setting(self):
        # Each zero-time trim must describe its own operating point, whatever
        # the engine reported before it.
        reference = {}
        for command in ALL_COMMANDS:
            fdm = self.trimmed_fdm()
            reference[command] = self.trim_at(fdm, command)
            del fdm

        fdm = self.trimmed_fdm()
        for command in (1.0, 0.0, 0.49, 0.2, 1.0, 0.35, 0.0):
            with self.subTest(command=command):
                self.assertAlmostEqual(self.trim_at(fdm, command)
                                       / reference[command], 1.0, places=6)

        dry = [reference[command] for command in DRY_COMMANDS]
        self.assertEqual(dry, sorted(dry))
        self.assertGreater(dry[-1], dry[0])
        self.assertGreater(reference[1.0], dry[-1])

    def test_trim_reports_the_tsfc_of_its_own_operating_point(self):
        fdm = self.trimmed_fdm()
        self.trim_at(fdm, 0.49)
        high = fdm["propulsion/engine[0]/tsfc"]
        self.trim_at(fdm, 0.0)
        idle = fdm["propulsion/engine[0]/tsfc"]
        # The simplified TSFC rises as N2norm falls, so the two operating
        # points must not report the same corrected value.
        self.assertGreater(idle, high)

    def test_zero_time_trim_preserves_never_started_fuel_state(self):
        fdm = self.create_turbine_fdm()
        self.assertEqual(fdm[ENGINE + "set-running"], 0)
        self.assertEqual(fdm[FUEL_FLOW], 0.0)
        # Dry and augmented commands; each RunIC repeats the initialization.
        self.assert_run_ic_preserves_off_fuel_state(fdm, OFF_COMMANDS)
        self.assert_burns_no_fuel(fdm)

    def test_zero_time_trim_preserves_fuel_state_after_cutoff(self):
        fdm = self.trimmed_fdm()
        fdm["fcs/throttle-cmd-norm[0]"] = 0.35
        for _ in range(240):  # 2 s running
            self.assertTrue(fdm.run())
        self.assertEqual(fdm[ENGINE + "set-running"], 1)
        self.assertGreater(fdm[FUEL_FLOW], MINIMUM_RUNNING_FLOW_GPH)

        fdm["propulsion/cutoff_cmd"] = 1
        for _ in range(1200):  # 10 s after cutoff
            self.assertTrue(fdm.run())
        # Shut down, still turning, and no longer burning fuel. That is this
        # engine's modelled shutdown at this operating point, not a claim
        # that every turbine's flow reaches zero the moment it is cut off.
        self.assertEqual(fdm[ENGINE + "set-running"], 0)
        self.assertGreater(fdm[ENGINE + "n1"], 0.0)
        self.assertGreater(fdm[ENGINE + "n2"], 0.0)
        self.assertEqual(fdm[FUEL_FLOW], 0.0)
        self.assertGreater(fdm[ENGINE + "fuel-used-lbs"], 0.0)

        # The second RunIC checks that nothing restores a flow later.
        self.assert_run_ic_preserves_off_fuel_state(fdm, OFF_COMMANDS)
        self.assert_burns_no_fuel(fdm)


RunTest(TestTurbineTrimFuelFlow)
