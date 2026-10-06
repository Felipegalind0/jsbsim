#include <cmath>
#include <limits>
#include <cxxtest/TestSuite.h>
#include <FGFDMExec.h>
#include <models/propulsion/FGTurbine.h>
#include "TestUtilities.h"

using namespace JSBSim;

// An inline direct-thrust engine isolates the observer from aircraft motion,
// atmosphere changes and external engine files.
class TurbineTemperatureFixture {
public:
  FGFDMExec exec;
  FGEngine::Inputs input{};
  Element_ptr document;
  std::unique_ptr<FGTurbine> engine;
  SGPropertyNode_ptr temperature;

  TurbineTemperatureFixture(unsigned int index = 0, double ambientC = 15.0,
                            const std::string& thermalXML = "") {
    input.TAT_c = ambientC;
    input.Temperature = (ambientC + 273.15) * 1.8;
    input.DensityRatio = 1.0;
    input.TotalDeltaT = 0.1;
    input.ThrottlePos.resize(index + 1, 0.0);
    std::string xml = R"(<engine>
      <turbine_engine name="temperature-observer-fixture">
        <milthrust unit="LBS">10000</milthrust>
        <maxthrust unit="LBS">15000</maxthrust>
        <augmented>1</augmented><augmethod>0</augmethod>
        <function name="IdleThrust"><value>0.05</value></function>
        <function name="MilThrust"><value>1</value></function>
        <function name="AugThrust"><value>1</value></function>
      </turbine_engine>
      <thruster><direct name="direct"/>
        <location unit="IN"><x>0</x><y>0</y><z>0</z></location>
        <orient unit="DEG"><roll>0</roll><pitch>0</pitch><yaw>0</yaw></orient>
      </thruster>
    </engine>)";
    xml.insert(xml.find("</turbine_engine>"), thermalXML);
    document = readFromXML(xml);
    engine = std::make_unique<FGTurbine>(&exec,
      document->FindElement("turbine_engine"), index, input);
    temperature = exec.GetPropertyManager()->GetNode(
      "propulsion/engine[" + std::to_string(index) + "]/egt-degc", false);
  }

  static std::string thermal(double capacity = 1000.0, double hg = 100.0,
                             double hc = 0.0, double area = 0.0,
                             double emissivity = 0.0, double gasK = 1000.0,
                             const std::string& extra = "") {
    std::ostringstream xml;
    xml << "<thermal><gas-mass-flow-kg-sec>10</gas-mass-flow-kg-sec>"
        << "<gas-specific-heat-j-kg-k>1200</gas-specific-heat-j-kg-k>"
        << "<fuel-heating-value-j-kg>43300000</fuel-heating-value-j-kg>"
        << "<afterburner-efficiency>0.9</afterburner-efficiency>"
        << "<wall-heat-capacity-j-k>" << capacity << "</wall-heat-capacity-j-k>"
        << "<gas-conductance-w-k>" << hg << "</gas-conductance-w-k>"
        << "<coolant-conductance-w-k>" << hc << "</coolant-conductance-w-k>"
        << "<radiation-area-sq-m>" << area << "</radiation-area-sq-m>"
        << "<emissivity>" << emissivity << "</emissivity>"
        << "<base-gas-temperature-k>" << gasK << "</base-gas-temperature-k>"
        << extra << "</thermal>";
    return xml.str();
  }

  static std::string core(const std::string& extra = "") {
    return "<solid-region name=\"core\">"
      "<wall-heat-capacity-j-k>2000</wall-heat-capacity-j-k>"
      "<gas-conductance-w-k>200</gas-conductance-w-k>"
      "<coolant-conductance-w-k>20</coolant-conductance-w-k>"
      "<radiation-area-sq-m>0.2</radiation-area-sq-m>"
      "<emissivity>0.8</emissivity>"
      "<gas-temperature-k>950</gas-temperature-k>" + extra + "</solid-region>";
  }

  ~TurbineTemperatureFixture() {
    // This engine is outside exec's owned propulsion list. Untie its pointer,
    // function and thruster bindings while every referenced object is alive.
    exec.GetPropertyManager()->Unbind();
  }
};

class FGTurbineTemperatureTest : public CxxTest::TestSuite {
public:
  ~FGTurbineTemperatureTest() { FGJSBBase::debug_lvl = 0; }

  void testReadOnlyIndexedCelsiusGetter() {
    TurbineTemperatureFixture f(2, -17.5);
    TS_ASSERT(f.temperature);
    TS_ASSERT(f.temperature->getAttribute(SGPropertyNode::READ));
    TS_ASSERT(!f.temperature->getAttribute(SGPropertyNode::WRITE));
    TS_ASSERT_EQUALS(f.temperature->getDoubleValue(), -17.5);
    TS_ASSERT_EQUALS(f.temperature->getDoubleValue(), f.engine->GetEGT());
    TS_ASSERT(!f.temperature->setDoubleValue(900.0));
    TS_ASSERT_EQUALS(f.engine->GetEGT(), -17.5);
    TS_ASSERT(!f.exec.GetPropertyManager()->HasNode("propulsion/engine[0]/egt-degc"));
  }

  void testColdStartIdleAugmentationAndShutdownObserveExistingDynamics() {
    TurbineTemperatureFixture f;
    f.engine->SetStarter(true);
    for (int i = 0; i < 60; ++i) f.engine->Calculate();
    TS_ASSERT(f.engine->GetN2() > 15.0);
    TS_ASSERT_EQUALS(f.engine->GetEGT(), 15.0);
    f.engine->SetCutoff(false);
    f.engine->Calculate();
    TS_ASSERT(!f.engine->GetRunning());
    TS_ASSERT_DELTA(f.engine->GetEGT(), 15.0 + 21.3 * 0.1, 1e-10);
    TS_ASSERT_EQUALS(f.temperature->getDoubleValue(), f.engine->GetEGT());
    for (int i = 0; i < 300 && !f.engine->GetRunning(); ++i) {
      f.engine->Calculate();
      TS_ASSERT_EQUALS(f.temperature->getDoubleValue(), f.engine->GetEGT());
    }
    TS_ASSERT(f.engine->GetRunning());
    f.engine->Calculate();
    TS_ASSERT_DELTA(f.engine->GetEGT(), 15.0 + 363.1, 1e-10);

    f.input.ThrottlePos[0] = 1.0;
    for (int i = 0; i < 1000; ++i) f.engine->Calculate();
    TS_ASSERT_DELTA(f.engine->GetN2(), 100.0, 1e-10);
    const double dryEGT = f.engine->GetEGT();
    const double dryThrust = f.engine->GetThrust();
    TS_ASSERT_DELTA(dryEGT, 15.0 + 363.1 + 357.1, 1e-10);
    f.engine->SetAugmentation(true);
    f.engine->Calculate();
    TS_ASSERT(f.engine->GetAugmentationActive());
    TS_ASSERT(f.engine->GetThrust() > dryThrust);
    TS_ASSERT_EQUALS(f.engine->GetEGT(), dryEGT);
    TS_ASSERT_EQUALS(f.temperature->getDoubleValue(), dryEGT);

    f.engine->SetCutoff(true);
    f.engine->Calculate();
    TS_ASSERT(!f.engine->GetRunning());
    TS_ASSERT_DELTA(f.temperature->getDoubleValue(), dryEGT - 7.3 * 0.1, 1e-10);
    TS_ASSERT(f.temperature->getDoubleValue() > 700.0);
    for (int i = 0; i < 1100; ++i) f.engine->Calculate();
    TS_ASSERT_EQUALS(f.temperature->getDoubleValue(), 15.0);
  }

  void testTrimRetentionAndResetRemainNativeSemantics() {
    TurbineTemperatureFixture f;
    f.engine->SetRunning(true);
    f.engine->SetCutoff(false);
    f.input.ThrottlePos[0] = 0.5;
    f.input.TotalDeltaT = 0.0;
    f.engine->Calculate();
    TS_ASSERT_EQUALS(f.temperature->getDoubleValue(), 15.0);
    f.input.TotalDeltaT = 0.1;
    f.engine->Calculate();
    TS_ASSERT(f.engine->GetEGT() > 500.0);
    const double hotEGT = f.engine->GetEGT();
    f.input.TotalDeltaT = 0.0;
    f.engine->Calculate();
    TS_ASSERT_EQUALS(f.temperature->getDoubleValue(), hotEGT);
    f.input.TAT_c = -5.0;
    f.engine->ResetToIC();
    TS_ASSERT_EQUALS(f.temperature->getDoubleValue(), -5.0);
    TS_ASSERT_EQUALS(f.temperature->getDoubleValue(), f.engine->GetEGT());
    TS_ASSERT(!f.engine->GetRunning());
  }

  void testOptionalThermalModelPreservesExistingEngineDynamics() {
    TurbineTemperatureFixture legacy;
    TurbineTemperatureFixture thermal(0, 15.0, TurbineTemperatureFixture::thermal());
    TS_ASSERT(!legacy.exec.GetPropertyManager()->HasNode("propulsion/engine/thermal/metal-temperature-k"));
    for (auto* f : {&legacy, &thermal}) {
      f->engine->SetRunning(true);
      f->engine->SetCutoff(false);
      f->input.TotalDeltaT = 0.0;
      f->engine->Calculate();
      f->input.TotalDeltaT = 0.1;
    }
    for (int i = 0; i < 500; ++i) {
      for (auto* f : {&legacy, &thermal}) {
        f->input.ThrottlePos[0] = i < 100 ? 0.3 : 1.0;
        f->engine->SetAugmentation(i >= 200 && i < 350);
        if (i == 400) f->engine->SetCutoff(true);
        f->engine->Calculate();
      }
      TS_ASSERT_EQUALS(thermal.engine->GetThrust(), legacy.engine->GetThrust());
      TS_ASSERT_EQUALS(thermal.engine->GetN1(), legacy.engine->GetN1());
      TS_ASSERT_EQUALS(thermal.engine->GetN2(), legacy.engine->GetN2());
      TS_ASSERT_EQUALS(thermal.engine->getFuelFlow_pph(), legacy.engine->getFuelFlow_pph());
      TS_ASSERT_EQUALS(thermal.engine->GetEGT(), legacy.engine->GetEGT());
      TS_ASSERT_EQUALS(thermal.engine->GetNozzle(), legacy.engine->GetNozzle());
      TS_ASSERT_EQUALS(thermal.engine->GetAugmentationActive(), legacy.engine->GetAugmentationActive());
    }
  }

  void testColdFirstStepDoesNotWarmInitializeOnRunningTransition() {
    TurbineTemperatureFixture f(0, 15.0, TurbineTemperatureFixture::thermal());
    TS_ASSERT(!f.engine->GetThermalInitialized());
    TS_ASSERT_DELTA(f.engine->GetMetalTemperatureK(), 288.15, 1e-10);
    f.engine->Calculate();
    TS_ASSERT(f.engine->GetThermalInitialized());
    TS_ASSERT(f.engine->GetMetalTemperatureK() > 288.15);
    TS_ASSERT(f.engine->GetMetalTemperatureK() < 300.0);
    const double coldStartedWall = f.engine->GetMetalTemperatureK();
    f.input.TotalDeltaT = 0.0;
    f.engine->InitRunning();
    TS_ASSERT_EQUALS(f.engine->GetMetalTemperatureK(), coldStartedWall);
    f.input.TotalDeltaT = 0.1;
    f.engine->Calculate();
    TS_ASSERT(f.engine->GetMetalTemperatureK() < 310.0);
  }

  void testColdSeedUsesFirstAcceptedInitialConditionAtmosphere() {
    TurbineTemperatureFixture f(0, 15.0,
      TurbineTemperatureFixture::thermal(1000, 0, 0, 0, 0));
    f.input.TAT_c = -35.0;
    f.input.TotalDeltaT = 0.0;
    f.engine->Calculate();
    TS_ASSERT(!f.engine->GetThermalInitialized());
    TS_ASSERT_EQUALS(f.engine->GetMetalTemperatureK(), 288.15);
    f.exec.SetTrimStatus(true);
    f.input.TotalDeltaT = 0.5;
    f.engine->Calculate();
    TS_ASSERT(!f.engine->GetThermalInitialized());
    f.exec.SetTrimStatus(false);
    f.input.TotalDeltaT = 0.1;
    f.engine->Calculate();
    TS_ASSERT(f.engine->GetThermalInitialized());
    TS_ASSERT_DELTA(f.engine->GetMetalTemperatureK(), 238.15, 1e-10);
    f.engine->SetMetalTemperatureStateK(800.0);
    f.input.TAT_c = 25.0;
    f.engine->Calculate();
    TS_ASSERT_EQUALS(f.engine->GetMetalTemperatureK(), 800.0);
  }

  void testWarmPendingTrimAndExplicitStateRestoration() {
    TurbineTemperatureFixture f(2, 15.0, TurbineTemperatureFixture::thermal(1000, 100, 100));
    auto pm = f.exec.GetPropertyManager();
    auto wall = pm->GetNode("propulsion/engine[2]/thermal/metal-temperature-k", false);
    auto state = pm->GetNode("propulsion/engine[2]/thermal/metal-temperature-state-k", false);
    TS_ASSERT(wall && state);
    TS_ASSERT(!wall->getAttribute(SGPropertyNode::WRITE));
    TS_ASSERT(state->getAttribute(SGPropertyNode::READ));
    TS_ASSERT(state->getAttribute(SGPropertyNode::WRITE));
    f.input.TotalDeltaT = 0.0;
    f.engine->InitRunning();
    TS_ASSERT(!f.engine->GetThermalInitialized());
    f.exec.SetTrimStatus(true);
    f.input.TotalDeltaT = 0.5;
    for (int i = 0; i < 200; ++i) f.engine->Calculate();
    TS_ASSERT(!f.engine->GetThermalInitialized());
    TS_ASSERT_EQUALS(wall->getDoubleValue(), 288.15);
    f.exec.SetTrimStatus(false);
    f.input.TotalDeltaT = 0.1;
    f.engine->Calculate();
    TS_ASSERT(f.engine->GetThermalInitialized());
    TS_ASSERT_DELTA(wall->getDoubleValue(), (1000.0 + 288.15) / 2.0, 1e-10);
    f.engine->ResetToIC();
    TS_ASSERT(!f.engine->GetThermalInitialized());
    TS_ASSERT_EQUALS(wall->getDoubleValue(), 288.15);
    TS_ASSERT(state->setDoubleValue(820.0));
    TS_ASSERT_EQUALS(state->getDoubleValue(), 820.0);
    f.input.TotalDeltaT = 0.0;
    f.engine->InitRunning();
    for (int i = 0; i < 10; ++i) f.engine->Calculate();
    TS_ASSERT_EQUALS(wall->getDoubleValue(), 820.0);
    state->setDoubleValue(-1.0);
    state->setDoubleValue(std::numeric_limits<double>::quiet_NaN());
    TS_ASSERT_EQUALS(wall->getDoubleValue(), 820.0);
    TS_ASSERT_EQUALS(f.engine->GetMetalTemperatureK(), 820.0);
  }

  void testInitRunningSuspensionOverridesCachedPositiveInputDeltaT() {
    TurbineTemperatureFixture f(0, 15.0, TurbineTemperatureFixture::thermal());
    f.exec.Setdt(0.1);
    f.input.TotalDeltaT = 0.1;
    const double cold = f.engine->GetMetalTemperatureK();
    f.engine->InitRunning();
    TS_ASSERT(!f.engine->GetThermalInitialized());
    TS_ASSERT_EQUALS(f.engine->GetMetalTemperatureK(), cold);
    TS_ASSERT(!f.exec.IntegrationSuspended());
    f.engine->Calculate();
    TS_ASSERT(f.engine->GetThermalInitialized());
    TS_ASSERT_DELTA(f.engine->GetMetalTemperatureK(), 1000.0, 1e-10);
    f.engine->SetMetalTemperatureStateK(820.0);
    f.engine->InitRunning();
    TS_ASSERT_EQUALS(f.engine->GetMetalTemperatureK(), 820.0);
    TS_ASSERT(f.engine->GetThermalInitialized());
    f.exec.SuspendIntegration();
    for (int i = 0; i < 10; ++i) f.engine->Calculate();
    TS_ASSERT_EQUALS(f.engine->GetMetalTemperatureK(), 820.0);
    f.exec.ResumeIntegration();
    f.engine->Calculate();
    TS_ASSERT(f.engine->GetMetalTemperatureK() > 820.0);
  }

  void testExplicitWarmInitialWallIsOneShot() {
    TurbineTemperatureFixture f(0, 15.0, TurbineTemperatureFixture::thermal(1000, 100, 0, 0, 0, 1000,
      "<initial-metal-temperature-k>750</initial-metal-temperature-k>"));
    f.input.TotalDeltaT = 0.0;
    f.engine->InitRunning();
    TS_ASSERT(!f.engine->GetThermalInitialized());
    f.input.TotalDeltaT = 0.1;
    f.engine->Calculate();
    TS_ASSERT_EQUALS(f.engine->GetMetalTemperatureK(), 750.0);
    f.engine->SetMetalTemperatureStateK(800.0);
    f.input.TotalDeltaT = 0.0;
    f.engine->InitRunning();
    TS_ASSERT_EQUALS(f.engine->GetMetalTemperatureK(), 800.0);
  }

  void testConvergenceToKnownExponentialAndLargeStepBounds() {
    const double exact = 1000.0 + (288.15 - 1000.0) * std::exp(-0.1);
    double previousError = 1e9;
    for (int hz : {30, 60, 120}) {
      TurbineTemperatureFixture f(0, 15.0, TurbineTemperatureFixture::thermal());
      f.input.TotalDeltaT = 1.0 / hz;
      for (int i = 0; i < hz; ++i) f.engine->Calculate();
      double error = std::abs(f.engine->GetMetalTemperatureK() - exact);
      TS_ASSERT(error < previousError);
      previousError = error;
    }
    TS_ASSERT(previousError < 0.03);
    TurbineTemperatureFixture large(0, 15.0, TurbineTemperatureFixture::thermal());
    large.input.TotalDeltaT = 1000.0;
    large.engine->Calculate();
    TS_ASSERT(large.engine->GetThermalValid());
    TS_ASSERT(large.engine->GetMetalTemperatureK() >= 288.15);
    TS_ASSERT(large.engine->GetMetalTemperatureK() <= 1000.0);
    large.engine->SetMetalTemperatureStateK(1000.0);
    large.engine->Calculate();
    TS_ASSERT_EQUALS(large.engine->GetMetalTemperatureK(), 1000.0);
  }

  void testRadiationCoolingAndHeatCapacity() {
    TurbineTemperatureFixture isolated(0, 15.0, TurbineTemperatureFixture::thermal(1000, 0));
    TurbineTemperatureFixture radiating(0, 15.0, TurbineTemperatureFixture::thermal(1000, 0, 0, 1, .8));
    TurbineTemperatureFixture heavy(0, 15.0, TurbineTemperatureFixture::thermal(10000, 0, 0, 1, .8));
    for (auto* f : {&isolated, &radiating, &heavy}) {
      f->engine->SetMetalTemperatureStateK(1000.0);
      f->input.TotalDeltaT = 1.0;
      f->engine->Calculate();
    }
    TS_ASSERT_EQUALS(isolated.engine->GetMetalTemperatureK(), 1000.0);
    TS_ASSERT(radiating.engine->GetMetalTemperatureK() < heavy.engine->GetMetalTemperatureK());
    TS_ASSERT(heavy.engine->GetMetalTemperatureK() < 1000.0);
    TS_ASSERT(radiating.engine->GetMetalTemperatureK() > 288.15);
    const double t = radiating.engine->GetMetalTemperatureK();
    const double residual = 1000.0 * (t - 1000.0)
      + 5.670374419e-8 * .8 * (std::pow(t, 4) - std::pow(288.15, 4));
    TS_ASSERT_DELTA(residual, 0.0, 1e-6);
  }

  void testAfterburnerUsesActualExcessFuelAndDoesNotResetWall() {
    TurbineTemperatureFixture f(0, 15.0, TurbineTemperatureFixture::thermal());
    f.input.ThrottlePos[0] = 1.0;
    f.input.TotalDeltaT = 0.0;
    f.engine->SetRunning(true);
    f.engine->SetCutoff(false);
    f.engine->Calculate();
    const double dryFuelPph = f.engine->getFuelFlow_pph();
    f.input.TotalDeltaT = 0.1;
    f.engine->Calculate();
    const double dryEGT = f.engine->GetEGT();
    f.engine->SetAugmentation(true);
    for (int i = 0; i < 100; ++i) f.engine->Calculate();
    TS_ASSERT(f.engine->GetAugmentationActive());
    const double abFuel = (f.engine->getFuelFlow_pph() - dryFuelPph) * .45359237 / 3600.0;
    TS_ASSERT_DELTA(f.engine->GetAfterburnerFuelFlowKgSec(), abFuel, 1e-12);
    TS_ASSERT_EQUALS(f.engine->GetAfterburnerBurnedFuelFlowKgSec(),
                     f.engine->GetAfterburnerFuelFlowKgSec());
    const double expectedGas = 298.15 + (10.0 * 1200.0 * (1000.0 - 298.15)
      + .9 * abFuel * 43300000.0) / ((10.0 + abFuel) * 1200.0);
    TS_ASSERT_DELTA(f.engine->GetNozzleGasTemperatureK(), expectedGas, 1e-10);
    TS_ASSERT_EQUALS(f.engine->GetEGT(), dryEGT);
    const double hotWall = f.engine->GetMetalTemperatureK();
    f.engine->SetAugmentation(false);
    f.engine->Calculate();
    TS_ASSERT_EQUALS(f.engine->GetAfterburnerFuelFlowKgSec(), 0.0);
    TS_ASSERT_EQUALS(f.engine->GetNozzleGasTemperatureK(), 1000.0);
    TS_ASSERT(f.engine->GetMetalTemperatureK() > 1000.0);
    TS_ASSERT(f.engine->GetMetalTemperatureK() < hotWall);
  }

  void testDynamicFunctionValidationAndZeroFlowSafety() {
    std::string xml = TurbineTemperatureFixture::thermal(1000, 0);
    const std::string original = "<gas-mass-flow-kg-sec>10</gas-mass-flow-kg-sec>";
    xml.replace(xml.find(original), original.size(),
      "<gas-mass-flow-kg-sec><function><property>thermal-test/flow</property></function></gas-mass-flow-kg-sec>");
    TurbineTemperatureFixture f(0, 15.0, xml);
    auto flow = f.exec.GetPropertyManager()->GetNode("thermal-test/flow", true);
    flow->setDoubleValue(0.0);
    f.engine->Calculate();
    TS_ASSERT(f.engine->GetThermalValid());
    TS_ASSERT_EQUALS(f.engine->GetNozzleGasTemperatureK(), 1000.0);
    const double wall = f.engine->GetMetalTemperatureK();
    flow->setDoubleValue(-1.0);
    f.engine->Calculate();
    TS_ASSERT(!f.engine->GetThermalValid());
    TS_ASSERT(std::isnan(f.engine->GetNozzleGasTemperatureK()));
    TS_ASSERT_EQUALS(f.engine->GetMetalTemperatureK(), wall);
    flow->setDoubleValue(10.0);
    f.input.TotalDeltaT = 0.0;
    f.engine->SetRunning(true);
    f.engine->SetCutoff(false);
    f.input.ThrottlePos[0] = 1.0;
    f.engine->SetAugmentation(true);
    f.engine->Calculate();
    flow->setDoubleValue(0.0);
    f.input.TotalDeltaT = 0.1;
    f.engine->Calculate();
    TS_ASSERT(!f.engine->GetThermalValid());
    TS_ASSERT_EQUALS(f.engine->GetMetalTemperatureK(), wall);
    flow->setDoubleValue(1e-15);
    f.engine->Calculate();
    TS_ASSERT(f.engine->GetThermalValid());
    TS_ASSERT(std::isfinite(f.engine->GetNozzleGasTemperatureK()));
    TS_ASSERT(f.engine->GetNozzleGasTemperatureK() <= 298.15 + .9 * 43300000.0 / 1200.0);
  }

  void testFuelRichAfterburnerHeatLimitRetainsAllSuppliedMass() {
    TurbineTemperatureFixture f(0, 15.0, TurbineTemperatureFixture::thermal(1000, 0, 0, 0, 0, 1000,
      "<stoichiometric-fuel-air-ratio>0.068</stoichiometric-fuel-air-ratio>"));
    f.input.ThrottlePos[0] = 1.0;
    f.input.TotalDeltaT = 0.0;
    f.engine->SetRunning(true);
    f.engine->SetCutoff(false);
    f.engine->Calculate();
    const double coreFuel = f.engine->getFuelFlow_pph() * .45359237 / 3600.0;
    f.input.TotalDeltaT = 0.1;
    f.engine->SetAugmentation(true);
    for (int i = 0; i < 100; ++i) f.engine->Calculate();
    const double supplied = f.engine->GetAfterburnerFuelFlowKgSec();
    const double burned = f.engine->GetAfterburnerBurnedFuelFlowKgSec();
    TS_ASSERT(supplied > burned);
    TS_ASSERT_DELTA(burned, std::max(0.0, (10.0 - coreFuel) * .068 - coreFuel), 1e-12);
    TS_ASSERT_DELTA(f.engine->GetNozzleGasTemperatureK(), 298.15 +
      (10.0 * 1200.0 * (1000.0 - 298.15) + .9 * burned * 43300000.0)
      / ((10.0 + supplied) * 1200.0), 1e-10);
    TS_ASSERT_DELTA(f.engine->getFuelFlow_pph(), 15000.0 * 1.7, 1e-10);
  }

  void testRejectedConstantThermalConfigurationUnwindsSafely() {
    TS_ASSERT_THROWS_ANYTHING(TurbineTemperatureFixture(0, 15.0,
      TurbineTemperatureFixture::thermal(0.0)));
    TS_ASSERT_THROWS_ANYTHING(TurbineTemperatureFixture(0, 15.0,
      TurbineTemperatureFixture::thermal(1000, -1.0)));
    TS_ASSERT_THROWS_ANYTHING(TurbineTemperatureFixture(0, 15.0,
      TurbineTemperatureFixture::thermal(1000, 100, 0, 1, 1.1)));
  }
  void testIndependentSolidRegionsAndIndexedReadOnlyObservations() {
    TurbineTemperatureFixture f(2, 15.0,
      TurbineTemperatureFixture::thermal(1000, 100, 100, 0, 0, 1000,
        TurbineTemperatureFixture::core()));
    auto pm = f.exec.GetPropertyManager();
    auto core = pm->GetNode("propulsion/engine[2]/thermal/core/metal-temperature-k", false);
    auto state = pm->GetNode("propulsion/engine[2]/thermal/core/metal-temperature-state-k", false);
    auto initialized = pm->GetNode("propulsion/engine[2]/thermal/core/initialized", false);
    TS_ASSERT(core && state && initialized);
    TS_ASSERT(!core->getAttribute(SGPropertyNode::WRITE));
    TS_ASSERT(!initialized->getAttribute(SGPropertyNode::WRITE));
    TS_ASSERT(!core->setDoubleValue(999.0));
    TS_ASSERT(!initialized->getBoolValue());
    f.engine->Calculate();
    TS_ASSERT(initialized->getBoolValue());
    TS_ASSERT(core->getDoubleValue() > 288.15);
    TS_ASSERT(core->getDoubleValue() < 300.0);
    for (int i = 0; i < 2000; ++i) f.engine->Calculate();
    TS_ASSERT(core->getDoubleValue() > f.engine->GetMetalTemperatureK() + 200.0);
    TS_ASSERT(core->getDoubleValue() < 950.0);
    const double downstream = f.engine->GetMetalTemperatureK();
    state->setDoubleValue(820.0);
    TS_ASSERT_EQUALS(core->getDoubleValue(), 820.0);
    TS_ASSERT_EQUALS(f.engine->GetMetalTemperatureK(), downstream);
    state->setDoubleValue(-1.0);
    state->setDoubleValue(std::numeric_limits<double>::quiet_NaN());
    TS_ASSERT_EQUALS(core->getDoubleValue(), 820.0);
  }

  void testEachSolidPreservesWarmStateAcrossSuspensionTrimAndResetRestore() {
    TurbineTemperatureFixture f(0, 15.0,
      TurbineTemperatureFixture::thermal(1000, 100, 100, 0, 0, 1000,
        TurbineTemperatureFixture::core("<initial-metal-temperature-k>780</initial-metal-temperature-k>")));
    auto pm = f.exec.GetPropertyManager();
    const std::string path = "propulsion/engine/thermal/core/";
    auto core = pm->GetNode(path + "metal-temperature-state-k", false);
    auto initialized = pm->GetNode(path + "initialized", false);
    f.exec.Setdt(0.1);
    f.engine->InitRunning();
    TS_ASSERT(!initialized->getBoolValue());
    TS_ASSERT_EQUALS(core->getDoubleValue(), 288.15);
    f.engine->Calculate();
    TS_ASSERT_EQUALS(core->getDoubleValue(), 780.0);
    core->setDoubleValue(840.0);
    f.engine->SetMetalTemperatureStateK(720.0);
    f.engine->InitRunning();
    f.exec.SuspendIntegration();
    for (int i = 0; i < 10; ++i) f.engine->Calculate();
    f.exec.ResumeIntegration();
    f.exec.SetTrimStatus(true);
    for (int i = 0; i < 10; ++i) f.engine->Calculate();
    f.exec.SetTrimStatus(false);
    TS_ASSERT_EQUALS(core->getDoubleValue(), 840.0);
    TS_ASSERT_EQUALS(f.engine->GetMetalTemperatureK(), 720.0);
    f.engine->ResetToIC();
    TS_ASSERT(!initialized->getBoolValue());
    TS_ASSERT_EQUALS(core->getDoubleValue(), 288.15);
    core->setDoubleValue(840.0);
    f.engine->SetMetalTemperatureStateK(720.0);
    f.input.TotalDeltaT = 0.0;
    f.engine->InitRunning();
    f.engine->Calculate();
    TS_ASSERT_EQUALS(core->getDoubleValue(), 840.0);
    TS_ASSERT_EQUALS(f.engine->GetMetalTemperatureK(), 720.0);
    f.input.TotalDeltaT = 0.1;
    f.engine->Calculate();
    TS_ASSERT(std::abs(core->getDoubleValue() - 840.0) < 2.0);
    TS_ASSERT(std::abs(f.engine->GetMetalTemperatureK() - 720.0) < 2.0);
  }

  void testAfterburnerDoesNotHeatSolidWithSeparateUpstreamGasBath() {
    TurbineTemperatureFixture f(0, 15.0,
      TurbineTemperatureFixture::thermal(1000, 100, 100, 0, 0, 1000,
        TurbineTemperatureFixture::core()));
    auto core = f.exec.GetPropertyManager()->GetNode(
      "propulsion/engine/thermal/core/metal-temperature-k", false);
    f.input.ThrottlePos[0] = 1.0;
    f.engine->InitRunning();
    f.engine->Calculate();
    const double dryCore = core->getDoubleValue();
    const double dryWall = f.engine->GetMetalTemperatureK();
    f.engine->SetAugmentation(true);
    for (int i = 0; i < 2000; ++i) f.engine->Calculate();
    TS_ASSERT(f.engine->GetAugmentationActive());
    TS_ASSERT_DELTA(core->getDoubleValue(), dryCore, 1e-10);
    TS_ASSERT(f.engine->GetMetalTemperatureK() > dryWall + 100.0);
  }

  void testInvalidRegionInputRetainsAllSolidsAtomically() {
    std::string coreXML = TurbineTemperatureFixture::core();
    const std::string original = "<gas-temperature-k>950</gas-temperature-k>";
    coreXML.replace(coreXML.find(original), original.size(),
      "<gas-temperature-k><function><property>thermal-test/core-k</property></function></gas-temperature-k>");
    TurbineTemperatureFixture f(0, 15.0,
      TurbineTemperatureFixture::thermal(1000, 100, 100, 0, 0, 1000, coreXML));
    auto pm = f.exec.GetPropertyManager();
    auto bath = pm->GetNode("thermal-test/core-k", true);
    auto core = pm->GetNode("propulsion/engine/thermal/core/metal-temperature-state-k", false);
    bath->setDoubleValue(950.0);
    f.engine->Calculate();
    const double wallBefore = f.engine->GetMetalTemperatureK();
    const double coreBefore = core->getDoubleValue();
    bath->setDoubleValue(-1.0);
    f.engine->Calculate();
    TS_ASSERT(!f.engine->GetThermalValid());
    TS_ASSERT(std::isnan(f.engine->GetNozzleGasTemperatureK()));
    TS_ASSERT_EQUALS(core->getDoubleValue(), coreBefore);
    TS_ASSERT_EQUALS(f.engine->GetMetalTemperatureK(), wallBefore);
    bath->setDoubleValue(950.0);
    f.engine->Calculate();
    TS_ASSERT(f.engine->GetThermalValid());
    TS_ASSERT(core->getDoubleValue() > coreBefore);
    TS_ASSERT(f.engine->GetMetalTemperatureK() > wallBefore);
  }

  void testRejectedRegionNamesAndConstantsUnwindBeforeBinding() {
    const std::string core = TurbineTemperatureFixture::core();
    TS_ASSERT_THROWS_ANYTHING(TurbineTemperatureFixture(0, 15.0,
      TurbineTemperatureFixture::thermal(1000, 100, 100, 0, 0, 1000, core + core)));
    std::string invalid = core;
    invalid.replace(invalid.find("name=\"core\""), 11, "name=\"../core\"");
    TS_ASSERT_THROWS_ANYTHING(TurbineTemperatureFixture(0, 15.0,
      TurbineTemperatureFixture::thermal(1000, 100, 100, 0, 0, 1000, invalid)));
    invalid = core;
    invalid.replace(invalid.find(">2000<"), 6, ">0<");
    TS_ASSERT_THROWS_ANYTHING(TurbineTemperatureFixture(0, 15.0,
      TurbineTemperatureFixture::thermal(1000, 100, 100, 0, 0, 1000, invalid)));
  }

};
