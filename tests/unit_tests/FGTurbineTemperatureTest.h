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

  TurbineTemperatureFixture(unsigned int index = 0, double ambientC = 15.0) {
    input.TAT_c = ambientC;
    input.TotalDeltaT = 0.1;
    input.ThrottlePos.resize(index + 1, 0.0);
    document = readFromXML(R"(<engine>
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
    </engine>)");
    engine = std::make_unique<FGTurbine>(&exec,
      document->FindElement("turbine_engine"), index, input);
    temperature = exec.GetPropertyManager()->GetNode(
      "propulsion/engine[" + std::to_string(index) + "]/egt-degc", false);
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
};
