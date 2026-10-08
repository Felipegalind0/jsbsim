// Openly specified synthetic mixed-flow augmented turbofan used by the plant
// tests. Round numbers, no relation to any real engine; every value here is a
// test fixture, not a calibration.
#ifndef TURBINEPLANTSYNTHETICFIXTURE_H
#define TURBINEPLANTSYNTHETICFIXTURE_H

#include <models/propulsion/plant/TurbinePlant.h>

namespace TurbinePlantFixtures {
using namespace JSBSim::TurbinePlant;

inline Table1D Table(std::initializer_list<std::pair<double, double>> rows)
{
  Table1D t;
  for (const auto& r : rows) { t.x.push_back(r.first); t.y.push_back(r.second); }
  return t;
}

inline PlantConfig Synthetic(bool lift = false)
{
  PlantConfig c;
  c.name = "synthetic-mixed-flow-augmented";
  c.augmented = true;
  DesignSpec& d = c.design;
  d.massFlow = 100.0;
  d.bypassRatio = 0.6;
  d.overallPressureRatio = 25.0;
  d.fan = CompressorSpec{0.0, 0.86, 0.45, 0.5, 0.6, 2.0, 0.75};
  d.compressor = CompressorSpec{0.0, 0.86, 0.45, 0.5, 0.6, 2.0, 0.75};
  d.hpTurbine = TurbineSpec{0.89, 0.6, 2.0};
  d.lpTurbine = TurbineSpec{0.90, 0.6, 2.0};
  d.turbineInletTemperature = 1800.0;
  d.coolingFraction = 0.10;
  d.accessoryPower = 150e3;
  c.lp = ShaftConfig{6.0, 1000.0, 5.0, 1.0, 0.02, 0.0};
  c.hp = ShaftConfig{2.5, 1500.0, 5.0, 1.0, 0.02, 0.0};
  c.starter = StarterConfig{350.0, 900.0};
  c.coreFuel = FuelPathConfig{0.05, 5.0, 5.0, 0.2, 0.02, 0.0};
  c.abFuel = FuelPathConfig{0.05, 10.0, 12.0, 1.0, 0.03, 0.0};
  CombustionConfig& cc = c.coreCombustion;
  cc.ignitionDelay = 0.3; cc.leanIgnitionEquivalence = 0.2; cc.leanBlowoutEquivalence = 0.06;
  cc.richLimitEquivalence = 3.0; cc.halfEfficiencyLoading = 0.006; cc.loadingExponent = 1.0;
  CombustionConfig& ac = c.abCombustion;
  ac.ignitionDelay = 0.15; ac.leanIgnitionEquivalence = 0.2; ac.leanBlowoutEquivalence = 0.1;
  ac.richLimitEquivalence = 1.4; ac.minimumInletTemperature = 600.0; ac.constantEfficiency = 0.9;
  ControllerConfig& k = c.control;
  k.fanSpeedDemand = Table({{0.0, 0.55}, {1.0, 1.0}});
  k.accelerationLimit = Table({{0.0, 0.8}, {0.5, 1.0}, {0.9, 1.4}, {1.1, 1.6}});
  k.decelerationLimit = Table({{0.0, 0.05}, {1.1, 0.12}});
  k.startSchedule = Table({{0.0, 1.0}, {0.6, 1.0}});
  k.dryThroatArea = Table({{0.5, 1.15}, {1.0, 1.0}});
  k.abFuelAirRatio = Table({{0.0, 0.02}, {1.0, 0.04}});
  k.exitAreaRatio = Table({{0.0, 1.05}, {1.0, 1.4}});
  k.throatAreaMin = 0.8; k.throatAreaMax = 2.0; k.throatAreaRate = 1.0; k.throatAreaTimeConstant = 0.1;
  k.idleCorrectedN2 = 0.62; k.lightoffN2 = 0.15; k.starterCutoutN2 = 0.5; k.abMinimumN2 = 0.95;
  k.abLightoffFuelAirRatio = 0.012; k.abRampRate = 0.02; k.abPreopenFraction = 0.05;
  k.maxN2 = 1.03; k.maxTurbineExitK = 1400.0;
  SolidConfig liner{"liner", 12000.0, kSt7, 1, 600.0, 600.0, 0.8, 0.02, 1.0, 0.8};
  SolidConfig core{"core", 1800.0, kSt5, 2, 600.0, 40.0, 0.8, 0.02, 0.2, 0.75};
  c.solids = {liner, core};
  if (lift) {
    LiftSystemConfig& l = c.lift;
    l.present = true;
    l.fanShaft = ShaftConfig{4.0, 700.0, 2.0, 1.0, 0.01, 0.0};
    l.gearRatio = 0.7;
    l.clutchCapacity = 15000.0; l.clutchSlipSpeed = 5.0; l.clutchRate = 1.0;
    l.fan = CompressorSpec{1.3, 0.85, 0.5, 0.5, 0.6, 2.0, 0.75};
    l.designFlow = 120.0;
    l.inletRecovery = 0.97;
    l.nozzle.dischargeCoefficient = 0.97; l.nozzle.velocityCoefficient = 0.97;
    l.doorRate = 1.0;
    l.rollPostArea = {{0.02, 0.02}};
  }
  return c;
}

inline PlantInputs SeaLevel(double throttle = 0.0, double dt = 1.0 / 120.0)
{
  PlantInputs in;
  in.dt = dt; in.advance = dt > 0.0;
  in.throttle = throttle;
  in.cutoff = false;
  return in;
}
} // namespace TurbinePlantFixtures

#endif
