/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Module:       TurbinePlantXML.cpp
 Date started: 2026-10-08
 Purpose:      Reads the <plant> configuration of a coupled turbine engine.

 ------------- Copyright (C) 2026 The JSBSim contributors ----------------------

 This program is free software; you can redistribute it and/or modify it under
 the terms of the GNU Lesser General Public License as published by the Free
 Software Foundation; either version 2 of the License, or (at your option) any
 later version.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
 details.

 You should have received a copy of the GNU Lesser General Public License along
 with this program; if not, write to the Free Software Foundation, Inc., 59
 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include <cmath>
#include <sstream>
#include <string>

#include "TurbinePlantXML.h"
#include "input_output/FGLog.h"
#include "input_output/FGXMLElement.h"

namespace JSBSim {
namespace TurbinePlant {

namespace {

[[noreturn]] void Fail(Element* el, const std::string& message)
{
  XMLLogException err(el);
  err << message << "\n";
  throw err;
}

/** Optional SI number: keeps the default when the element is absent. */
void Number(Element* parent, const char* tag, double& value)
{
  if (!parent) return;
  Element* field = parent->FindElement(tag);
  if (!field) return;
  const double v = field->GetDataAsNumber();
  if (!std::isfinite(v)) Fail(field, std::string("<") + tag + "> must be one finite SI number.");
  value = v;
}

void Required(Element* parent, const char* tag, double& value)
{
  if (!parent || !parent->FindElement(tag))
    Fail(parent, std::string("<plant> requires <") + tag + ">.");
  Number(parent, tag, value);
}

void Integer(Element* parent, const char* tag, int& value)
{
  double v = value;
  Number(parent, tag, v);
  if (v != std::floor(v)) Fail(parent->FindElement(tag), std::string("<") + tag + "> must be an integer.");
  value = static_cast<int>(v);
}

/** A table is one "x y" pair per data line, x strictly increasing. */
void Table(Element* parent, const char* tag, Table1D& table, bool required)
{
  Element* field = parent ? parent->FindElement(tag) : nullptr;
  if (!field) {
    if (required) Fail(parent, std::string("<controller> requires the table <") + tag + ">.");
    return;
  }
  table = Table1D();
  for (unsigned int i = 0; i < field->GetNumDataLines(); ++i) {
    std::istringstream line(field->GetDataLine(i));
    line.imbue(std::locale::classic());
    double x, y;
    if (!(line >> x >> y) || !std::isfinite(x) || !std::isfinite(y))
      Fail(field, std::string("<") + tag + "> rows are two finite numbers, x then y.");
    std::string rest;
    if (line >> rest) Fail(field, std::string("<") + tag + "> rows are two numbers.");
    table.x.push_back(x);
    table.y.push_back(y);
  }
  if (!table.Valid()) Fail(field, std::string("<") + tag + "> needs at least one row and increasing x.");
}

void Compressor(Element* el, CompressorSpec& c)
{
  if (!el) return;
  Number(el, "pressure-ratio", c.pressureRatio);
  Number(el, "efficiency", c.efficiency);
  Number(el, "work-coefficient", c.workCoefficient);
  Number(el, "flow-coefficient", c.flowCoefficient);
  Number(el, "work-line-slope", c.workLineSlope);
  Number(el, "incidence-loss-factor", c.incidenceLossFactor);
  Number(el, "stall-flow-fraction", c.stallFlowFraction);
}

void Turbine(Element* el, TurbineSpec& t)
{
  if (!el) return;
  Number(el, "efficiency", t.efficiency);
  Number(el, "velocity-ratio", t.velocityRatio);
  Number(el, "stodola-exponent", t.stodolaExponent);
}

void Shaft(Element* el, ShaftConfig& s, const char* owner)
{
  if (!el) Fail(nullptr, std::string("<plant> requires <") + owner + ">.");
  Required(el, "inertia-kg-m2", s.inertia);
  Required(el, "design-speed-rad-sec", s.designSpeed);
  Number(el, "coulomb-torque-n-m", s.coulombTorque);
  Number(el, "coulomb-speed-rad-sec", s.coulombSpeed);
  Number(el, "viscous-n-m-s", s.viscousCoefficient);
  Number(el, "aerodynamic-n-m-s2", s.aerodynamicCoefficient);
}

void FuelPath(Element* el, FuelPathConfig& f)
{
  if (!el) return;
  Number(el, "valve-time-constant-s", f.valveTimeConstant);
  Number(el, "valve-rate-limit-kg-sec2", f.valveRateLimit);
  Number(el, "max-flow-kg-sec", f.maxFlow);
  Number(el, "manifold-prime-mass-kg", f.manifoldPrimeMass);
  Number(el, "manifold-time-constant-s", f.manifoldTimeConstant);
  Number(el, "drain-time-constant-s", f.drainTimeConstant);
}

void Combustion(Element* el, CombustionConfig& c)
{
  if (!el) return;
  Number(el, "ignition-delay-s", c.ignitionDelay);
  Number(el, "light-around-time-s", c.lightAroundTime);
  Number(el, "lean-ignition-equivalence", c.leanIgnitionEquivalence);
  Number(el, "lean-blowout-equivalence", c.leanBlowoutEquivalence);
  Number(el, "rich-limit-equivalence", c.richLimitEquivalence);
  Number(el, "minimum-pressure-pa", c.minimumPressure);
  Number(el, "minimum-inlet-temperature-k", c.minimumInletTemperature);
  Number(el, "maximum-efficiency", c.maximumEfficiency);
  Number(el, "half-efficiency-loading", c.halfEfficiencyLoading);
  Number(el, "loading-exponent", c.loadingExponent);
  Number(el, "constant-efficiency", c.constantEfficiency);
}

/** FNV-1a over names, attributes and data in document order. */
void Digest(Element* el, std::uint64_t& h)
{
  const auto mix = [&](const std::string& text) {
    for (unsigned char ch : text) { h ^= ch; h *= 1099511628211ull; }
    h ^= 0xff; h *= 1099511628211ull;
  };
  mix(el->GetName());
  for (unsigned int i = 0; i < el->GetNumDataLines(); ++i) mix(el->GetDataLine(i));
  for (const char* key : {"name", "index", "unit"})
    if (el->HasAttribute(key)) { mix(key); mix(el->GetAttributeValue(key)); }
  for (unsigned int i = 0; i < el->GetNumElements(); ++i) Digest(el->GetElement(i), h);
}
} // namespace

PlantConfig ReadPlantConfig(Element* plant, NumericalSettings& settings)
{
  PlantConfig c;
  c.name = plant->GetAttributeValue("name");
  std::uint64_t h = 1469598103934665603ull;
  Digest(plant, h);
  c.digest = h & ((1ull << 53) - 1);   // exact as a double property

  if (Element* model = plant->FindElement("gas-model")) {
    const std::string kind = model->GetDataLine();
    if (kind == "thermally-perfect") c.referenceGas = GasModelKind::ThermallyPerfect;
    else if (kind == "calorically-perfect") c.referenceGas = GasModelKind::CaloricallyPerfect;
    else Fail(model, "<gas-model> is thermally-perfect or calorically-perfect.");
  }
  Number(plant, "perfect-cp-j-kg-k", c.perfectCp);
  Number(plant, "perfect-r-j-kg-k", c.perfectR);
  if (Element* fuel = plant->FindElement("fuel")) {
    Number(fuel, "carbon-atoms", c.fuel.carbonAtoms);
    Number(fuel, "hydrogen-atoms", c.fuel.hydrogenAtoms);
    Number(fuel, "lower-heating-value-j-kg", c.fuel.lowerHeatingValue);
    Number(fuel, "specific-heat-j-kg-k", c.fuel.specificHeat);
    Number(fuel, "temperature-k", c.fuelTemperature);
  }

  Element* d = plant->FindElement("design");
  if (!d) Fail(plant, "<plant> requires <design>.");
  DesignSpec& s = c.design;
  Number(d, "ambient-pressure-pa", s.ambientPressure);
  Number(d, "ambient-temperature-k", s.ambientTemperature);
  Number(d, "mass-flow-kg-sec", s.massFlow);
  Number(d, "thrust-n", s.designThrust);
  Required(d, "bypass-ratio", s.bypassRatio);
  Number(d, "fan-pressure-ratio", s.fanPressureRatio);
  Number(d, "overall-pressure-ratio", s.overallPressureRatio);
  Required(d, "turbine-inlet-temperature-k", s.turbineInletTemperature);
  Number(d, "inlet-recovery", s.inletRecovery);
  Number(d, "cooling-fraction", s.coolingFraction);
  Number(d, "combustor-pressure-loss", s.combustorPressureLoss);
  Number(d, "combustor-hot-loss-fraction", s.combustorHotLossFraction);
  Number(d, "bypass-pressure-loss", s.bypassPressureLoss);
  Number(d, "mixer-pressure-loss", s.mixerPressureLoss);
  Number(d, "augmentor-dry-pressure-loss", s.augmentorDryPressureLoss);
  Number(d, "augmentor-hot-loss-coefficient", s.augmentorHotLossCoefficient);
  Number(d, "combustion-efficiency", s.designCombustionEfficiency);
  Number(d, "accessory-power-w", s.accessoryPower);
  Number(d, "nozzle-discharge-coefficient", s.nozzleDischarge);
  Number(d, "nozzle-velocity-coefficient", s.nozzleVelocityCoefficient);
  Number(d, "nozzle-separation-pressure-ratio", s.nozzleSeparationRatio);
  Compressor(d->FindElement("fan"), s.fan);
  Compressor(d->FindElement("compressor"), s.compressor);
  Turbine(d->FindElement("hp-turbine"), s.hpTurbine);
  Turbine(d->FindElement("lp-turbine"), s.lpTurbine);

  Shaft(plant->FindElement("lp-shaft"), c.lp, "lp-shaft");
  Shaft(plant->FindElement("hp-shaft"), c.hp, "hp-shaft");
  if (Element* starter = plant->FindElement("starter")) {
    Number(starter, "stall-torque-n-m", c.starter.stallTorque);
    Number(starter, "free-speed-rad-sec", c.starter.freeSpeed);
  }
  FuelPath(plant->FindElement("core-fuel"), c.coreFuel);
  FuelPath(plant->FindElement("augmentor-fuel"), c.abFuel);
  Combustion(plant->FindElement("core-combustion"), c.coreCombustion);
  Combustion(plant->FindElement("augmentor-combustion"), c.abCombustion);
  c.augmented = plant->FindElement("augmentor-fuel") != nullptr;

  Element* k = plant->FindElement("controller");
  if (!k) Fail(plant, "<plant> requires <controller>.");
  ControllerConfig& ctl = c.control;
  Table(k, "fan-speed-demand", ctl.fanSpeedDemand, true);
  Table(k, "acceleration-limit", ctl.accelerationLimit, true);
  Table(k, "deceleration-limit", ctl.decelerationLimit, true);
  Table(k, "start-schedule", ctl.startSchedule, true);
  Table(k, "dry-throat-area", ctl.dryThroatArea, true);
  Table(k, "exit-area-ratio", ctl.exitAreaRatio, true);
  Table(k, "augmentor-fuel-air-ratio", ctl.abFuelAirRatio, c.augmented);
  Number(k, "proportional-gain", ctl.proportionalGain);
  Number(k, "integral-gain", ctl.integralGain);
  Number(k, "idle-corrected-n2", ctl.idleCorrectedN2);
  Number(k, "idle-gain", ctl.idleGain);
  Number(k, "max-n2", ctl.maxN2);
  Number(k, "max-n2-gain", ctl.maxN2Gain);
  Number(k, "max-turbine-exit-temperature-k", ctl.maxTurbineExitK);
  Number(k, "temperature-gain", ctl.temperatureGain);
  Number(k, "lightoff-n2", ctl.lightoffN2);
  Number(k, "starter-cutout-n2", ctl.starterCutoutN2);
  Number(k, "running-n2-margin", ctl.runningN2Margin);
  Number(k, "augmentor-minimum-n2", ctl.abMinimumN2);
  Number(k, "augmentor-lightoff-fuel-air-ratio", ctl.abLightoffFuelAirRatio);
  Number(k, "augmentor-ramp-rate-per-sec", ctl.abRampRate);
  Number(k, "augmentor-preopen-fraction", ctl.abPreopenFraction);
  Number(k, "throat-area-min-fraction", ctl.throatAreaMin);
  Number(k, "throat-area-max-fraction", ctl.throatAreaMax);
  Number(k, "throat-area-rate-per-sec", ctl.throatAreaRate);
  Number(k, "throat-area-time-constant-s", ctl.throatAreaTimeConstant);
  Number(k, "egt-sensor-time-constant-s", ctl.egtSensorTimeConstant);

  for (Element* el = plant->FindElement("solid"); el; el = plant->FindNextElement("solid")) {
    SolidConfig solid;
    solid.name = el->GetAttributeValue("name");
    if (solid.name.empty()
        || solid.name.find_first_not_of("abcdefghijklmnopqrstuvwxyz0123456789-") != std::string::npos)
      Fail(el, "<solid> requires a lowercase alphanumeric/hyphen name.");
    for (const auto& prior : c.solids)
      if (prior.name == solid.name) Fail(el, "<solid> names must be unique.");
    Required(el, "heat-capacity-j-k", solid.capacity);
    double station = 7.0;
    Number(el, "gas-station", station);
    if (station != 5.0 && station != 7.0) Fail(el, "<gas-station> is 5 (turbine exit) or 7 (augmentor exit).");
    solid.gasStation = station == 5.0 ? kSt5 : kSt7;
    if (Element* coolant = el->FindElement("coolant")) {
      const std::string kind = coolant->GetDataLine();
      if (kind == "none") solid.coolant = 0;
      else if (kind == "bypass") solid.coolant = 1;
      else if (kind == "cooling") solid.coolant = 2;
      else Fail(coolant, "<coolant> is none, bypass or cooling.");
    }
    Number(el, "gas-conductance-w-k", solid.gasConductance);
    Number(el, "coolant-conductance-w-k", solid.coolantConductance);
    Number(el, "flow-exponent", solid.flowExponent);
    Number(el, "minimum-conductance-fraction", solid.minimumConductanceFraction);
    Number(el, "radiation-area-sq-m", solid.radiationArea);
    Number(el, "emissivity", solid.emissivity);
    if (!(solid.capacity > 0.0 && solid.gasConductance >= 0.0 && solid.coolantConductance >= 0.0
          && solid.radiationArea >= 0.0 && solid.emissivity >= 0.0 && solid.emissivity <= 1.0))
      Fail(el, "<solid> needs a positive capacity, nonnegative conductances and area, emissivity in [0, 1].");
    c.solids.push_back(solid);
  }

  if (Element* l = plant->FindElement("lift-system")) {
    LiftSystemConfig& lift = c.lift;
    lift.present = true;
    Shaft(l->FindElement("fan-shaft"), lift.fanShaft, "fan-shaft");
    Number(l, "gear-ratio", lift.gearRatio);
    Number(l, "clutch-capacity-n-m", lift.clutchCapacity);
    Number(l, "clutch-slip-speed-rad-sec", lift.clutchSlipSpeed);
    Number(l, "clutch-rate-per-sec", lift.clutchRate);
    Compressor(l->FindElement("fan"), lift.fan);
    Required(l, "design-flow-kg-sec", lift.designFlow);
    Number(l, "inlet-recovery", lift.inletRecovery);
    if (Element* nozzle = l->FindElement("nozzle")) {
      Number(nozzle, "throat-area-sq-m", lift.nozzle.throatArea);
      lift.nozzle.exitArea = lift.nozzle.throatArea;
      Number(nozzle, "discharge-coefficient", lift.nozzle.dischargeCoefficient);
      Number(nozzle, "velocity-coefficient", lift.nozzle.velocityCoefficient);
    }
    Number(l, "door-rate-per-sec", lift.doorRate);
    Number(l, "left-roll-post-area-sq-m", lift.rollPostArea[0]);
    Number(l, "right-roll-post-area-sq-m", lift.rollPostArea[1]);
    Number(l, "roll-post-discharge-coefficient", lift.rollPostDischarge);
    Number(l, "roll-post-velocity-coefficient", lift.rollPostVelocityCoefficient);
    Number(l, "roll-post-rate-per-sec", lift.rollPostRate);
  }

  if (Element* n = plant->FindElement("numerics")) {
    if (Element* algorithm = n->FindElement("algorithm")) {
      const std::string kind = algorithm->GetDataLine();
      if (kind == "reduced") settings.requested = Algorithm::Reduced;
      else if (kind == "component") settings.requested = Algorithm::Component;
      else Fail(algorithm, "<algorithm> is reduced or component.");
    }
    Integer(n, "iteration-cap", settings.iterationCap);
    Integer(n, "subdivision-cap", settings.subdivisionCap);
    Number(n, "tolerance", settings.tolerance);
    Number(n, "closure-budget-bytes", settings.closureBudgetBytes);
    if (!ValidSettings(settings)) Fail(n, "<numerics> out of range: iteration-cap 1..32, subdivision-cap 1..16, tolerance 1e-9..1e-3, closure-budget-bytes 65536..16777216.");
  }
  return c;
}

} // namespace TurbinePlant
} // namespace JSBSim
