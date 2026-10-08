/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Module:       FGTurbinePlant.cpp
 Date started: 2026-10-08
 Purpose:      FGTurbine adapter for the coupled turbine plant (<plant>).

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

FUNCTIONAL DESCRIPTION
--------------------------------------------------------------------------------
An engine opts in with a <plant> element. The plant then owns fuel metering,
ignition, combustion, the gas path, shafts, nozzle thrust and thermal solids;
FGTurbine maps its accepted state onto the legacy engine properties (n1, n2,
egt-degc, augmentation, nozzle-pos-norm, fuel flow and the thermal
observations) and publishes
the plant's own observations under propulsion/engine[n]/plant/. Every value
is computed once per accepted step or zero-time evaluation; reading a
property never evaluates the plant. See doc/turbine-plant-model.md.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "FGFDMExec.h"
#include "FGTurbine.h"
#include "FGThruster.h"
#include "input_output/FGLog.h"
#include "input_output/FGXMLElement.h"
#include "math/FGPropertyValue.h"
#include "plant/TurbinePlant.h"
#include "plant/TurbinePlantXML.h"

namespace JSBSim {

using namespace TurbinePlant;

namespace {
constexpr double kPaPerPsf = 47.880258980335840;
constexpr double kNewtonsPerLbf = 4.4482216152605;
constexpr double kKgPerLb = 0.45359237;
constexpr double kMetersPerFoot = 0.3048;

struct ValueSpec {
  const char* name;
  double (*get)(const PlantOutputs&, const PlantState&);
};

double Flag(bool b) { return b ? 1.0 : 0.0; }

#define PV(NAME, EXPR) {NAME, [](const PlantOutputs& o, const PlantState& s) -> double { (void)o; (void)s; return (EXPR); }}
const ValueSpec kValues[] = {
  PV("nozzle/gross-thrust-lbs", o.nozzle.grossThrust / kNewtonsPerLbf),
  PV("nozzle/gross-thrust-n", o.nozzle.grossThrust),
  PV("nozzle/mass-flow-kg-sec", o.nozzle.massFlow),
  PV("nozzle/exit-velocity-mps", o.nozzle.exitVelocity),
  PV("nozzle/exit-mach", o.nozzle.exitMach),
  PV("nozzle/exit-static-pressure-pa", o.nozzle.exitStaticPressure),
  PV("nozzle/exit-static-temperature-k", o.nozzle.exitStaticTemperature),
  PV("nozzle/exit-area-sq-m", o.nozzle.exitArea),
  PV("nozzle/regime", o.nozzle.regime),
  PV("nozzle/choked", Flag(o.nozzle.choked)),
  PV("nozzle/throat-area-sq-m", o.throatArea),
  PV("nozzle/position-norm", o.nozzlePosition),
  PV("lift-fan/gross-thrust-lbs", o.liftFan.grossThrust / kNewtonsPerLbf),
  PV("lift-fan/mass-flow-kg-sec", o.liftFan.massFlow),
  PV("lift-fan/exit-velocity-mps", o.liftFan.exitVelocity),
  PV("lift-fan/exit-static-temperature-k", o.liftFan.exitStaticTemperature),
  PV("lift-fan/speed-percent", o.liftFanPercent),
  PV("lift-fan/shaft-power-w", o.liftFanPowerW),
  PV("lift-fan/clutch-torque-n-m", o.clutchTorque),
  PV("roll-post[0]/gross-thrust-lbs", o.rollPost[0].grossThrust / kNewtonsPerLbf),
  PV("roll-post[0]/mass-flow-kg-sec", o.rollPost[0].massFlow),
  PV("roll-post[0]/exit-velocity-mps", o.rollPost[0].exitVelocity),
  PV("roll-post[0]/exit-static-temperature-k", o.rollPost[0].exitStaticTemperature),
  PV("roll-post[1]/gross-thrust-lbs", o.rollPost[1].grossThrust / kNewtonsPerLbf),
  PV("roll-post[1]/mass-flow-kg-sec", o.rollPost[1].massFlow),
  PV("roll-post[1]/exit-velocity-mps", o.rollPost[1].exitVelocity),
  PV("roll-post[1]/exit-static-temperature-k", o.rollPost[1].exitStaticTemperature),
  PV("ram-drag-lbs", o.ramDragN / kNewtonsPerLbf),
  PV("inlet-ram-drag-lbs", o.inletRamDragN / kNewtonsPerLbf),
  PV("lift-fan/ram-drag-lbs", o.liftFanRamDragN / kNewtonsPerLbf),
  PV("captured-air-kg-sec", o.capturedAirKgSec),
  PV("net-thrust-lbs", o.netThrustN / kNewtonsPerLbf),
  PV("shaft/n1-percent", o.n1Percent),
  PV("shaft/n2-percent", o.n2Percent),
  PV("shaft/lp-rad-sec", s.omegaLP),
  PV("shaft/hp-rad-sec", s.omegaHP),
  PV("shaft/lift-fan-rad-sec", s.omegaLF),
  PV("shaft/lp-turbine-torque-n-m", o.lpTurbineTorque),
  PV("shaft/fan-torque-n-m", o.fanTorque),
  PV("shaft/hp-turbine-torque-n-m", o.hpTurbineTorque),
  PV("shaft/compressor-torque-n-m", o.compressorTorque),
  PV("shaft/starter-torque-n-m", o.starterTorque),
  PV("fuel/core-command-kg-sec", o.coreCommandKgSec),
  PV("fuel/core-metered-kg-sec", o.coreMeteredKgSec),
  PV("fuel/core-spray-kg-sec", o.coreSprayKgSec),
  PV("fuel/core-burned-kg-sec", o.coreBurnedKgSec),
  PV("fuel/ab-command-kg-sec", o.abCommandKgSec),
  PV("fuel/ab-metered-kg-sec", o.abMeteredKgSec),
  PV("fuel/ab-spray-kg-sec", o.abSprayKgSec),
  PV("fuel/ab-burned-kg-sec", o.abBurnedKgSec),
  PV("fuel/total-metered-kg-sec", o.totalMeteredKgSec),
  PV("fuel/unburned-exhaust-kg-sec", o.unburnedExhaustKgSec),
  PV("fuel/core-manifold-kg", s.core.manifold),
  PV("fuel/ab-manifold-kg", s.ab.manifold),
  PV("fuel/used-kg", s.fuelUsedKg),
  PV("combustion/core-lit", Flag(o.coreLit)),
  PV("combustion/ab-lit", Flag(o.abLit)),
  PV("combustion/ab-selected", Flag(o.abSelected)),
  PV("combustion/igniter", Flag(o.igniter)),
  PV("combustion/core-lit-fraction", s.coreFlame.fraction),
  PV("combustion/ab-lit-fraction", s.abFlame.fraction),
  PV("combustion/core-efficiency", o.coreEfficiency),
  PV("combustion/core-heat-release-w", o.coreHeatReleaseW),
  PV("combustion/ab-heat-release-w", o.abHeatReleaseW),
  PV("combustion/core-equivalence-ratio", o.coreEquivalence),
  PV("combustion/ab-equivalence-ratio", o.abEquivalence),
  PV("combustion/egt-gauge-k", o.egtGaugeK),
  PV("control/limiter", s.control.limiter),
  PV("control/split-trim", s.control.splitTrim),
  PV("control/split-vane", s.control.splitVane),
  PV("control/clutch", s.clutch),
  PV("control/lift-nozzle", s.door),
  PV("control/roll-post[0]", s.rollPost[0]),
  PV("control/roll-post[1]", s.rollPost[1]),
  PV("running", Flag(s.running)),
  PV("starter-engaged", Flag(s.starter)),
  PV("ledger/air-in-j", o.ledger.airInJ),
  PV("ledger/fuel-sensible-j", o.ledger.fuelSensibleInJ),
  PV("ledger/chemical-j", o.ledger.chemicalJ),
  PV("ledger/starter-j", o.ledger.starterJ),
  PV("ledger/outflow-j", o.ledger.outflowJ),
  PV("ledger/accessory-j", o.ledger.accessoryJ),
  PV("ledger/friction-j", o.ledger.frictionJ),
  PV("ledger/clutch-j", o.ledger.clutchJ),
  PV("ledger/radiation-j", o.ledger.radiationJ),
  PV("ledger/dump-j", o.ledger.dumpJ),
  PV("ledger/rotor-stored-j", o.ledger.rotorStoredJ),
  PV("ledger/solid-stored-j", o.ledger.solidStoredJ),
  PV("ledger/manifold-stored-j", o.ledger.manifoldStoredJ),
  PV("ledger/energy-residual-j", o.ledger.residualJ),
  PV("ledger/energy-scale-j", o.ledger.scaleJ),
  PV("ledger/energy-relative", o.ledger.relative),
  PV("ledger/mass-relative", o.ledger.massRelative),
  PV("ledger/fuel-in-kg", o.ledger.fuelInKg),
  PV("ledger/burned-kg", o.ledger.burnedKg),
  PV("ledger/unburned-out-kg", o.ledger.unburnedOutKg),
  PV("ledger/cumulative-energy-residual-j", o.ledger.cumulativeResidualJ),
  PV("ledger/cumulative-energy-scale-j", o.ledger.cumulativeScaleJ),
  PV("ledger/valid", Flag(o.ledger.valid)),
  PV("numerics/valid", Flag(o.valid)),
  PV("numerics/algorithm", o.status.algorithm),
  PV("numerics/iterations", o.status.iterations),
  PV("numerics/residual-evaluations", o.status.residualEvaluations),
  PV("numerics/substeps", o.status.substeps),
  PV("numerics/max-residual", o.status.maxResidual),
  PV("numerics/converged", Flag(o.status.converged)),
  PV("numerics/fallback-reason", o.status.fallbackReason),
  PV("numerics/failure", Flag(o.status.failure)),
  PV("numerics/nozzle-backoff", o.status.nozzleBackoff),
  PV("numerics/domain-clamps", static_cast<double>(o.status.domainClamps)),
  PV("numerics/resident-bytes", o.status.residentBytes),
  PV("numerics/sequence", o.status.sequence),
  PV("numerics/epoch", o.status.epoch),
  PV("numerics/time-s", o.status.time),
  PV("numerics/step-seconds", o.status.stepSeconds),
  PV("numerics/fan-stall-margin", o.status.fanStallMargin),
  PV("numerics/compressor-stall-margin", o.status.compressorStallMargin),
  PV("numerics/bypass-blocked", Flag(o.status.reverseBypassBlocked)),
};
#undef PV
constexpr int kValueCount = static_cast<int>(sizeof(kValues) / sizeof(kValues[0]));
constexpr int kStationValues = 5;
constexpr int kSolidValues = 4;
constexpr int kRecordHeader = 2;     // schema version, configuration digest

/** Property path components cannot start with a digit: ARP755 station 21
    is published as st21. */
std::string StationProperty(int station)
{
  const std::string name = StationName(station);
  return !name.empty() && name[0] >= '0' && name[0] <= '9' ? "st" + name : name;
}

enum Setting : int { kAlgorithm = 0, kIterationCap, kSubdivisionCap, kTolerance, kClosureBudget,
                     kCommit, kSettingCount };
} // namespace

struct FGTurbine::PlantAdapter {
  Plant plant;
  PlantInputs inputs;
  double supplyKg = std::numeric_limits<double>::infinity();
  double stepFuelKg = 0.0;
  double thrustLbs = 0.0;
  double commitStatus = 0.0;     ///< 1 accepted, -1 rejected, 0 none since load or reset
  std::shared_ptr<FGPropertyValue> augment, throatTrim, liftSplit, clutch, door, rollPost[2];
  std::vector<double> values;    ///< published; tied by pointer, never resized after binding
  std::vector<double> record;    ///< staged state record: header, then PackState
  double eventCount = 0.0, lastEventCode = 0.0, lastEventTime = 0.0;
};

FGTurbine::~FGTurbine() = default;

void FGTurbine::PlantAdapterDeleter::operator()(PlantAdapter* adapter) const { delete adapter; }

const Plant* FGTurbine::GetPlant(void) const
{
  return PlantModel ? &PlantModel->plant : nullptr;
}

void FGTurbine::LoadPlant(FGFDMExec* exec, Element* el)
{
  Element* block = el->FindElement("plant");
  if (!block) return;
  if (el->FindElement("thermal")) {
    XMLLogException err(block);
    err << "<plant> computes gas and metal temperatures itself; remove <thermal>.\n";
    throw err;
  }
  std::unique_ptr<PlantAdapter, PlantAdapterDeleter> adapter(new PlantAdapter());
  NumericalSettings settings;
  const PlantConfig config = ReadPlantConfig(block, settings);
  if (!adapter->plant.Load(config)) {
    XMLLogException err(block);
    err << "<plant> design is not realizable: " << adapter->plant.Design().failure << "\n";
    throw err;
  }
  adapter->plant.settings = settings;
  if (!adapter->plant.SelectAlgorithm(settings.requested)) {
    XMLLogException err(block);
    err << "<numerics> closure budget is smaller than the reduced algorithm's tables.\n";
    throw err;
  }
  auto pm = exec->GetPropertyManager();
  const auto source = [&](Element* parent, const char* tag) -> std::shared_ptr<FGPropertyValue> {
    Element* field = parent ? parent->FindElement(tag) : nullptr;
    if (!field) return nullptr;
    return std::make_shared<FGPropertyValue>(field->GetDataLine(), pm, field);
  };
  adapter->augment = source(block, "augment-demand");
  adapter->throatTrim = source(block, "throat-area-trim");
  Element* lift = block->FindElement("lift-system");
  adapter->clutch = source(lift, "clutch-command");
  adapter->liftSplit = source(lift, "thrust-split-command");
  adapter->door = source(lift, "door-command");
  adapter->rollPost[0] = source(lift, "left-roll-post-command");
  adapter->rollPost[1] = source(lift, "right-roll-post-command");
  const std::size_t solids = config.solids.size();
  adapter->values.assign(kValueCount + kStationCount * kStationValues + solids * kSolidValues, 0.0);
  adapter->record.assign(kRecordHeader + StateRecordSize(solids), 0.0);
  PlantModel = std::move(adapter);
}

void FGTurbine::ResetPlant(void)
{
  PlantAdapter& a = *PlantModel;
  a.plant.InitializeCold(in.TAT_c + 273.15);
  a.plant.time = 0.0;
  a.supplyKg = std::numeric_limits<double>::infinity();
  a.stepFuelKg = a.thrustLbs = 0.0;
  a.commitStatus = 0.0;
  std::fill(a.values.begin(), a.values.end(), 0.0);
  a.eventCount = a.lastEventCode = a.lastEventTime = 0.0;
}

void FGTurbine::InitializePlantRunning(void)
{
  PlantAdapter& a = *PlantModel;
  FillPlantInputs(a.inputs, false);
  a.inputs.cutoff = false;
  a.plant.InitializeRunning(a.inputs);
  PublishPlant();
}

double FGTurbine::PlantStepFuelLbs(void) const
{
  return PlantModel->stepFuelKg / kKgPerLb;
}

void FGTurbine::SetFuelSupply(double lbs)
{
  if (PlantModel) PlantModel->supplyKg = std::isfinite(lbs) ? std::max(0.0, lbs) * kKgPerLb : lbs;
}

void FGTurbine::FillPlantInputs(PlantInputs& pi, bool advance) const
{
  pi.dt = in.TotalDeltaT;
  pi.advance = advance;
  pi.ambientPressure = in.Pressure * kPaPerPsf;
  pi.ambientTemperature = in.Temperature / 1.8;
  pi.totalPressure = std::max(in.TotalPressure, in.Pressure) * kPaPerPsf;
  pi.totalTemperature = in.TAT_c + 273.15;
  pi.airspeed = in.Vt * kMetersPerFoot;
  pi.mach = in.Soundspeed > 0.0 ? in.Vt / in.Soundspeed : 0.0;
  pi.throttle = std::clamp(ThrottlePos, 0.0, 1.0);
  const PlantAdapter& a = *PlantModel;
  double demand = 0.0;
  if (a.augment) demand = a.augment->GetValue();
  else if (AugMethod == 2) demand = AugmentCmd;
  else if (AugMethod == 1) demand = ThrottlePos > 0.99 ? 1.0 : 0.0;
  pi.augmentDemand = std::isfinite(demand) ? std::clamp(demand, 0.0, 1.0) : 0.0;
  pi.starter = Starter;
  pi.cutoff = Cutoff;
  pi.windmillAir = in.qbar > 30.0;
  pi.stalled = Stalled;
  pi.seized = Seized;
  pi.fuelFrozen = FuelFreeze || FDMExec->GetTrimStatus();
  pi.fuelSupplyKg = pi.fuelFrozen ? std::numeric_limits<double>::infinity() : a.supplyKg;
  const auto command = [](const std::shared_ptr<FGPropertyValue>& p) {
    const double v = p ? p->GetValue() : 0.0;
    return std::isfinite(v) ? std::clamp(v, 0.0, 1.0) : 0.0;
  };
  pi.clutchCommand = command(a.clutch);
  pi.doorCommand = command(a.door);
  pi.rollPostCommand = {{command(a.rollPost[0]), command(a.rollPost[1])}};
  const double trim = a.throatTrim ? a.throatTrim->GetValue() : 1.0;
  pi.throatAreaTrim = std::isfinite(trim) ? trim : 1.0;
  const double split = a.liftSplit ? a.liftSplit->GetValue() : 0.0;
  pi.liftSplit = std::isfinite(split) && split > 0.0 ? split : 0.0;
}

double FGTurbine::CalculatePlant(void)
{
  PlantAdapter& a = *PlantModel;
  const bool advance = in.TotalDeltaT > 0.0 && !FDMExec->IntegrationSuspended()
    && !FDMExec->GetTrimStatus();
  FillPlantInputs(a.inputs, advance);
  // A latched numerical failure is retried each step: the iteration cap
  // bounds the work, and changed conditions often admit a solution.
  a.plant.ClearNumericalFailure();
  const bool ok = a.plant.Advance(a.inputs);
  a.stepFuelKg = ok && advance ? a.plant.Outputs().ledger.fuelInKg : 0.0;
  // The supply is a per-step quantity; an engine outside FGPropulsion keeps
  // an unlimited supply.
  a.supplyKg = std::numeric_limits<double>::infinity();
  PublishPlant();
  return a.thrustLbs;
}

void FGTurbine::PublishPlant(void)
{
  PlantAdapter& a = *PlantModel;
  const PlantOutputs& o = a.plant.Outputs();
  const PlantState& s = a.plant.State();
  if (o.valid) {
    // Legacy observations from the same accepted state.
    N1 = o.n1Percent;
    N2 = o.n2Percent;
    Running = s.running;
    Cranking = s.starter;
    if (Running) Starter = false;       // as the empirical engine releases it
    FuelFlow_pph = o.totalMeteredKgSec / kKgPerLb * 3600.0;
    EGT_degC = o.egtGaugeK - 273.15;
    NozzlePosition = o.nozzlePosition;
    Augmentation = o.abSelected;
    AugmentationActive = o.abBurnedKgSec > 0.0;
    EPR = o.station[kSt2].Pt > 0.0 ? o.station[kSt7].Pt / o.station[kSt2].Pt : 1.0;
    OilPressure_psi = N2 * 0.62;
    phase = Running ? tpRun : tpOff;
    a.thrustLbs = o.nozzle.grossThrust / kNewtonsPerLbf;
  }
  double* v = a.values.data();
  for (int i = 0; i < kValueCount; ++i) v[i] = kValues[i].get(o, s);
  v += kValueCount;
  for (int i = 0; i < kStationCount; ++i, v += kStationValues) {
    const StationObservation& st = o.station[i];
    v[0] = st.Tt; v[1] = st.Pt; v[2] = st.W; v[3] = st.fuelAir; v[4] = st.unburnedFraction;
  }
  for (std::size_t i = 0; i < s.solids.size(); ++i, v += kSolidValues) {
    v[0] = s.solids[i].temperature;
    v[1] = i < o.solidGasHeatW.size() ? o.solidGasHeatW[i] : 0.0;
    v[2] = i < o.solidCoolantHeatW.size() ? o.solidCoolantHeatW[i] : 0.0;
    v[3] = i < o.solidRadiationW.size() ? o.solidRadiationW[i] : 0.0;
  }
  a.record[0] = kSchemaVersion;
  a.record[1] = static_cast<double>(a.plant.Config().digest);
  PackState(s, a.record.data() + kRecordHeader);
  const int count = a.plant.EventCount();
  a.eventCount = count;
  if (count > 0) {
    const PlantEvent& e = a.plant.Events()[(count - 1) % Plant::kEventCapacity];
    a.lastEventCode = e.code;
    a.lastEventTime = e.time;
  }
}

double FGTurbine::GetPlantValue(int index) const
{
  return PlantModel && index >= 0 && index < static_cast<int>(PlantModel->values.size())
    ? PlantModel->values[index] : 0.0;
}

double FGTurbine::GetPlantRecord(int index) const
{
  return PlantModel->record[index];
}

void FGTurbine::SetPlantRecord(int index, double value)
{
  PlantModel->record[index] = value;
}

double FGTurbine::GetPlantSetting(int index) const
{
  const NumericalSettings& s = PlantModel->plant.settings;
  switch (index) {
  case kAlgorithm: return static_cast<double>(static_cast<int>(s.requested));
  case kIterationCap: return s.iterationCap;
  case kSubdivisionCap: return s.subdivisionCap;
  case kTolerance: return s.tolerance;
  case kClosureBudget: return s.closureBudgetBytes;
  case kCommit: return PlantModel->commitStatus;
  default: return 0.0;
  }
}

void FGTurbine::SetPlantSetting(int index, double value)
{
  PlantAdapter& a = *PlantModel;
  Plant& p = a.plant;
  if (index == kCommit) {
    // Restore the staged record: same schema and configuration, valid values.
    if (!(value > 0.0)) return;
    const std::size_t solids = p.Config().solids.size();
    PlantState staged = p.State();
    std::string reason;
    const bool matches = a.record[0] == kSchemaVersion
      && a.record[1] == static_cast<double>(p.Config().digest);
    const bool ok = matches && UnpackState(a.record.data() + kRecordHeader, solids, staged)
      && p.RestoreState(staged, &reason);
    a.commitStatus = ok ? 1.0 : -1.0;
    if (ok) {
      PlantInputs zero;
      FillPlantInputs(zero, false);
      zero.dt = 0.0;
      p.Refresh(zero);
    }
    PublishPlant();     // a rejected record is replaced by the current state
    return;
  }
  if (!std::isfinite(value)) return;
  NumericalSettings next = p.settings;
  switch (index) {
  case kAlgorithm:
    if (value == 0.0) p.SelectAlgorithm(Algorithm::Component);
    else if (value == 1.0) p.SelectAlgorithm(Algorithm::Reduced);
    return;
  case kIterationCap: next.iterationCap = static_cast<int>(std::lround(value)); break;
  case kSubdivisionCap: next.subdivisionCap = static_cast<int>(std::lround(value)); break;
  case kTolerance: next.tolerance = value; break;
  case kClosureBudget: next.closureBudgetBytes = value; break;
  default: return;
  }
  if (!ValidSettings(next)) return;
  const bool reducedFits = next.requested != Algorithm::Reduced
    || index != kClosureBudget || p.SelectAlgorithm(Algorithm::Reduced);
  if (index == kClosureBudget && !reducedFits) {
    // The selected algorithm no longer fits: the reference takes over.
    p.settings.closureBudgetBytes = next.closureBudgetBytes;
    p.SelectAlgorithm(Algorithm::Component);
    return;
  }
  p.settings = next;
}

double FGTurbine::GetPlantSolidTemperature(int index) const
{
  const auto& solids = PlantModel->plant.State().solids;
  return index >= 0 && index < static_cast<int>(solids.size()) ? solids[index].temperature : 0.0;
}

void FGTurbine::SetPlantSolidTemperature(int index, double kelvin)
{
  if (index >= 0 && PlantModel->plant.SetSolidTemperature(static_cast<std::size_t>(index), kelvin))
    PublishPlant();
}

double FGTurbine::GetPlantSolidInitialized(int index) const
{
  const auto& solids = PlantModel->plant.State().solids;
  return index >= 0 && index < static_cast<int>(solids.size()) && solids[index].initialized ? 1.0 : 0.0;
}

void FGTurbine::BindPlant(FGPropertyManager* pm)
{
  PlantAdapter& a = *PlantModel;
  const std::string engine = CreateIndexedPropertyName("propulsion/engine", EngineNumber);
  const std::string base = engine + "/plant/";
  const auto readOnly = [&](const std::string& name, double* value) {
    pm->Tie(name, value);
    pm->GetNode(name, false)->setAttribute(SGPropertyNode::WRITE, false);
  };
  double* v = a.values.data();
  for (int i = 0; i < kValueCount; ++i) readOnly(base + kValues[i].name, v++);
  static const char* const stationFields[kStationValues] = {
    "total-temperature-k", "total-pressure-pa", "mass-flow-kg-sec", "fuel-air-ratio", "unburned-fraction"};
  for (int i = 0; i < kStationCount; ++i)
    for (int k = 0; k < kStationValues; ++k)
      readOnly(base + "station/" + StationProperty(i) + "/" + stationFields[k], v++);
  const auto& solids = a.plant.Config().solids;
  static const char* const solidFields[kSolidValues] = {
    "temperature-k", "gas-heat-w", "coolant-heat-w", "radiation-w"};
  for (const auto& solid : solids)
    for (int k = 0; k < kSolidValues; ++k)
      readOnly(base + "solid/" + solid.name + "/" + solidFields[k], v++);
  readOnly(base + "events/count", &a.eventCount);
  readOnly(base + "events/last-code", &a.lastEventCode);
  readOnly(base + "events/last-time-s", &a.lastEventTime);

  static const char* const settings[kSettingCount] = {
    "settings/algorithm", "settings/iteration-cap", "settings/subdivision-cap",
    "settings/tolerance", "settings/closure-budget-bytes", "state/commit"};
  for (int i = 0; i < kSettingCount; ++i)
    pm->Tie(base + settings[i], this, i, &FGTurbine::GetPlantSetting, &FGTurbine::SetPlantSetting);

  // Staged state record: reads give the accepted state, writes stage a
  // record, and writing state/commit restores it after validation.
  pm->Tie(base + "state/schema-version", this, 0, &FGTurbine::GetPlantRecord, &FGTurbine::SetPlantRecord);
  pm->Tie(base + "state/config-digest", this, 1, &FGTurbine::GetPlantRecord, &FGTurbine::SetPlantRecord);
  for (int i = 0; i < kStateFixedFields; ++i)
    pm->Tie(base + "state/" + StateRecordName(i), this, kRecordHeader + i,
            &FGTurbine::GetPlantRecord, &FGTurbine::SetPlantRecord);
  for (std::size_t i = 0; i < solids.size(); ++i) {
    const int at = kRecordHeader + kStateFixedFields + 2 * static_cast<int>(i);
    pm->Tie(base + "state/solid/" + solids[i].name + "/temperature-k", this, at,
            &FGTurbine::GetPlantRecord, &FGTurbine::SetPlantRecord);
    pm->Tie(base + "state/solid/" + solids[i].name + "/initialized", this, at + 1,
            &FGTurbine::GetPlantRecord, &FGTurbine::SetPlantRecord);
  }

  // Legacy thermal observations, mapped from the plant: the first solid is
  // the wall; every solid also appears under its own name.
  const auto valueIndex = [](const char* name) {
    for (int i = 0; i < kValueCount; ++i) if (std::string(kValues[i].name) == name) return i;
    return -1;
  };
  const std::string thermal = engine + "/thermal/";
  readOnly(thermal + "nozzle-gas-temperature-k", &a.values[kValueCount + kSt7 * kStationValues]);
  readOnly(thermal + "valid", &a.values[valueIndex("numerics/valid")]);
  readOnly(thermal + "afterburner-fuel-flow-kg-sec", &a.values[valueIndex("fuel/ab-spray-kg-sec")]);
  readOnly(thermal + "afterburner-burned-fuel-flow-kg-sec", &a.values[valueIndex("fuel/ab-burned-kg-sec")]);
  for (std::size_t i = 0; i < solids.size(); ++i) {
    const int index = static_cast<int>(i);
    const std::vector<std::string> prefixes = i == 0
      ? std::vector<std::string>{thermal, thermal + solids[i].name + "/"}
      : std::vector<std::string>{thermal + solids[i].name + "/"};
    for (const auto& prefix : prefixes) {
      pm->Tie(prefix + "metal-temperature-k", this, index, &FGTurbine::GetPlantSolidTemperature);
      pm->Tie(prefix + "metal-temperature-state-k", this, index,
              &FGTurbine::GetPlantSolidTemperature, &FGTurbine::SetPlantSolidTemperature);
      pm->Tie(prefix + "initialized", this, index, &FGTurbine::GetPlantSolidInitialized);
    }
  }
}

} // namespace JSBSim
