/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Module:       TurbinePlant.cpp
 Date started: 2026-10-07
 Purpose:      Coupled turbine plant: design derivation, controller, fuel and
               combustion states, matched gas path, energy-conserving shaft
               integration, steady solution and ledgers.

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

#include <algorithm>
#include <cmath>
#include <cstring>

#include "TurbinePlant.h"

namespace JSBSim {
namespace TurbinePlant {

namespace {
constexpr int kMaxSolids = 8;
constexpr int kMaxUnknowns = 10;
constexpr int kPathUnknowns = 5;   // core, bypass, mixer bypass, ln PR HPT, ln PR LPT
constexpr double kSigma = 5.670374419e-8;

bool Finite(double v) { return std::isfinite(v); }

double LagExact(double value, double target, double tau, double rate, double dt, double& average)
{
  double end = tau > 0.0 ? target + (value - target) * std::exp(-dt / tau) : target;
  average = tau > 0.0 && dt > 0.0
    ? target + (value - target) * tau / dt * (1.0 - std::exp(-dt / tau)) : end;
  if (rate > 0.0 && std::fabs(end - value) > rate * dt) {
    end = value + std::copysign(rate * dt, end - value);
    average = 0.5 * (value + end);
  }
  return end;
}

double RateLimit(double value, double target, double rate, double dt, double& average)
{
  double end = target;
  if (std::fabs(target - value) > rate * dt) end = value + std::copysign(rate * dt, target - value);
  average = 0.5 * (value + end);
  return end;
}

bool SolveLinear(int n, double* A, double* b)
{
  for (int k = 0; k < n; ++k) {
    int p = k;
    for (int i = k + 1; i < n; ++i) if (std::fabs(A[i * n + k]) > std::fabs(A[p * n + k])) p = i;
    if (!(std::fabs(A[p * n + k]) > 1e-300)) return false;
    if (p != k) {
      for (int j = 0; j < n; ++j) std::swap(A[k * n + j], A[p * n + j]);
      std::swap(b[k], b[p]);
    }
    for (int i = k + 1; i < n; ++i) {
      const double f = A[i * n + k] / A[k * n + k];
      for (int j = k; j < n; ++j) A[i * n + j] -= f * A[k * n + j];
      b[i] -= f * b[k];
    }
  }
  for (int i = n - 1; i >= 0; --i) {
    double s = b[i];
    for (int j = i + 1; j < n; ++j) s -= A[i * n + j] * b[j];
    b[i] = s / A[i * n + i];
    if (!Finite(b[i])) return false;
  }
  return true;
}

double SumSquares(const double* v, int n)
{
  double s = 0.0;
  for (int i = 0; i < n; ++i) s += v[i] * v[i];
  return s;
}

double MaxAbs(const double* v, int n)
{
  double m = 0.0;
  for (int i = 0; i < n; ++i) m = std::max(m, std::fabs(v[i]));
  return m;
}

/** Fan-law similarity seed for near-zero stored flows: pressure-form
    residuals are quadratic in flow at zero flow, so Newton needs a nonzero
    start once a rotor turns or ram pressure drives flow. */
void SeedFlows(double* x, double n1, double n2, double ram)
{
  // Only from rest: a blocked bypass legitimately stores zero bypass flow.
  const double floor = 1e-6;
  if (!(x[0] < floor) || !(n1 > 0.0 || n2 > 0.0 || ram > 0.0)) return;
  x[0] = std::max(0.3 * n2, 1e-3 * (ram > 0.0));
  if (x[1] < floor) { x[1] = std::max(0.3 * n1, 1e-3 * (ram > 0.0)); x[2] = x[1]; }
}

/** Halves the flow guesses (the first three unknowns) until the gas path
    can be evaluated: an abrupt change, such as a seizure, can leave the
    previous flows outside the domain of the new operating point. */
template <class F>
bool FeasibleStart(F&& f, double* x, double* r, int& evaluations)
{
  for (int k = 0; k < 12; ++k) {
    ++evaluations;
    if (f(x, r)) return true;
    for (int i = 0; i < 3; ++i) x[i] *= 0.5;
  }
  return false;
}

double LossTorque(const ShaftConfig& s, double omega)
{
  return s.coulombTorque * std::tanh(omega / std::max(1e-9, s.coulombSpeed))
    + s.viscousCoefficient * omega + s.aerodynamicCoefficient * omega * std::fabs(omega);
}

Stream AmbientStream(const GasProperties& gas, double Pt, double Tt, double W)
{
  Stream s;
  s.W = W; s.Pt = Pt; s.Tt = Tt;
  s.c = Composition();
  s.ht = gas.H(s.c, Tt);
  return s;
}

bool NozzleFlowConvergent(const GasProperties& gas, const Stream& s, double area, double cd,
                          double ambient, double& W)
{
  W = 0.0;
  if (!(area > 0.0) || !(s.Pt > ambient)) return true;
  double Tstar, xstar, gstar;
  if (!SonicState(gas, s.c, s.Tt, Tstar, xstar, gstar)) return false;
  const double x = ambient / s.Pt;
  if (x <= xstar) { W = cd * area * s.Pt * gstar; return Finite(W); }
  double flux, Ts, V;
  if (!FluxParameter(gas, s.c, s.Tt, x, flux, Ts, V)) return false;
  W = cd * area * s.Pt * flux;
  return Finite(W);
}
} // namespace

const char* StationName(int station)
{
  static const char* names[kStationCount] = {
    "0", "2", "21", "13", "16", "25", "3", "31", "4", "41", "45", "5", "6", "7", "8", "9",
    "liftfan-inlet", "liftfan-exit"};
  return station >= 0 && station < kStationCount ? names[station] : "";
}

double Table1D::operator()(double v) const
{
  if (x.empty()) return 0.0;
  if (v <= x.front()) return y.front();
  if (v >= x.back()) return y.back();
  const auto it = std::upper_bound(x.begin(), x.end(), v);
  const std::size_t i = static_cast<std::size_t>(it - x.begin()) - 1;
  const double f = (v - x[i]) / (x[i + 1] - x[i]);
  return y[i] + f * (y[i + 1] - y[i]);
}

bool Table1D::Valid() const
{
  if (x.empty() || x.size() != y.size()) return false;
  for (std::size_t i = 0; i < x.size(); ++i) {
    if (!Finite(x[i]) || !Finite(y[i])) return false;
    if (i > 0 && !(x[i] > x[i - 1])) return false;
  }
  return true;
}

bool ValidSettings(const NumericalSettings& s)
{
  return s.iterationCap >= 1 && s.iterationCap <= 32
    && s.subdivisionCap >= 1 && s.subdivisionCap <= 16
    && s.tolerance >= 1e-9 && s.tolerance <= 1e-3
    && s.closureBudgetBytes >= 65536.0 && s.closureBudgetBytes <= 16.0 * 1048576.0;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Per-evaluation inputs and solution of the gas path.

struct Plant::Operating {
  const GasProperties* gas = nullptr;
  double p0 = 0.0, Pt0 = 0.0, Tt0 = 0.0, V0 = 0.0, T0 = 0.0, mach = 0.0;
  double wLP = 0.0, wHP = 0.0, wLF = 0.0;
  double coreSpray = 0.0, abSpray = 0.0;
  bool coreLit = false, abLit = false, stalled = false;
  double coreFraction = 1.0, abFraction = 1.0;
  double throatArea = 0.0, exitArea = 0.0;
  std::array<double, 2> rollPostArea{{0.0, 0.0}};
  double door = 0.0, clutch = 0.0, starterTorque = 0.0;
  std::array<double, kMaxSolids> solidK{};
  int solidCount = 0;
  double liftFlowGuess = 0.0;
  double abFuelAir = 0.0;   ///< steady mode: AB spray = fuel/air times W6
  bool bypassBlocked = false;  ///< mixer junction regime
};

struct Plant::GasPath {
  std::array<Stream, kStationCount> st{};
  CompressorResult fan, hpc, lf;
  TurbineResult hpt, lpt;
  BurnerResult core, ab;
  NozzleResult nozzle, lfNozzle;
  std::array<NozzleResult, 2> rp{};
  std::array<double, 2> rpFlow{{0.0, 0.0}};
  double liftFlow = 0.0;
  double lpNet = 0.0, hpNet = 0.0, lfNet = 0.0;
  double clutchTorque = 0.0, lossLP = 0.0, lossHP = 0.0, lossLF = 0.0, accessory = 0.0;
  std::array<double, kMaxSolids> qGas{}, qCoolant{}, qRadiation{};
  std::array<double, kMaxSolids> gasK{}, coolantK{}, hGas{}, hCoolant{};
  double coreEfficiency = 0.0;
  double bypassRaw = 0.0, bypassDrive = 0.0;
  bool reverseBypassBlocked = false;
  bool valid = false;
};

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Plant::Plant() : midpointStorage_(new GasPath()), endStorage_(new GasPath()) {}
Plant::~Plant() = default;

bool Plant::Load(const PlantConfig& config)
{
  config_ = config;
  design_ = DerivedDesign();
  if (config_.solids.size() > static_cast<std::size_t>(kMaxSolids)) {
    design_.failure = "at most 8 thermal solids are supported";
    return false;
  }
  referenceGas_.Configure(config_.referenceGas, config_.fuel, config_.perfectCp, config_.perfectR);
  tabulatedGas_.Configure(config_.referenceGas == GasModelKind::CaloricallyPerfect
                            ? GasModelKind::CaloricallyPerfect : GasModelKind::Tabulated,
                          config_.fuel, config_.perfectCp, config_.perfectR);
  if (!DeriveDesign()) return false;
  state_.solids.assign(config_.solids.size(), SolidState());
  out_.solidGasHeatW.assign(config_.solids.size(), 0.0);
  out_.solidCoolantHeatW.assign(config_.solids.size(), 0.0);
  out_.solidRadiationW.assign(config_.solids.size(), 0.0);
  InitializeCold(config_.design.ambientTemperature);
  return true;
}

bool Plant::DeriveDesign()
{
  const DesignSpec& d = config_.design;
  const GasProperties& gas = referenceGas_;
  DerivedDesign& out = design_;
  const auto fail = [&](const char* why) { out.failure = why; out.valid = false; return false; };
  if (!(d.ambientPressure > 0 && d.ambientTemperature > 0 && d.bypassRatio >= 0
        && d.turbineInletTemperature > d.ambientTemperature && config_.lp.designSpeed > 0
        && config_.hp.designSpeed > 0 && config_.lp.inertia > 0 && config_.hp.inertia > 0))
    return fail("design requires ambient, bypass ratio, turbine inlet temperature, shaft speeds and inertias");
  if (!(d.fan.efficiency > 0 && d.fan.efficiency <= 1 && d.compressor.efficiency > 0
        && d.compressor.efficiency <= 1 && d.hpTurbine.efficiency > 0 && d.hpTurbine.efficiency <= 1
        && d.lpTurbine.efficiency > 0 && d.lpTurbine.efficiency <= 1))
    return fail("component efficiencies must be in (0, 1]");
  if (!(d.fanPressureRatio > 1.0 || d.overallPressureRatio > 1.0))
    return fail("design requires a fan pressure ratio or an overall pressure ratio");
  if (d.fanPressureRatio <= 1.0 && !(d.overallPressureRatio > 1.0))
    return fail("overall pressure ratio required when the fan pressure ratio is solved");
  if (d.fanPressureRatio > 1.0 && !(d.compressor.pressureRatio > 1.0) && !(d.overallPressureRatio > 1.0))
    return fail("compressor pressure ratio required");
  const ControllerConfig& c = config_.control;
  if (!(c.fanSpeedDemand.Valid() && c.accelerationLimit.Valid() && c.decelerationLimit.Valid()
        && c.startSchedule.Valid() && c.dryThroatArea.Valid() && c.exitAreaRatio.Valid()))
    return fail("controller schedules must be monotone tables");
  if (config_.augmented && !c.abFuelAirRatio.Valid()) return fail("augmented plant requires an AB schedule");
  if (!(c.throatAreaMax > 1.0 && c.throatAreaMin > 0.0 && c.throatAreaMin < 1.0))
    return fail("nozzle throat area range (fractions of design) must bracket 1");

  const double lossPowerLP = LossTorque(config_.lp, config_.lp.designSpeed) * config_.lp.designSpeed;
  const double lossPowerHP = LossTorque(config_.hp, config_.hp.designSpeed) * config_.hp.designSpeed;

  // Builds the design chain for a fan pressure ratio and a mass flow; returns
  // Pt5 - Pt16 (mixer total-pressure imbalance) and stores maps and stations.
  const auto chain = [&](double prf, double W2, double& imbalance) -> bool {
    const double prc = d.fanPressureRatio > 1.0 && d.compressor.pressureRatio > 1.0
      ? d.compressor.pressureRatio : d.overallPressureRatio / prf;
    Stream s2 = AmbientStream(gas, d.ambientPressure * d.inletRecovery, d.ambientTemperature, W2);
    out.inletCapacity = W2 * std::sqrt(d.ambientTemperature) / d.ambientPressure;
    const auto deriveCompressor = [&](const CompressorSpec& spec, double pr, const Stream& in,
                                      double omega, CompressorMap& map, Stream& outStream) -> bool {
      double Tis;
      if (!gas.IsentropicT(in.c, in.Tt, pr, Tis)) return false;
      const double dhIs = gas.H(in.c, Tis) - in.ht;
      const double dh = dhIs / spec.efficiency;
      const double U = std::sqrt(dh / spec.workCoefficient);
      map.radius = U / omega;
      const double Va = spec.flowCoefficient * U;
      const double rho = in.Pt / (gas.R(in.c) * in.Tt);
      map.area = in.W / (rho * Va);
      map.b = spec.workLineSlope * spec.workCoefficient / spec.flowCoefficient;
      map.a = spec.workCoefficient + map.b * spec.flowCoefficient;
      map.kp = 2.0 * (dh - dhIs) / (Va * Va);
      map.ki = spec.incidenceLossFactor * map.kp;
      map.phiDesign = spec.flowCoefficient;
      map.phiStall = spec.stallFlowFraction * spec.flowCoefficient;
      const CompressorResult check = Compress(gas, map, in, omega);
      outStream = check.out;
      return check.valid && std::fabs(check.out.Pt / in.Pt / pr - 1.0) < 1e-9;
    };
    Stream s21;
    if (!deriveCompressor(d.fan, prf, s2, config_.lp.designSpeed, out.fan, s21)) return false;
    const double W25 = W2 / (1.0 + d.bypassRatio);
    const double W13 = W2 - W25;
    Stream s25 = s21; s25.W = W25;
    Stream s13 = s21; s13.W = W13;
    Stream s3;
    if (!deriveCompressor(d.compressor, prc, s25, config_.hp.designSpeed, out.compressor, s3)) return false;
    Stream cooling = s3; cooling.W = d.coolingFraction * s3.W;
    Stream burnerAir = s3; burnerAir.W = s3.W - cooling.W;
    // Fuel for the design turbine inlet temperature.
    BurnerInput bi;
    bi.lit = true;
    bi.efficiency = d.designCombustionEfficiency;
    bi.fuelSensibleH = gas.H(Composition{{{0.0, 0.0, 1.0}}}, config_.fuelTemperature);
    LossModel noLoss;
    double wf = burnerAir.W * gas.Cp(burnerAir.c, 0.5 * (s3.Tt + d.turbineInletTemperature))
      * (d.turbineInletTemperature - s3.Tt) / (bi.efficiency * config_.fuel.lowerHeatingValue);
    BurnerResult br;
    for (int i = 0; i < 40; ++i) {
      bi.sprayKgSec = wf;
      br = Burn(gas, burnerAir, bi, noLoss);
      if (!br.valid) return false;
      const double err = br.out.Tt - d.turbineInletTemperature;
      if (std::fabs(err) < 1e-10 * d.turbineInletTemperature) break;
      const double cp = gas.Cp(br.out.c, br.out.Tt);
      const double dT = (bi.efficiency * config_.fuel.lowerHeatingValue - cp * (br.out.Tt - 298.15))
        / (cp * br.out.W);
      wf -= err / dT;
    }
    out.fuelFlow = wf;
    const double dpb = d.combustorPressureLoss;
    out.combustor.designCapacity = burnerAir.W * std::sqrt(burnerAir.Tt) / burnerAir.Pt;
    out.combustor.cold = (1.0 - d.combustorHotLossFraction) * dpb;
    out.combustor.hot = d.combustorHotLossFraction * dpb / (d.turbineInletTemperature / s3.Tt - 1.0);
    Stream s4 = br.out;
    s4.Pt = burnerAir.Pt * (1.0 - dpb);
    out.ratioUnit = wf / s3.Pt;
    out.loading = std::pow(s3.Pt, 1.75) * std::exp(s3.Tt / 300.0) / burnerAir.W;
    const double accessory = d.accessoryPower;
    const auto deriveTurbine = [&](const TurbineSpec& spec, const Stream& in, double power,
                                   double omega, TurbineMap& map, Stream& outStream,
                                   double& torque) -> bool {
      const double dh = power / in.W;
      const double dhIs = dh / spec.efficiency;
      double Tis;
      if (!gas.TemperatureFromH(in.c, in.ht - dhIs, in.Tt, Tis)) return false;
      const double pr = std::exp((gas.Phi(in.c, in.Tt) - gas.Phi(in.c, Tis)) / gas.R(in.c));
      if (!(pr > 1.0)) return false;
      const double capacity = in.W * std::sqrt(in.Tt) / in.Pt;
      map.exponent = spec.stodolaExponent;
      map.capacityMax = capacity / std::sqrt(1.0 - std::pow(pr, -spec.stodolaExponent));
      map.velocityRatioOptimum = spec.velocityRatio;
      map.peakEfficiency = spec.efficiency;
      const double c0 = std::sqrt(2.0 * dhIs);
      map.radius = spec.velocityRatio * c0 / omega;
      const TurbineResult check = Expand(gas, map, in, omega);
      outStream = check.out;
      torque = check.torque;
      return check.valid && std::fabs(check.dh / dh - 1.0) < 1e-8;
    };
    Stream s41, s5;
    const double hpPower = W25 * (s3.ht - s25.ht) + accessory + lossPowerHP;
    if (!deriveTurbine(d.hpTurbine, s4, hpPower, config_.hp.designSpeed, out.hpt, s41, out.hpTorque))
      return false;
    bool mixOk = false;
    LossModel none;
    Stream s45 = Mix(gas, s41, cooling, none, mixOk);
    if (!mixOk) return false;
    s45.Pt = s41.Pt;
    const double lpPower = W2 * (s21.ht - s2.ht) + lossPowerLP;
    if (!deriveTurbine(d.lpTurbine, s45, lpPower, config_.lp.designSpeed, out.lpt, s5, out.lpTorque))
      return false;
    Stream s16 = s13;
    out.bypass.designCapacity = W13 > 0 ? W13 * std::sqrt(s13.Tt) / s13.Pt : 1.0;
    out.bypass.cold = d.bypassPressureLoss;
    s16.Pt = s13.Pt * (1.0 - d.bypassPressureLoss);
    imbalance = s5.Pt - s16.Pt;
    Stream s6 = Mix(gas, s5, s16, none, mixOk);
    if (!mixOk) return false;
    out.mixer.designCapacity = s6.W * std::sqrt(s6.Tt) / s6.Pt;
    out.mixer.cold = d.mixerPressureLoss;
    s6.Pt *= 1.0 - d.mixerPressureLoss;
    out.augmentor.designCapacity = s6.W * std::sqrt(s6.Tt) / s6.Pt;
    out.augmentor.cold = d.augmentorDryPressureLoss;
    out.augmentor.hot = d.augmentorHotLossCoefficient;
    Stream s7 = s6;
    s7.Pt = s6.Pt * (1.0 - d.augmentorDryPressureLoss);
    // Nozzle throat area that passes the design flow.
    NozzleGeometry g;
    g.dischargeCoefficient = d.nozzleDischarge;
    g.velocityCoefficient = d.nozzleVelocityCoefficient;
    g.separationPressureRatio = d.nozzleSeparationRatio;
    double lo = 1e-4, hi = 50.0;
    NozzleResult nr;
    const double designPosition = (1.0 - c.throatAreaMin) / (c.throatAreaMax - c.throatAreaMin);
    const double designRatio = std::max(1.0, c.exitAreaRatio(designPosition));
    for (int i = 0; i < 200; ++i) {
      const double mid = std::sqrt(lo * hi);
      g.throatArea = mid;
      g.exitArea = mid * designRatio;
      if (!NozzleRequiredPressure(gas, s7, g, d.ambientPressure, nr)) return false;
      if (nr.requiredPt > s7.Pt) lo = mid; else hi = mid;
      if (hi / lo - 1.0 < 1e-13) break;
    }
    out.throatArea = std::sqrt(lo * hi);
    out.throatMin = c.throatAreaMin * out.throatArea;
    out.throatMax = c.throatAreaMax * out.throatArea;
    out.throatRate = c.throatAreaRate * out.throatArea;
    g.throatArea = out.throatArea;
    g.exitArea = out.throatArea * designRatio;
    if (!NozzleThrust(gas, s7, g, d.ambientPressure, nr)) return false;
    out.thrust = nr.grossThrust;
    out.gaugeRatio = s7.Pt / d.ambientPressure - 1.0;
    out.coreFlow = W25; out.bypassFlow = W13; out.fanFlow = W2;
    out.coolingFlow = cooling.W;
    out.fanPressureRatio = prf;
    out.compressorPressureRatio = prc;
    const Stream* stations[] = {&s2, &s2, &s21, &s13, &s16, &s25, &s3, &burnerAir, &s4, &s41,
                                &s45, &s5, &s6, &s7, &s7, &s7};
    for (int i = 0; i < 16; ++i) {
      out.stationPt[i] = stations[i]->Pt; out.stationTt[i] = stations[i]->Tt;
      out.stationW[i] = stations[i]->W;
    }
    out.stationPt[kSt0] = d.ambientPressure;
    return true;
  };

  double W2 = d.massFlow > 0.0 ? d.massFlow : 100.0;
  double prf = d.fanPressureRatio;
  double imbalance = 0.0;
  if (!(prf > 1.0)) {
    // Mixer total-pressure balance determines the fan pressure ratio.
    double lo = 1.05, hi = std::max(1.1, d.overallPressureRatio / 2.0);
    double flo, fhi;
    if (!chain(lo, W2, flo) || !chain(hi, W2, fhi) || flo * fhi > 0.0)
      return fail("mixer balance has no fan pressure ratio in range; check bypass ratio and turbine inlet temperature");
    for (int i = 0; i < 200 && hi - lo > 1e-12 * hi; ++i) {
      const double mid = 0.5 * (lo + hi);
      double fm;
      if (!chain(mid, W2, fm)) return fail("design chain failed during the mixer balance");
      if ((fm < 0.0) == (flo < 0.0)) { lo = mid; flo = fm; } else hi = mid;
    }
    prf = 0.5 * (lo + hi);
  }
  if (!chain(prf, W2, imbalance)) return fail("design chain failed");
  if (!(d.massFlow > 0.0)) {
    if (!(d.designThrust > 0.0)) return fail("design requires a mass flow or a thrust");
    W2 *= d.designThrust / out.thrust;     // every extensive quantity scales with flow
    if (!chain(prf, W2, imbalance)) return fail("design chain failed after sizing");
  }
  // Lift fan surrogate at its own design speed.
  if (config_.lift.present) {
    LiftSystemConfig& lift = config_.lift;
    if (!(lift.fanShaft.inertia > 0 && lift.fanShaft.designSpeed > 0 && lift.designFlow > 0
          && lift.fan.pressureRatio > 1.0 && lift.fan.efficiency > 0.0))
      return fail("lift system requires fan inertia, speed, flow, pressure ratio and efficiency");
    Stream in = AmbientStream(gas, d.ambientPressure * lift.inletRecovery, d.ambientTemperature,
                              lift.designFlow);
    double Tis;
    if (!gas.IsentropicT(in.c, in.Tt, lift.fan.pressureRatio, Tis)) return fail("lift fan design");
    const double dhIs = gas.H(in.c, Tis) - in.ht;
    const double dh = dhIs / lift.fan.efficiency;
    CompressorMap& map = out.liftFan;
    const double U = std::sqrt(dh / lift.fan.workCoefficient);
    map.radius = U / lift.fanShaft.designSpeed;
    const double Va = lift.fan.flowCoefficient * U;
    map.area = in.W / (in.Pt / (gas.R(in.c) * in.Tt) * Va);
    map.b = lift.fan.workLineSlope * lift.fan.workCoefficient / lift.fan.flowCoefficient;
    map.a = lift.fan.workCoefficient + map.b * lift.fan.flowCoefficient;
    map.kp = 2.0 * (dh - dhIs) / (Va * Va);
    map.ki = lift.fan.incidenceLossFactor * map.kp;
    map.phiDesign = lift.fan.flowCoefficient;
    map.phiStall = lift.fan.stallFlowFraction * lift.fan.flowCoefficient;
    const CompressorResult r = Compress(gas, map, in, lift.fanShaft.designSpeed);
    if (!r.valid) return fail("lift fan design check failed");
    out.liftFanTorque = r.torque;
    if (!(lift.nozzle.throatArea > 0.0)) {
      // Size the fan nozzle to pass the design flow at the fan exit pressure.
      double lo = 1e-3, hi = 20.0;
      NozzleGeometry g = lift.nozzle;
      NozzleResult nr;
      for (int i = 0; i < 200; ++i) {
        const double mid = std::sqrt(lo * hi);
        g.throatArea = g.exitArea = mid;
        if (!NozzleRequiredPressure(gas, r.out, g, d.ambientPressure, nr)) return fail("lift fan nozzle");
        if (nr.requiredPt > r.out.Pt) lo = mid; else hi = mid;
        if (hi / lo - 1.0 < 1e-13) break;
      }
      lift.nozzle.throatArea = lift.nozzle.exitArea = std::sqrt(lo * hi);
    }
  }
  out.valid = true;
  return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

void Plant::RecordEvent(double t, int code)
{
  events_[eventCount_ % kEventCapacity] = PlantEvent{t, code};
  ++eventCount_;
}

void Plant::InitializeCold(double ambientTemperature)
{
  const std::vector<SolidState> solids(config_.solids.size(), SolidState());
  state_ = PlantState();
  state_.solids = solids;
  // Solids start at ambient but are not yet established: the first accepted
  // step, a restored temperature or a warm initialization establishes them.
  for (auto& s : state_.solids) { s.temperature = ambientTemperature; s.initialized = false; }
  state_.throatArea = std::clamp(design_.throatArea * config_.control.dryThroatArea(0.0),
                                 design_.throatMin, design_.throatMax);
  state_.control.egtGaugeK = ambientTemperature;
  state_.control.sensedT5 = state_.control.sensedT6 = state_.control.sensedT7 = ambientTemperature;
  state_.control.sensedP3 = config_.design.ambientPressure;
  eventCount_ = 0;
  epoch_ += 1.0;
  sequence_ = 0.0;
  jacobianValid_ = endJacobianValid_ = false;
  bypassBlocked_ = true;   // at rest the open regime leaves bypass flow undetermined
  flowSpeedLP_ = flowSpeedHP_ = 0.0;
  lastAccelLP_ = lastAccelHP_ = 0.0;
  out_.ledger = Ledger();
  out_.status = NumericalStatus();
  out_.status.epoch = epoch_;
  out_.valid = false;
}

bool Plant::SelectAlgorithm(Algorithm algorithm)
{
  if (algorithm == Algorithm::Reduced
      && static_cast<double>(tabulatedGas_.ResidentBytes()) > settings.closureBudgetBytes)
    return false;
  settings.requested = algorithm;
  jacobianValid_ = false;      // numerical warm-start data only; physics unchanged
  return true;
}

bool Plant::RestoreState(const PlantState& s, std::string* reason)
{
  const auto reject = [&](const char* why) { if (reason) *reason = why; return false; };
  if (s.solids.size() != config_.solids.size()) return reject("solid count mismatch");
  const auto nonnegative = [](double v) { return Finite(v) && v >= 0.0; };
  if (!(nonnegative(s.omegaLP) && nonnegative(s.omegaHP) && nonnegative(s.omegaLF)
        && nonnegative(s.core.metered) && nonnegative(s.core.manifold) && nonnegative(s.core.spray)
        && nonnegative(s.ab.metered) && nonnegative(s.ab.manifold) && nonnegative(s.ab.spray)
        && nonnegative(s.coreFlame.progress) && nonnegative(s.abFlame.progress)
        && s.coreFlame.fraction >= 0.0 && s.coreFlame.fraction <= 1.0
        && s.abFlame.fraction >= 0.0 && s.abFlame.fraction <= 1.0
        && nonnegative(s.fuelUsedKg) && Finite(s.control.integrator)
        && nonnegative(s.control.abFuelAir) && nonnegative(s.control.egtGaugeK)))
    return reject("nonfinite or negative state");
  if (!(s.clutch >= 0 && s.clutch <= 1 && s.door >= 0 && s.door <= 1
        && s.rollPost[0] >= 0 && s.rollPost[0] <= 1 && s.rollPost[1] >= 0 && s.rollPost[1] <= 1))
    return reject("actuator fraction out of range");
  if (!(s.throatArea >= design_.throatMin * (1 - 1e-12) && s.throatArea <= design_.throatMax * (1 + 1e-12)))
    return reject("throat area out of range");
  for (const auto& solid : s.solids)
    if (!(Finite(solid.temperature) && solid.temperature > 0.0)) return reject("solid temperature");
  state_ = s;
  epoch_ += 1.0;
  // Numerical warm-start data follows from the restored state: the junction
  // regime of its stored solution, flows at its speeds, no acceleration.
  bypassBlocked_ = !(s.flows[2] > 0.0);
  flowSpeedLP_ = s.omegaLP; flowSpeedHP_ = s.omegaHP;
  lastAccelLP_ = lastAccelHP_ = 0.0;
  lastStarterTorque_ = 0.0;
  lastStarved_ = false;
  jacobianValid_ = endJacobianValid_ = false;
  out_.status.failure = false;
  return true;
}

bool Plant::SetSolidTemperature(std::size_t index, double kelvin)
{
  if (index >= state_.solids.size() || !Finite(kelvin) || !(kelvin > 0.0)) return false;
  SolidState& solid = state_.solids[index];
  solid.temperature = kelvin;
  solid.initialized = true;
  solid.warmPending = false;
  return true;
}

namespace {
const char* const kStateNames[kStateFixedFields] = {
  "omega-lp-rad-sec", "omega-hp-rad-sec", "omega-lf-rad-sec", "clutch", "door",
  "roll-post-0", "roll-post-1",
  "core-metered-kg-sec", "core-manifold-kg", "core-spray-kg-sec",
  "ab-metered-kg-sec", "ab-manifold-kg", "ab-spray-kg-sec",
  "core-lit", "core-ignition-progress-s", "core-lit-fraction",
  "ab-lit", "ab-ignition-progress-s", "ab-lit-fraction",
  "throat-area-sq-m", "control-integrator", "control-ab-fuel-air", "control-ab-selected",
  "control-igniter", "control-egt-gauge-k", "sensed-p3-pa", "sensed-t5-k", "sensed-t6-k",
  "sensed-t7-k", "sensed-w3b-kg-sec", "sensed-air6-kg-sec", "sensed-w6-kg-sec",
  "control-limiter", "running", "starter", "fuel-used-kg",
  "warm-start-0", "warm-start-1", "warm-start-2", "warm-start-3", "warm-start-4", "warm-start-5"};
} // namespace

const char* StateRecordName(int index)
{
  return index >= 0 && index < kStateFixedFields ? kStateNames[index] : "";
}

void PackState(const PlantState& s, double* r)
{
  const ControllerState& c = s.control;
  const double fixed[kStateFixedFields] = {
    s.omegaLP, s.omegaHP, s.omegaLF, s.clutch, s.door, s.rollPost[0], s.rollPost[1],
    s.core.metered, s.core.manifold, s.core.spray, s.ab.metered, s.ab.manifold, s.ab.spray,
    s.coreFlame.lit ? 1.0 : 0.0, s.coreFlame.progress, s.coreFlame.fraction,
    s.abFlame.lit ? 1.0 : 0.0, s.abFlame.progress, s.abFlame.fraction,
    s.throatArea, c.integrator, c.abFuelAir, c.abSelected ? 1.0 : 0.0, c.igniter ? 1.0 : 0.0,
    c.egtGaugeK, c.sensedP3, c.sensedT5, c.sensedT6, c.sensedT7, c.sensedW3b, c.sensedAir6,
    c.sensedW6, static_cast<double>(c.limiter), s.running ? 1.0 : 0.0, s.starter ? 1.0 : 0.0,
    s.fuelUsedKg, s.flows[0], s.flows[1], s.flows[2], s.flows[3], s.flows[4], s.flows[5]};
  std::memcpy(r, fixed, sizeof(fixed));
  for (std::size_t i = 0; i < s.solids.size(); ++i) {
    r[kStateFixedFields + 2 * i] = s.solids[i].temperature;
    r[kStateFixedFields + 2 * i + 1] = s.solids[i].initialized ? 1.0 : 0.0;
  }
}

bool UnpackState(const double* r, std::size_t solids, PlantState& s)
{
  const int n = StateRecordSize(solids);
  for (int i = 0; i < n; ++i) if (!Finite(r[i])) return false;
  const auto flag = [&](int i, bool& out) {
    if (r[i] != 0.0 && r[i] != 1.0) return false;
    out = r[i] == 1.0;
    return true;
  };
  ControllerState& c = s.control;
  s.omegaLP = r[0]; s.omegaHP = r[1]; s.omegaLF = r[2]; s.clutch = r[3]; s.door = r[4];
  s.rollPost = {{r[5], r[6]}};
  s.core.metered = r[7]; s.core.manifold = r[8]; s.core.spray = r[9];
  s.ab.metered = r[10]; s.ab.manifold = r[11]; s.ab.spray = r[12];
  if (!flag(13, s.coreFlame.lit)) return false;
  s.coreFlame.progress = r[14]; s.coreFlame.fraction = r[15];
  if (!flag(16, s.abFlame.lit)) return false;
  s.abFlame.progress = r[17]; s.abFlame.fraction = r[18];
  s.throatArea = r[19]; c.integrator = r[20]; c.abFuelAir = r[21];
  if (!flag(22, c.abSelected) || !flag(23, c.igniter)) return false;
  c.egtGaugeK = r[24]; c.sensedP3 = r[25]; c.sensedT5 = r[26]; c.sensedT6 = r[27];
  c.sensedT7 = r[28]; c.sensedW3b = r[29]; c.sensedAir6 = r[30]; c.sensedW6 = r[31];
  if (!(r[32] >= 0.0 && r[32] <= 7.0 && r[32] == std::floor(r[32]))) return false;
  c.limiter = static_cast<int>(r[32]);
  if (!flag(33, s.running) || !flag(34, s.starter)) return false;
  s.fuelUsedKg = r[35];
  for (int i = 0; i < 6; ++i) s.flows[i] = r[36 + i];
  s.solids.resize(solids);
  for (std::size_t i = 0; i < solids; ++i) {
    s.solids[i].temperature = r[kStateFixedFields + 2 * i];
    if (!flag(kStateFixedFields + 2 * static_cast<int>(i) + 1, s.solids[i].initialized)) return false;
    s.solids[i].warmPending = false;
  }
  return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Representative controller: min/max selection of governor errors with
// acceleration/deceleration ratio-unit limits (cf. NASA C-MAPSS40k).

void Plant::UpdateController(const PlantInputs& in, PlantState& s, double dt, double& coreCmd,
                             double& abCmd, double& starterTorque, double& throatCmd)
{
  const ControllerConfig& c = config_.control;
  ControllerState& cs = s.control;
  const double theta = std::sqrt(std::max(1e-6, in.totalTemperature / config_.design.ambientTemperature));
  const double N1 = s.omegaLP / config_.lp.designSpeed, N2 = s.omegaHP / config_.hp.designSpeed;
  const double N1c = N1 / theta, N2c = N2 / theta;
  const double P3 = std::max(cs.sensedP3, 1.0);
  const bool fuelAllowed = !in.cutoff && !in.seized && in.fuelSupplyKg > 0.0;
  cs.igniter = fuelAllowed && !s.coreFlame.lit && (in.starter || in.windmillAir) && N2 >= c.lightoffN2;
  if (!fuelAllowed) {
    coreCmd = 0.0; cs.limiter = 7; cs.integrator = 0.0;
  } else if (!s.coreFlame.lit) {
    const double ru = cs.igniter ? c.startSchedule(N2c) : 0.0;
    coreCmd = ru * design_.ratioUnit * P3;
    cs.integrator = ru;   // bumpless transfer into the running governor
    cs.limiter = 6;
  } else {
    const double eN1 = c.fanSpeedDemand(in.throttle) - N1c;
    const double eIdle = (c.idleCorrectedN2 - N2c) * c.idleGain;
    const double eN2 = (c.maxN2 - N2) * c.maxN2Gain;
    const double eT = (c.maxTurbineExitK - cs.sensedT5) * c.temperatureGain;
    double e = std::max(eN1, eIdle);
    int limiter = eN1 >= eIdle ? 0 : 5;
    if (eN2 < e) { e = eN2; limiter = 3; }
    if (eT < e) { e = eT; limiter = 4; }
    double ru = cs.integrator + c.proportionalGain * e;
    const double hi = c.accelerationLimit(N2c), lo = c.decelerationLimit(N2c);
    if (ru > hi) { ru = hi; limiter = 1; }
    else if (ru < lo) { ru = lo; limiter = 2; }
    if (limiter == 1 || limiter == 2) cs.integrator = ru - c.proportionalGain * e;  // back-calculation
    else cs.integrator += c.integralGain * e * dt;
    cs.limiter = limiter;
    coreCmd = std::max(0.0, ru) * design_.ratioUnit * P3;
  }
  // Augmentor: selection is not burning; light-off flow, then a ramp when lit.
  const bool wasSelected = cs.abSelected;
  cs.abSelected = config_.augmented && fuelAllowed && in.augmentDemand > 0.0 && s.coreFlame.lit
    && s.running && N2 >= c.abMinimumN2;
  if (cs.abSelected != wasSelected) RecordEvent(time, cs.abSelected ? 7 : 8);
  if (!cs.abSelected) cs.abFuelAir = 0.0;
  else if (!s.abFlame.lit) cs.abFuelAir = c.abLightoffFuelAirRatio;
  else {
    const double target = c.abFuelAirRatio(std::min(1.0, in.augmentDemand));
    double avg;
    cs.abFuelAir = RateLimit(cs.abFuelAir, target, c.abRampRate, dt, avg);
  }
  abCmd = cs.abFuelAir * std::max(0.0, cs.sensedW6);
  // Nozzle: dry schedule, pre-open on selection, constant corrected flow when lit.
  double area = design_.throatArea * c.dryThroatArea(N1c);
  if (cs.abSelected) area *= 1.0 + c.abPreopenFraction;
  if (s.abFlame.lit && cs.sensedT6 > 0.0 && cs.sensedT7 > cs.sensedT6)
    area *= std::sqrt(cs.sensedT7 / cs.sensedT6);
  throatCmd = std::clamp(area, design_.throatMin, design_.throatMax);
  // Air turbine starter: linear torque-speed line, released at cutout or running.
  s.starter = in.starter && !s.running && N2 < c.starterCutoutN2;
  starterTorque = s.starter
    ? config_.starter.stallTorque * std::max(0.0, 1.0 - s.omegaHP / config_.starter.freeSpeed) : 0.0;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Gas path at given speeds, fuel sprays, flames and geometry. x holds the
// flow unknowns scaled by design flows (core W25, bypass W13, mixer bypass
// W16) and the natural logs of the HP and LP turbine pressure ratios.
// Residuals: nozzle total pressure, mixer junction, bypass/roll-post mass
// split, and the two turbine flow capacities. Turbine pressure ratios are
// unknowns because the ellipse law makes pressure ratio a nearly singular
// function of flow near choke, whereas capacity is a smooth bounded
// function of pressure ratio.


namespace {
double Conductance(double designValue, double flow, double designFlow, double exponent, double floor)
{
  if (!(designValue > 0.0)) return 0.0;
  const double ratio = designFlow > 0.0 ? std::max(0.0, flow) / designFlow : 0.0;
  return designValue * std::max(floor, std::pow(ratio, exponent));
}

/** Effective conductance W cp (1 - exp(-NTU)) of a stream passing a wall at
    uniform temperature: tends to h at high flow and to W cp, so the stream
    can at most reach the wall temperature, as flow vanishes. */
double StreamConductance(double h, double W, double cp)
{
  if (!(h > 0.0) || !(W > 0.0) || !(cp > 0.0)) return 0.0;
  const double capacity = W * cp;
  return capacity * -std::expm1(-h / capacity);
}
} // namespace

bool Plant::EvaluatePath(const Operating& op, const double* x, GasPath& g, double* residual) const
{
  const GasProperties& gas = *op.gas;
  const DesignSpec& d = config_.design;
  g.valid = false;
  const double coreScale = design_.coreFlow;
  const double bypassScale = std::max(design_.bypassFlow, 1e-3 * design_.fanFlow);
  const double W25 = std::max(0.0, x[0] * coreScale);
  // Bypass flows are signed in iteration: the open junction regime extends
  // smoothly through zero (a converged negative flow switches the regime to
  // blocked, so no reverse flow is ever accepted); blocked forces zero.
  const double W13raw = x[1] * bypassScale;
  const double W13 = W13raw;
  const double W16raw = x[2] * bypassScale;
  const double W16 = op.bypassBlocked ? 0.0 : W16raw;
  if (!(W25 + W13 > -1e-9 * coreScale)) return false;
  if (!(Finite(W25) && Finite(W13) && Finite(W16raw))) return false;

  // Inlet: MIL-E-5008B supersonic shock recovery, and a subsonic duct loss
  // proportional to the square of corrected flow (design recovery at design flow).
  double shock = op.mach > 1.0 ? 1.0 - 0.075 * std::pow(op.mach - 1.0, 1.35) : 1.0;
  const double inletFlow = (W25 + W13) * std::sqrt(op.Tt0) / std::max(1.0, op.Pt0 * shock)
    / design_.inletCapacity;
  const double recovery = shock * (1.0 - (1.0 - d.inletRecovery) * inletFlow * inletFlow);
  if (!(recovery > 0.1)) return false;
  Stream s2 = AmbientStream(gas, op.Pt0 * recovery, op.Tt0, W25 + W13);
  g.st[kSt0] = AmbientStream(gas, op.Pt0, op.Tt0, s2.W);
  g.st[kSt2] = s2;
  g.fan = Compress(gas, design_.fan, s2, op.wLP);
  if (!g.fan.valid) return false;
  const Stream& s21 = g.fan.out;
  g.st[kSt21] = s21;
  Stream s13 = s21; s13.W = W13;
  Stream s25 = s21; s25.W = W25;
  g.st[kSt13] = s13; g.st[kSt25] = s25;

  // Bypass-fed roll posts (convergent nozzles) at station 13 conditions.
  double rollPost = 0.0;
  for (int i = 0; i < 2; ++i) {
    if (!NozzleFlowConvergent(gas, s13, op.rollPostArea[i], config_.lift.rollPostDischarge, op.p0,
                              g.rpFlow[i])) return false;
    rollPost += g.rpFlow[i];
  }
  Stream s16 = s13; s16.W = W16;
  const double bypassLoss = PressureLossFraction(design_.bypass, s16, s16.Tt);
  if (!(bypassLoss < 0.9)) return false;
  s16.Pt *= 1.0 - bypassLoss;

  CompressorMap hpcMap = design_.compressor;
  if (op.stalled) { hpcMap.a *= 0.5; hpcMap.kp *= 2.0; hpcMap.ki *= 2.0; }
  g.hpc = Compress(gas, hpcMap, s25, op.wHP);
  if (!g.hpc.valid) return false;
  const Stream& s3 = g.hpc.out;
  g.st[kSt3] = s3;
  Stream s31 = s3; s31.W = d.coolingFraction * s3.W;
  Stream burnerAir = s3; burnerAir.W = s3.W - s31.W;

  // Coolant-side solid exchanges enter internal streams (internal heat).
  const int solids = op.solidCount;
  for (int i = 0; i < solids; ++i) { g.qGas[i] = 0.0; g.hGas[i] = 0.0; g.gasK[i] = op.T0; }
  for (int i = 0; i < solids; ++i) {
    const SolidConfig& sc = config_.solids[i];
    g.qCoolant[i] = 0.0; g.hCoolant[i] = 0.0; g.coolantK[i] = op.T0;
    Stream* coolant = sc.coolant == 1 ? &s16 : sc.coolant == 2 ? &s31 : nullptr;
    if (coolant && coolant->W > 0.0) {
      const double designFlow = sc.coolant == 1 ? design_.bypassFlow : design_.coolingFlow;
      g.hCoolant[i] = StreamConductance(
        Conductance(sc.coolantConductance, coolant->W, designFlow, sc.flowExponent,
                    sc.minimumConductanceFraction),
        coolant->W, gas.Cp(coolant->c, coolant->Tt));
      g.coolantK[i] = coolant->Tt;
      g.qCoolant[i] = g.hCoolant[i] * (op.solidK[i] - coolant->Tt);
      if (!AddHeat(gas, *coolant, g.qCoolant[i])) return false;
    }
    g.qRadiation[i] = kSigma * sc.emissivity * sc.radiationArea
      * (std::pow(op.solidK[i], 4) - std::pow(op.T0, 4));
  }
  g.st[kSt16] = s16; g.st[kSt31] = s31;

  // Combustor: loading-based efficiency (Lefebvre theta parameter, normalized).
  const CombustionConfig& cc = config_.coreCombustion;
  double eta = 0.0;
  if (op.coreLit && burnerAir.W > 0.0) {
    const double loading = std::pow(burnerAir.Pt, 1.75) * std::exp(burnerAir.Tt / 300.0) / burnerAir.W
      / design_.loading;
    const double s = cc.loadingExponent, h50 = cc.halfEfficiencyLoading;
    const double etaMax = std::min(1.0, d.designCombustionEfficiency * (1.0 + std::pow(h50, s)));
    eta = etaMax / (1.0 + std::pow(h50 / std::max(loading, 1e-300), s));
  }
  g.coreEfficiency = eta;
  BurnerInput core;
  core.sprayKgSec = op.coreSpray;
  core.fuelSensibleH = gas.H(Composition{{{0.0, 0.0, 1.0}}}, config_.fuelTemperature);
  core.lit = op.coreLit;
  core.efficiency = eta * std::clamp(op.coreFraction, 0.0, 1.0);
  g.core = Burn(gas, burnerAir, core, design_.combustor);
  if (!g.core.valid) return false;
  g.st[kSt4] = g.core.out;
  if (!(Finite(x[3]) && Finite(x[4]) && x[3] < 20.0 && x[4] < 20.0)) return false;
  g.hpt = ExpandAt(gas, design_.hpt, g.core.out, op.wHP, std::exp(std::max(0.0, x[3])));
  if (!g.hpt.valid) return false;
  // Capacity residual in squared form, smooth at zero flow and at choke.
  const double fh = g.hpt.capacityFraction;
  residual[3] = fh * std::fabs(fh) - CapacityFraction(design_.hpt, g.hpt.pressureRatio)
    * CapacityFraction(design_.hpt, g.hpt.pressureRatio);
  g.st[kSt41] = g.hpt.out;
  bool ok = false;
  LossModel none;
  Stream s45 = Mix(gas, g.hpt.out, s31, none, ok);
  if (!ok) return false;
  s45.Pt = g.hpt.out.Pt;   // cooling re-enters at the higher HPC delivery pressure
  g.st[kSt45] = s45;
  g.lpt = ExpandAt(gas, design_.lpt, s45, op.wLP, std::exp(std::max(0.0, x[4])));
  if (!g.lpt.valid) return false;
  const double fl = g.lpt.capacityFraction;
  residual[4] = fl * std::fabs(fl) - CapacityFraction(design_.lpt, g.lpt.pressureRatio)
    * CapacityFraction(design_.lpt, g.lpt.pressureRatio);
  Stream s5 = g.lpt.out;
  for (int i = 0; i < solids; ++i) {
    const SolidConfig& sc = config_.solids[i];
    if (sc.gasStation != kSt5) continue;
    g.gasK[i] = s5.Tt;
    g.hGas[i] = StreamConductance(Conductance(sc.gasConductance, s5.W, design_.stationW[kSt5],
                                              sc.flowExponent, sc.minimumConductanceFraction),
                                  s5.W, gas.Cp(s5.c, s5.Tt));
    g.qGas[i] = g.hGas[i] * (s5.Tt - op.solidK[i]);
    if (s5.W > 0.0 && !AddHeat(gas, s5, -g.qGas[i])) return false;
  }
  g.st[kSt5] = s5;

  // Pressure residuals are relative to a gauge pressure that depends only on
  // the evaluation speeds and flight condition (fan-law similarity of the
  // design nozzle gauge, plus ram, floored at 1e-3 of ambient), never on the
  // unknowns: a scale taken from the iterated pressures is constant, so the
  // residual is flat, wherever the two sides straddle ambient.
  const double n1 = op.wLP / config_.lp.designSpeed;
  const double pScale = std::max(1e-3 * op.p0, std::max(0.0, op.Pt0 - op.p0)
                                 + op.Pt0 * design_.gaugeRatio * n1 * n1);
  // Mixer entry is a one-way junction solved as an active set: open (equal
  // total pressures) or blocked (no bypass flow). The caller switches regime
  // when a converged solution violates its sign condition; each regime is smooth.
  residual[1] = op.bypassBlocked ? W16raw / bypassScale : (s5.Pt - s16.Pt) / pScale;
  g.bypassRaw = W16raw;
  g.bypassDrive = s16.Pt - s5.Pt;
  g.reverseBypassBlocked = op.bypassBlocked;
  Stream s16m = s16; s16m.W = W16;
  Stream s6 = Mix(gas, s5, s16m, design_.mixer, ok);
  if (!ok) return false;
  g.st[kSt6] = s6;
  BurnerInput ab;
  ab.sprayKgSec = op.abFuelAir > 0.0 ? op.abFuelAir * s6.W : op.abSpray;
  ab.fuelSensibleH = core.fuelSensibleH;
  ab.lit = op.abLit;
  ab.efficiency = (config_.abCombustion.constantEfficiency > 0.0
    ? config_.abCombustion.constantEfficiency : 0.9) * std::clamp(op.abFraction, 0.0, 1.0);
  g.ab = Burn(gas, s6, ab, design_.augmentor);
  if (!g.ab.valid) return false;
  Stream s7 = g.ab.out;
  for (int i = 0; i < solids; ++i) {
    const SolidConfig& sc = config_.solids[i];
    if (sc.gasStation != kSt7) continue;
    g.gasK[i] = s7.Tt;
    g.hGas[i] = StreamConductance(Conductance(sc.gasConductance, s7.W, design_.stationW[kSt7],
                                              sc.flowExponent, sc.minimumConductanceFraction),
                                  s7.W, gas.Cp(s7.c, s7.Tt));
    g.qGas[i] = g.hGas[i] * (s7.Tt - op.solidK[i]);
    if (s7.W > 0.0 && !AddHeat(gas, s7, -g.qGas[i])) return false;
  }
  g.st[kSt7] = s7;
  NozzleGeometry geometry;
  geometry.throatArea = op.throatArea;
  geometry.exitArea = op.exitArea;
  geometry.dischargeCoefficient = d.nozzleDischarge;
  geometry.velocityCoefficient = d.nozzleVelocityCoefficient;
  geometry.separationPressureRatio = d.nozzleSeparationRatio;
  if (!NozzleRequiredPressure(gas, s7, geometry, op.p0, g.nozzle)) return false;
  residual[0] = (s7.Pt - g.nozzle.requiredPt) / pScale;
  residual[2] = (W13raw - W16raw - rollPost) / bypassScale;

  // Lift fan: independent air path; scalar flow solve against its nozzle.
  g.liftFlow = 0.0;
  g.lf = CompressorResult();
  g.lfNozzle = NozzleResult();
  if (config_.lift.present) {
    const LiftSystemConfig& lift = config_.lift;
    Stream in = AmbientStream(gas, op.Pt0, op.Tt0, 0.0);
    g.st[kStLiftFanIn] = in;
    NozzleGeometry ng = lift.nozzle;
    ng.throatArea *= op.door; ng.exitArea *= op.door;
    const double inletLoss = 1.0 - lift.inletRecovery;
    const auto excess = [&](double W, CompressorResult& cr, NozzleResult& nr) -> double {
      const double q = W / lift.designFlow;
      in.W = W;
      in.Pt = op.Pt0 * (1.0 - inletLoss * q * q);
      cr = Compress(gas, design_.liftFan, in, op.wLF);
      if (!cr.valid) return NAN;
      if (!NozzleRequiredPressure(gas, cr.out, ng, op.p0, nr)) return NAN;
      return (cr.out.Pt - nr.requiredPt)
        / std::max(1e-3 * op.p0, 0.5 * (std::fabs(cr.out.Pt - op.p0) + std::fabs(nr.requiredPt - op.p0)));
    };
    CompressorResult cr; NozzleResult nr;
    if (ng.throatArea > 0.0 && op.wLF > 0.0) {
      const double f0 = excess(0.0, cr, nr);
      if (!Finite(f0)) return false;
      if (f0 > 0.0) {
        double lo = 0.0, flo = f0;
        double hi = std::max(op.liftFlowGuess * 1.5, 0.05 * lift.designFlow), fhi = excess(hi, cr, nr);
        for (int i = 0; i < 40 && Finite(fhi) && fhi > 0.0; ++i) { lo = hi; flo = fhi; hi *= 2.0; fhi = excess(hi, cr, nr); }
        if (!Finite(fhi)) return false;
        // Illinois false position.
        int side = 0;
        double w = lo;
        for (int i = 0; i < 100; ++i) {
          w = (lo * fhi - hi * flo) / (fhi - flo);
          const double fw = excess(w, cr, nr);
          if (!Finite(fw)) return false;
          if (std::fabs(fw) < 1e-13 || (hi - lo) < 1e-13 * std::max(1.0, hi)) break;
          if ((fw > 0.0) == (flo > 0.0)) { lo = w; flo = fw; if (side == -1) fhi *= 0.5; side = -1; }
          else { hi = w; fhi = fw; if (side == 1) flo *= 0.5; side = 1; }
        }
        g.liftFlow = w;
      }
    }
    in.W = g.liftFlow;
    in.Pt = op.Pt0 * (1.0 - inletLoss * (g.liftFlow / lift.designFlow) * (g.liftFlow / lift.designFlow));
    g.st[kStLiftFanIn] = in;
    g.lf = Compress(gas, design_.liftFan, in, op.wLF);
    if (!g.lf.valid) return false;
    g.st[kStLiftFanOut] = g.lf.out;
    NozzleRequiredPressure(gas, g.lf.out, ng, op.p0, g.lfNozzle);
  }

  // Shaft torques at the evaluation speeds.
  const LiftSystemConfig& lift = config_.lift;
  g.lossLP = LossTorque(config_.lp, op.wLP);
  g.lossHP = LossTorque(config_.hp, op.wHP);
  g.lossLF = lift.present ? LossTorque(lift.fanShaft, op.wLF) : 0.0;
  g.accessory = d.accessoryPower / config_.hp.designSpeed * (op.wHP / config_.hp.designSpeed);
  const double slip = lift.gearRatio * op.wLP - op.wLF;
  g.clutchTorque = lift.present
    ? op.clutch * lift.clutchCapacity * std::tanh(slip / std::max(1e-9, lift.clutchSlipSpeed)) : 0.0;
  g.lpNet = g.lpt.torque - g.fan.torque - g.lossLP - lift.gearRatio * g.clutchTorque;
  g.hpNet = g.hpt.torque - g.hpc.torque - g.lossHP - g.accessory + op.starterTorque;
  g.lfNet = g.clutchTorque - g.lf.torque - g.lossLF;
  g.valid = Finite(residual[0]) && Finite(residual[1]) && Finite(residual[2])
    && Finite(residual[3]) && Finite(residual[4])
    && Finite(g.lpNet) && Finite(g.hpNet) && Finite(g.lfNet);
  return g.valid;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Damped Newton with bound projection and optional Jacobian reuse (Broyden).

namespace {
struct NewtonResult { int status = 0; double norm = 0.0; };

template <class F>
NewtonResult Newton(F&& f, int n, double* x, const double* lower, double tol, int cap,
                    double* J, bool& jValid, bool broyden, int& iterations, int& evaluations)
{
  NewtonResult result;
  double r[kMaxUnknowns], rn[kMaxUnknowns], dx[kMaxUnknowns], xn[kMaxUnknowns];
  double A[kMaxUnknowns * kMaxUnknowns], rp[kMaxUnknowns], xp[kMaxUnknowns];
  ++evaluations;
  if (!f(x, r)) { result.status = 2; return result; }
  double norm = MaxAbs(r, n);
  bool fresh = false;
  int used = 0;
  while (norm > tol) {
    if (used >= cap) { result.status = 1; result.norm = norm; return result; }
    if (!jValid) {
      for (int j = 0; j < n; ++j) {
        std::memcpy(xp, x, sizeof(double) * n);
        double h = 1e-6 * std::max(std::fabs(x[j]), 1e-2);
        xp[j] += h;
        ++evaluations;
        if (!f(xp, rp)) {
          xp[j] = x[j] - h; h = -h;
          ++evaluations;
          if (!f(xp, rp)) { result.status = 2; result.norm = norm; return result; }
        }
        for (int i = 0; i < n; ++i) J[i * n + j] = (rp[i] - r[i]) / h;
      }
      jValid = true;
      fresh = true;
    }
    std::memcpy(A, J, sizeof(double) * n * n);
    for (int i = 0; i < n; ++i) dx[i] = -r[i];
    if (!SolveLinear(n, A, dx)) {
      if (!fresh) { jValid = false; continue; }
      // A momentarily singular fresh Jacobian (for example an undetermined
      // flow at rest): take a Levenberg step on the normal equations.
      double scale = 0.0;
      for (int i = 0; i < n * n; ++i) scale = std::max(scale, std::fabs(J[i]));
      for (int i = 0; i < n; ++i) {
        dx[i] = 0.0;
        for (int k = 0; k < n; ++k) dx[i] -= J[k * n + i] * r[k];
        for (int j = 0; j < n; ++j) {
          double v = 0.0;
          for (int k = 0; k < n; ++k) v += J[k * n + i] * J[k * n + j];
          A[i * n + j] = v + (i == j ? 1e-12 * scale * scale + 1e-300 : 0.0);
        }
      }
      if (!SolveLinear(n, A, dx)) { result.status = 3; result.norm = norm; return result; }
    }
    ++used; ++iterations;
    // Armijo on the 2-norm merit: a Newton direction always descends it,
    // whereas the max norm used for convergence need not decrease.
    double alpha = 1.0, nn = 0.0;
    bool accepted = false;
    const double merit = SumSquares(r, n);
    for (int ls = 0; ls < 10; ++ls) {
      for (int i = 0; i < n; ++i) xn[i] = std::max(lower[i], x[i] + alpha * dx[i]);
      ++evaluations;
      if (f(xn, rn)) {
        nn = MaxAbs(rn, n);
        if (nn <= tol || SumSquares(rn, n) < (1.0 - 2e-4 * alpha) * merit) { accepted = true; break; }
      }
      alpha *= 0.5;
    }
    if (!accepted) {
      if (!fresh) { jValid = false; continue; }
      result.status = 1; result.norm = norm; return result;
    }
    if (broyden) {
      double s[kMaxUnknowns], ss = 0.0;
      for (int i = 0; i < n; ++i) { s[i] = xn[i] - x[i]; ss += s[i] * s[i]; }
      if (ss > 0.0) {
        for (int i = 0; i < n; ++i) {
          double Js = 0.0;
          for (int j = 0; j < n; ++j) Js += J[i * n + j] * s[j];
          const double c = (rn[i] - r[i] - Js) / ss;
          for (int j = 0; j < n; ++j) J[i * n + j] += c * s[j];
        }
      }
      fresh = false;
    } else {
      jValid = false;
    }
    std::memcpy(x, xn, sizeof(double) * n);
    std::memcpy(r, rn, sizeof(double) * n);
    norm = nn;
  }
  result.norm = norm;
  return result;
}

/** Solves with the mixer junction regime as an active set: a converged open
    solution with reverse bypass flow is re-solved blocked, and a blocked one
    whose bypass would flow is re-solved open. */
template <class F, class Op, class G>
NewtonResult RegimeNewton(F&& f, int n, double* x, const double* lower, double tol, int cap,
                          double* J, bool& jValid, bool broyden, int& iterations, int& evaluations,
                          Op& op, G& g)
{
  NewtonResult result;
  for (int attempt = 0; attempt < 3; ++attempt) {
    const int before = iterations;
    result = Newton(f, n, x, lower, tol, cap, J, jValid, broyden, iterations, evaluations);
    cap -= iterations - before;
    if (result.status != 0) return result;
    double r[kMaxUnknowns];
    ++evaluations;
    if (!f(x, r)) { result.status = 2; return result; }
    const bool switchToBlocked = !op.bypassBlocked && g.bypassRaw < 0.0;
    const bool switchToOpen = op.bypassBlocked && g.bypassDrive > 0.0;
    if (!switchToBlocked && !switchToOpen) return result;
    op.bypassBlocked = switchToBlocked;
    if (switchToBlocked) x[2] = 0.0;
    jValid = false;
    if (cap < 1) { result.status = 1; return result; }
  }
  return result;
}
} // namespace

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

namespace {
void FillOperatingEnvironment(const PlantInputs& in, double& p0, double& Pt0, double& Tt0,
                              double& V0, double& T0, double& mach)
{
  p0 = in.ambientPressure; Pt0 = in.totalPressure; Tt0 = in.totalTemperature;
  V0 = in.airspeed; T0 = in.ambientTemperature; mach = in.mach;
}
} // namespace

void Plant::Fill(const GasPath& g, const Operating& op, const PlantState& s, PlantOutputs& out) const
{
  const GasProperties& gas = *op.gas;
  const double far = gas.StoichiometricFAR();
  for (int i = 0; i < kStationCount; ++i) {
    const Stream& st = g.st[i];
    out.station[i].W = st.W; out.station[i].Tt = st.Tt; out.station[i].Pt = st.Pt;
    out.station[i].fuelAir = st.c.FuelAirRatioEquivalent(far);
    out.station[i].unburnedFraction = st.c.y[kFuel];
  }
  // Nozzle stations 8/9 carry the exit state; Tt and composition are station 7's.
  const DesignSpec& d = config_.design;
  NozzleGeometry geometry;
  geometry.throatArea = op.throatArea; geometry.exitArea = op.exitArea;
  geometry.dischargeCoefficient = d.nozzleDischarge;
  geometry.velocityCoefficient = d.nozzleVelocityCoefficient;
  geometry.separationPressureRatio = d.nozzleSeparationRatio;
  NozzleResult nr;
  const bool nozzleOk = NozzleThrust(gas, g.st[kSt7], geometry, op.p0, nr);
  out.nozzle.massFlow = g.st[kSt7].W;
  out.nozzle.grossThrust = nozzleOk ? nr.grossThrust : NAN;
  out.nozzle.exitVelocity = nr.exitVelocity * d.nozzleVelocityCoefficient;
  out.nozzle.exitMach = nr.exitMach;
  out.nozzle.exitStaticPressure = nr.exitStaticPressure;
  out.nozzle.exitStaticTemperature = nr.exitStaticTemperature;
  out.nozzle.exitArea = nr.exitArea;
  out.nozzle.regime = static_cast<int>(nr.regime);
  out.nozzle.choked = nr.choked;
  out.station[kSt8] = out.station[kSt7];
  out.station[kSt9] = out.station[kSt7];
  out.station[kSt9].Pt = nr.requiredPt;
  // Roll posts share the bypass mass not delivered to the mixer.
  const double rpTotal = g.rpFlow[0] + g.rpFlow[1];
  const double rpMass = std::max(0.0, g.st[kSt13].W - std::max(0.0, g.st[kSt16].W));
  for (int i = 0; i < 2; ++i) {
    OutletObservation& o = out.rollPost[i];
    o = OutletObservation();
    if (rpTotal > 0.0 && op.rollPostArea[i] > 0.0) {
      Stream post = g.st[kSt13];
      post.W = rpMass * g.rpFlow[i] / rpTotal;
      NozzleGeometry rg;
      rg.throatArea = rg.exitArea = op.rollPostArea[i];
      rg.dischargeCoefficient = config_.lift.rollPostDischarge;
      rg.velocityCoefficient = config_.lift.rollPostVelocityCoefficient;
      NozzleResult rr;
      if (NozzleThrust(gas, post, rg, op.p0, rr)) {
        o.massFlow = post.W; o.grossThrust = rr.grossThrust;
        o.exitVelocity = rr.exitVelocity * rg.velocityCoefficient; o.exitMach = rr.exitMach;
        o.exitStaticPressure = rr.exitStaticPressure; o.exitStaticTemperature = rr.exitStaticTemperature;
        o.exitArea = rr.exitArea; o.regime = static_cast<int>(rr.regime); o.choked = rr.choked;
      }
    }
  }
  out.liftFan = OutletObservation();
  if (config_.lift.present && g.liftFlow > 0.0) {
    NozzleGeometry ng = config_.lift.nozzle;
    ng.throatArea *= op.door; ng.exitArea *= op.door;
    NozzleResult lr;
    if (NozzleThrust(gas, g.st[kStLiftFanOut], ng, op.p0, lr)) {
      out.liftFan.massFlow = g.liftFlow; out.liftFan.grossThrust = lr.grossThrust;
      out.liftFan.exitVelocity = lr.exitVelocity * ng.velocityCoefficient;
      out.liftFan.exitMach = lr.exitMach; out.liftFan.exitStaticPressure = lr.exitStaticPressure;
      out.liftFan.exitStaticTemperature = lr.exitStaticTemperature; out.liftFan.exitArea = lr.exitArea;
      out.liftFan.regime = static_cast<int>(lr.regime); out.liftFan.choked = lr.choked;
    }
  }
  out.throatArea = op.throatArea;
  out.exitArea = op.exitArea;
  const ControllerConfig& c = config_.control;
  out.nozzlePosition = (op.throatArea - design_.throatMin) / (design_.throatMax - design_.throatMin);
  (void)c;
  out.capturedAirKgSec = g.st[kSt2].W + g.liftFlow;
  out.ramDragN = out.capturedAirKgSec * op.V0;
  out.netThrustN = out.nozzle.grossThrust + out.liftFan.grossThrust + out.rollPost[0].grossThrust
    + out.rollPost[1].grossThrust - out.ramDragN;
  out.n1Percent = 100.0 * s.omegaLP / config_.lp.designSpeed;
  out.n2Percent = 100.0 * s.omegaHP / config_.hp.designSpeed;
  out.liftFanPercent = config_.lift.present
    ? 100.0 * s.omegaLF / config_.lift.fanShaft.designSpeed : 0.0;
  out.lpTurbineTorque = g.lpt.torque; out.fanTorque = g.fan.torque;
  out.hpTurbineTorque = g.hpt.torque; out.compressorTorque = g.hpc.torque;
  out.clutchTorque = g.clutchTorque; out.liftFanTorque = g.lf.torque;
  out.liftFanPowerW = g.lf.torque * op.wLF;
  out.starterTorque = op.starterTorque;
  out.coreSprayKgSec = op.coreSpray;
  out.abSprayKgSec = op.abFuelAir > 0.0 ? op.abFuelAir * g.st[kSt6].W : op.abSpray;
  out.coreBurnedKgSec = g.core.burnedKgSec;
  out.abBurnedKgSec = g.ab.burnedKgSec;
  out.coreHeatReleaseW = g.core.heatReleaseW;
  out.abHeatReleaseW = g.ab.heatReleaseW;
  out.coreEfficiency = g.coreEfficiency;
  out.coreMeteredKgSec = s.core.metered;
  out.abMeteredKgSec = s.ab.metered;
  out.totalMeteredKgSec = s.core.metered + s.ab.metered;
  out.unburnedExhaustKgSec = g.ab.unburnedOutKgSec;
  const double burnerAir = g.st[kSt31].W > 0.0 || g.st[kSt3].W > 0.0
    ? g.st[kSt3].W - g.st[kSt31].W : 0.0;
  out.coreEquivalence = burnerAir > 0.0 ? op.coreSpray / burnerAir / far : 0.0;
  const double air6 = g.st[kSt6].W * g.st[kSt6].c.y[kAir];
  out.abEquivalence = air6 > 0.0
    ? (out.abSprayKgSec + g.st[kSt6].W * g.st[kSt6].c.y[kFuel]) / air6 / far : 0.0;
  out.coreLit = s.coreFlame.lit; out.abLit = s.abFlame.lit;
  out.abSelected = s.control.abSelected; out.igniter = s.control.igniter;
  out.egtGaugeK = s.control.egtGaugeK;
  for (std::size_t i = 0; i < config_.solids.size(); ++i) {
    out.solidGasHeatW[i] = g.qGas[i];
    out.solidCoolantHeatW[i] = g.qCoolant[i];
    out.solidRadiationW[i] = g.qRadiation[i];
  }
  out.status.lpOverCapacity = g.lpt.overCapacity;
  out.status.hpOverCapacity = g.hpt.overCapacity;
  out.status.fanStallMargin = g.fan.stallMargin;
  out.status.compressorStallMargin = g.hpc.stallMargin;
  out.status.reverseBypassBlocked = g.reverseBypassBlocked;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Operating point of a state: geometry, sprays and flames at one instant.

void Plant::OperatingFromState(const PlantState& s, const PlantInputs& in, Algorithm algorithm,
                               Operating& op) const
{
  op = Operating();
  op.gas = &Gas(algorithm);
  FillOperatingEnvironment(in, op.p0, op.Pt0, op.Tt0, op.V0, op.T0, op.mach);
  op.wLP = s.omegaLP; op.wHP = s.omegaHP; op.wLF = s.omegaLF;
  op.coreSpray = s.core.spray; op.abSpray = s.ab.spray;
  op.coreLit = s.coreFlame.lit; op.abLit = s.abFlame.lit;
  op.coreFraction = s.coreFlame.fraction; op.abFraction = s.abFlame.fraction;
  op.stalled = in.stalled;
  op.throatArea = s.throatArea;
  op.exitArea = s.throatArea * ExitAreaRatio(s.throatArea);
  for (int i = 0; i < 2; ++i) op.rollPostArea[i] = config_.lift.rollPostArea[i] * s.rollPost[i];
  op.door = s.door; op.clutch = s.clutch;
  op.solidCount = static_cast<int>(s.solids.size());
  for (int i = 0; i < op.solidCount; ++i) op.solidK[i] = s.solids[i].temperature;
  op.liftFlowGuess = s.flows[3];
}

void Plant::ScaleFlowsBySpeed(double* x, double wLP, double wHP) const
{
  // Fan-law similarity: flows proportional to rotor speed. Numerical warm
  // start only; the solve decides the flows.
  const auto ratio = [](double now, double then) {
    return then > 0.0 && now > 0.0 ? std::clamp(now / then, 0.5, 2.0) : 1.0;
  };
  x[0] *= ratio(wHP, flowSpeedHP_);
  const double r = ratio(wLP, flowSpeedLP_);
  x[1] *= r; x[2] *= r;
}

double Plant::ExitAreaRatio(double throatArea) const
{
  const double position = (throatArea - design_.throatMin) / (design_.throatMax - design_.throatMin);
  return std::max(1.0, config_.control.exitAreaRatio(position));
}

bool Plant::SolveAt(Operating& op, const std::array<double, 5>& guess, Algorithm algorithm,
                    GasPath& g, double* J, bool& jValid, double tolerance, int cap,
                    int& iterations, int& evaluations, double& norm) const
{
  double x[kPathUnknowns] = {guess[0], guess[1], guess[2], guess[3], guess[4]};
  const double lower[kPathUnknowns] = {0.0, -1e3, -1e3, 0.0, 0.0};
  SeedFlows(x, op.wLP / config_.lp.designSpeed, op.wHP / config_.hp.designSpeed, op.Pt0 - op.p0);
  const auto f = [&](const double* y, double* r) { return EvaluatePath(op, y, g, r); };
  double r0[kPathUnknowns];
  if (!FeasibleStart(f, x, r0, evaluations)) return false;
  const NewtonResult result = RegimeNewton(f, kPathUnknowns, x, lower, tolerance, cap, J, jValid,
                                           algorithm == Algorithm::Reduced, iterations, evaluations,
                                           op, g);
  norm = result.norm;
  if (result.status != 0) return false;
  // The last evaluation is the accepted point unless the start converged.
  double r[kPathUnknowns];
  ++evaluations;
  if (!EvaluatePath(op, x, g, r)) return false;
  norm = MaxAbs(r, kPathUnknowns);
  return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// One accepted (sub)step: actuators and fuel paths exactly over dt, then the
// implicit-midpoint shaft update solved jointly with the quasi-steady gas path
// at mid-step speeds. Rotor energy change equals the midpoint shaft power
// times dt exactly, so the whole-installation ledger closes to the solver
// residual. Flames change state at the end of the step on conditions
// evaluated over it; burning starts in the following step.

bool Plant::StepOnce(const PlantInputs& in, const StepCommands& cmd, double dt, Algorithm algorithm,
                     int& iterations, int& evaluations, int cap)
{
  PlantState& s = trial_;
  const GasProperties& gas = Gas(algorithm);
  const Composition fuelComposition{{{0.0, 0.0, 1.0}}};
  const double hFuel = gas.H(fuelComposition, config_.fuelTemperature);
  const bool shut = in.cutoff || in.seized;
  const ControllerConfig& c = config_.control;
  const LiftSystemConfig& lift = config_.lift;

  // Fuel metering valves, then actual supply.
  const auto valve = [&](FuelPathState& f, const FuelPathConfig& fc, double command, double& average) {
    const double target = shut ? 0.0 : std::clamp(command, 0.0, fc.maxFlow);
    f.metered = std::max(0.0, LagExact(f.metered, target, fc.valveTimeConstant, fc.valveRateLimit, dt, average));
    average = std::max(0.0, average);
  };
  double coreIn, abIn;
  valve(s.core, config_.coreFuel, cmd.core, coreIn);
  valve(s.ab, config_.abFuel, cmd.ab, abIn);
  const bool supplyEmpty = !(in.fuelSupplyKg > 0.0);
  bool starved = false;
  if (supplyEmpty) {
    starved = coreIn > 0.0 || abIn > 0.0;
    coreIn = abIn = 0.0; s.core.metered = s.ab.metered = 0.0;
  } else if (!in.fuelFrozen && (coreIn + abIn) * dt > in.fuelSupplyKg) {
    // A final partially supplied step: the core path has priority.
    const double coreMass = std::min(coreIn * dt, in.fuelSupplyKg);
    const double abMass = std::min(abIn * dt, in.fuelSupplyKg - coreMass);
    coreIn = coreMass / dt; abIn = abMass / dt;
    s.core.metered = std::min(s.core.metered, coreIn);
    s.ab.metered = std::min(s.ab.metered, abIn);
    starved = true;
  }
  if (starved && !lastStarved_) RecordEvent(time, 9);
  lastStarved_ = starved;

  // Manifold inventories: fill to the prime mass, then deliver with a first-
  // order primed response; drained mass goes to dump. Exact for constant inflow.
  const auto manifold = [&](FuelPathState& f, const FuelPathConfig& fc, double inflow,
                            double& sprayAverage, double& dump) {
    const double Mp = std::max(0.0, fc.manifoldPrimeMass);
    const double tau = std::max(1e-6, fc.manifoldTimeConstant);
    double m = f.manifold, t = 0.0, spray = 0.0;
    dump = 0.0;
    if (m < Mp) {
      if (inflow > 0.0) {
        const double fill = (Mp - m) / inflow;
        if (fill >= dt) { m += inflow * dt; t = dt; }
        else { m = Mp; t = fill; }
      } else t = dt;
    }
    if (t < dt) {
      const double rest = dt - t, e0 = m - Mp, eInf = inflow * tau;
      const double e1 = eInf + (e0 - eInf) * std::exp(-rest / tau);
      spray = inflow * rest - (e1 - e0);
      m = Mp + e1;
    }
    if (shut && inflow <= 0.0 && fc.drainTimeConstant > 0.0 && m > 0.0) {
      dump = m * (1.0 - std::exp(-dt / fc.drainTimeConstant));
      m -= dump;
    }
    f.manifold = std::max(0.0, m);
    f.spray = std::max(0.0, f.manifold - Mp) / tau;
    sprayAverage = std::max(0.0, spray) / dt;
  };
  const double coreManifold0 = s.core.manifold, abManifold0 = s.ab.manifold;
  double coreSpray, abSpray, coreDump, abDump;
  manifold(s.core, config_.coreFuel, coreIn, coreSpray, coreDump);
  manifold(s.ab, config_.abFuel, abIn, abSpray, abDump);

  // Actuators.
  double throatAverage;
  s.throatArea = LagExact(s.throatArea, cmd.throat, c.throatAreaTimeConstant, design_.throatRate, dt,
                          throatAverage);
  double clutchAverage = 0.0, doorAverage = 0.0;
  std::array<double, 2> postAverage{{0.0, 0.0}};
  if (lift.present) {
    s.clutch = RateLimit(s.clutch, std::clamp(in.clutchCommand, 0.0, 1.0), lift.clutchRate, dt, clutchAverage);
    s.door = RateLimit(s.door, std::clamp(in.doorCommand, 0.0, 1.0), lift.doorRate, dt, doorAverage);
    for (int i = 0; i < 2; ++i)
      s.rollPost[i] = RateLimit(s.rollPost[i], std::clamp(in.rollPostCommand[i], 0.0, 1.0),
                                lift.rollPostRate, dt, postAverage[i]);
  }

  // Seizure stops the core rotor instantly; its energy is dissipated. The
  // collapse of core airflow quenches both flames at that instant; fuel left
  // in the manifolds still leaves, unburned.
  double seizureJ = 0.0;
  if (in.seized && s.omegaHP > 0.0) {
    seizureJ = 0.5 * config_.hp.inertia * s.omegaHP * s.omegaHP;
    s.omegaHP = 0.0;
    const double tStart = time + stepElapsed_;
    if (s.coreFlame.lit) RecordEvent(tStart, 2);
    if (s.abFlame.lit) RecordEvent(tStart, 4);
    s.coreFlame = FlameState();
    s.abFlame = FlameState();
  }

  Operating op;
  op.gas = &gas;
  FillOperatingEnvironment(in, op.p0, op.Pt0, op.Tt0, op.V0, op.T0, op.mach);
  op.coreSpray = coreSpray; op.abSpray = abSpray;
  op.coreLit = s.coreFlame.lit; op.abLit = s.abFlame.lit;
  // Light-around: the lit fraction grows linearly over the step; the step
  // average weights the burning. Fractions reach their end value below.
  const auto spread = [&](FlameState& flame, const CombustionConfig& cc, double& average) {
    if (!flame.lit) { average = 0.0; return; }
    const double rate = cc.lightAroundTime > 0.0 ? 1.0 / cc.lightAroundTime : INFINITY;
    const double f0 = flame.fraction, f1 = std::min(1.0, f0 + rate * dt);
    // Exact average of a linear ramp that may saturate within the step.
    const double tSat = rate > 0.0 && f0 < 1.0 ? (1.0 - f0) / rate : 0.0;
    average = tSat >= dt ? 0.5 * (f0 + f1)
      : (tSat * 0.5 * (f0 + 1.0) + (dt - tSat)) / dt;
    flame.fraction = f1;
  };
  double coreFractionAverage, abFractionAverage;
  spread(s.coreFlame, config_.coreCombustion, coreFractionAverage);
  spread(s.abFlame, config_.abCombustion, abFractionAverage);
  op.coreFraction = coreFractionAverage; op.abFraction = abFractionAverage;
  op.stalled = in.stalled;
  op.throatArea = throatAverage;
  op.exitArea = throatAverage * ExitAreaRatio(throatAverage);
  for (int i = 0; i < 2; ++i) op.rollPostArea[i] = lift.rollPostArea[i] * postAverage[i];
  op.door = doorAverage; op.clutch = clutchAverage;
  op.starterTorque = cmd.starterTorque;
  op.solidCount = static_cast<int>(s.solids.size());
  for (int i = 0; i < op.solidCount; ++i) op.solidK[i] = s.solids[i].temperature;
  op.liftFlowGuess = s.flows[3];

  const double wLPd = config_.lp.designSpeed, wHPd = config_.hp.designSpeed;
  const double wLFd = lift.present ? lift.fanShaft.designSpeed : 1.0;
  const double tLP = std::max(1.0, std::fabs(design_.lpTorque));
  const double tHP = std::max(1.0, std::fabs(design_.hpTorque));
  const double tLF = std::max(1.0, std::fabs(design_.liftFanTorque));
  // Unknowns: the five gas-path unknowns, then end-of-step speeds.
  const int n = kPathUnknowns + (lift.present ? 3 : 2);
  const int iLP = kPathUnknowns, iHP = kPathUnknowns + 1, iLF = kPathUnknowns + 2;
  const double coreScale = design_.coreFlow;
  const double bypassScale = std::max(design_.bypassFlow, 1e-3 * design_.fanFlow);
  // Speed predictor: last accepted acceleration, or the starter from rest.
  const double accelHP = lastAccelHP_ != 0.0 ? lastAccelHP_ : cmd.starterTorque / config_.hp.inertia;
  double y[kMaxUnknowns] = {s.flows[0] / coreScale, s.flows[1] / bypassScale, s.flows[2] / bypassScale,
                            s.flows[4], s.flows[5],
                            (s.omegaLP + dt * lastAccelLP_) / wLPd, (s.omegaHP + dt * accelHP) / wHPd,
                            s.omegaLF / wLFd};
  y[iLP] = std::max(0.0, y[iLP]); y[iHP] = in.seized ? 0.0 : std::max(0.0, y[iHP]);
  if (seizureJ > 0.0) {
    // The stopped compressor blocks the core: start from a small core flow
    // through turbines with little expansion, not from the running solution.
    y[0] *= 0.1; y[3] *= 0.1; y[4] *= 0.1;
  }
  ScaleFlowsBySpeed(y, 0.5 * (s.omegaLP + y[iLP] * wLPd), 0.5 * (s.omegaHP + y[iHP] * wHPd));
  SeedFlows(y, 0.5 * (s.omegaLP / wLPd + y[iLP]), 0.5 * (s.omegaHP / wHPd + y[iHP]),
            in.totalPressure - in.ambientPressure);
  const double lower[kMaxUnknowns] = {0.0, -1e3, -1e3, 0.0, 0.0, 0.0, 0.0, 0.0};
  GasPath& g = *midpointStorage_;
  const auto f = [&](const double* x, double* r) {
    const double wLP = x[iLP] * wLPd, wHP = x[iHP] * wHPd, wLF = lift.present ? x[iLF] * wLFd : 0.0;
    op.wLP = 0.5 * (s.omegaLP + wLP);
    op.wHP = 0.5 * (s.omegaHP + wHP);
    op.wLF = 0.5 * (s.omegaLF + wLF);
    if (!EvaluatePath(op, x, g, r)) return false;
    r[iLP] = (config_.lp.inertia * (wLP - s.omegaLP) / dt - g.lpNet) / tLP;
    r[iHP] = in.seized ? x[iHP] : (config_.hp.inertia * (wHP - s.omegaHP) / dt - g.hpNet) / tHP;
    if (lift.present) r[iLF] = (lift.fanShaft.inertia * (wLF - s.omegaLF) / dt - g.lfNet) / tLF;
    return Finite(r[iLP]) && Finite(r[iHP]) && (!lift.present || Finite(r[iLF]));
  };
  bool jValid = algorithm == Algorithm::Reduced && jacobianValid_ && jacobianSize_ == n;
  op.bypassBlocked = bypassBlocked_;
  double r0[kMaxUnknowns];
  if (!FeasibleStart(f, y, r0, evaluations)) return false;
  const NewtonResult result = RegimeNewton(f, n, y, lower, settings.tolerance, cap, jacobian_.data(),
                                           jValid, algorithm == Algorithm::Reduced, iterations,
                                           evaluations, op, g);
  bypassBlocked_ = op.bypassBlocked;
  jacobianValid_ = jValid && algorithm == Algorithm::Reduced;
  jacobianSize_ = n;
  lastNorm_ = result.norm;
  if (result.status != 0) { jacobianValid_ = false; return false; }
  double r[kMaxUnknowns];
  ++evaluations;
  if (!f(y, r)) return false;
  lastNorm_ = MaxAbs(r, n);

  // Accept: shafts, solids, ledger, flows.
  const double wLP1 = y[iLP] * wLPd, wHP1 = y[iHP] * wHPd, wLF1 = lift.present ? y[iLF] * wLFd : 0.0;
  Ledger& L = stepLedger_;
  const double rotor0 = 0.5 * (config_.lp.inertia * s.omegaLP * s.omegaLP
    + config_.hp.inertia * s.omegaHP * s.omegaHP
    + (lift.present ? lift.fanShaft.inertia * s.omegaLF * s.omegaLF : 0.0));
  const double rotor1 = 0.5 * (config_.lp.inertia * wLP1 * wLP1 + config_.hp.inertia * wHP1 * wHP1
    + (lift.present ? lift.fanShaft.inertia * wLF1 * wLF1 : 0.0));
  lastAccelLP_ = (wLP1 - s.omegaLP) / dt;
  lastAccelHP_ = (wHP1 - s.omegaHP) / dt;
  s.omegaLP = wLP1; s.omegaHP = wHP1; s.omegaLF = wLF1;
  double solidStored = 0.0, radiation = 0.0;
  for (int i = 0; i < op.solidCount; ++i) {
    SolidState& solid = s.solids[i];
    const double q = g.qGas[i] - g.qCoolant[i] - g.qRadiation[i];
    const double T1 = solid.temperature + dt * q / config_.solids[i].capacity;
    if (!(Finite(T1) && T1 > 0.0)) return false;
    solidStored += config_.solids[i].capacity * (T1 - solid.temperature);
    radiation += dt * g.qRadiation[i];
    solid.temperature = T1;
    solid.warmPending = false;
    solid.initialized = true;
  }
  // Roll-post mass is the bypass flow not delivered to the mixer, so the
  // installation mass balance is exact by construction.
  const double rpMass = g.st[kSt13].W - g.st[kSt16].W;
  const double fuelIn = (coreIn + abIn) * dt;
  const double manifoldKg = (s.core.manifold - coreManifold0) + (s.ab.manifold - abManifold0);
  Ledger step;
  step.airInKg = dt * (g.st[kSt2].W + g.liftFlow);
  step.airInJ = dt * (g.st[kSt2].W * g.st[kSt0].ht + g.liftFlow * g.st[kStLiftFanIn].ht);
  step.fuelInKg = fuelIn;
  step.fuelSensibleInJ = fuelIn * hFuel;
  step.chemicalJ = dt * (g.core.heatReleaseW + g.ab.heatReleaseW);
  step.starterJ = dt * cmd.starterTorque * op.wHP;
  step.outKg = dt * (g.st[kSt7].W + rpMass + g.liftFlow);
  step.outflowJ = dt * (g.st[kSt7].W * g.st[kSt7].ht + rpMass * g.st[kSt13].ht
    + g.liftFlow * g.st[kStLiftFanOut].ht);
  step.accessoryJ = dt * g.accessory * op.wHP;
  step.frictionJ = dt * (g.lossLP * op.wLP + g.lossHP * op.wHP + g.lossLF * op.wLF) + seizureJ;
  step.clutchJ = dt * g.clutchTorque * (lift.gearRatio * op.wLP - op.wLF);
  step.radiationJ = radiation;
  step.dumpKg = coreDump + abDump;
  step.dumpJ = step.dumpKg * hFuel;
  step.rotorStoredJ = rotor1 - rotor0 + seizureJ * 0.0;
  // The seizure dissipates stored rotor energy: count it as a storage loss.
  step.rotorStoredJ -= seizureJ;
  step.solidStoredJ = solidStored;
  step.manifoldKg = manifoldKg;
  step.manifoldStoredJ = manifoldKg * hFuel;
  step.burnedKg = dt * (g.core.burnedKgSec + g.ab.burnedKgSec);
  step.unburnedOutKg = dt * g.ab.unburnedOutKgSec;
  const double sources = step.airInJ + step.fuelSensibleInJ + step.chemicalJ + step.starterJ
    - step.outflowJ - step.accessoryJ - step.frictionJ - step.clutchJ - step.radiationJ - step.dumpJ;
  const double stored = step.rotorStoredJ + step.solidStoredJ + step.manifoldStoredJ;
  step.residualJ = stored - sources;
  step.scaleJ = std::fabs(step.airInJ) + std::fabs(step.fuelSensibleInJ) + std::fabs(step.chemicalJ)
    + std::fabs(step.starterJ) + std::fabs(step.outflowJ) + std::fabs(step.accessoryJ)
    + std::fabs(step.frictionJ) + std::fabs(step.clutchJ) + std::fabs(step.radiationJ)
    + std::fabs(step.dumpJ) + std::fabs(step.rotorStoredJ) + std::fabs(step.solidStoredJ)
    + std::fabs(step.manifoldStoredJ);
  step.massResidualKg = step.manifoldKg - (step.airInKg + step.fuelInKg - step.outKg - step.dumpKg);
  const double massScale = step.airInKg + step.fuelInKg + step.outKg + step.dumpKg + std::fabs(step.manifoldKg);
  // Accumulate substeps into the accepted-step ledger.
  L.airInJ += step.airInJ; L.fuelSensibleInJ += step.fuelSensibleInJ; L.chemicalJ += step.chemicalJ;
  L.starterJ += step.starterJ; L.outflowJ += step.outflowJ; L.accessoryJ += step.accessoryJ;
  L.frictionJ += step.frictionJ; L.clutchJ += step.clutchJ; L.radiationJ += step.radiationJ;
  L.dumpJ += step.dumpJ; L.rotorStoredJ += step.rotorStoredJ; L.solidStoredJ += step.solidStoredJ;
  L.manifoldStoredJ += step.manifoldStoredJ; L.residualJ += step.residualJ; L.scaleJ += step.scaleJ;
  L.airInKg += step.airInKg; L.fuelInKg += step.fuelInKg; L.outKg += step.outKg;
  L.dumpKg += step.dumpKg; L.manifoldKg += step.manifoldKg; L.massResidualKg += step.massResidualKg;
  L.massRelative += massScale;   // temporarily the mass scale; normalized in Advance
  L.burnedKg += step.burnedKg; L.unburnedOutKg += step.unburnedOutKg;
  if (!in.fuelFrozen) s.fuelUsedKg += fuelIn;
  s.flows = {y[0] * coreScale, y[1] * bypassScale, y[2] * bypassScale, g.liftFlow, y[3], y[4]};
  flowSpeedLP_ = op.wLP; flowSpeedHP_ = op.wHP;

  // Flame state at the end of the step, from conditions over it.
  const double far = gas.StoichiometricFAR();
  const CombustionConfig& cc = config_.coreCombustion;
  const double burnerAir = g.st[kSt3].W - g.st[kSt31].W;
  const double phiCore = burnerAir > 0.0 ? coreSpray / burnerAir / far : INFINITY;
  const double P3 = g.st[kSt3].Pt;
  const double tEnd = time + stepElapsed_ + dt;
  if (!s.coreFlame.lit) {
    const bool ignitable = s.control.igniter && coreSpray > 0.0 && phiCore >= cc.leanIgnitionEquivalence
      && phiCore <= cc.richLimitEquivalence && P3 >= cc.minimumPressure;
    s.coreFlame.progress = ignitable ? s.coreFlame.progress + dt : 0.0;
    if (ignitable && s.coreFlame.progress >= cc.ignitionDelay) {
      s.coreFlame.lit = true; s.coreFlame.progress = 0.0; s.coreFlame.fraction = 0.0;
      RecordEvent(tEnd, 1);
    }
  } else if (!(coreSpray > 0.0) || phiCore < cc.leanBlowoutEquivalence
             || phiCore > cc.richLimitEquivalence || P3 < cc.minimumPressure) {
    s.coreFlame = FlameState(); RecordEvent(tEnd, 2);
  }
  const CombustionConfig& ac = config_.abCombustion;
  const double air6 = g.st[kSt6].W * g.st[kSt6].c.y[kAir];
  const double unburned6 = g.st[kSt6].W * g.st[kSt6].c.y[kFuel];
  const double phiAB = air6 > 0.0 ? (abSpray + unburned6) / air6 / far : INFINITY;
  const double T6 = g.st[kSt6].Tt;
  if (!s.abFlame.lit) {
    const bool ignitable = s.control.abSelected && abSpray > 0.0 && phiAB >= ac.leanIgnitionEquivalence
      && phiAB <= ac.richLimitEquivalence && T6 >= ac.minimumInletTemperature
      && g.st[kSt6].Pt >= ac.minimumPressure;
    s.abFlame.progress = ignitable ? s.abFlame.progress + dt : 0.0;
    if (ignitable && s.abFlame.progress >= ac.ignitionDelay) {
      s.abFlame.lit = true; s.abFlame.progress = 0.0; s.abFlame.fraction = 0.0;
      RecordEvent(tEnd, 3);
    }
  } else if (!(abSpray > 0.0) || phiAB < ac.leanBlowoutEquivalence || phiAB > ac.richLimitEquivalence
             || g.st[kSt6].Pt < ac.minimumPressure) {
    s.abFlame = FlameState(); RecordEvent(tEnd, 4);
  }
  const double theta = std::sqrt(std::max(1e-6, in.totalTemperature / config_.design.ambientTemperature));
  const double N2c = s.omegaHP / wHPd / theta;
  const bool wasRunning = s.running;
  if (!s.coreFlame.lit) s.running = false;
  else if (N2c >= c.idleCorrectedN2 - c.runningN2Margin) s.running = true;
  if (s.running != wasRunning) RecordEvent(tEnd, s.running ? 5 : 6);
  stepElapsed_ += dt;
  return true;
}

bool Plant::PublishEnd(const PlantInputs& in, Algorithm algorithm, bool updateSensors,
                       int& iterations, int& evaluations, int cap)
{
  Operating op;
  OperatingFromState(state_, in, algorithm, op);
  op.starterTorque = lastStarterTorque_;
  op.bypassBlocked = bypassBlocked_;
  GasPath& g = *endStorage_;
  double norm = 0.0;
  bool jValid = algorithm == Algorithm::Reduced && endJacobianValid_;
  const double coreScale = design_.coreFlow;
  const double bypassScale = std::max(design_.bypassFlow, 1e-3 * design_.fanFlow);
  std::array<double, 5> guess{{state_.flows[0] / coreScale, state_.flows[1] / bypassScale,
                               state_.flows[2] / bypassScale, state_.flows[4], state_.flows[5]}};
  ScaleFlowsBySpeed(guess.data(), state_.omegaLP, state_.omegaHP);
  if (!SolveAt(op, guess, algorithm, g, endJacobian_.data(), jValid, settings.tolerance, cap,
               iterations, evaluations, norm)) {
    endJacobianValid_ = false;
    return false;
  }
  endJacobianValid_ = jValid && algorithm == Algorithm::Reduced;
  if (updateSensors) bypassBlocked_ = op.bypassBlocked;
  Fill(g, op, state_, out_);
  out_.status.maxResidual = std::max(lastNorm_, norm);
  if (updateSensors) {
    ControllerState& cs = state_.control;
    cs.sensedP3 = g.st[kSt3].Pt;
    cs.sensedT5 = g.st[kSt5].Tt;
    cs.sensedT6 = g.st[kSt6].Tt;
    cs.sensedT7 = g.st[kSt7].Tt;
    cs.sensedW3b = g.st[kSt3].W - g.st[kSt31].W;
    cs.sensedAir6 = g.st[kSt6].W * g.st[kSt6].c.y[kAir];
    cs.sensedW6 = g.st[kSt6].W;
    if (in.dt > 0.0) {
      const double tau = config_.control.egtSensorTimeConstant;
      const double a = tau > 0.0 ? 1.0 - std::exp(-in.dt / tau) : 1.0;
      cs.egtGaugeK += (g.st[kSt5].Tt - cs.egtGaugeK) * a;
    }
    state_.flows = {g.st[kSt25].W, g.st[kSt13].W, std::max(0.0, g.st[kSt16].W), g.liftFlow,
                    std::log(g.hpt.pressureRatio), std::log(g.lpt.pressureRatio)};
    flowSpeedLP_ = state_.omegaLP; flowSpeedHP_ = state_.omegaHP;
    out_.egtGaugeK = cs.egtGaugeK;
  }
  return true;
}

bool Plant::Advance(const PlantInputs& in)
{
  if (!in.advance || !(in.dt > 0.0)) {
    const bool eligible = state_.coreFlame.lit && state_.running && !in.cutoff && !in.seized
      && !in.stalled && in.fuelSupplyKg > 0.0;
    return eligible ? Steady(in) : Refresh(in);
  }
  if (out_.status.failure) return false;
  int iterations = 0, evaluations = 0;
  trial_ = state_;
  StepCommands cmd;
  const int eventsAtStart = eventCount_;   // a failed step records no controller event
  UpdateController(in, trial_, in.dt, cmd.core, cmd.ab, cmd.starterTorque, cmd.throat);
  lastStarterTorque_ = cmd.starterTorque;
  out_.coreCommandKgSec = cmd.core;
  out_.abCommandKgSec = cmd.ab;
  retry_ = trial_;
  const int eventsBefore = eventCount_;
  const bool lastStarvedBefore = lastStarved_;
  Algorithm first = settings.requested;
  if (first == Algorithm::Reduced
      && static_cast<double>(tabulatedGas_.ResidentBytes()) > settings.closureBudgetBytes) {
    first = Algorithm::Component;
    out_.status.fallbackReason = 4;
  }
  const Algorithm order[2] = {first, Algorithm::Component};
  const int attempts = first == Algorithm::Reduced ? 2 : 1;
  // Hard work bound: every Newton iteration of the step, fallbacks and the
  // end-state publication included, counts against the cap. A quarter is
  // reserved for publication; the first algorithm gets half of the rest when
  // a fallback exists.
  const int cap = settings.iterationCap;
  const int publishReserve = std::max(1, cap / 4);
  const int stepBudget = std::max(1, cap - publishReserve);
  const int reserve = attempts == 2 ? stepBudget / 2 : 0;
  bool done = false;
  Algorithm used = first;
  int substeps = 0, fallback = 0;
  for (int a = 0; a < attempts && !done; ++a) {
    const int budgetEnd = a == 0 ? stepBudget - reserve : stepBudget;
    for (int k = 1; k <= settings.subdivisionCap && !done && iterations < budgetEnd; k *= 2) {
      trial_ = retry_;
      eventCount_ = eventsBefore;
      lastStarved_ = lastStarvedBefore;
      stepLedger_ = Ledger();
      stepElapsed_ = 0.0;
      bool ok = true;
      for (int j = 0; j < k && ok; ++j) {
        const int left = budgetEnd - iterations;
        ok = left >= 1 && StepOnce(in, cmd, in.dt / k, order[a], iterations, evaluations, left);
      }
      if (ok) { done = true; used = order[a]; substeps = k; }
    }
    if (!done) fallback = iterations >= budgetEnd ? 1 : 2;
  }
  const auto fail = [&](int reason) {
    eventCount_ = eventsAtStart;
    lastStarved_ = lastStarvedBefore;
    out_.status.failure = true;
    out_.status.fallbackReason = reason;
    out_.valid = false;
    RecordEvent(time, 10);
    return false;
  };
  if (!done) return fail(fallback);
  previous_ = state_;
  state_ = trial_;
  int endIterations = 0;
  const bool published = PublishEnd(in, used, true, endIterations, evaluations, cap - iterations)
    || (used == Algorithm::Reduced && cap - iterations - endIterations >= 1
        && PublishEnd(in, Algorithm::Component, true, endIterations, evaluations,
                      cap - iterations - endIterations));
  if (!published) {
    state_ = previous_;    // the last accepted state is kept
    return fail(1);
  }
  time += in.dt;
  sequence_ += 1.0;
  iterations += endIterations;
  Ledger& L = stepLedger_;
  const double massScale = L.massRelative;
  L.relative = std::fabs(L.residualJ) / std::max(1.0, L.scaleJ);
  L.massRelative = std::fabs(L.massResidualKg) / std::max(1e-9, massScale);
  L.cumulativeResidualJ = out_.ledger.cumulativeResidualJ + L.residualJ;
  L.cumulativeScaleJ = out_.ledger.cumulativeScaleJ + L.scaleJ;
  L.cumulativeMassResidualKg = out_.ledger.cumulativeMassResidualKg + L.massResidualKg;
  L.cumulativeMassScaleKg = out_.ledger.cumulativeMassScaleKg + massScale;
  L.valid = true;
  out_.ledger = L;
  NumericalStatus& st = out_.status;
  st.algorithm = static_cast<int>(used);
  st.iterations = iterations;
  st.residualEvaluations = evaluations;
  st.substeps = substeps;
  st.converged = true;
  st.fallbackReason = used == first ? (st.fallbackReason == 4 ? 4 : 0) : fallback;
  st.domainClamps = referenceGas_.domainClamps + tabulatedGas_.domainClamps;
  st.residentBytes = static_cast<double>(used == Algorithm::Reduced ? tabulatedGas_.ResidentBytes() : 0)
    + sizeof(jacobian_) + sizeof(endJacobian_);
  st.sequence = sequence_;
  st.epoch = epoch_;
  st.time = time;
  st.stepSeconds = in.dt;
  out_.valid = true;
  return true;
}

bool Plant::Refresh(const PlantInputs& in)
{
  int iterations = 0, evaluations = 0;
  const Algorithm algorithm = settings.requested;
  if (!PublishEnd(in, algorithm, false, iterations, evaluations, settings.iterationCap)
      && !PublishEnd(in, Algorithm::Component, false, iterations, evaluations, 32)) {
    out_.valid = false;
    return false;
  }
  out_.ledger = Ledger();
  out_.ledger.cumulativeResidualJ = 0.0;
  out_.status.iterations = iterations;
  out_.status.residualEvaluations = evaluations;
  out_.status.stepSeconds = 0.0;
  out_.status.time = time;
  out_.status.epoch = epoch_;
  out_.valid = true;
  return true;
}

bool Plant::Evaluate(const PlantState& state, const PlantInputs& in, PlantOutputs& out) const
{
  Operating op;
  OperatingFromState(state, in, settings.requested, op);
  op.bypassBlocked = bypassBlocked_;
  GasPath g;
  std::array<double, 100> J{};
  bool jValid = false;
  int iterations = 0, evaluations = 0;
  double norm;
  const double coreScale = design_.coreFlow;
  const double bypassScale = std::max(design_.bypassFlow, 1e-3 * design_.fanFlow);
  std::array<double, 5> guess{{state.flows[0] / coreScale, state.flows[1] / bypassScale,
                               state.flows[2] / bypassScale, state.flows[4], state.flows[5]}};
  if (!SolveAt(op, guess, settings.requested, g, J.data(), jValid, settings.tolerance, 32,
               iterations, evaluations, norm)) return false;
  out = out_;
  Fill(g, op, state, out);
  out.status.maxResidual = norm;
  out.valid = true;
  return true;
}

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Steady operating point: shaft torque balance, governor and nozzle schedule
// solved with the gas path. Slow histories (solids, cumulative fuel) are held
// unless they are pending warm initialization.

bool Plant::Steady(const PlantInputs& in) { return SolveSteady(in, false); }

bool Plant::InitializeRunning(const PlantInputs& in)
{
  const PlantState saved = state_;
  state_.coreFlame.lit = true;
  state_.coreFlame.progress = 0.0;
  state_.coreFlame.fraction = 1.0;
  state_.running = true;
  for (auto& solid : state_.solids) solid.warmPending = !solid.initialized;
  if (!SolveSteady(in, true)) { state_ = saved; return false; }
  return true;
}

bool Plant::SolveSteady(const PlantInputs& in, bool thermalEquilibrium)
{
  const ControllerConfig& c = config_.control;
  const LiftSystemConfig& lift = config_.lift;
  const Algorithm algorithm = Algorithm::Component;
  const GasProperties& gas = Gas(algorithm);
  const double theta = std::sqrt(std::max(1e-6, in.totalTemperature / config_.design.ambientTemperature));
  const double wLPd = config_.lp.designSpeed, wHPd = config_.hp.designSpeed;
  const double wLFd = lift.present ? lift.fanShaft.designSpeed : 1.0;
  const double coreScale = design_.coreFlow;
  const double bypassScale = std::max(design_.bypassFlow, 1e-3 * design_.fanFlow);
  const double tLP = std::max(1.0, std::fabs(design_.lpTorque));
  const double tHP = std::max(1.0, std::fabs(design_.hpTorque));
  const double tLF = std::max(1.0, std::fabs(design_.liftFanTorque));
  const double A8d = design_.throatArea;
  const double demand = c.fanSpeedDemand(in.throttle);
  double target = demand;     // N1 governor target; stepped by continuation

  PlantState s = state_;
  const double clutch = lift.present ? std::clamp(in.clutchCommand, 0.0, 1.0) : 0.0;
  const double door = lift.present ? std::clamp(in.doorCommand, 0.0, 1.0) : 0.0;
  const std::array<double, 2> posts{{lift.present ? std::clamp(in.rollPostCommand[0], 0.0, 1.0) : 0.0,
                                     lift.present ? std::clamp(in.rollPostCommand[1], 0.0, 1.0) : 0.0}};
  const bool solveLF = lift.present && clutch > 0.0;
  // Unknowns: the five gas-path unknowns, N1, N2, core fuel, throat area and
  // (with an engaged lift fan) its speed.
  constexpr int kN1 = kPathUnknowns, kN2 = kN1 + 1, kWf = kN1 + 2, kA8 = kN1 + 3, kLF = kN1 + 4;
  const int n = solveLF ? kLF + 1 : kLF;
  Operating op;
  op.gas = &gas;
  FillOperatingEnvironment(in, op.p0, op.Pt0, op.Tt0, op.V0, op.T0, op.mach);
  op.bypassBlocked = false;
  op.coreLit = true;
  op.stalled = false;
  op.door = door; op.clutch = clutch;
  for (int i = 0; i < 2; ++i) op.rollPostArea[i] = lift.rollPostArea[i] * posts[i];
  op.solidCount = static_cast<int>(s.solids.size());
  GasPath& g = *midpointStorage_;
  int mode = 0;               // 0 N1, 3 N2 max, 4 T5 max, 5 idle
  bool abActive = false;
  double y[kMaxUnknowns];
  const bool running = s.running && s.coreFlame.lit && s.omegaHP > 0.0 && s.flows[4] > 0.0;
  const double N1guess = running ? s.omegaLP / wLPd : std::max(0.3, demand * theta);
  const double lnHPd = std::log(design_.stationPt[kSt4] / design_.stationPt[kSt41]);
  const double lnLPd = std::log(design_.stationPt[kSt45] / design_.stationPt[kSt5]);
  y[0] = running ? s.flows[0] / coreScale : std::pow(N1guess, 1.5);
  y[1] = running ? s.flows[1] / bypassScale : std::pow(N1guess, 1.5);
  y[2] = running ? s.flows[2] / bypassScale : y[1];
  y[3] = running ? s.flows[4] : lnHPd * N1guess * N1guess;
  y[4] = running ? s.flows[5] : lnLPd * N1guess * N1guess;
  y[kN1] = N1guess;
  y[kN2] = running ? s.omegaHP / wHPd : std::max(0.6, 0.6 + 0.4 * N1guess);
  y[kWf] = running ? std::max(0.05, s.core.metered / design_.fuelFlow) : std::pow(N1guess, 3.0);
  y[kA8] = running ? s.throatArea / A8d : c.dryThroatArea(N1guess);
  y[kLF] = running && lift.present ? s.omegaLF / wLFd : lift.gearRatio * N1guess * wLPd / wLFd;
  double lower[kMaxUnknowns] = {0, -1e3, -1e3, 0, 0, 0, 0, 0, 0, 0};
  lower[kA8] = c.throatAreaMin;
  const auto residual = [&](const double* x, double* r) {
    op.wLP = x[kN1] * wLPd; op.wHP = x[kN2] * wHPd;
    op.wLF = solveLF ? x[kLF] * wLFd : s.omegaLF;
    op.coreSpray = x[kWf] * design_.fuelFlow;
    op.abLit = abActive;
    op.abFuelAir = abActive ? c.abFuelAirRatio(std::min(1.0, in.augmentDemand)) : 0.0;
    op.abSpray = 0.0;
    op.throatArea = x[kA8] * A8d;
    op.exitArea = op.throatArea * ExitAreaRatio(op.throatArea);
    for (int i = 0; i < op.solidCount; ++i) op.solidK[i] = s.solids[i].temperature;
    if (!EvaluatePath(op, x, g, r)) return false;
    r[kN1] = g.lpNet / tLP;
    r[kN2] = g.hpNet / tHP;
    const double N1c = x[kN1] / theta, N2 = x[kN2], N2c = N2 / theta;
    switch (mode) {
    case 3: r[kWf] = N2 - c.maxN2; break;
    case 4: r[kWf] = (c.maxTurbineExitK - g.st[kSt5].Tt) / c.maxTurbineExitK; break;
    case 5: r[kWf] = N2c - c.idleCorrectedN2; break;
    default: r[kWf] = N1c - target;
    }
    double area = A8d * c.dryThroatArea(N1c);
    if (abActive) {
      area *= 1.0 + c.abPreopenFraction;
      if (g.st[kSt7].Tt > g.st[kSt6].Tt) area *= std::sqrt(g.st[kSt7].Tt / g.st[kSt6].Tt);
    }
    r[kA8] = (x[kA8] * A8d - std::clamp(area, design_.throatMin, design_.throatMax)) / A8d;
    if (solveLF) r[kLF] = g.lfNet / tLF;
    return true;
  };
  std::array<double, 100> J{};
  int iterations = 0, evaluations = 0;
  const auto solve = [&]() {
    bool jValid = false;
    const NewtonResult result = RegimeNewton(residual, n, y, lower, std::min(settings.tolerance, 1e-8),
                                             60, J.data(), jValid, false, iterations, evaluations, op, g);
    return result.status == 0;
  };
  const auto converge = [&]() -> bool {
    mode = 0;
    for (int pass = 0; pass < 4; ++pass) {
      if (!solve()) return false;
      const double N2 = y[kN2], N2c = N2 / theta;
      double r[kMaxUnknowns];
      residual(y, r);
      int next = mode;
      if (N2 > c.maxN2 + 1e-9) next = 3;
      else if (g.st[kSt5].Tt > c.maxTurbineExitK + 1e-6) next = 4;
      else if (mode == 0 && N2c < c.idleCorrectedN2 - 1e-9) next = 5;
      if (next == mode) return true;
      mode = next;
    }
    return false;
  };
  // A large change of demand from a running point (mil to idle, say) leaves
  // the linearization far from the answer: continue in the N1 target from
  // the present corrected speed, halving the stride on failure.
  double start[kMaxUnknowns];
  std::memcpy(start, y, sizeof(start));
  if (!converge()) {
    std::memcpy(y, start, sizeof(start));
    const double from = y[kN1] / theta;
    double lambda = 0.0, stride = 0.25;
    while (lambda < 1.0) {
      const double next = std::min(1.0, lambda + stride);
      target = from + (demand - from) * next;
      double keep[kMaxUnknowns];
      std::memcpy(keep, y, sizeof(keep));
      if (converge()) { lambda = next; stride = std::min(0.5, 2.0 * stride); }
      else {
        std::memcpy(y, keep, sizeof(keep));
        stride *= 0.5;
        if (stride < 1.0 / 64.0) return false;
      }
    }
  }
  if (config_.augmented && in.augmentDemand > 0.0 && !in.cutoff && y[kN2] >= c.abMinimumN2) {
    abActive = true;
    if (!converge()) return false;
  }
  // Thermal policy: pending warm solids (or an explicit request) take their
  // equilibrium at this operating point; others are held.
  const bool anyPending = std::any_of(s.solids.begin(), s.solids.end(),
                                      [](const SolidState& solid) { return solid.warmPending; });
  if (thermalEquilibrium || anyPending) {
    for (int pass = 0; pass < 3; ++pass) {
      double r[kMaxUnknowns];
      if (!residual(y, r)) return false;
      for (int i = 0; i < op.solidCount; ++i) {
        SolidState& solid = s.solids[i];
        if (!(thermalEquilibrium || solid.warmPending)) continue;
        const SolidConfig& sc = config_.solids[i];
        const double hg = g.hGas[i], hc = g.hCoolant[i], Tg = g.gasK[i], Tc = g.coolantK[i];
        const double rad = kSigma * sc.emissivity * sc.radiationArea;
        const auto q = [&](double T) {
          return hg * (Tg - T) - hc * (T - Tc) - rad * (std::pow(T, 4) - std::pow(op.T0, 4));
        };
        double lo = std::min({Tg, Tc, op.T0}), hi = std::max({Tg, Tc, op.T0});
        if (hg + hc + rad <= 0.0) continue;
        for (int k = 0; k < 200 && hi - lo > 1e-10 * hi; ++k) {
          const double mid = 0.5 * (lo + hi);
          if (q(mid) > 0.0) lo = mid; else hi = mid;
        }
        solid.temperature = 0.5 * (lo + hi);
        solid.initialized = true;
      }
      if (!converge()) return false;
    }
  }
  double r[kMaxUnknowns];
  if (!residual(y, r)) return false;
  // Commit the steady operating point.
  s.omegaLP = y[kN1] * wLPd; s.omegaHP = y[kN2] * wHPd;
  s.omegaLF = solveLF ? y[kLF] * wLFd : (lift.present ? s.omegaLF : 0.0);
  s.clutch = clutch; s.door = door; s.rollPost = posts;
  const double wf = y[kWf] * design_.fuelFlow;
  s.core.metered = s.core.spray = wf;
  s.core.manifold = config_.coreFuel.manifoldPrimeMass + wf * std::max(1e-6, config_.coreFuel.manifoldTimeConstant);
  s.coreFlame.lit = true; s.coreFlame.progress = 0.0; s.coreFlame.fraction = 1.0;
  s.running = true;
  s.throatArea = y[kA8] * A8d;
  if (abActive) {
    const double far = c.abFuelAirRatio(std::min(1.0, in.augmentDemand));
    const double wab = far * g.st[kSt6].W;
    s.ab.metered = s.ab.spray = wab;
    s.ab.manifold = config_.abFuel.manifoldPrimeMass + wab * std::max(1e-6, config_.abFuel.manifoldTimeConstant);
    s.abFlame.lit = true; s.abFlame.progress = 0.0; s.abFlame.fraction = 1.0;
    s.control.abSelected = true; s.control.abFuelAir = far;
  } else {
    s.ab.metered = s.ab.spray = 0.0;
    s.abFlame = FlameState();
    s.control.abSelected = false; s.control.abFuelAir = 0.0;
  }
  s.control.integrator = wf / std::max(1e-12, g.st[kSt3].Pt) / design_.ratioUnit;
  s.control.limiter = mode;
  s.control.igniter = false;
  s.control.sensedP3 = g.st[kSt3].Pt;
  s.control.sensedT5 = g.st[kSt5].Tt;
  s.control.sensedT6 = g.st[kSt6].Tt;
  s.control.sensedT7 = g.st[kSt7].Tt;
  s.control.sensedW3b = g.st[kSt3].W - g.st[kSt31].W;
  s.control.sensedAir6 = g.st[kSt6].W * g.st[kSt6].c.y[kAir];
  s.control.sensedW6 = g.st[kSt6].W;
  s.control.egtGaugeK = g.st[kSt5].Tt;
  s.starter = false;
  s.flows = {g.st[kSt25].W, g.st[kSt13].W, std::max(0.0, g.st[kSt16].W), g.liftFlow, y[3], y[4]};
  state_ = s;
  bypassBlocked_ = op.bypassBlocked;
  lastAccelLP_ = lastAccelHP_ = 0.0;
  jacobianValid_ = endJacobianValid_ = false;
  lastStarterTorque_ = 0.0;
  // Publish from the same operating point.
  Operating pub;
  OperatingFromState(state_, in, algorithm, pub);
  pub.bypassBlocked = op.bypassBlocked;
  Fill(g, pub, state_, out_);
  out_.ledger = Ledger();
  out_.status.iterations = iterations;
  out_.status.residualEvaluations = evaluations;
  out_.status.maxResidual = MaxAbs(r, n);
  out_.status.converged = true;
  out_.status.algorithm = static_cast<int>(algorithm);
  out_.status.stepSeconds = 0.0;
  out_.status.time = time;
  out_.status.epoch = epoch_;
  out_.valid = true;
  return true;
}

} // namespace TurbinePlant
} // namespace JSBSim
