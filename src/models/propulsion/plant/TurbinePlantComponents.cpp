/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Module:       TurbinePlantComponents.cpp
 Date started: 2026-10-07
 Purpose:      Component equations of the coupled turbine plant.

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

#include "TurbinePlantComponents.h"

namespace JSBSim {
namespace TurbinePlant {

namespace {
bool Finite(double v) { return std::isfinite(v); }

bool SetEnthalpy(const GasProperties& gas, Stream& s, double ht, double guess) {
  s.ht = ht;
  return gas.TemperatureFromH(s.c, ht, guess, s.Tt);
}

/** Exit velocity of a subsonic exit at static pressure p: W = A p V/(R Ts(V)). */
bool SubsonicExitVelocity(const GasProperties& gas, const Composition& c, double Tt,
                          double ht, double W, double effectiveArea, double p,
                          double& V, double& Ts) {
  const double R = gas.R(c);
  V = 0.0; Ts = Tt;
  if (W <= 0.0) return true;
  // Velocity bound: sonic state at this Tt.
  double Tstar, xstar, gstar;
  if (!SonicState(gas, c, Tt, Tstar, xstar, gstar)) return false;
  const double vMax = std::sqrt(std::max(0.0, 2.0 * (ht - gas.H(c, Tstar))));
  // Initial guess from the incompressible limit.
  V = std::min(0.99 * vMax, W * R * Tt / (effectiveArea * p));
  double low = 0.0, high = vMax;
  for (int i = 0; i < 60; ++i) {
    if (!gas.TemperatureFromH(c, ht - 0.5 * V * V, Ts, Ts)) return false;
    const double f = effectiveArea * p * V / (R * Ts) - W;
    if (f > 0.0) high = V; else low = V;
    const double cp = gas.Cp(c, Ts);
    const double df = effectiveArea * p / (R * Ts) * (1.0 + V * V / (cp * Ts));
    double next = V - f / df;
    if (!(next > low && next < high)) next = 0.5 * (low + high);
    if (std::fabs(next - V) <= 1e-13 * std::max(1.0, vMax)) { V = next; break; }
    V = next;
  }
  return gas.TemperatureFromH(c, ht - 0.5 * V * V, Ts, Ts) && Finite(V);
}

/** Root of flux(x) = target on the subsonic (x > x*) or supersonic branch. */
bool AreaRatioPressure(const GasProperties& gas, const Composition& c, double Tt, double xstar,
                       double target, bool supersonic, double& x) {
  double low = supersonic ? 1e-8 : xstar, high = supersonic ? xstar : 1.0;
  double lnx = std::log(supersonic ? 0.5 * xstar : 0.5 * (1.0 + xstar));
  const double R = gas.R(c);
  for (int i = 0; i < 80; ++i) {
    double flux, Ts, V;
    const double xi = std::exp(lnx);
    if (!FluxParameter(gas, c, Tt, xi, flux, Ts, V)) return false;
    const double f = std::log(std::max(flux, 1e-300)) - std::log(target);
    // Flux falls away from x* on both branches.
    const bool tooMuch = f > 0.0;
    if (supersonic) { if (tooMuch) high = xi; else low = xi; }
    else { if (tooMuch) low = xi; else high = xi; }
    double slope = 1.0 - (V > 0.0 ? R * Ts / (V * V) : 1e300) - R / gas.Cp(c, Ts);
    double next = lnx - f / slope;
    if (!(Finite(next) && std::exp(next) > low && std::exp(next) < high))
      next = std::log(0.5 * (low + high));
    if (std::fabs(next - lnx) < 1e-14 || high - low < 1e-15 * high) { lnx = next; break; }
    lnx = next;
  }
  x = std::exp(lnx);
  return Finite(x);
}
} // namespace

CompressorResult Compress(const GasProperties& gas, const CompressorMap& map,
                          const Stream& in, double omega)
{
  CompressorResult r;
  r.out = in;
  const double R = gas.R(in.c);
  if (!(in.Pt > 0.0 && in.Tt > 0.0 && map.area > 0.0 && map.radius > 0.0)) return r;
  const double rho = in.Pt / (R * in.Tt);
  const double Va = in.W / (rho * map.area);
  const double U = omega * map.radius;
  r.torque = in.W * map.radius * (map.a * U - map.b * Va);
  r.dh = U * (map.a * U - map.b * Va);
  const double incidence = Va - map.phiDesign * U;
  r.dhIs = r.dh - 0.5 * (map.kp * Va * Va + map.ki * incidence * incidence);
  double Tis;
  if (!gas.TemperatureFromH(in.c, in.ht + r.dhIs, in.Tt, Tis)) return r;
  const double pr = std::exp((gas.Phi(in.c, Tis) - gas.Phi(in.c, in.Tt)) / R);
  if (!SetEnthalpy(gas, r.out, in.ht + r.dh, in.Tt)) return r;
  r.out.Pt = in.Pt * pr;
  r.efficiency = r.dh > 0.0 ? r.dhIs / r.dh : std::nan("");
  r.flowCoefficient = U > 0.0 ? Va / U : (Va > 0.0 ? INFINITY : 0.0);
  r.stallMargin = map.phiStall > 0.0 && U > 0.0
    ? (r.flowCoefficient - map.phiStall) / map.phiStall : INFINITY;
  r.valid = Finite(r.out.Pt) && Finite(r.torque) && r.out.Pt > 0.0;
  return r;
}

TurbineResult Expand(const GasProperties& gas, const TurbineMap& map, const Stream& in,
                     double omega)
{
  TurbineResult r;
  r.out = in;
  if (!(in.Pt > 0.0 && in.Tt > 0.0 && map.capacityMax > 0.0)) return r;
  double fraction = in.W * std::sqrt(in.Tt) / in.Pt / map.capacityMax;
  fraction = std::max(0.0, fraction);
  r.capacityFraction = fraction;
  // Beyond 99.5 % of capacity the ellipse is continued with its slope in
  // ln PR, so solver iterates stay finite. A solution there is over capacity
  // and is rejected by the caller; it is never accepted as physics.
  constexpr double kLimit = 0.995;
  if (fraction < kLimit) {
    r.pressureRatio = std::pow(1.0 - fraction * fraction, -1.0 / map.exponent);
  } else {
    r.overCapacity = true;
    const double lnLimit = -std::log(1.0 - kLimit * kLimit) / map.exponent;
    const double slope = 2.0 * kLimit / (map.exponent * (1.0 - kLimit * kLimit));
    r.pressureRatio = std::exp(std::min(lnLimit + slope * (fraction - kLimit), 50.0));
  }
  const TurbineResult at = ExpandAt(gas, map, in, omega, r.pressureRatio);
  const bool over = r.overCapacity;
  r = at;
  r.capacityFraction = fraction;
  r.overCapacity = over;
  return r;
}

double CapacityFraction(const TurbineMap& map, double pressureRatio)
{
  return pressureRatio > 1.0 ? std::sqrt(-std::expm1(-map.exponent * std::log(pressureRatio))) : 0.0;
}

TurbineResult ExpandAt(const GasProperties& gas, const TurbineMap& map, const Stream& in,
                       double omega, double pressureRatio)
{
  TurbineResult r;
  r.out = in;
  if (!(in.Pt > 0.0 && in.Tt > 0.0 && pressureRatio >= 1.0 && Finite(pressureRatio))) return r;
  r.pressureRatio = pressureRatio;
  r.capacityFraction = map.capacityMax > 0.0 ? in.W * std::sqrt(in.Tt) / in.Pt / map.capacityMax : 0.0;
  r.overCapacity = r.capacityFraction >= 1.0;
  double Tis;
  if (!gas.IsentropicT(in.c, in.Tt, 1.0 / r.pressureRatio, Tis)) return r;
  r.dhIs = in.ht - gas.H(in.c, Tis);
  const double c0 = std::sqrt(std::max(0.0, 2.0 * r.dhIs));
  const double U = omega * map.radius;
  const double nu = map.velocityRatioOptimum;
  const double eta = map.peakEfficiency;
  r.velocityRatio = c0 > 0.0 ? U / c0 : (U > 0.0 ? INFINITY : 0.0);
  r.torque = in.W * eta * map.radius * (2.0 * c0 - U / nu) / (2.0 * nu);
  r.dh = eta * U * (2.0 * c0 - U / nu) / (2.0 * nu);
  if (!SetEnthalpy(gas, r.out, in.ht - r.dh, in.Tt)) return r;
  r.out.Pt = in.Pt / r.pressureRatio;
  r.valid = Finite(r.torque) && Finite(r.out.Pt) && r.out.Pt > 0.0;
  return r;
}

double PressureLossFraction(const LossModel& loss, const Stream& in, double TtOut)
{
  if (!(in.Pt > 0.0) || loss.designCapacity <= 0.0) return 0.0;
  const double q = in.W * std::sqrt(in.Tt) / in.Pt / loss.designCapacity;
  const double heat = in.Tt > 0.0 ? std::max(0.0, TtOut / in.Tt - 1.0) : 0.0;
  return q * std::fabs(q) * (loss.cold + loss.hot * heat);
}

BurnerResult Burn(const GasProperties& gas, const Stream& in, const BurnerInput& input,
                  const LossModel& loss)
{
  BurnerResult r;
  r.out = in;
  const double far = gas.StoichiometricFAR();
  const double air = std::max(0.0, in.W * in.c.y[kAir]);
  const double products = std::max(0.0, in.W * in.c.y[kProducts]);
  const double fuelIn = std::max(0.0, in.W * in.c.y[kFuel]);
  const double spray = std::max(0.0, input.sprayKgSec);
  const double burnable = fuelIn + spray;
  const double wanted = input.lit ? std::clamp(input.efficiency, 0.0, 1.0) * burnable : 0.0;
  r.burnedKgSec = std::min(wanted, air * far);
  r.oxygenLimitedKgSec = wanted - r.burnedKgSec;
  r.heatReleaseW = r.burnedKgSec * gas.Fuel().lowerHeatingValue;
  const double W = in.W + spray;
  if (!(W > 0.0)) { r.valid = true; return r; }
  r.out.W = W;
  r.out.c.y[kAir] = (air - r.burnedKgSec / far) / W;
  r.out.c.y[kProducts] = (products + r.burnedKgSec * (1.0 + 1.0 / far)) / W;
  r.out.c.y[kFuel] = (burnable - r.burnedKgSec) / W;
  r.unburnedOutKgSec = burnable - r.burnedKgSec;
  const double H = in.W * in.ht + spray * input.fuelSensibleH + r.heatReleaseW - input.wallHeatW;
  if (!SetEnthalpy(gas, r.out, H / W, in.Tt)) return r;
  r.pressureLoss = PressureLossFraction(loss, in, r.out.Tt);
  r.out.Pt = in.Pt * (1.0 - r.pressureLoss);
  r.valid = r.pressureLoss < 0.9 && Finite(r.out.Pt) && Finite(r.out.Tt);
  return r;
}

Stream Mix(const GasProperties& gas, const Stream& a, const Stream& b, const LossModel& loss,
           bool& valid)
{
  // Signed flows are accepted as a smooth extension for solver iterates; a
  // physical solution never carries a negative stream.
  Stream m;
  const double Wa = a.W, Wb = b.W;
  m.W = Wa + Wb;
  valid = false;
  if (!(m.W > 0.0)) {
    if (m.W < 0.0) return m;
    m = a.Pt >= b.Pt ? a : b;
    m.W = 0.0;
    valid = true;
    return m;
  }
  for (int s = 0; s < kSpeciesCount; ++s) m.c.y[s] = (Wa * a.c.y[s] + Wb * b.c.y[s]) / m.W;
  const double H = Wa * a.ht + Wb * b.ht;
  if (!SetEnthalpy(gas, m, H / m.W, Wa >= Wb ? a.Tt : b.Tt)) return m;
  m.Pt = (Wa * a.Pt + Wb * b.Pt) / m.W;
  m.Pt *= 1.0 - PressureLossFraction(loss, m, m.Tt);
  valid = Finite(m.Pt) && m.Pt > 0.0;
  return m;
}

bool AddHeat(const GasProperties& gas, Stream& s, double heatW)
{
  if (heatW == 0.0) return true;
  if (!(s.W > 0.0)) return heatW == 0.0;
  return SetEnthalpy(gas, s, s.ht + heatW / s.W, s.Tt);
}

bool SonicState(const GasProperties& gas, const Composition& c, double Tt, double& Tstar,
                double& pressureRatio, double& fluxPerPt)
{
  const double R = gas.R(c);
  const double ht = gas.H(c, Tt);
  if (gas.Kind() == GasModelKind::CaloricallyPerfect) {
    const double g = gas.Gamma(c, Tt);
    Tstar = 2.0 * Tt / (g + 1.0);
  } else {
    // f(T) = h(Tt) - h(T) - gamma(T) R T / 2; secant iteration from the
    // frozen-gamma estimate. f is monotone decreasing in T.
    const auto f = [&](double T) { return ht - gas.H(c, T) - 0.5 * gas.Gamma(c, T) * R * T; };
    double t0 = 2.0 * Tt / (gas.Gamma(c, Tt) + 1.0);
    double t1 = t0 * (1.0 + 1e-4);
    double f0 = f(t0), f1 = f(t1);
    for (int i = 0; i < 50 && f1 != f0; ++i) {
      const double t2 = t1 - f1 * (t1 - t0) / (f1 - f0);
      t0 = t1; f0 = f1; t1 = t2; f1 = f(t1);
      if (std::fabs(t1 - t0) <= 1e-13 * Tt) break;
    }
    Tstar = t1;
  }
  pressureRatio = std::exp((gas.Phi(c, Tstar) - gas.Phi(c, Tt)) / R);
  const double V = std::sqrt(gas.Gamma(c, Tstar) * R * Tstar);
  fluxPerPt = pressureRatio * V / (R * Tstar);
  return Finite(Tstar) && Finite(pressureRatio) && Finite(fluxPerPt) && Tstar > 0.0;
}

bool FluxParameter(const GasProperties& gas, const Composition& c, double Tt, double x,
                   double& flux, double& Ts, double& V)
{
  const double R = gas.R(c);
  if (!gas.IsentropicT(c, Tt, x, Ts)) return false;
  V = std::sqrt(std::max(0.0, 2.0 * (gas.H(c, Tt) - gas.H(c, Ts))));
  flux = x * V / (R * Ts);
  return Finite(flux);
}

bool NozzleRequiredPressure(const GasProperties& gas, const Stream& stream,
                            const NozzleGeometry& geometry, double ambientPa,
                            NozzleResult& r)
{
  r = NozzleResult();
  const double At = geometry.dischargeCoefficient * geometry.throatArea;
  const double Ae = geometry.dischargeCoefficient * std::max(geometry.exitArea, geometry.throatArea);
  if (!(At > 0.0 && ambientPa > 0.0 && stream.Tt > 0.0)) return false;
  r.exitArea = std::max(geometry.exitArea, geometry.throatArea);
  if (!(stream.W > 0.0)) {
    r.requiredPt = ambientPa;
    r.exitStaticPressure = ambientPa;
    r.exitStaticTemperature = stream.Tt;
    r.regime = NozzleRegime::NoFlow;
    r.valid = true;
    return true;
  }
  double Tstar, xstar, gstar;
  if (!SonicState(gas, stream.c, stream.Tt, Tstar, xstar, gstar)) return false;
  r.criticalPressureRatio = xstar;
  const double areaRatio = Ae / At;
  double xsub = xstar;
  if (areaRatio > 1.0 + 1e-12 &&
      !AreaRatioPressure(gas, stream.c, stream.Tt, xstar, gstar / areaRatio, false, xsub))
    return false;
  const double ptChoked = stream.W / (At * gstar);
  if (ambientPa / ptChoked <= xsub) {
    r.choked = true;
    r.requiredPt = ptChoked;
    r.regime = NozzleRegime::UnderexpandedOrIdeal; // refined by NozzleThrust
  } else {
    r.regime = NozzleRegime::Subsonic;
    double V, Ts;
    if (!SubsonicExitVelocity(gas, stream.c, stream.Tt, stream.ht, stream.W, Ae, ambientPa, V, Ts))
      return false;
    const double x = std::exp((gas.Phi(stream.c, Ts) - gas.Phi(stream.c, stream.Tt)) / gas.R(stream.c));
    r.requiredPt = ambientPa / x;
  }
  r.valid = Finite(r.requiredPt);
  return r.valid;
}

bool NozzleThrust(const GasProperties& gas, const Stream& stream, const NozzleGeometry& geometry,
                  double ambientPa, NozzleResult& r)
{
  if (!NozzleRequiredPressure(gas, stream, geometry, ambientPa, r)) return false;
  if (r.regime == NozzleRegime::NoFlow) return true;
  const Composition& c = stream.c;
  const double R = gas.R(c);
  const double At = geometry.dischargeCoefficient * geometry.throatArea;
  const double Ae = geometry.dischargeCoefficient * std::max(geometry.exitArea, geometry.throatArea);
  const double cv = geometry.velocityCoefficient;
  const auto subsonicExit = [&](NozzleRegime regime) {
    double V, Ts;
    if (!SubsonicExitVelocity(gas, c, stream.Tt, stream.ht, stream.W, Ae, ambientPa, V, Ts))
      return false;
    r.regime = regime;
    r.exitStaticPressure = ambientPa;
    r.exitStaticTemperature = Ts;
    r.exitVelocity = V;
    r.exitMach = V / std::sqrt(gas.Gamma(c, Ts) * R * Ts);
    r.exitDensity = ambientPa / (R * Ts);
    r.exitArea = std::max(geometry.exitArea, geometry.throatArea);
    r.grossThrust = stream.W * cv * V;
    return true;
  };
  if (!r.choked) { r.valid = subsonicExit(NozzleRegime::Subsonic); return r.valid; }

  const double Pt = r.requiredPt;
  double Tstar, xstar, gstar;
  if (!SonicState(gas, c, stream.Tt, Tstar, xstar, gstar)) return false;
  const double areaRatio = Ae / At;
  double x9 = xstar;
  if (areaRatio > 1.0 + 1e-12 &&
      !AreaRatioPressure(gas, c, stream.Tt, xstar, gstar / areaRatio, true, x9))
    return false;
  double flux9, T9, V9;
  if (!FluxParameter(gas, c, stream.Tt, x9, flux9, T9, V9)) return false;
  const double p9 = x9 * Pt;
  const double gamma9 = gas.Gamma(c, T9);
  const double M9 = V9 / std::sqrt(gamma9 * R * T9);
  const double geometricExit = std::max(geometry.exitArea, geometry.throatArea);
  const auto supersonicExit = [&](NozzleRegime regime, double p, double T, double V, double area) {
    r.regime = regime;
    r.exitStaticPressure = p;
    r.exitStaticTemperature = T;
    r.exitVelocity = V;
    r.exitMach = V / std::sqrt(gas.Gamma(c, T) * R * T);
    r.exitDensity = p / (R * T);
    r.exitArea = area;
    r.grossThrust = stream.W * cv * V + (p - ambientPa) * area;
  };
  if (p9 >= ambientPa) {
    supersonicExit(NozzleRegime::UnderexpandedOrIdeal, p9, T9, V9, geometricExit);
  } else {
    // Normal shock standing at the exit plane, frozen gamma at exit.
    const double pShock = p9 * (1.0 + 2.0 * gamma9 / (gamma9 + 1.0) * (M9 * M9 - 1.0));
    if (areaRatio > 1.0 + 1e-12 && ambientPa >= pShock) {
      r.valid = subsonicExit(NozzleRegime::ChokedInternalShock);
      r.choked = true;
      return r.valid;
    }
    const double pSeparation = geometry.separationPressureRatio * ambientPa;
    if (p9 >= pSeparation) {
      supersonicExit(NozzleRegime::OverexpandedAttached, p9, T9, V9, geometricExit);
    } else {
      // Summerfield: the jet leaves the wall where the wall pressure falls to
      // the separation criterion; the downstream wall carries ambient pressure.
      double xs = pSeparation / Pt, flux, Ts, Vs, area;
      if (xs >= xstar) {
        xs = xstar; Ts = Tstar; Vs = std::sqrt(gas.Gamma(c, Tstar) * R * Tstar);
        area = geometry.throatArea;
      } else {
        if (!FluxParameter(gas, c, stream.Tt, xs, flux, Ts, Vs)) return false;
        area = geometry.throatArea * gstar / flux;
      }
      supersonicExit(NozzleRegime::OverexpandedSeparated, xs * Pt, Ts, Vs, area);
    }
  }
  r.valid = Finite(r.grossThrust);
  return r.valid;
}


bool SubsonicStaticState(const GasProperties& gas, const Stream& s, double area,
                         double& pressureRatio, double& Ts, double& V)
{
  pressureRatio = 1.0; Ts = s.Tt; V = 0.0;
  if (!(area > 0.0 && s.Pt > 0.0 && s.Tt > 0.0)) return false;
  if (!(s.W > 0.0)) return true;
  const double R = gas.R(s.c);
  double Tstar, xstar, gstar;
  if (!SonicState(gas, s.c, s.Tt, Tstar, xstar, gstar)) return false;
  if (!(s.W < area * s.Pt * gstar)) return false;
  // Continuity in velocity: f(V) = A Ps(V) V / (R Ts(V)) - W rises through
  // the subsonic range and is well behaved at vanishing flow.
  const double phiT = gas.Phi(s.c, s.Tt);
  const double vMax = std::sqrt(gas.Gamma(s.c, Tstar) * R * Tstar);
  double low = 0.0, high = vMax;
  V = std::min(0.9 * vMax, s.W * R * s.Tt / (area * s.Pt));
  for (int i = 0; i < 60; ++i) {
    if (!gas.TemperatureFromH(s.c, s.ht - 0.5 * V * V, Ts, Ts)) return false;
    const double x = std::exp((gas.Phi(s.c, Ts) - phiT) / R);
    const double f = area * s.Pt * x * V / (R * Ts) - s.W;
    if (f > 0.0) high = V; else low = V;
    // d ln(rho V)/dV = 1/V - V/(gamma R Ts) for an isentropic stream.
    const double slope = area * s.Pt * x / (R * Ts) * (1.0 - V * V / (gas.Gamma(s.c, Ts) * R * Ts));
    double next = slope > 0.0 ? V - f / slope : 0.5 * (low + high);
    if (!(next > low && next < high)) next = 0.5 * (low + high);
    if (std::fabs(next - V) <= 1e-13 * vMax) { V = next; break; }
    V = next;
  }
  if (!gas.TemperatureFromH(s.c, s.ht - 0.5 * V * V, Ts, Ts)) return false;
  pressureRatio = std::exp((gas.Phi(s.c, Ts) - phiT) / R);
  return Finite(pressureRatio) && Finite(V);
}

MixerResult MixAtStaticPressure(const GasProperties& gas, const Stream& core,
                                const Stream& bypass, const MixerGeometry& geometry)
{
  MixerResult r;
  r.out = core;
  double x, Ts, V5;
  if (!SubsonicStaticState(gas, core, geometry.coreArea, x, Ts, V5)) return r;
  const double Ps = x * core.Pt;
  r.staticPressure = Ps;
  // Bypass air enters through its own area against the core's static pressure.
  Stream injected = bypass;
  injected.W = std::fabs(bypass.W);
  NozzleGeometry entry;
  entry.throatArea = entry.exitArea = geometry.bypassArea;
  NozzleResult nozzle;
  if (!NozzleThrust(gas, injected, entry, Ps, nozzle)) return r;
  const double sign = bypass.W < 0.0 ? -1.0 : 1.0;
  r.bypassRequiredPt = Ps + sign * (nozzle.requiredPt - Ps);
  r.bypassChoked = nozzle.choked;

  Stream m;
  m.W = core.W + bypass.W;
  if (!(m.W > 0.0)) {
    if (m.W < 0.0) return r;
    m = core;
    m.W = 0.0;
    m.Pt = Ps;
    r.out = m;
    r.valid = true;
    return r;
  }
  for (int k = 0; k < kSpeciesCount; ++k)
    m.c.y[k] = (core.W * core.c.y[k] + bypass.W * bypass.c.y[k]) / m.W;
  if (!SetEnthalpy(gas, m, (core.W * core.ht + bypass.W * bypass.ht) / m.W,
                   core.W >= bypass.W ? core.Tt : bypass.Tt)) return r;
  // Impulse at the mixing plane: momentum plus pressure on each entry area.
  const double area = geometry.coreArea + geometry.bypassArea;
  const double impulse = core.W * V5 + sign * nozzle.grossThrust + Ps * area;
  // Mixed-out state: (I - W V) V / (R T(V)) = W on the subsonic branch.
  const double R = gas.R(m.c);
  double Tstar, xstar, gstar;
  if (!SonicState(gas, m.c, m.Tt, Tstar, xstar, gstar)) return r;
  const double vMax = std::sqrt(gas.Gamma(m.c, Tstar) * R * Tstar);
  double low = 0.0, high = vMax, T = m.Tt;
  double V = std::min(0.9 * vMax, m.W * R * m.Tt * area / (impulse * area));
  const auto f = [&](double v, double& t) {
    if (!gas.TemperatureFromH(m.c, m.ht - 0.5 * v * v, t, t)) return std::nan("");
    return (impulse - m.W * v) * v / (R * t) - m.W;
  };
  double tHigh = Tstar;
  const double fHigh = f(vMax, tHigh);
  if (!(fHigh >= 0.0)) return r;          // the mixed duct cannot pass the flow
  for (int i = 0; i < 60; ++i) {
    const double fv = f(V, T);
    if (!Finite(fv)) return r;
    if (fv > 0.0) high = V; else low = V;
    const double cp = gas.Cp(m.c, T);
    const double slope = (impulse - 2.0 * m.W * V) / (R * T)
      + (impulse - m.W * V) * V * V / (R * T * T * cp);
    double next = slope > 0.0 ? V - fv / slope : 0.5 * (low + high);
    if (!(next > low && next < high)) next = 0.5 * (low + high);
    if (std::fabs(next - V) <= 1e-13 * vMax) { V = next; break; }
    V = next;
  }
  if (!Finite(f(V, T))) return r;
  const double P = (impulse - m.W * V) / area;
  m.Pt = P * std::exp((gas.Phi(m.c, m.Tt) - gas.Phi(m.c, T)) / R);
  r.out = m;
  r.valid = Finite(m.Pt) && m.Pt > 0.0;
  return r;
}

} // namespace TurbinePlant
} // namespace JSBSim
