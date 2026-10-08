/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Module:       TurbinePlantGas.cpp
 Date started: 2026-10-07
 Purpose:      Pseudo-species thermodynamics of the coupled turbine plant.

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

#include "TurbinePlantGas.h"

namespace JSBSim {
namespace TurbinePlant {

namespace {
#include "TurbinePlantGasData.inc"

// Molar masses (kg/mol) for element bookkeeping; they match the generator.
constexpr double kMC = 12.0107e-3, kMH = 1.00794e-3, kMO = 15.9994e-3;

double Horner(const double* a, double T) {
  return a[0] + T * (a[1] + T * (a[2] + T * (a[3] + T * a[4])));
}
double PolyCp(const double* a, double T) { return Horner(a, T); }
double PolyH(const double* a, double T) {
  return T * (a[0] + T * (a[1] / 2 + T * (a[2] / 3 + T * (a[3] / 4 + T * a[4] / 5)))) + a[5];
}
double PolyPhi(const double* a, double T) {
  return a[0] * std::log(T) + T * (a[1] + T * (a[2] / 2 + T * (a[3] / 3 + T * a[4] / 4))) + a[6];
}
const double* Coefficients(int species, double T) {
  if (species == kAir) return T >= 1000.0 ? kPoly0High : kPoly0Low;
  return T >= 1000.0 ? kPoly1High : kPoly1Low;
}
const double* Table(int species, int quantity) {
  static const double* const tables[2][4] = {
    {kTable0_h, kTable0_cp, kTable0_dcp, kTable0_phi},
    {kTable1_h, kTable1_cp, kTable1_dcp, kTable1_phi}};
  return tables[species][quantity];
}
} // namespace

double Composition::FuelAirRatioEquivalent(double stoichiometricFAR) const
{
  // Fuel mass per unit air mass that produced this gas, counting unburned fuel.
  const double burnedFuel = y[kProducts] * stoichiometricFAR / (1.0 + stoichiometricFAR);
  const double air = y[kAir] + y[kProducts] - burnedFuel;
  const double fuel = burnedFuel + y[kFuel];
  return air > 0.0 ? fuel / air : 0.0;
}

GasProperties::GasProperties() { Configure(GasModelKind::ThermallyPerfect, FuelSpec()); }

void GasProperties::Configure(GasModelKind kind, const FuelSpec& fuel,
                              double perfectCp, double perfectR)
{
  kind_ = kind;
  fuel_ = fuel;
  perfectCp_ = perfectCp;
  perfectR_ = perfectR;
  for (int s = 0; s < 2; ++s) {
    for (int i = 0; i < 7; ++i) {
      poly_[s].low[i] = (s == kAir ? kPoly0Low : kPoly1Low)[i];
      poly_[s].high[i] = (s == kAir ? kPoly0High : kPoly1High)[i];
    }
    h298_[s] = PolyH(Coefficients(s, kReferenceK), kReferenceK);
  }
  // Species gas constants follow from the polynomial a1 terms only for a
  // constant-composition mixture: cp - cv = R for ideal gases, and the NASA
  // fits are per R, so recover R from the generator's mass-weighted sum of
  // R_k per unit mass. That sum is not in the polynomial itself, so compute
  // it from the documented compositions here.
  const double xN2 = 0.78084, xO2 = 0.209476, xAr = 0.00934, xCO2 = 0.000314;
  const double sum = xN2 + xO2 + xAr + xCO2;
  const double mN2 = 28.0134e-3, mO2 = 31.9988e-3, mAr = 39.948e-3;
  const double mCO2 = kMC + 2 * kMO, mH2O = 2 * kMH + kMO;
  const double airMolar = (xN2 * mN2 + xO2 * mO2 + xAr * mAr + xCO2 * mCO2) / sum;
  const double x = fuel.carbonAtoms, yH = fuel.hydrogenAtoms;
  const double fuelMolar = x * kMC + yH * kMH;
  const double nO2 = x + yH / 4.0;
  const double nAir = nO2 / (xO2 / sum);
  const double nProducts = (x + nAir * xCO2 / sum) + yH / 2.0 + nAir * (xN2 + xAr) / sum;
  const double productsMass = fuelMolar + nAir * airMolar;
  far_ = fuelMolar / (nAir * airMolar);
  if (kind_ == GasModelKind::CaloricallyPerfect) {
    R_ = {perfectR_, perfectR_, perfectR_};
  } else {
    R_[kAir] = kUniversalR / airMolar;
    R_[kProducts] = kUniversalR * nProducts / productsMass;
    R_[kFuel] = kUniversalR / fuelMolar;
  }
  // Element mass fractions (C, H, O) for conservation bookkeeping.
  carbon_[kAir] = (xCO2 / sum) * kMC / airMolar;
  oxygen_[kAir] = ((xO2 + xCO2) / sum) * 2 * kMO / airMolar;
  hydrogen_[kAir] = 0.0;
  carbon_[kProducts] = (x + nAir * xCO2 / sum) * kMC / productsMass;
  hydrogen_[kProducts] = yH * kMH / productsMass;
  oxygen_[kProducts] = (nAir * (xO2 + xCO2) / sum) * 2 * kMO / productsMass;
  carbon_[kFuel] = x * kMC / fuelMolar;
  hydrogen_[kFuel] = yH * kMH / fuelMolar;
  oxygen_[kFuel] = 0.0;
  (void)mH2O;
}

double GasProperties::R(const Composition& c) const
{
  return c.y[kAir] * R_[kAir] + c.y[kProducts] * R_[kProducts] + c.y[kFuel] * R_[kFuel];
}

double GasProperties::PolynomialH(int s, double T) const
{
  if (s == kFuel) return fuel_.specificHeat * (T - kReferenceK);
  return PolyH(Coefficients(s, T), T) - h298_[s];
}
double GasProperties::PolynomialCp(int s, double T) const
{
  if (s == kFuel) return fuel_.specificHeat;
  return PolyCp(Coefficients(s, T), T);
}
double GasProperties::PolynomialPhi(int s, double T) const
{
  if (s == kFuel) return fuel_.specificHeat * std::log(T / kReferenceK);
  return PolyPhi(Coefficients(s, T), T);
}

double GasProperties::TableValue(int species, int quantity, double T) const
{
  // Cubic Hermite with analytic node derivatives; quantity 0 h, 1 cp, 3 phi.
  const double* v = Table(species, quantity);
  const double u = (T - kTableMinK) / kTableStepK;
  int i = static_cast<int>(std::floor(u));
  i = std::min(std::max(i, 0), kTableCount - 2);
  const double s = u - i;
  const double s2 = s * s, s1 = 1.0 - s;
  const double h00 = (1.0 + 2.0 * s) * s1 * s1, h10 = s * s1 * s1;
  const double h01 = s2 * (3.0 - 2.0 * s), h11 = s2 * (s - 1.0);
  double d0, d1;
  if (quantity == 0) { d0 = Table(species, 1)[i]; d1 = Table(species, 1)[i + 1]; }
  else if (quantity == 1) { d0 = Table(species, 2)[i]; d1 = Table(species, 2)[i + 1]; }
  else {
    const double* cp = Table(species, 1);
    d0 = cp[i] / (kTableMinK + i * kTableStepK);
    d1 = cp[i + 1] / (kTableMinK + (i + 1) * kTableStepK);
  }
  return h00 * v[i] + h10 * kTableStepK * d0 + h01 * v[i + 1] + h11 * kTableStepK * d1;
}

double GasProperties::SpeciesH(int s, double T) const
{
  switch (kind_) {
  case GasModelKind::CaloricallyPerfect: return perfectCp_ * (T - kReferenceK);
  case GasModelKind::Tabulated:
    return s == kFuel ? PolynomialH(s, T) : TableValue(s, 0, T);
  default: return PolynomialH(s, T);
  }
}
double GasProperties::SpeciesCp(int s, double T) const
{
  switch (kind_) {
  case GasModelKind::CaloricallyPerfect: return perfectCp_;
  case GasModelKind::Tabulated:
    return s == kFuel ? PolynomialCp(s, T) : TableValue(s, 1, T);
  default: return PolynomialCp(s, T);
  }
}
double GasProperties::SpeciesPhi(int s, double T) const
{
  switch (kind_) {
  case GasModelKind::CaloricallyPerfect: return perfectCp_ * std::log(T / kReferenceK);
  case GasModelKind::Tabulated:
    return s == kFuel ? PolynomialPhi(s, T) : TableValue(s, 3, T);
  default: return PolynomialPhi(s, T);
  }
}

double GasProperties::H(const Composition& c, double T) const
{
  double h = 0.0;
  for (int s = 0; s < kSpeciesCount; ++s) if (c.y[s] != 0.0) h += c.y[s] * SpeciesH(s, T);
  return h;
}
double GasProperties::Cp(const Composition& c, double T) const
{
  double cp = 0.0;
  for (int s = 0; s < kSpeciesCount; ++s) if (c.y[s] != 0.0) cp += c.y[s] * SpeciesCp(s, T);
  return cp;
}
double GasProperties::Phi(const Composition& c, double T) const
{
  double phi = 0.0;
  for (int s = 0; s < kSpeciesCount; ++s) if (c.y[s] != 0.0) phi += c.y[s] * SpeciesPhi(s, T);
  return phi;
}
double GasProperties::Gamma(const Composition& c, double T) const
{
  const double cp = Cp(c, T);
  return cp / (cp - R(c));
}

bool GasProperties::TemperatureFromH(const Composition& c, double h, double guess, double& T) const
{
  if (!std::isfinite(h)) return false;
  if (kind_ == GasModelKind::CaloricallyPerfect) {
    T = kReferenceK + h / perfectCp_;
    if (T < kMinK || T > kMaxK) ++domainClamps;
    return std::isfinite(T);
  }
  T = std::isfinite(guess) && guess > kMinK && guess < kMaxK ? guess : kReferenceK + h / 1100.0;
  T = std::min(std::max(T, kMinK), kMaxK);
  for (int i = 0; i < 30; ++i) {
    const double f = H(c, T) - h;
    const double step = f / Cp(c, T);
    double next = T - step;
    if (next < kMinK) next = 0.5 * (T + kMinK);
    if (next > kMaxK) next = 0.5 * (T + kMaxK);
    if (std::fabs(next - T) <= 1e-12 * T) { T = next; break; }
    T = next;
  }
  // Only a result at a bound can be out of domain; avoid two extra
  // evaluations on every ordinary inversion.
  if (T <= kMinK * (1.0 + 1e-9) && h < H(c, kMinK)) { T = kMinK; ++domainClamps; }
  else if (T >= kMaxK * (1.0 - 1e-9) && h > H(c, kMaxK)) { T = kMaxK; ++domainClamps; }
  return std::isfinite(T);
}

bool GasProperties::TemperatureFromPhi(const Composition& c, double phi, double guess, double& T) const
{
  if (!std::isfinite(phi)) return false;
  if (kind_ == GasModelKind::CaloricallyPerfect) {
    T = kReferenceK * std::exp(phi / perfectCp_);
    if (T < kMinK || T > kMaxK) ++domainClamps;
    return std::isfinite(T);
  }
  T = std::isfinite(guess) && guess > kMinK && guess < kMaxK ? guess : kReferenceK;
  for (int i = 0; i < 30; ++i) {
    const double f = Phi(c, T) - phi;
    // dphi/dT = cp/T: Newton in ln T is better conditioned.
    const double step = f / Cp(c, T);
    double next = T * std::exp(-step);
    if (next < kMinK) next = 0.5 * (T + kMinK);
    if (next > kMaxK) next = 0.5 * (T + kMaxK);
    if (std::fabs(next - T) <= 1e-12 * T) { T = next; break; }
    T = next;
  }
  if (T <= kMinK * (1.0 + 1e-9) && phi < Phi(c, kMinK)) { T = kMinK; ++domainClamps; }
  else if (T >= kMaxK * (1.0 - 1e-9) && phi > Phi(c, kMaxK)) { T = kMaxK; ++domainClamps; }
  return std::isfinite(T);
}

bool GasProperties::IsentropicT(const Composition& c, double Tin, double pressureRatio,
                                double& Tout) const
{
  if (!(pressureRatio > 0.0) || !std::isfinite(pressureRatio)) return false;
  if (kind_ == GasModelKind::CaloricallyPerfect) {
    Tout = Tin * std::pow(pressureRatio, perfectR_ / perfectCp_);
    return std::isfinite(Tout);
  }
  const double target = Phi(c, Tin) + R(c) * std::log(pressureRatio);
  const double gamma = Gamma(c, Tin);
  const double guess = Tin * std::pow(pressureRatio, (gamma - 1.0) / gamma);
  return TemperatureFromPhi(c, target, guess, Tout);
}

std::size_t GasProperties::ResidentBytes() const
{
  return kind_ == GasModelKind::Tabulated
    ? static_cast<std::size_t>(2 * 4 * kTableCount) * sizeof(double) : 0;
}

} // namespace TurbinePlant
} // namespace JSBSim
