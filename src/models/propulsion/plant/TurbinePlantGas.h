/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Header:       TurbinePlantGas.h
 Date started: 2026-10-07

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

#ifndef TURBINEPLANTGAS_H
#define TURBINEPLANTGAS_H

#include <array>
#include <cstddef>

namespace JSBSim {
namespace TurbinePlant {

/** Pseudo-species basis of the coupled turbine plant.

    Lean complete combustion of a CxHy fuel in air produces a gas that is
    exactly a mixture of unreacted air and the products of stoichiometric
    combustion. Unburned delivered fuel is the third pseudo-species. Mass
    fractions of these three describe composition at every station, and the
    element balance is exact at this resolution. Dissociation, CO, NOx and
    soot are not represented; see doc/turbine-plant-model.md.

    Energy convention: sensible enthalpies with a 298.15 K datum, plus burned
    fuel times the configured lower heating value of the liquid fuel. Formation
    enthalpies are never added, so chemical energy is counted exactly once. */
enum Species : int { kAir = 0, kProducts = 1, kFuel = 2, kSpeciesCount = 3 };

struct Composition {
  std::array<double, kSpeciesCount> y{{1.0, 0.0, 0.0}};
  double FuelAirRatioEquivalent(double stoichiometricFAR) const;
};

/** Thermal model selected for a plant. */
enum class GasModelKind : int {
  ThermallyPerfect = 0, ///< NASA 7-coefficient polynomials (reference)
  Tabulated = 1,        ///< Generated Hermite tables (reduced algorithm)
  CaloricallyPerfect = 2 ///< Constant cp and R for analytical fixtures
};

struct FuelSpec {
  double carbonAtoms = 12.0;     ///< Jet-A surrogate C12H23 by default
  double hydrogenAtoms = 23.0;
  double lowerHeatingValue = 43.2e6; ///< J/kg, liquid fuel, H2O vapour, 298.15 K
  double specificHeat = 2000.0;  ///< J/kg/K, unburned/injected fuel sensible model
};

/** Thermodynamic properties of the three pseudo-species.

    Every function is pure and allocation-free. Temperatures outside the
    validated range are clamped by the inversion and reported through the
    domain counter rather than silently extrapolated. */
class GasProperties {
public:
  static constexpr double kReferenceK = 298.15;
  static constexpr double kMinK = 200.0;
  static constexpr double kMaxK = 3500.0;
  static constexpr double kUniversalR = 8.31446261815324; // J/mol/K

  GasProperties();
  void Configure(GasModelKind kind, const FuelSpec& fuel,
                 double perfectCp = 1004.5, double perfectR = 287.05);

  GasModelKind Kind() const { return kind_; }
  const FuelSpec& Fuel() const { return fuel_; }
  double StoichiometricFAR() const { return far_; }
  double SpeciesR(int species) const { return R_[species]; }
  double R(const Composition& c) const;

  // Pure-species sensible enthalpy (J/kg), cp (J/kg/K) and standard entropy
  // function phi = s0 (J/kg/K), each with the selected model.
  double SpeciesH(int species, double T) const;
  double SpeciesCp(int species, double T) const;
  double SpeciesPhi(int species, double T) const;

  double H(const Composition& c, double T) const;
  double Cp(const Composition& c, double T) const;
  double Phi(const Composition& c, double T) const;
  double Gamma(const Composition& c, double T) const;

  /** Inverse of H at fixed composition. Returns false only for nonfinite
      input; a clamped result increments the domain counter. */
  bool TemperatureFromH(const Composition& c, double h, double guess, double& T) const;
  /** Inverse of Phi at fixed composition. */
  bool TemperatureFromPhi(const Composition& c, double phi, double guess, double& T) const;
  /** Isentropic temperature for a total-pressure ratio pOut/pIn. */
  bool IsentropicT(const Composition& c, double Tin, double pressureRatio, double& Tout) const;

  /** Number of property evaluations clamped to the validated range since reset. */
  mutable unsigned long domainClamps = 0;
  /** Bytes of resident data required by the selected model. */
  std::size_t ResidentBytes() const;

  /** Raw NASA evaluation, independent of the selected model (used by the
      generator self-checks and tests). */
  double PolynomialH(int species, double T) const;
  double PolynomialCp(int species, double T) const;
  double PolynomialPhi(int species, double T) const;

  /** Element mass fractions of pseudo-species, for conservation tests. */
  double CarbonMassFraction(int species) const { return carbon_[species]; }
  double HydrogenMassFraction(int species) const { return hydrogen_[species]; }
  double OxygenMassFraction(int species) const { return oxygen_[species]; }

private:
  struct Poly { std::array<double, 7> low{}, high{}; };
  GasModelKind kind_ = GasModelKind::ThermallyPerfect;
  FuelSpec fuel_;
  double far_ = 0.0;
  std::array<double, kSpeciesCount> R_{};
  std::array<Poly, 2> poly_{};         // per unit mass, air and products
  std::array<double, 2> h298_{};       // polynomial absolute H at 298.15 K
  std::array<double, kSpeciesCount> carbon_{}, hydrogen_{}, oxygen_{};
  double perfectCp_ = 1004.5, perfectR_ = 287.05;

  double TableValue(int species, int quantity, double T) const;
};

/** Total conditions of a stream with its composition. */
struct Stream {
  double W = 0.0;   ///< kg/s
  Composition c;
  double Tt = 0.0;  ///< K
  double Pt = 0.0;  ///< Pa
  double ht = 0.0;  ///< J/kg, sensible total enthalpy, 298.15 K datum
};

} // namespace TurbinePlant
} // namespace JSBSim

#endif
