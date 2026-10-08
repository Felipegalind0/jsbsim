/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Header:       TurbinePlantComponents.h
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

#ifndef TURBINEPLANTCOMPONENTS_H
#define TURBINEPLANTCOMPONENTS_H

#include "TurbinePlantGas.h"

namespace JSBSim {
namespace TurbinePlant {

/** Lumped Euler-stage compressor surrogate.

    Work follows Euler's turbomachinery equation with a straight work line,
    dh = U (a U - b Va), so torque = W r (a U - b Va) stays finite at zero
    speed. Losses are 0.5 (kp Va^2 + ki (Va - phiDesign U)^2): a stationary
    row is a pressure loss and windmilling flow drives the rotor. Speed lines
    fall monotonically with flow (no positive-slope region, so surge dynamics
    are not represented; the flow coefficient margin is only observed).
    Compressibility and stage matching are not modeled: this is a surrogate
    calibrated at one design point, not a measured map. */
struct CompressorMap {
  double radius = 0.0;      ///< m, effective blade radius
  double area = 0.0;        ///< m^2, axial-velocity reference area
  double a = 0.0, b = 0.0;  ///< work-line coefficients
  double kp = 0.0, ki = 0.0;///< profile and incidence loss coefficients
  double phiDesign = 0.0;   ///< design flow coefficient Va/U
  double phiStall = 0.0;    ///< flow coefficient at the declared stall line
};

struct CompressorResult {
  Stream out;
  double torque = 0.0;      ///< N m on the shaft, positive absorbs power
  double dh = 0.0;          ///< J/kg actual total enthalpy rise
  double dhIs = 0.0;        ///< J/kg isentropic rise (negative: pressure loss)
  double efficiency = 0.0;  ///< NaN where dh <= 0
  double flowCoefficient = 0.0;
  double stallMargin = 0.0; ///< (phi - phiStall)/phiStall; infinite at zero speed
  bool valid = false;
};

CompressorResult Compress(const GasProperties& gas, const CompressorMap& map,
                          const Stream& in, double omega);

/** Turbine surrogate: Stodola ellipse flow capacity and a parabolic
    velocity-ratio efficiency, written as torque so it is finite at zero
    speed (stall torque W eta r c0/nu_opt) and braking when overspeeding. */
struct TurbineMap {
  double capacityMax = 0.0;   ///< kg/s sqrt(K)/Pa, limit of W sqrt(Tt)/Pt
  double exponent = 2.0;      ///< Stodola exponent
  double radius = 0.0;        ///< m
  double velocityRatioOptimum = 0.6;
  double peakEfficiency = 0.9;
};

struct TurbineResult {
  Stream out;
  double torque = 0.0;        ///< N m delivered to the shaft
  double dh = 0.0;            ///< J/kg actual total enthalpy drop
  double dhIs = 0.0;
  double pressureRatio = 1.0; ///< Pt_in / Pt_out
  double velocityRatio = 0.0;
  double capacityFraction = 0.0;
  bool overCapacity = false;
  bool valid = false;
};

TurbineResult Expand(const GasProperties& gas, const TurbineMap& map,
                     const Stream& in, double omega);
/** Expansion through a given total-pressure ratio Pt_in/Pt_out >= 1; the
    caller balances the stream's capacity fraction against CapacityFraction. */
TurbineResult ExpandAt(const GasProperties& gas, const TurbineMap& map,
                       const Stream& in, double omega, double pressureRatio);
/** Stodola ellipse: capacity fraction sqrt(1 - PR^-n) passed at a pressure ratio. */
double CapacityFraction(const TurbineMap& map, double pressureRatio);

/** Pressure-loss coefficients of ducts, burners and mixers, scaled with the
    square of the corrected flow relative to design. */
struct LossModel {
  double designCapacity = 0.0;  ///< kg/s sqrt(K)/Pa at design
  double cold = 0.0;
  double hot = 0.0;             ///< multiplies (Tt_out/Tt_in - 1)
};

double PressureLossFraction(const LossModel& loss, const Stream& in, double TtOut);

struct BurnerInput {
  double sprayKgSec = 0.0;      ///< fuel leaving the injectors this instant
  double fuelSensibleH = 0.0;   ///< J/kg of injected fuel
  bool lit = false;
  double efficiency = 1.0;      ///< fraction of burnable fuel converted
  double wallHeatW = 0.0;       ///< heat leaving the gas to solids
};

struct BurnerResult {
  Stream out;
  double burnedKgSec = 0.0;
  double heatReleaseW = 0.0;
  double oxygenLimitedKgSec = 0.0;  ///< burnable fuel refused for lack of oxygen
  double unburnedOutKgSec = 0.0;
  double pressureLoss = 0.0;
  bool valid = false;
};

BurnerResult Burn(const GasProperties& gas, const Stream& in, const BurnerInput& input,
                  const LossModel& loss);

/** Adiabatic mixing of two streams at the lower entry pressure basis:
    Pt is mass weighted and reduced by the loss model. */
Stream Mix(const GasProperties& gas, const Stream& a, const Stream& b, const LossModel& loss,
           bool& valid);

/** Static-to-total pressure ratio, static temperature and velocity of a
    stream passing an area subsonically. Returns false when the area cannot
    pass the flow. */
bool SubsonicStaticState(const GasProperties& gas, const Stream& s, double area,
                         double& pressureRatio, double& Ts, double& V);

struct MixerGeometry {
  double coreArea = 0.0;     ///< m^2 of the core stream at the mixing plane
  double bypassArea = 0.0;   ///< m^2 through which bypass air enters
};

struct MixerResult {
  Stream out;                      ///< mixed stream; friction loss not applied
  double staticPressure = 0.0;     ///< Pa at the mixing plane, set by the core stream
  /// Pa of bypass total pressure that drives the bypass flow into the mixer
  /// (odd extension about the static pressure for negative solver iterates).
  double bypassRequiredPt = 0.0;
  bool bypassChoked = false;
  bool valid = false;
};

/** Constant-area mixing of a core stream with bypass air injected through
    its own area against the core's static pressure. Mass, energy and
    momentum are conserved, so unequal total pressures (a bypass stream
    choked at the mixer when the turbine exit pressure is low) pay their
    mixing loss. A negative bypass flow is accepted only as a smooth
    extension for solver iterates. */
MixerResult MixAtStaticPressure(const GasProperties& gas, const Stream& core,
                                const Stream& bypass, const MixerGeometry& geometry);

/** Adds heat (positive into the gas) at constant pressure. */
bool AddHeat(const GasProperties& gas, Stream& s, double heatW);

enum class NozzleRegime : int {
  NoFlow = 0, Subsonic = 1, ChokedInternalShock = 2, OverexpandedAttached = 3,
  OverexpandedSeparated = 4, UnderexpandedOrIdeal = 5
};

struct NozzleGeometry {
  double throatArea = 0.0;      ///< m^2, geometric
  double exitArea = 0.0;        ///< m^2, geometric; equal to throat for convergent
  double dischargeCoefficient = 1.0;
  double velocityCoefficient = 1.0;  ///< multiplies exit momentum only
  double separationPressureRatio = 0.4; ///< Summerfield wall/ambient criterion
};

struct NozzleResult {
  NozzleRegime regime = NozzleRegime::NoFlow;
  bool choked = false;
  double requiredPt = 0.0;      ///< Pa that passes W at these conditions
  double criticalPressureRatio = 0.0; ///< p*/Pt
  double exitStaticPressure = 0.0;
  double exitStaticTemperature = 0.0;
  double exitVelocity = 0.0;    ///< m/s, ideal before the velocity coefficient
  double exitMach = 0.0;
  double exitDensity = 0.0;
  double exitArea = 0.0;        ///< m^2, flow exit (separation point if separated)
  double grossThrust = 0.0;     ///< N along the nozzle axis
  bool valid = false;
};

/** Total pressure required to pass stream.W (stream.Pt ignored). */
bool NozzleRequiredPressure(const GasProperties& gas, const Stream& stream,
                            const NozzleGeometry& geometry, double ambientPa,
                            NozzleResult& result);
/** Exit state and gross thrust at the stream's actual Pt and W. */
bool NozzleThrust(const GasProperties& gas, const Stream& stream,
                  const NozzleGeometry& geometry, double ambientPa, NozzleResult& result);

/** Sonic state of an isentropic expansion from (Tt, composition). */
bool SonicState(const GasProperties& gas, const Composition& c, double Tt,
                double& Tstar, double& pressureRatio, double& fluxPerPt);
/** Mass flux per unit area per unit Pt at static/total pressure ratio x. */
bool FluxParameter(const GasProperties& gas, const Composition& c, double Tt, double x,
                   double& flux, double& Ts, double& V);

} // namespace TurbinePlant
} // namespace JSBSim

#endif
