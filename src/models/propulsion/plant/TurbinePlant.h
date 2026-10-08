/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 Header:       TurbinePlant.h
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

#ifndef TURBINEPLANT_H
#define TURBINEPLANT_H

#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "TurbinePlantComponents.h"

namespace JSBSim {
namespace TurbinePlant {

/** Schema of the plant's published snapshot, state record and XML. */
constexpr int kSchemaVersion = 1;

/** Piecewise-linear schedule, clamped at its ends. Allocated at load only. */
struct Table1D {
  std::vector<double> x, y;
  double operator()(double v) const;
  bool Valid() const;
};

/** Station identifiers follow SAE ARP755 numbering. */
enum Station : int {
  kSt0 = 0, kSt2, kSt21, kSt13, kSt16, kSt25, kSt3, kSt31, kSt4, kSt41, kSt45, kSt5,
  kSt6, kSt7, kSt8, kSt9, kStLiftFanIn, kStLiftFanOut, kStationCount
};
const char* StationName(int station);

enum class Algorithm : int { Component = 0, Reduced = 1 };

/** A thermal solid coupled to a gas station and a coolant stream. */
struct SolidConfig {
  std::string name;
  double capacity = 0.0;         ///< J/K
  int gasStation = kSt7;         ///< kSt5 or kSt7
  int coolant = 0;               ///< 0 none, 1 bypass air (station 16), 2 cooling air (station 31)
  double gasConductance = 0.0;   ///< W/K at design gas flow
  double coolantConductance = 0.0; ///< W/K at design coolant flow
  double flowExponent = 0.8;     ///< conductance proportional to (W/Wdesign)^n
  double minimumConductanceFraction = 0.02;
  double radiationArea = 0.0;    ///< m^2 to the surroundings
  double emissivity = 0.0;
};

struct CompressorSpec {
  double pressureRatio = 0.0, efficiency = 0.0;
  double workCoefficient = 0.45;  ///< design dh/U^2 (lumped)
  double flowCoefficient = 0.5;   ///< design Va/U
  double workLineSlope = 0.6;     ///< b phiDesign / psiDesign
  double incidenceLossFactor = 2.0; ///< ki / kp
  double stallFlowFraction = 0.75;
};

struct TurbineSpec {
  double efficiency = 0.0;
  double velocityRatio = 0.6;
  double stodolaExponent = 2.0;
};

struct FuelPathConfig {
  double valveTimeConstant = 0.05;   ///< s, metering actuator lag
  double valveRateLimit = 10.0;      ///< kg/s/s
  double maxFlow = 10.0;             ///< kg/s
  double manifoldPrimeMass = 0.0;    ///< kg filled before injectors deliver
  double manifoldTimeConstant = 0.02;///< s, primed manifold response
  double drainTimeConstant = 0.0;    ///< s, 0 disables drain to dump when shut off
};

struct CombustionConfig {
  double ignitionDelay = 0.2;        ///< s of continuous ignitable conditions
  /// s for the flame to spread from the igniter to every injector (annular
  /// light-around, or across the augmentor flameholders); burning fuel is the
  /// lit fraction times efficiency times burnable fuel.
  double lightAroundTime = 0.1;
  double leanIgnitionEquivalence = 0.15;
  double leanBlowoutEquivalence = 0.08;
  double richLimitEquivalence = 1.6;
  double minimumPressure = 0.0;      ///< Pa, below which no flame holds
  double minimumInletTemperature = 0.0; ///< K, AB vaporization requirement
  // Core efficiency: eta = etaMax / (1 + (theta50/theta)^s), theta = loading/design loading.
  double maximumEfficiency = 0.995;
  double halfEfficiencyLoading = 0.01;
  double loadingExponent = 1.0;
  double constantEfficiency = 0.0;   ///< AB: used when > 0
};

struct ControllerConfig {
  Table1D fanSpeedDemand;      ///< corrected N1 fraction vs throttle 0..1
  Table1D accelerationLimit;   ///< ratio units (Wf/P3 / design) vs corrected N2 fraction
  Table1D decelerationLimit;
  Table1D startSchedule;       ///< ratio units during light-off and start vs corrected N2
  Table1D dryThroatArea;       ///< throat area / design throat area vs corrected N1 fraction
  Table1D abFuelAirRatio;      ///< AB fuel/air (on augmentor inlet air) vs AB demand 0..1
  double proportionalGain = 3.0;   ///< ratio units per unit N1 error
  double integralGain = 2.0;       ///< ratio units per unit N1 error second
  double idleCorrectedN2 = 0.6;
  double idleGain = 5.0;
  double maxN2 = 1.05;             ///< physical fraction
  /// The N2 limit shares the governor integrator; the HP spool answers fuel
  /// fast, so a gain much above 10 limit-cycles against the N1 loop.
  double maxN2Gain = 5.0;
  double maxTurbineExitK = 1e9;    ///< station 5 limit
  double temperatureGain = 0.01;   ///< ratio units per K
  double lightoffN2 = 0.15;        ///< physical fraction for igniter enable
  double starterCutoutN2 = 0.5;
  double runningN2Margin = 0.01;   ///< Running when N2 >= idle - margin (corrected)
  double abMinimumN2 = 0.95;
  double abLightoffFuelAirRatio = 0.005;
  double abRampRate = 0.02;        ///< fuel/air per second when lit
  double abPreopenFraction = 0.05;
  /// Throat area multiplier with the lift fan fully loaded (clutch engaged,
  /// nozzle door open; it scales with their product): opening the nozzle
  /// lowers turbine back pressure so the LP turbine can drive the fan.
  double liftThroatAreaFactor = 1.0;
  double liftSplitGain = 12.0;     ///< 1/s, thrust-split loop on relative split error
  double liftSplitWashout = 1.0;   ///< 1/s, rate at which the throat takes over from closed vanes
  double liftVaneMin = 0.4;        ///< lowest guide-vane work multiplier
  /// LP speed limit (physical fraction) held with the lift fan engaged by
  /// opening the vanes and closing the throat; per second per unit excess.
  double maxN1 = 1e9, maxN1Gain = 100.0;
  double throatAreaMin = 0.0, throatAreaMax = 0.0; ///< fractions of the design throat area
  double throatAreaRate = 0.5;     ///< design throat areas per second
  double throatAreaTimeConstant = 0.1;
  Table1D exitAreaRatio;           ///< A9/A8 vs normalized throat position 0..1
  double egtSensorTimeConstant = 1.0; ///< s, first-order gauge on station 5
};

struct ShaftConfig {
  double inertia = 0.0;            ///< kg m^2
  double designSpeed = 0.0;        ///< rad/s at 100 percent
  double coulombTorque = 0.0;      ///< N m
  double coulombSpeed = 1.0;       ///< rad/s smoothing
  double viscousCoefficient = 0.0; ///< N m s
  double aerodynamicCoefficient = 0.0; ///< N m s^2
};

struct StarterConfig {
  double stallTorque = 0.0;        ///< N m at zero speed
  double freeSpeed = 1.0;          ///< rad/s at zero torque
};

/** Optional shaft-driven lift fan with clutch and bypass-fed roll posts. */
struct LiftSystemConfig {
  bool present = false;
  ShaftConfig fanShaft;
  double gearRatio = 1.0;          ///< fan speed per LP speed when locked
  double clutchCapacity = 0.0;     ///< N m at the fan shaft
  double clutchSlipSpeed = 1.0;    ///< rad/s, tanh smoothing
  double clutchRate = 1.0;         ///< engagement fraction per second
  CompressorSpec fan;              ///< design PR/efficiency/shape at the design fan speed
  double designFlow = 0.0;         ///< kg/s
  double inletRecovery = 0.98;
  NozzleGeometry nozzle;           ///< convergent fan nozzle at full door opening
  double doorRate = 1.0;           ///< fraction per second
  std::array<double, 2> rollPostArea{{0.0, 0.0}}; ///< m^2 full open
  double rollPostDischarge = 0.95;
  double rollPostVelocityCoefficient = 0.97;
  double rollPostRate = 2.0;       ///< fraction per second
  /// Bypass entry area into the mixer at full clutch engagement, as a
  /// fraction of its design area. A declared surrogate for however the real
  /// engine holds its fan operating line while the nozzle opens in lift mode:
  /// a smaller entry chokes the bypass stream, so fan pressure and core
  /// supercharge stay up as turbine exit pressure falls. 1 disables it.
  double bypassEntryAreaFactor = 1.0;
};

struct DesignSpec {
  double ambientPressure = 101325.0, ambientTemperature = 288.15;
  double massFlow = 0.0;           ///< kg/s; 0 means size from designThrust
  double designThrust = 0.0;       ///< N static gross at design if massFlow is 0
  double bypassRatio = 0.0;
  double fanPressureRatio = 0.0;   ///< 0 means solve from mixer balance
  double overallPressureRatio = 0.0; ///< used when fanPressureRatio is solved
  CompressorSpec fan, compressor;
  TurbineSpec hpTurbine, lpTurbine;
  double turbineInletTemperature = 0.0;
  double inletRecovery = 0.99;
  double coolingFraction = 0.0;
  double combustorPressureLoss = 0.05, combustorHotLossFraction = 0.3;
  double bypassPressureLoss = 0.02, mixerPressureLoss = 0.01;
  /// Design static/total pressure ratio of the core stream at the mixing
  /// plane (0.95 is about Mach 0.27); sizes the core and bypass entry areas.
  double mixerStaticRatio = 0.95;
  double augmentorDryPressureLoss = 0.03, augmentorHotLossCoefficient = 0.03;
  double designCombustionEfficiency = 0.995;
  double accessoryPower = 0.0;     ///< W on the HP shaft at design speed
  double nozzleDischarge = 0.98, nozzleVelocityCoefficient = 0.985;
  double nozzleSeparationRatio = 0.4;
};

struct PlantConfig {
  std::string name;
  std::uint64_t digest = 0;        ///< hash of the configuration text
  GasModelKind referenceGas = GasModelKind::ThermallyPerfect;
  double perfectCp = 1004.5, perfectR = 287.05; ///< calorically perfect fixtures only
  FuelSpec fuel;
  double fuelTemperature = 300.0;
  DesignSpec design;
  ShaftConfig lp, hp;
  StarterConfig starter;
  FuelPathConfig coreFuel, abFuel;
  CombustionConfig coreCombustion, abCombustion;
  ControllerConfig control;
  LiftSystemConfig lift;
  std::vector<SolidConfig> solids;
  bool augmented = false;
};

/** Derived at load from the design point; immutable afterwards. */
struct DerivedDesign {
  CompressorMap fan, compressor, liftFan;
  TurbineMap hpt, lpt;
  LossModel combustor, bypass, mixer, augmentor;
  double coreFlow = 0.0, bypassFlow = 0.0, fanFlow = 0.0;
  double inletCapacity = 1.0;      ///< design W sqrt(Tt0)/Pt0
  double fuelFlow = 0.0;           ///< kg/s at design
  double ratioUnit = 0.0;          ///< design Wf/P3, kg/s/Pa
  double throatArea = 0.0;         ///< m^2 design A8
  double throatMin = 0.0, throatMax = 0.0, throatRate = 0.0; ///< m^2, m^2/s
  double thrust = 0.0;             ///< N design gross
  double lpTorque = 0.0, hpTorque = 0.0; ///< N m design turbine torque scales
  double liftFanTorque = 0.0;
  double fanPressureRatio = 0.0;
  double compressorPressureRatio = 0.0;
  double loading = 0.0;            ///< design combustor loading parameter
  double gaugeRatio = 0.0;         ///< design Pt7/p0 - 1: pressure residual scale
  MixerGeometry mixerGeometry;     ///< core and bypass entry areas at the mixing plane
  double bypassCoolantFlow = 0.0, coolingFlow = 0.0;
  std::array<double, kStationCount> stationPt{}, stationTt{}, stationW{};
  bool valid = false;
  std::string failure;
};

struct FuelPathState {
  double metered = 0.0;    ///< kg/s actual metering valve output (tank debit rate)
  double manifold = 0.0;   ///< kg in the manifold
  double spray = 0.0;      ///< kg/s leaving the injectors at the accepted end state
};

struct FlameState {
  bool lit = false;
  double progress = 0.0;   ///< s of continuous ignitable conditions
  double fraction = 0.0;   ///< lit fraction of the burner, 0..1
};

struct ControllerState {
  double integrator = 0.0;     ///< ratio units
  double abFuelAir = 0.0;      ///< attained AB command, fuel/air
  bool abSelected = false;
  bool igniter = false;
  double egtGaugeK = 0.0;
  double sensedP3 = 0.0, sensedT5 = 0.0, sensedT6 = 0.0, sensedT7 = 0.0;
  double sensedW3b = 0.0, sensedAir6 = 0.0, sensedW6 = 0.0;
  double sensedMainThrust = 0.0, sensedLiftThrust = 0.0; ///< N, gross
  double splitTrim = 1.0;      ///< nozzle area multiplier held by the thrust-split loop
  double splitVane = 1.0;      ///< lift-fan guide-vane work multiplier (fast split effector), 1 open
  int limiter = 0;             ///< 0 N1, 1 accel, 2 decel, 3 N2 max, 4 T5 max, 5 idle, 6 start, 7 off
};

struct SolidState {
  double temperature = 0.0;
  bool initialized = false;
  bool warmPending = false;
};

/** Canonical physical and controller state shared by every algorithm. */
struct PlantState {
  double omegaLP = 0.0, omegaHP = 0.0, omegaLF = 0.0;
  double clutch = 0.0, door = 0.0;
  std::array<double, 2> rollPost{{0.0, 0.0}};
  FuelPathState core, ab;
  FlameState coreFlame, abFlame;
  double throatArea = 0.0;
  ControllerState control;
  std::vector<SolidState> solids;
  bool running = false;
  bool starter = false;
  double fuelUsedKg = 0.0;     ///< cumulative metered fuel debited (not frozen)
  /// Numerical warm start, not physical history: last accepted core,
  /// bypass, mixer-bypass and lift-fan flows (kg/s), then the natural logs
  /// of the HP and LP turbine total-pressure ratios.
  std::array<double, 6> flows{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
};

/** Inputs sampled once per accepted step; never re-evaluated in a trial. */
struct PlantInputs {
  double dt = 0.0;
  bool advance = false;        ///< false: zero-time evaluation
  double ambientPressure = 101325.0, ambientTemperature = 288.15;
  double totalPressure = 101325.0, totalTemperature = 288.15;
  double airspeed = 0.0, mach = 0.0;
  double throttle = 0.0;       ///< dry lever 0..1
  double augmentDemand = 0.0;  ///< 0..1, already zero when inhibited upstream
  bool starter = false, cutoff = true, windmillAir = false;
  bool stalled = false, seized = false;
  double fuelSupplyKg = INFINITY; ///< deliverable this step at the current priority
  bool fuelFrozen = false;
  double clutchCommand = 0.0, doorCommand = 0.0;
  std::array<double, 2> rollPostCommand{{0.0, 0.0}};
  /// Multiplies the scheduled throat area (an open-loop nozzle trim).
  double throatAreaTrim = 1.0;
  /// Commanded ratio of lift-fan to main-nozzle gross thrust while the lift
  /// fan is engaged. The controller holds it with two effectors: lift-fan
  /// guide vanes act at once (they can only reduce fan thrust), and the
  /// throat area follows, shifting LP turbine work between the lift fan and
  /// the jet until the vanes are open again. 0 disables.
  double liftSplit = 0.0;
};

/** Solver controls exposed to the application; one home per setting. */
struct NumericalSettings {
  Algorithm requested = Algorithm::Reduced;
  /// 1..32 total Newton iterations per accepted step, all solves and fallback
  /// included. Default 24: each accepted step needs a midpoint and an
  /// end-state solve, and the first step of an engine at rest in moving air
  /// (flow growing as the square root of a tiny ram pressure) needs about 12
  /// component iterations after the reduced attempt has spent its share.
  int iterationCap = 24;
  int subdivisionCap = 4;          ///< 1..16 substeps on failure
  double tolerance = 1e-6;         ///< 1e-9..1e-3 scaled residual
  double closureBudgetBytes = 1048576.0; ///< 64 KiB..16 MiB
};

struct StationObservation {
  double W = 0.0, Tt = 0.0, Pt = 0.0, fuelAir = 0.0, unburnedFraction = 0.0;
};

struct OutletObservation {
  double massFlow = 0.0, grossThrust = 0.0, exitVelocity = 0.0, exitMach = 0.0;
  double exitStaticPressure = 0.0, exitStaticTemperature = 0.0, exitArea = 0.0;
  int regime = 0;
  bool choked = false;
};

/** Energy and mass ledger of one accepted step and since the last reset. */
struct Ledger {
  // Signed per step, J or kg. Positive enters the installation control volume.
  double airInJ = 0.0, fuelSensibleInJ = 0.0, chemicalJ = 0.0, starterJ = 0.0;
  double outflowJ = 0.0, accessoryJ = 0.0, frictionJ = 0.0, clutchJ = 0.0;
  double radiationJ = 0.0, dumpJ = 0.0;
  double rotorStoredJ = 0.0, solidStoredJ = 0.0, manifoldStoredJ = 0.0;
  double residualJ = 0.0, scaleJ = 0.0, relative = 0.0;
  double airInKg = 0.0, fuelInKg = 0.0, outKg = 0.0, dumpKg = 0.0, manifoldKg = 0.0;
  double massResidualKg = 0.0, massRelative = 0.0;
  double unburnedOutKg = 0.0, burnedKg = 0.0;
  double cumulativeResidualJ = 0.0, cumulativeScaleJ = 0.0;
  double cumulativeMassResidualKg = 0.0, cumulativeMassScaleKg = 0.0;
  bool valid = false;
};

struct NumericalStatus {
  int algorithm = 0;           ///< algorithm that produced the accepted solution
  int iterations = 0, residualEvaluations = 0, substeps = 0;
  double maxResidual = 0.0;
  bool converged = false;
  int fallbackReason = 0;      ///< 0 none, 1 iteration cap, 2 domain, 3 nonfinite, 4 budget, 5 no steady point (zero-time)
  double nozzleBackoff = 1.0;  ///< 1 normally; below 1 while the nozzle command is being held back
  bool failure = false;        ///< latched numerical failure; outputs invalid
  unsigned long domainClamps = 0;
  double residentBytes = 0.0;
  double sequence = 0.0;       ///< accepted-step sequence
  double epoch = 0.0;          ///< reset/restore epoch
  double time = 0.0;           ///< plant time of the published state, s
  double stepSeconds = 0.0;
  bool lpOverCapacity = false, hpOverCapacity = false;
  double fanStallMargin = 0.0, compressorStallMargin = 0.0;
  bool reverseBypassBlocked = false;
};

struct PlantOutputs {
  std::array<StationObservation, kStationCount> station{};
  OutletObservation nozzle, liftFan;
  std::array<OutletObservation, 2> rollPost{};
  double throatArea = 0.0, exitArea = 0.0, nozzlePosition = 0.0;
  double ramDragN = 0.0, capturedAirKgSec = 0.0, netThrustN = 0.0;
  double inletRamDragN = 0.0, liftFanRamDragN = 0.0; ///< parts of ramDragN, at each inlet
  double n1Percent = 0.0, n2Percent = 0.0, liftFanPercent = 0.0;
  double lpTurbineTorque = 0.0, fanTorque = 0.0, hpTurbineTorque = 0.0, compressorTorque = 0.0;
  double clutchTorque = 0.0, liftFanTorque = 0.0, liftFanPowerW = 0.0, starterTorque = 0.0;
  double coreCommandKgSec = 0.0, abCommandKgSec = 0.0;
  double coreMeteredKgSec = 0.0, coreSprayKgSec = 0.0, coreBurnedKgSec = 0.0;
  double abMeteredKgSec = 0.0, abSprayKgSec = 0.0, abBurnedKgSec = 0.0;
  double coreHeatReleaseW = 0.0, abHeatReleaseW = 0.0, coreEfficiency = 0.0;
  double totalMeteredKgSec = 0.0, unburnedExhaustKgSec = 0.0, dumpKgSec = 0.0;
  double coreEquivalence = 0.0, abEquivalence = 0.0;
  bool coreLit = false, abLit = false, abSelected = false, igniter = false;
  double egtGaugeK = 0.0;
  std::vector<double> solidGasHeatW, solidCoolantHeatW, solidRadiationW;
  Ledger ledger;
  NumericalStatus status;
  bool valid = false;
};

/** Event history entry, bounded ring buffer. */
struct PlantEvent {
  double time = 0.0;
  int code = 0;  ///< 1 core lit, 2 core out, 3 AB lit, 4 AB out, 5 running, 6 stopped, 7 AB selected, 8 AB deselected, 9 starved, 10 numerical failure
};

class Plant {
public:
  Plant();
  ~Plant();
  Plant(const Plant&) = delete;
  Plant& operator=(const Plant&) = delete;
  /** Derives the design; returns false with Design().failure on error. */
  bool Load(const PlantConfig& config);
  const PlantConfig& Config() const { return config_; }
  const DerivedDesign& Design() const { return design_; }

  /** Explicit cold state at the given ambient. Resets the epoch. */
  void InitializeCold(double ambientTemperature);
  /** Running operating point at the inputs' controls; solids marked warm-pending
      unless already initialized. Returns false if no supported equilibrium. */
  bool InitializeRunning(const PlantInputs& inputs);

  /** Advance accepted elapsed time, or a zero-time Steady/Refresh by policy.
      Returns false on a numerical failure; the last accepted state is kept. */
  bool Advance(const PlantInputs& inputs);
  /** Steady operating point for an eligible engine at fixed slow histories. */
  bool Steady(const PlantInputs& inputs);
  /** Recompute outputs from unchanged histories. */
  bool Refresh(const PlantInputs& inputs);

  /** Pure evaluation for causality tests: outputs from a state and inputs
      without changing the plant. */
  bool Evaluate(const PlantState& state, const PlantInputs& inputs, PlantOutputs& out) const;

  const PlantState& State() const { return state_; }
  /** Replaces the state after validation; does not refresh. */
  bool RestoreState(const PlantState& state, std::string* reason = nullptr);
  /** Establishes one solid at a restored temperature (K), as the legacy
      metal-temperature-state property does; false if invalid. */
  bool SetSolidTemperature(std::size_t index, double kelvin);
  const PlantOutputs& Outputs() const { return out_; }
  PlantOutputs& MutableOutputs() { return out_; }

  NumericalSettings settings;
  /** Events since the last reset, at most kEventCapacity retained. */
  static constexpr int kEventCapacity = 64;
  const std::array<PlantEvent, kEventCapacity>& Events() const { return events_; }
  int EventCount() const { return eventCount_; }
  void ClearNumericalFailure() { out_.status.failure = false; }

  /** Validates a numerical setting change and applies it at zero time. */
  bool SelectAlgorithm(Algorithm algorithm);

  double time = 0.0;
private:
  struct Operating;            // per-evaluation inputs to the gas path
  struct GasPath;              // per-evaluation solution
  struct StepCommands { double core = 0.0, ab = 0.0, starterTorque = 0.0, throat = 0.0; };

  PlantConfig config_;
  DerivedDesign design_;
  GasProperties referenceGas_, tabulatedGas_;
  PlantState state_;
  PlantOutputs out_;
  std::array<PlantEvent, kEventCapacity> events_{};
  int eventCount_ = 0;
  double sequence_ = 0.0, epoch_ = 0.0;
  // Numerical warm-start data (reconstructible; not physical history).
  std::array<double, 100> jacobian_{}, endJacobian_{};
  bool jacobianValid_ = false, endJacobianValid_ = false;
  int jacobianSize_ = 0;
  double lastAccelLP_ = 0.0, lastAccelHP_ = 0.0, lastNorm_ = 0.0, lastStarterTorque_ = 0.0;
  double stepElapsed_ = 0.0;
  double flowSpeedLP_ = 0.0, flowSpeedHP_ = 0.0;  ///< speeds of the stored warm-start flows
  bool lastStarved_ = false;
  bool bypassBlocked_ = false;   ///< mixer junction regime of the stored solution
  /// Fraction of the commanded nozzle move taken this step: quartered after
  /// a step that could not be solved, doubled after each accepted one, so a
  /// nozzle commanded beyond where the gas path has a solution stops there
  /// instead of freezing the engine.
  double nozzleBackoff_ = 1.0;
  // Preallocated working storage: no allocation on an accepted step.
  PlantState trial_, retry_, previous_;
  Ledger stepLedger_;
  std::unique_ptr<GasPath> midpointStorage_, endStorage_;

  const GasProperties& Gas(Algorithm a) const {
    return a == Algorithm::Reduced ? tabulatedGas_ : referenceGas_;
  }
  bool DeriveDesign();
  void RecordEvent(double t, int code);
  void UpdateController(const PlantInputs& in, PlantState& s, double dt, double& coreCmd, double& abCmd, double& starterTorque, double& throatCmd);
  bool EvaluatePath(const Operating& op, const double* x, GasPath& g, double* residual) const;
  void OperatingFromState(const PlantState& s, const PlantInputs& in, Algorithm algorithm,
                          Operating& op) const;
  double ExitAreaRatio(double throatArea) const;
  void ScaleFlowsBySpeed(double* x, double wLP, double wHP) const;
  bool SolveAt(Operating& op, const std::array<double, 5>& guess, Algorithm algorithm, GasPath& g,
               double* J, bool& jValid, double tolerance, int cap, int& iterations,
               int& evaluations, double& norm) const;
  bool StepOnce(const PlantInputs& in, const StepCommands& cmd, double dt, Algorithm algorithm,
                int& iterations, int& evaluations, int cap);
  bool PublishEnd(const PlantInputs& in, Algorithm algorithm, bool updateSensors, int& iterations,
                  int& evaluations, int cap);
  void Fill(const GasPath& g, const Operating& op, const PlantState& s, PlantOutputs& out) const;
  bool SolveSteady(const PlantInputs& in, bool thermalEquilibrium);
};

/** Validates numeric ranges of settings; returns false if out of bounds. */
bool ValidSettings(const NumericalSettings& settings);

/** Flat numeric record of a PlantState for staged capture and restore
    through properties: kStateFixedFields named fields, then a temperature
    (K) and an initialized flag for each solid. Booleans are 0 or 1. The
    record carries the warm-start flows so a restored step starts where the
    captured one would have. */
constexpr int kStateFixedFields = 46;
inline int StateRecordSize(std::size_t solids) { return kStateFixedFields + 2 * static_cast<int>(solids); }
const char* StateRecordName(int index);
void PackState(const PlantState& state, double* record);
/** Fills state from a record; false if a flag is not 0/1 or a value is not finite. */
bool UnpackState(const double* record, std::size_t solids, PlantState& state);

} // namespace TurbinePlant
} // namespace JSBSim

#endif
