#include <algorithm>
#include <cmath>
#include <vector>
#include <cxxtest/TestSuite.h>
#include "TurbinePlantSyntheticFixture.h"

using namespace JSBSim::TurbinePlant;
using namespace TurbinePlantFixtures;

namespace {
constexpr double kDt = 1.0 / 120.0;

GasProperties PerfectAir()
{
  GasProperties g;
  g.Configure(GasModelKind::CaloricallyPerfect, FuelSpec(), 1004.5, 287.0);  // gamma 1.4
  return g;
}

Stream AirStream(const GasProperties& g, double W, double Tt, double Pt)
{
  Stream s;
  s.W = W; s.Tt = Tt; s.Pt = Pt;
  s.ht = g.H(s.c, Tt);
  return s;
}

double RelativeError(double value, double reference)
{
  return std::fabs(value - reference) / std::max(1e-300, std::fabs(reference));
}

// Supersonic or subsonic Mach number for an area ratio A/A*, by bisection.
double MachFromAreaRatio(double ratio, double gamma, bool supersonic)
{
  const auto area = [&](double M) {
    const double e = (gamma + 1.0) / (2.0 * (gamma - 1.0));
    return std::pow(2.0 / (gamma + 1.0) * (1.0 + 0.5 * (gamma - 1.0) * M * M), e) / M;
  };
  double lo = supersonic ? 1.0 : 1e-9, hi = supersonic ? 50.0 : 1.0;
  for (int i = 0; i < 200; ++i) {
    const double mid = 0.5 * (lo + hi);
    const bool tooBig = area(mid) > ratio;
    if (supersonic) { if (tooBig) hi = mid; else lo = mid; }
    else { if (tooBig) lo = mid; else hi = mid; }
  }
  return 0.5 * (lo + hi);
}

struct Trace {
  double time, n1, n2, thrust, abBurned, abSpray, coreBurned, t6, t7;
};

/** Runs a scripted transient and checks every accepted step's ledger. */
class Run {
public:
  Plant plant;
  PlantInputs in = SeaLevel(0.0);
  double worstEnergy = 0.0, worstMass = 0.0;
  int maxIterations = 0;
  bool failed = false;

  explicit Run(bool lift = false) { plant.Load(Synthetic(lift)); }

  bool Step()
  {
    if (!plant.Advance(in)) { failed = true; return false; }
    const PlantOutputs& o = plant.Outputs();
    worstEnergy = std::max(worstEnergy, o.ledger.relative);
    worstMass = std::max(worstMass, o.ledger.massRelative);
    maxIterations = std::max(maxIterations, o.status.iterations);
    return true;
  }
  bool For(double seconds)
  {
    const int n = static_cast<int>(std::lround(seconds / in.dt));
    for (int i = 0; i < n; ++i) if (!Step()) return false;
    return true;
  }
  bool StartRunning(double throttle)
  {
    PlantInputs zero = in;
    zero.throttle = throttle;
    zero.advance = false;
    zero.cutoff = false;
    in.throttle = throttle;
    in.cutoff = false;
    return plant.InitializeRunning(zero);
  }
};
} // namespace

class FGTurbinePlantTest : public CxxTest::TestSuite
{
public:
  // ---------------------------------------------------------------- thermo
  void testTabulatedGasMatchesPolynomials()
  {
    GasProperties poly, table;
    poly.Configure(GasModelKind::ThermallyPerfect, FuelSpec());
    table.Configure(GasModelKind::Tabulated, FuelSpec());
    for (int species = 0; species < 2; ++species) {
      for (double T = 203.7; T < 3490.0; T += 37.3) {
        TS_ASSERT_DELTA(table.SpeciesH(species, T), poly.SpeciesH(species, T), 1.0);
        TS_ASSERT_DELTA(table.SpeciesCp(species, T), poly.SpeciesCp(species, T), 0.05);
        TS_ASSERT_DELTA(table.SpeciesPhi(species, T), poly.SpeciesPhi(species, T), 2e-3);
      }
    }
  }

  void testEnthalpyInversion()
  {
    GasProperties g;
    g.Configure(GasModelKind::ThermallyPerfect, FuelSpec());
    Composition c;
    c.y = {{0.7, 0.28, 0.02}};
    for (double T = 250.0; T < 3300.0; T += 97.0) {
      double back = 0.0;
      TS_ASSERT(g.TemperatureFromH(c, g.H(c, T), 1000.0, back));
      TS_ASSERT_DELTA(back, T, 1e-8 * T);
      TS_ASSERT(g.TemperatureFromPhi(c, g.Phi(c, T), 1000.0, back));
      TS_ASSERT_DELTA(back, T, 1e-8 * T);
    }
    TS_ASSERT_DELTA(g.H(Composition(), GasProperties::kReferenceK), 0.0, 1e-9);
  }

  // ------------------------------------------------------------ components
  void testBurnerConservesEnergyAndElements()
  {
    GasProperties g;
    g.Configure(GasModelKind::ThermallyPerfect, FuelSpec());
    const Stream air = AirStream(g, 50.0, 800.0, 2.5e6);
    const double far = g.StoichiometricFAR();
    for (double phi : {0.3, 0.9, 1.5}) {
      BurnerInput b;
      b.lit = true;
      b.efficiency = 0.97;
      b.sprayKgSec = phi * far * air.W;
      b.fuelSensibleH = g.H(Composition{{{0.0, 0.0, 1.0}}}, 300.0);
      b.wallHeatW = 1e5;
      const BurnerResult r = Burn(g, air, b, LossModel());
      TS_ASSERT(r.valid);
      TS_ASSERT_DELTA(r.out.W, air.W + b.sprayKgSec, 1e-12 * air.W);
      const double energyIn = air.W * air.ht + b.sprayKgSec * b.fuelSensibleH + r.heatReleaseW - b.wallHeatW;
      TS_ASSERT_DELTA(r.out.W * r.out.ht, energyIn, 1e-10 * std::fabs(energyIn));
      TS_ASSERT_DELTA(r.heatReleaseW, r.burnedKgSec * g.Fuel().lowerHeatingValue, 1e-6);
      TS_ASSERT_LESS_THAN_EQUALS(r.burnedKgSec, air.W * far * (1.0 + 1e-12));
      TS_ASSERT_DELTA(r.unburnedOutKgSec + r.burnedKgSec, b.sprayKgSec, 1e-12);
      // Elements: carbon, hydrogen and oxygen leave as they entered.
      const auto element = [&](double (GasProperties::*fraction)(int) const, const Stream& s) {
        double m = 0.0;
        for (int k = 0; k < kSpeciesCount; ++k) m += s.W * s.c.y[k] * (g.*fraction)(k);
        return m;
      };
      const double cIn = element(&GasProperties::CarbonMassFraction, air) + b.sprayKgSec * g.CarbonMassFraction(kFuel);
      const double hIn = element(&GasProperties::HydrogenMassFraction, air) + b.sprayKgSec * g.HydrogenMassFraction(kFuel);
      const double oIn = element(&GasProperties::OxygenMassFraction, air);
      TS_ASSERT_DELTA(element(&GasProperties::CarbonMassFraction, r.out), cIn, 1e-10 * air.W);
      TS_ASSERT_DELTA(element(&GasProperties::HydrogenMassFraction, r.out), hIn, 1e-10 * air.W);
      TS_ASSERT_DELTA(element(&GasProperties::OxygenMassFraction, r.out), oIn, 1e-10 * air.W);
      if (phi > 1.0) TS_ASSERT_DELTA(r.out.c.y[kAir], 0.0, 1e-12);
    }
  }

  void testMixerConservesMassAndEnergy()
  {
    const GasProperties g = PerfectAir();
    const Stream a = AirStream(g, 30.0, 900.0, 3e5), b = AirStream(g, 10.0, 400.0, 2.9e5);
    bool ok = false;
    const Stream m = Mix(g, a, b, LossModel(), ok);
    TS_ASSERT(ok);
    TS_ASSERT_DELTA(m.W, 40.0, 1e-12);
    // Calorically perfect, same composition: mass-weighted temperature.
    TS_ASSERT_DELTA(m.Tt, (30.0 * 900.0 + 10.0 * 400.0) / 40.0, 1e-9);
    TS_ASSERT_DELTA(m.Pt, (30.0 * 3e5 + 10.0 * 2.9e5) / 40.0, 1e-6);
    bool zeroOk = false;
    const Stream z = Mix(g, a, AirStream(g, 0.0, 400.0, 1e5), LossModel(), zeroOk);
    TS_ASSERT(zeroOk);
    TS_ASSERT_DELTA(z.Tt, a.Tt, 1e-9);
  }

  void testConvergentNozzleChokedAnalytic()
  {
    const GasProperties g = PerfectAir();
    const double gamma = 1.4, R = 287.0, Tt = 900.0, Pt = 3e5, A = 0.2, p0 = 101325.0;
    const double Wc = A * Pt * std::sqrt(gamma / (R * Tt))
      * std::pow(2.0 / (gamma + 1.0), (gamma + 1.0) / (2.0 * (gamma - 1.0)));
    NozzleGeometry geom;
    geom.throatArea = geom.exitArea = A;
    geom.dischargeCoefficient = 1.0; geom.velocityCoefficient = 1.0;
    NozzleResult r;
    TS_ASSERT(NozzleThrust(g, AirStream(g, Wc, Tt, Pt), geom, p0, r));
    TS_ASSERT(r.choked);
    TS_ASSERT_LESS_THAN(RelativeError(r.requiredPt, Pt), 1e-8);
    const double Tstar = 2.0 * Tt / (gamma + 1.0);
    const double pstar = Pt * std::pow(2.0 / (gamma + 1.0), gamma / (gamma - 1.0));
    const double Vstar = std::sqrt(gamma * R * Tstar);
    TS_ASSERT_LESS_THAN(RelativeError(r.exitVelocity, Vstar), 1e-8);
    TS_ASSERT_LESS_THAN(RelativeError(r.grossThrust, Wc * Vstar + (pstar - p0) * A), 1e-8);
    TS_ASSERT_LESS_THAN(RelativeError(r.exitMach, 1.0), 1e-8);
  }

  void testConvergentNozzleSubsonicAnalytic()
  {
    const GasProperties g = PerfectAir();
    const double gamma = 1.4, R = 287.0, Tt = 600.0, p0 = 101325.0, Pt = 1.3 * p0, A = 0.3;
    const double M = std::sqrt(2.0 / (gamma - 1.0) * (std::pow(Pt / p0, (gamma - 1.0) / gamma) - 1.0));
    const double T = Tt / (1.0 + 0.5 * (gamma - 1.0) * M * M);
    const double V = M * std::sqrt(gamma * R * T);
    const double W = p0 / (R * T) * V * A;
    NozzleGeometry geom;
    geom.throatArea = geom.exitArea = A;
    geom.dischargeCoefficient = 1.0; geom.velocityCoefficient = 1.0;
    NozzleResult r;
    TS_ASSERT(NozzleThrust(g, AirStream(g, W, Tt, 0.0), geom, p0, r));
    TS_ASSERT(!r.choked);
    TS_ASSERT_EQUALS(static_cast<int>(r.regime), static_cast<int>(NozzleRegime::Subsonic));
    TS_ASSERT_LESS_THAN(RelativeError(r.requiredPt, Pt), 1e-8);
    TS_ASSERT_LESS_THAN(RelativeError(r.grossThrust, W * V), 1e-8);
  }

  void testDivergentNozzleIdealAndSeparatedAnalytic()
  {
    const GasProperties g = PerfectAir();
    const double gamma = 1.4, R = 287.0, Tt = 1500.0, Pt = 4e5, At = 0.2, ratio = 1.4;
    const double Wc = At * Pt * std::sqrt(gamma / (R * Tt))
      * std::pow(2.0 / (gamma + 1.0), (gamma + 1.0) / (2.0 * (gamma - 1.0)));
    const double Me = MachFromAreaRatio(ratio, gamma, true);
    const double Te = Tt / (1.0 + 0.5 * (gamma - 1.0) * Me * Me);
    const double pe = Pt * std::pow(Tt / Te, -gamma / (gamma - 1.0));
    const double Ve = Me * std::sqrt(gamma * R * Te);
    NozzleGeometry geom;
    geom.throatArea = At; geom.exitArea = At * ratio;
    geom.dischargeCoefficient = 1.0; geom.velocityCoefficient = 1.0;
    NozzleResult r;
    TS_ASSERT(NozzleThrust(g, AirStream(g, Wc, Tt, Pt), geom, pe, r));
    TS_ASSERT_EQUALS(static_cast<int>(r.regime), static_cast<int>(NozzleRegime::UnderexpandedOrIdeal));
    TS_ASSERT_LESS_THAN(RelativeError(r.exitMach, Me), 1e-8);
    TS_ASSERT_LESS_THAN(RelativeError(r.grossThrust, Wc * Ve), 1e-8);
    // Underexpanded: pressure thrust on the geometric exit.
    TS_ASSERT(NozzleThrust(g, AirStream(g, Wc, Tt, Pt), geom, 0.5 * pe, r));
    TS_ASSERT_LESS_THAN(RelativeError(r.grossThrust, Wc * Ve + 0.5 * pe * At * ratio), 1e-8);
    // Strongly overexpanded but still choked: Summerfield separation at 0.4 p0.
    const double p0 = 2.0 * pe / 0.4 * 0.9;   // p9 below 0.4 p0 and above the exit-shock pressure
    TS_ASSERT(NozzleThrust(g, AirStream(g, Wc, Tt, Pt), geom, p0, r));
    if (r.regime == NozzleRegime::OverexpandedSeparated) {
      const double ps = 0.4 * p0;
      const double Ts = Tt * std::pow(ps / Pt, (gamma - 1.0) / gamma);
      const double Ms = std::sqrt(2.0 / (gamma - 1.0) * (Tt / Ts - 1.0));
      const double As = At * std::pow(2.0 / (gamma + 1.0) * (1.0 + 0.5 * (gamma - 1.0) * Ms * Ms),
                                      (gamma + 1.0) / (2.0 * (gamma - 1.0))) / Ms;
      const double Vs = Ms * std::sqrt(gamma * R * Ts);
      TS_ASSERT_LESS_THAN(RelativeError(r.grossThrust, Wc * Vs + (ps - p0) * As), 1e-8);
    } else {
      TS_ASSERT_EQUALS(static_cast<int>(r.regime), static_cast<int>(NozzleRegime::ChokedInternalShock));
    }
  }

  void testTurbomachineryIsFiniteAtZeroSpeed()
  {
    const GasProperties g = PerfectAir();
    CompressorMap c;
    c.radius = 0.3; c.area = 0.2; c.a = 0.75; c.b = 0.6; c.kp = 0.4; c.ki = 0.8; c.phiDesign = 0.5;
    const Stream in = AirStream(g, 5.0, 288.15, 101325.0);
    const CompressorResult cr = Compress(g, c, in, 0.0);
    TS_ASSERT(cr.valid);
    const double Va = in.W / (in.Pt / (287.0 * in.Tt) * c.area);
    TS_ASSERT_LESS_THAN(RelativeError(cr.torque, -in.W * c.radius * c.b * Va), 1e-12);
    TS_ASSERT_LESS_THAN(cr.out.Pt, in.Pt);          // stationary row is a pressure loss
    const CompressorResult spun = Compress(g, c, in, 800.0);
    TS_ASSERT_LESS_THAN(RelativeError(spun.torque * 800.0, in.W * spun.dh), 1e-12);

    TurbineMap t;
    t.capacityMax = 2e-3; t.radius = 0.35; t.velocityRatioOptimum = 0.6; t.peakEfficiency = 0.9;
    const Stream hot = AirStream(g, 40.0, 1600.0, 2e6);
    const TurbineResult stall = Expand(g, t, hot, 0.0);
    TS_ASSERT(stall.valid);
    const double c0 = std::sqrt(2.0 * stall.dhIs);
    TS_ASSERT_LESS_THAN(RelativeError(stall.torque, hot.W * 0.9 * 0.35 * c0 / 0.6), 1e-12);
    TS_ASSERT_DELTA(stall.dh, 0.0, 1e-9);
    const TurbineResult turning = Expand(g, t, hot, 900.0);
    TS_ASSERT_LESS_THAN(RelativeError(turning.torque * 900.0, hot.W * turning.dh), 1e-12);
  }

  // ------------------------------------------------------------- the plant
  void testDesignPointIsSelfConsistent()
  {
    Plant p;
    TS_ASSERT(p.Load(Synthetic()));
    const DerivedDesign& d = p.Design();
    TS_ASSERT(d.valid);
    TS_ASSERT_DELTA(d.fanFlow, 100.0, 1e-9);
    // Mixer balance chose the fan ratio: equal total pressures at the mixer.
    TS_ASSERT_LESS_THAN(RelativeError(d.stationPt[kSt5], d.stationPt[kSt16]), 1e-9);
    Run run;
    TS_ASSERT(run.StartRunning(1.0));
    const PlantOutputs& o = run.plant.Outputs();
    TS_ASSERT_DELTA(o.n1Percent, 100.0, 1e-3);
    TS_ASSERT_LESS_THAN(RelativeError(o.nozzle.grossThrust, d.thrust), 1e-3);
    TS_ASSERT_DELTA(o.station[kSt4].Tt, 1800.0, 1.0);
  }

  void testSteadyStatesMatchSettledTransient()
  {
    Run run;
    TS_ASSERT(run.StartRunning(1.0));
    PlantInputs zero = run.in;
    zero.advance = false;
    zero.throttle = 0.0;
    TS_ASSERT(run.plant.Steady(zero));        // mil to idle needs the demand continuation
    const double idleThrust = run.plant.Outputs().nozzle.grossThrust;
    const double idleN2 = run.plant.Outputs().n2Percent;
    run.in.throttle = 0.0;
    TS_ASSERT(run.For(10.0));
    TS_ASSERT_LESS_THAN(RelativeError(run.plant.Outputs().nozzle.grossThrust, idleThrust), 2e-3);
    TS_ASSERT_DELTA(run.plant.Outputs().n2Percent, idleN2, 0.05);
  }

  void testColdStartToAfterburnerConservesMassAndEnergy()
  {
    Run run;
    run.in.starter = true;
    run.in.cutoff = true;
    std::vector<int> codes;
    for (int i = 0; i < static_cast<int>(70.0 / kDt) && !run.failed; ++i) {
      const double t = i * kDt;
      const PlantState& s = run.plant.State();
      if (s.omegaHP / 1500.0 > 0.2) run.in.cutoff = false;
      if (s.running) run.in.starter = false;
      run.in.throttle = t > 20.0 ? 1.0 : 0.0;
      run.in.augmentDemand = t > 35.0 && t < 50.0 ? 1.0 : 0.0;
      if (t > 60.0) { run.in.cutoff = true; run.in.throttle = 0.0; }
      run.Step();
    }
    TS_ASSERT(!run.failed);
    TS_ASSERT_LESS_THAN_EQUALS(run.worstEnergy, 1e-5);
    TS_ASSERT_LESS_THAN_EQUALS(run.worstMass, 1e-6);
    TS_ASSERT_LESS_THAN_EQUALS(run.maxIterations, run.plant.settings.iterationCap);
    const Ledger& L = run.plant.Outputs().ledger;
    TS_ASSERT_LESS_THAN(std::fabs(L.cumulativeResidualJ) / L.cumulativeScaleJ, 1e-6);
    TS_ASSERT_LESS_THAN(std::fabs(L.cumulativeMassResidualKg) / L.cumulativeMassScaleKg, 1e-9);
    // Lifecycle order: lit, running, AB selected, AB lit, AB deselected, AB out, core out, stopped.
    for (int e = 0; e < run.plant.EventCount(); ++e) codes.push_back(run.plant.Events()[e].code);
    const std::vector<int> expected{1, 5, 7, 3, 8, 4, 2, 6};
    TS_ASSERT_EQUALS(codes.size(), expected.size());
    for (std::size_t k = 0; k < std::min(codes.size(), expected.size()); ++k)
      TS_ASSERT_EQUALS(codes[k], expected[k]);
  }

  // The legacy engine published reheat thrust for 4.1333 s while burned AB
  // fuel was zero (0sfs docs/validation/f135-ab-onset.md). The replacement
  // contract: unburned augmentor fuel can at most leave at the jet velocity,
  // so until AB fuel burns the thrust exceeds that of the same attained state
  // without AB spray by no more than the spray's momentum.
  void testAugmentorThrustNeedsBurnedFuel()
  {
    Run run;
    TS_ASSERT(run.StartRunning(1.0));
    TS_ASSERT(run.For(1.0));
    const double dryThrust = run.plant.Outputs().nozzle.grossThrust;
    run.in.augmentDemand = 1.0;
    bool burned = false;
    double firstBurnTime = -1.0, litEventTime = -1.0;
    for (int i = 0; i < 600 && !run.failed; ++i) {
      run.Step();
      const PlantOutputs& o = run.plant.Outputs();
      if (o.abBurnedKgSec > 0.0) {
        if (!burned) firstBurnTime = run.plant.time;
        burned = true;
        continue;
      }
      TS_ASSERT_EQUALS(o.abHeatReleaseW, 0.0);
      PlantState dry = run.plant.State();
      dry.ab.spray = 0.0;
      dry.abFlame = FlameState();
      PlantInputs zero = run.in;
      zero.advance = false;
      PlantOutputs reference;
      TS_ASSERT(run.plant.Evaluate(dry, zero, reference));
      const double sprayMomentum = o.abSprayKgSec * o.nozzle.exitVelocity;
      TS_ASSERT_LESS_THAN_EQUALS(o.nozzle.grossThrust - reference.nozzle.grossThrust,
                                 sprayMomentum + 1e-6 * dryThrust);
      // No temperature rise across the augmentor without a reaction source
      // (the liner here is colder than the gas, so it only removes heat).
      TS_ASSERT_LESS_THAN_EQUALS(o.station[kSt7].Tt, o.station[kSt6].Tt + 1e-9);
    }
    TS_ASSERT(burned);
    for (int e = 0; e < run.plant.EventCount(); ++e)
      if (run.plant.Events()[e].code == 3) litEventTime = run.plant.Events()[e].time;
    TS_ASSERT_LESS_THAN(0.0, litEventTime);
    // Burning starts on the step after the accepted light-off event.
    TS_ASSERT_DELTA(firstBurnTime - litEventTime, kDt, 1e-9);
    TS_ASSERT(run.For(5.0));
    TS_ASSERT_LESS_THAN(1.3 * dryThrust, run.plant.Outputs().nozzle.grossThrust);
  }

  // Changing only the selection labels, at an identical attained physical
  // state, cannot change the evaluated force or source.
  void testAugmentorLabelsDoNotChangeEvaluation()
  {
    Run run;
    TS_ASSERT(run.StartRunning(1.0));
    run.in.augmentDemand = 1.0;
    TS_ASSERT(run.For(0.6));
    PlantState s = run.plant.State();
    PlantInputs zero = run.in;
    zero.advance = false;
    PlantOutputs a, b;
    TS_ASSERT(run.plant.Evaluate(s, zero, a));
    s.control.abSelected = !s.control.abSelected;
    s.control.abFuelAir = 0.0;
    zero.augmentDemand = 0.0;
    TS_ASSERT(run.plant.Evaluate(s, zero, b));
    TS_ASSERT_EQUALS(a.nozzle.grossThrust, b.nozzle.grossThrust);
    TS_ASSERT_EQUALS(a.abHeatReleaseW, b.abHeatReleaseW);
    TS_ASSERT_EQUALS(a.station[kSt7].Tt, b.station[kSt7].Tt);
  }

  // Negative control: after an AB cutoff the hot liner returns heat to the
  // gas, so station 7 can stay above station 6 while no AB fuel burns. That
  // energy must come from solid storage, never be forbidden by fiat.
  void testHotCutoffRetainsStoredHeat()
  {
    Run run;
    TS_ASSERT(run.StartRunning(1.0));
    run.in.augmentDemand = 1.0;
    TS_ASSERT(run.For(20.0));
    run.in.augmentDemand = 0.0;
    bool sawStoredHeat = false;
    for (int i = 0; i < 600 && !run.failed; ++i) {
      run.Step();
      const PlantOutputs& o = run.plant.Outputs();
      if (o.abBurnedKgSec > 0.0 || o.abLit) continue;
      if (o.station[kSt7].Tt > o.station[kSt6].Tt) {
        sawStoredHeat = true;
        TS_ASSERT_LESS_THAN(o.ledger.solidStoredJ, 0.0);
        TS_ASSERT_LESS_THAN(o.solidGasHeatW[0], 0.0);
      }
    }
    TS_ASSERT(sawStoredHeat);
    TS_ASSERT_LESS_THAN_EQUALS(run.worstEnergy, 1e-5);
  }

  void testFailedIgnitionLeavesFuelUnburned()
  {
    PlantConfig c = Synthetic();
    c.coreCombustion.leanIgnitionEquivalence = 5.0;   // never ignitable
    c.coreCombustion.richLimitEquivalence = 6.0;
    Run run;
    run.plant.Load(c);
    run.in.starter = true;
    run.in.cutoff = false;
    TS_ASSERT(run.For(15.0));
    const PlantOutputs& o = run.plant.Outputs();
    TS_ASSERT(!o.coreLit);
    TS_ASSERT_LESS_THAN(0.0, o.coreSprayKgSec);
    TS_ASSERT_EQUALS(o.coreHeatReleaseW, 0.0);
    TS_ASSERT_DELTA(o.unburnedExhaustKgSec, o.coreSprayKgSec, 1e-9);
    TS_ASSERT_EQUALS(run.plant.EventCount(), 0);
    TS_ASSERT_LESS_THAN_EQUALS(run.worstMass, 1e-6);
  }

  void testStarvationDebitsExactlyTheSupply()
  {
    Run run;
    TS_ASSERT(run.StartRunning(1.0));
    const double supply = 0.5;   // kg
    double remaining = supply;
    for (int i = 0; i < 240 && !run.failed; ++i) {
      run.in.fuelSupplyKg = remaining;
      const double before = run.plant.State().fuelUsedKg;
      run.Step();
      remaining = std::max(0.0, remaining - (run.plant.State().fuelUsedKg - before));
    }
    TS_ASSERT(!run.failed);
    TS_ASSERT_DELTA(run.plant.State().fuelUsedKg, supply, 1e-12);
    TS_ASSERT(!run.plant.State().coreFlame.lit);
    bool starved = false, out = false;
    for (int e = 0; e < run.plant.EventCount(); ++e) {
      starved |= run.plant.Events()[e].code == 9;
      out |= run.plant.Events()[e].code == 2;
    }
    TS_ASSERT(starved && out);
  }

  void testEngineAtRestStaysAtRest()
  {
    Run run;
    TS_ASSERT(run.For(1.0));
    const PlantOutputs& o = run.plant.Outputs();
    TS_ASSERT(o.valid);
    TS_ASSERT_EQUALS(run.plant.State().omegaHP, 0.0);
    TS_ASSERT_EQUALS(run.plant.State().omegaLP, 0.0);
    TS_ASSERT_DELTA(o.nozzle.grossThrust, 0.0, 1e-9);
  }

  void testSeizureDissipatesRotorEnergy()
  {
    Run run;
    TS_ASSERT(run.StartRunning(0.5));
    TS_ASSERT(run.For(0.5));
    run.in.seized = true;
    TS_ASSERT(run.Step());
    TS_ASSERT_EQUALS(run.plant.State().omegaHP, 0.0);
    TS_ASSERT(!run.plant.State().coreFlame.lit || run.plant.Outputs().coreSprayKgSec == 0.0);
    TS_ASSERT_LESS_THAN_EQUALS(run.worstEnergy, 1e-5);
  }

  void testRestoreReproducesTheTrajectory()
  {
    Run a;
    TS_ASSERT(a.StartRunning(0.3));
    a.in.throttle = 1.0;
    TS_ASSERT(a.For(0.5));
    const PlantState saved = a.plant.State();
    const double savedTime = a.plant.time;
    TS_ASSERT(a.For(2.0));
    Run b;
    std::string reason;
    TS_ASSERT(b.plant.RestoreState(saved, &reason));
    b.plant.time = savedTime;
    b.in = a.in;
    TS_ASSERT(b.For(2.0));
    // Warm-start data is not restored, so iterates differ; the state agrees to the tolerance.
    TS_ASSERT_DELTA(b.plant.State().omegaLP, a.plant.State().omegaLP, 1e-5 * 1000.0);
    TS_ASSERT_LESS_THAN(RelativeError(b.plant.Outputs().nozzle.grossThrust,
                                      a.plant.Outputs().nozzle.grossThrust), 1e-5);
    PlantState bad = saved;
    bad.omegaLP = -1.0;
    TS_ASSERT(!b.plant.RestoreState(bad, &reason));
  }

  void testZeroTimeCallsAreIdempotent()
  {
    Run run;
    TS_ASSERT(run.StartRunning(0.5));
    PlantInputs zero = run.in;
    zero.advance = false;
    TS_ASSERT(run.plant.Advance(zero));
    const PlantState first = run.plant.State();
    const double thrust = run.plant.Outputs().nozzle.grossThrust;
    TS_ASSERT(run.plant.Advance(zero));
    TS_ASSERT_DELTA(run.plant.State().omegaLP, first.omegaLP, 1e-9 * first.omegaLP);
    TS_ASSERT_DELTA(run.plant.Outputs().nozzle.grossThrust, thrust, 1e-7 * thrust);
    // A stopped engine refreshes outputs without changing its state.
    Run cold;
    PlantInputs z = cold.in;
    z.advance = false;
    TS_ASSERT(cold.plant.Advance(z));
    TS_ASSERT_EQUALS(cold.plant.State().omegaHP, 0.0);
  }

  // The controller samples once per accepted step from the previous step's
  // sensors, so its error is first order and need not fall monotonically;
  // the gate is the output envelope against a converged fine-step reference.
  void testTimestepConvergence()
  {
    const double rates[4] = {60.0, 120.0, 240.0, 1920.0};
    double n1[4][2], thrust[4][2];
    double scale = 0.0;
    for (int k = 0; k < 4; ++k) {
      Run run;
      run.in.dt = 1.0 / rates[k];
      TS_ASSERT(run.StartRunning(0.0));
      scale = run.plant.Design().thrust;
      run.in.throttle = 1.0;
      for (int m = 0; m < 2; ++m) {
        TS_ASSERT(run.For(m == 0 ? 0.5 : 1.0));
        n1[k][m] = run.plant.Outputs().n1Percent;
        thrust[k][m] = run.plant.Outputs().nozzle.grossThrust;
      }
    }
    for (int k = 0; k < 3; ++k) {
      for (int m = 0; m < 2; ++m) {
        TS_ASSERT_LESS_THAN(std::fabs(n1[k][m] - n1[3][m]), 0.5);
        TS_ASSERT_LESS_THAN(std::fabs(thrust[k][m] - thrust[3][m]), 0.01 * scale);
      }
    }
  }

  void testReducedAlgorithmTracksComponentReference()
  {
    Run reduced, component;
    component.plant.settings.requested = Algorithm::Component;
    for (Run* r : {&reduced, &component}) {
      TS_ASSERT(r->StartRunning(0.0));
      r->in.throttle = 1.0;
    }
    const double scale = reduced.plant.Design().thrust;
    double worstForce = 0.0, worstT = 0.0;
    for (int i = 0; i < 360; ++i) {
      if (i == 240) { reduced.in.augmentDemand = component.in.augmentDemand = 1.0; }
      TS_ASSERT(reduced.Step() && component.Step());
      const PlantOutputs& a = reduced.plant.Outputs();
      const PlantOutputs& b = component.plant.Outputs();
      worstForce = std::max(worstForce, std::fabs(a.nozzle.grossThrust - b.nozzle.grossThrust) / scale);
      worstT = std::max(worstT, std::fabs(a.station[kSt7].Tt - b.station[kSt7].Tt));
    }
    TS_ASSERT_LESS_THAN(worstForce, 0.01);
    TS_ASSERT_LESS_THAN(worstT, 5.0);
    TS_ASSERT_EQUALS(reduced.plant.Outputs().status.algorithm, static_cast<int>(Algorithm::Reduced));
  }

  void testLiftSystemAccountsEveryOutletOnce()
  {
    Run run(true);
    TS_ASSERT(run.StartRunning(0.6));
    run.in.clutchCommand = 1.0;
    run.in.doorCommand = 1.0;
    run.in.rollPostCommand = {{1.0, 1.0}};
    TS_ASSERT(run.For(6.0));
    const PlantOutputs& o = run.plant.Outputs();
    TS_ASSERT(!run.failed);
    TS_ASSERT_LESS_THAN(0.0, o.liftFan.grossThrust);
    TS_ASSERT_LESS_THAN(0.0, o.rollPost[0].grossThrust);
    TS_ASSERT_DELTA(o.rollPost[0].massFlow + o.rollPost[1].massFlow,
                    o.station[kSt13].W - o.station[kSt16].W, 1e-9);
    TS_ASSERT_DELTA(o.netThrustN, o.nozzle.grossThrust + o.liftFan.grossThrust
                    + o.rollPost[0].grossThrust + o.rollPost[1].grossThrust - o.ramDragN, 1e-6);
    TS_ASSERT_LESS_THAN_EQUALS(run.worstEnergy, 1e-5);
    TS_ASSERT_LESS_THAN_EQUALS(run.worstMass, 1e-6);
  }

  void testSettingsBounds()
  {
    NumericalSettings s;
    TS_ASSERT(ValidSettings(s));
    s.iterationCap = 0;
    TS_ASSERT(!ValidSettings(s));
    s = NumericalSettings();
    s.tolerance = 1e-2;
    TS_ASSERT(!ValidSettings(s));
    Plant p;
    TS_ASSERT(p.Load(Synthetic()));
    p.settings.closureBudgetBytes = 65536.0;
    TS_ASSERT(p.SelectAlgorithm(Algorithm::Reduced));   // 21 KiB of tables fit
    TS_ASSERT(p.SelectAlgorithm(Algorithm::Component));
  }
};
