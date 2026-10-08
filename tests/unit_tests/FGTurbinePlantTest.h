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

/** Sea-level ISA flight at true airspeed V (m/s): static and total conditions. */
void Flight(PlantInputs& in, double V)
{
  const double a = std::sqrt(1.4 * 287.05 * in.ambientTemperature);
  const double ratio = 1.0 + 0.2 * (V / a) * (V / a);
  in.airspeed = V;
  in.mach = V / a;
  in.totalTemperature = in.ambientTemperature * ratio;
  in.totalPressure = in.ambientPressure * std::pow(ratio, 3.5);
}

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

  void testMomentumMixerConservesMassEnergyAndMomentum()
  {
    const GasProperties g = PerfectAir();
    const double cp = 1004.5, R = 287.0;
    const MixerGeometry geometry{0.25, 0.12};
    const double area = geometry.coreArea + geometry.bypassArea;
    // Uniform streams at one flux mix without loss.
    const Stream core = AirStream(g, 60.0, 700.0, 2.5e5);
    const Stream matched = AirStream(g, 60.0 * geometry.bypassArea / geometry.coreArea, 700.0, 2.5e5);
    const MixerResult uniform = MixAtStaticPressure(g, core, matched, geometry);
    TS_ASSERT(uniform.valid);
    TS_ASSERT_LESS_THAN(RelativeError(uniform.out.Pt, core.Pt), 1e-9);
    TS_ASSERT_DELTA(uniform.out.Tt, core.Tt, 1e-9);
    TS_ASSERT_LESS_THAN(RelativeError(uniform.bypassRequiredPt, matched.Pt), 1e-9);
    // Unequal streams: mass, energy and impulse are conserved and entropy rises.
    const Stream hot = AirStream(g, 60.0, 900.0, 2.5e5), cold = AirStream(g, 20.0, 400.0, 2.7e5);
    const MixerResult m = MixAtStaticPressure(g, hot, cold, geometry);
    TS_ASSERT(m.valid);
    TS_ASSERT_DELTA(m.out.W, 80.0, 1e-12);
    TS_ASSERT_LESS_THAN(RelativeError(m.out.ht, (60.0 * hot.ht + 20.0 * cold.ht) / 80.0), 1e-12);
    double x, Ts, V;
    TS_ASSERT(SubsonicStaticState(g, hot, geometry.coreArea, x, Ts, V));
    const double Ps = x * hot.Pt;
    TS_ASSERT_LESS_THAN(RelativeError(m.staticPressure, Ps), 1e-12);
    NozzleGeometry entry;
    entry.throatArea = entry.exitArea = geometry.bypassArea;
    NozzleResult jet;
    TS_ASSERT(NozzleThrust(g, cold, entry, Ps, jet));
    const double impulseIn = hot.W * V + jet.grossThrust + Ps * area;
    double xo, To, Vo;
    TS_ASSERT(SubsonicStaticState(g, m.out, area, xo, To, Vo));
    TS_ASSERT_LESS_THAN(RelativeError(m.out.W * Vo + xo * m.out.Pt * area, impulseIn), 1e-9);
    const auto entropy = [&](const Stream& st) { return cp * std::log(st.Tt) - R * std::log(st.Pt); };
    TS_ASSERT_LESS_THAN(60.0 * entropy(hot) + 20.0 * entropy(cold), 80.0 * entropy(m.out));
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

  void testSteadyStartAwayFromTheDesignAmbient()
  {
    // 1500 m at Mach 0.4: the steady solve continues from the design point
    // across flight condition and must agree with the settled transient.
    Run run;
    const double p0 = 84556.0, T0 = 278.4, M = 0.4, ram = 1.0 + 0.2 * M * M;
    run.in.ambientPressure = p0; run.in.ambientTemperature = T0;
    run.in.mach = M; run.in.airspeed = M * std::sqrt(1.4 * 287.05 * T0);
    run.in.totalTemperature = T0 * ram; run.in.totalPressure = p0 * std::pow(ram, 3.5);
    TS_ASSERT(run.StartRunning(0.7));
    const double n1 = run.plant.Outputs().n1Percent, thrust = run.plant.Outputs().nozzle.grossThrust;
    TS_ASSERT(run.For(10.0));
    TS_ASSERT_DELTA(run.plant.Outputs().n1Percent, n1, 0.05);
    TS_ASSERT_LESS_THAN(RelativeError(run.plant.Outputs().nozzle.grossThrust, thrust), 2e-3);
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
  void testAugmentorSelectableAtAColdInlet()
  {
    // At -50 C a full-power engine turns below the augmentor's minimum as a
    // physical speed, but not as a corrected one, which is what the limit is.
    Run run;
    run.in.ambientTemperature = run.in.totalTemperature = 223.15;
    run.in.ambientPressure = run.in.totalPressure = 30000.0;
    TS_ASSERT(run.StartRunning(1.0));
    run.in.augmentDemand = 1.0;
    TS_ASSERT(run.For(5.0));
    const PlantOutputs& o = run.plant.Outputs();
    const double theta = std::sqrt(223.15 / 288.15);
    TS_ASSERT_LESS_THAN(o.n2Percent / 100.0, run.plant.Config().control.abMinimumN2);
    TS_ASSERT_LESS_THAN(run.plant.Config().control.abMinimumN2, o.n2Percent / 100.0 / theta);
    TS_ASSERT(o.abSelected);
    TS_ASSERT_LESS_THAN(0.0, o.abBurnedKgSec);
    PlantInputs zero = run.in;
    zero.advance = false;
    TS_ASSERT(run.plant.Steady(zero));
    TS_ASSERT(run.plant.Outputs().abSelected);
    TS_ASSERT(!run.failed);
  }

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

  void testColdSolidsFollowTheLocalAmbient()
  {
    Plant p;
    TS_ASSERT(p.Load(Synthetic()));
    PlantInputs in = SeaLevel(0.0);
    in.ambientTemperature = in.totalTemperature = 250.0;
    in.advance = false;
    TS_ASSERT(p.Advance(in));   // zero-time, at rest: not established, at the local ambient
    for (const SolidState& solid : p.State().solids) {
      TS_ASSERT_EQUALS(solid.temperature, 250.0);
      TS_ASSERT(!solid.initialized);
    }
    in.ambientTemperature = in.totalTemperature = 260.0;
    in.advance = true;
    TS_ASSERT(p.Advance(in));   // the first accepted step establishes them where they stand
    for (const SolidState& solid : p.State().solids) {
      TS_ASSERT(solid.initialized);
      TS_ASSERT_DELTA(solid.temperature, 260.0, 0.5);
    }
    in.ambientTemperature = in.totalTemperature = 300.0;
    TS_ASSERT(p.Advance(in));   // established solids exchange heat; they do not jump
    for (const SolidState& solid : p.State().solids) TS_ASSERT_LESS_THAN(solid.temperature, 261.0);
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
    TS_ASSERT_DELTA(o.ramDragN, o.inletRamDragN + o.liftFanRamDragN, 1e-9);
    TS_ASSERT_LESS_THAN_EQUALS(run.worstEnergy, 1e-5);
    TS_ASSERT_LESS_THAN_EQUALS(run.worstMass, 1e-6);
  }

  void testThrustSplitIsHeldByGuideVanesAndThroat()
  {
    Run run(true);
    TS_ASSERT(run.StartRunning(0.9));
    run.in.clutchCommand = 1.0;
    run.in.doorCommand = 1.0;
    run.in.rollPostCommand = {{1.0, 1.0}};
    const auto ratio = [&run]() {
      const PlantOutputs& o = run.plant.Outputs();
      return o.liftFan.grossThrust / o.nozzle.grossThrust;
    };
    // Below the split the fan gives with the throat at its minimum, the
    // guide vanes take out the rest; the steady solve finds the same point.
    run.in.liftSplit = 0.3;
    TS_ASSERT(run.For(10.0));
    TS_ASSERT_DELTA(ratio(), 0.3, 1e-3);
    const double vane = run.plant.State().control.splitVane;
    TS_ASSERT_LESS_THAN(vane, 0.95);
    TS_ASSERT_LESS_THAN(RelativeError(run.plant.Outputs().throatArea, run.plant.Design().throatMin), 1e-9);
    PlantInputs zero = run.in;
    zero.advance = false;
    TS_ASSERT(run.plant.Steady(zero));
    TS_ASSERT_DELTA(ratio(), 0.3, 1e-6);
    TS_ASSERT_DELTA(run.plant.State().control.splitVane, vane, 0.01);
    // Inside the throat range the vanes reopen and the throat holds the split.
    run.in.liftSplit = 0.42;
    TS_ASSERT(run.For(10.0));
    TS_ASSERT_DELTA(ratio(), 0.42, 0.01);
    TS_ASSERT_LESS_THAN(0.99, run.plant.State().control.splitVane);
    zero = run.in;
    zero.advance = false;
    TS_ASSERT(run.plant.Steady(zero));
    TS_ASSERT_DELTA(ratio(), 0.42, 1e-6);
    TS_ASSERT_EQUALS(run.plant.State().control.splitVane, 1.0);
    // A cut in the split is taken at once by the vanes, not the spool.
    run.in.liftSplit = 0.37;
    TS_ASSERT(run.For(0.5));
    TS_ASSERT_LESS_THAN(std::fabs(ratio() - 0.37), 0.3 * 0.05);
    TS_ASSERT_LESS_THAN(run.plant.State().control.splitVane, 1.0);
    TS_ASSERT(!run.failed);
    TS_ASSERT_LESS_THAN_EQUALS(run.worstEnergy, 1e-5);
    TS_ASSERT_LESS_THAN_EQUALS(run.worstMass, 1e-6);
  }

  void testCutOffLiftFanSpoolsDownSmoothly()
  {
    // After cutoff the split loop gives up its authority as the lift fan's
    // thrust falls through 2 % to 1 % of design thrust: no switching on and
    // off, so the outlet thrust falls on every step, and no rotor gains energy.
    Run run(true);
    TS_ASSERT(run.StartRunning(0.9));
    run.in.clutchCommand = 1.0;
    run.in.doorCommand = 1.0;
    run.in.liftSplit = 0.42;
    TS_ASSERT(run.For(8.0));
    run.in.cutoff = true;
    TS_ASSERT(run.For(0.5));
    const auto rotorEnergy = [&run]() {
      const PlantState& s = run.plant.State();
      const PlantConfig& c = run.plant.Config();
      return 0.5 * (c.lp.inertia * s.omegaLP * s.omegaLP + c.hp.inertia * s.omegaHP * s.omegaHP
                    + c.lift.fanShaft.inertia * s.omegaLF * s.omegaLF);
    };
    double thrust = run.plant.Outputs().liftFan.grossThrust, energy = rotorEnergy();
    int rises = 0;
    for (int i = 0; i < 1200; ++i) {
      TS_ASSERT(run.Step());
      const double now = run.plant.Outputs().liftFan.grossThrust;
      rises += now > thrust;
      TS_ASSERT_LESS_THAN_EQUALS(rotorEnergy(), energy);
      thrust = now;
      energy = rotorEnergy();
    }
    TS_ASSERT_EQUALS(rises, 0);
    TS_ASSERT_LESS_THAN(thrust, 0.01 * run.plant.Design().thrust);
  }

  void testSteadyConversionAtForwardSpeed()
  {
    // From conventional flight the steady solve reaches the converted point
    // at any airspeed. It starts with the lift fan stopped, where ram air
    // already passes the open nozzle, so the residual must be continuous
    // in fan speed there.
    for (const double V : {0.0, 10.0, 40.0, 80.0}) {
      Run run(true);
      Flight(run.in, V);
      TS_ASSERT(run.StartRunning(0.9));
      TS_ASSERT_EQUALS(run.plant.Outputs().liftFan.grossThrust, 0.0);
      PlantInputs zero = run.in;
      zero.advance = false;
      zero.clutchCommand = 1.0;
      zero.doorCommand = 1.0;
      zero.liftSplit = 0.42;
      TS_ASSERT(run.plant.Steady(zero));
      const PlantOutputs& o = run.plant.Outputs();
      TS_ASSERT_DELTA(o.liftFan.grossThrust / o.nozzle.grossThrust, 0.42, 1e-6);
      TS_ASSERT_EQUALS(run.plant.State().clutch, 1.0);
    }
  }

  void testStoppedLiftFanPassesRamFlow()
  {
    // Nozzle open, clutch released, in forward flight: ram air flows through
    // the stopped fan and windmills it; the engine gives it no power.
    Run run(true);
    Flight(run.in, 60.0);
    TS_ASSERT(run.StartRunning(0.6));
    run.in.doorCommand = 1.0;
    TS_ASSERT(run.For(0.5));
    const PlantOutputs& o = run.plant.Outputs();
    TS_ASSERT_EQUALS(run.plant.State().clutch, 0.0);
    TS_ASSERT_EQUALS(o.clutchTorque, 0.0);
    TS_ASSERT_LESS_THAN(0.0, o.liftFan.massFlow);
    TS_ASSERT_LESS_THAN(0.0, run.plant.State().omegaLF);
    TS_ASSERT(!run.failed);
    TS_ASSERT_LESS_THAN_EQUALS(run.worstEnergy, 1e-5);
    TS_ASSERT_LESS_THAN_EQUALS(run.worstMass, 1e-6);
  }

  void testLpSpeedLimitHoldsWhileTheFanUnloads()
  {
    // Closing the lift-fan door with the split following it unloads the LP
    // spool faster than fuel can answer; the limit gives the fan its load
    // back and closes the throat.
    constexpr double kMargin = 0.004;
    double peak[2];
    for (int k = 0; k < 2; ++k) {
      PlantConfig cfg = Synthetic(true);
      Run run(true);
      run.plant.Load(cfg);
      TS_ASSERT(run.StartRunning(0.9));
      run.in.clutchCommand = 1.0;
      run.in.doorCommand = 1.0;
      run.in.liftSplit = 0.42;
      TS_ASSERT(run.For(8.0));
      const double n1 = run.plant.Outputs().n1Percent / 100.0;
      if (k == 1) {
        cfg.control.maxN1 = n1 + kMargin;
        PlantState keep = run.plant.State();
        run.plant.Load(cfg);
        TS_ASSERT(run.plant.RestoreState(keep));
      }
      peak[k] = 0.0;
      run.in.doorCommand = 0.0;
      for (int i = 0; i < 240 && !run.failed; ++i) {
        run.in.liftSplit = 0.42 * run.plant.State().door;
        run.Step();
        peak[k] = std::max(peak[k], run.plant.Outputs().n1Percent / 100.0 - n1);
      }
      TS_ASSERT(!run.failed);
      TS_ASSERT_LESS_THAN_EQUALS(run.worstEnergy, 1e-5);
    }
    TS_ASSERT_LESS_THAN(2.0 * kMargin, peak[0]);
    TS_ASSERT_LESS_THAN(peak[1] - kMargin, 0.5 * (peak[0] - kMargin));
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

  void testRefreshReportsTheAlgorithmAndWorkOfItsPublicationFallback()
  {
    Run run;
    TS_ASSERT(run.StartRunning(0.8));
    TS_ASSERT(run.Step());
    TS_ASSERT_EQUALS(run.plant.Outputs().status.algorithm, static_cast<int>(Algorithm::Reduced));
    const PlantState before = run.plant.State();
    const double time = run.plant.time;
    const double sequence = run.plant.Outputs().status.sequence;
    // A changed atmosphere requires more than one Newton iteration. Refresh
    // retries with its existing component budget, without integrating history.
    PlantInputs refresh = run.in;
    refresh.ambientPressure = refresh.totalPressure = 80000.0;
    refresh.ambientTemperature = refresh.totalTemperature = 270.0;
    run.plant.settings.iterationCap = 1;
    TS_ASSERT(run.plant.Refresh(refresh));
    const NumericalStatus& st = run.plant.Outputs().status;
    TS_ASSERT_EQUALS(st.algorithm, static_cast<int>(Algorithm::Component));
    TS_ASSERT_EQUALS(st.stepAlgorithm, -1);
    TS_ASSERT_EQUALS(st.fallbackReason, 1);
    TS_ASSERT_EQUALS(st.publicationFallbackReason, 1);
    TS_ASSERT_EQUALS(st.stepIterations, 0);
    TS_ASSERT_EQUALS(st.iterations, st.publicationIterations);
    TS_ASSERT_LESS_THAN(1, st.publicationIterations);
    TS_ASSERT(st.converged);
    TS_ASSERT(!st.failure);
    TS_ASSERT_EQUALS(st.sequence, sequence);
    TS_ASSERT_EQUALS(run.plant.time, time);
    std::vector<double> oldRecord(StateRecordSize(before.solids.size()));
    std::vector<double> newRecord(oldRecord.size());
    PackState(before, oldRecord.data());
    PackState(run.plant.State(), newRecord.data());
    for (std::size_t i = 0; i < oldRecord.size(); ++i) TS_ASSERT_EQUALS(newRecord[i], oldRecord[i]);
    // A later ordinary refresh clears this call's fallback receipt.
    run.plant.settings.iterationCap = 24;
    TS_ASSERT(run.plant.Refresh(refresh));
    TS_ASSERT_EQUALS(st.algorithm, static_cast<int>(Algorithm::Reduced));
    TS_ASSERT_EQUALS(st.fallbackReason, 0);
    TS_ASSERT_EQUALS(st.publicationFallbackReason, 0);
  }

  void testFailedPublicationCountsItsWorkAndKeepsTheAcceptedState()
  {
    Run run(true);
    run.in.dt = 0.05;
    TS_ASSERT(run.StartRunning(0.9));
    run.in.clutchCommand = run.in.doorCommand = 1.0;
    run.in.rollPostCommand = {{1.0, 1.0}};
    run.in.liftSplit = 0.42;
    TS_ASSERT(run.plant.Steady(run.in));
    const PlantState before = run.plant.State();
    const double time = run.plant.time;
    const double sequence = run.plant.Outputs().status.sequence;
    // The split cut integrates within four iterations, then publication
    // exhausts the remaining one. No integrated state may leak through.
    run.in.liftSplit = 0.3;
    run.in.throttle = 1.0;
    run.plant.settings.iterationCap = 5;
    TS_ASSERT(!run.plant.Advance(run.in));
    const NumericalStatus& st = run.plant.Outputs().status;
    TS_ASSERT(st.failure);
    TS_ASSERT(!st.converged);
    TS_ASSERT(!run.plant.Outputs().valid);
    TS_ASSERT(!run.plant.Outputs().ledger.valid);
    TS_ASSERT_EQUALS(st.stepAlgorithm, -1);
    TS_ASSERT_LESS_THAN(0, st.stepIterations);
    TS_ASSERT_LESS_THAN(0, st.publicationIterations);
    TS_ASSERT_EQUALS(st.iterations, st.stepIterations + st.publicationIterations);
    TS_ASSERT_EQUALS(st.iterations, run.plant.settings.iterationCap);
    TS_ASSERT_EQUALS(st.fallbackReason, 1);
    TS_ASSERT_EQUALS(st.sequence, sequence);
    TS_ASSERT_EQUALS(run.plant.time, time);
    TS_ASSERT_EQUALS(st.nozzleBackoff, 0.25);
    std::vector<double> oldRecord(StateRecordSize(before.solids.size()));
    std::vector<double> newRecord(oldRecord.size());
    PackState(before, oldRecord.data());
    PackState(run.plant.State(), newRecord.data());
    for (std::size_t i = 0; i < oldRecord.size(); ++i) TS_ASSERT_EQUALS(newRecord[i], oldRecord[i]);
    // Retry is bounded and advances exactly one accepted step.
    run.plant.settings.iterationCap = 24;
    run.plant.ClearNumericalFailure();
    TS_ASSERT(run.plant.Advance(run.in));
    TS_ASSERT(st.converged);
    TS_ASSERT_EQUALS(st.sequence, sequence + 1.0);
    TS_ASSERT_EQUALS(st.iterations, st.stepIterations + st.publicationIterations);
    TS_ASSERT_LESS_THAN_EQUALS(st.iterations, run.plant.settings.iterationCap);
    TS_ASSERT_EQUALS(st.nozzleBackoff, 0.5);
    TS_ASSERT_LESS_THAN_EQUALS(run.plant.Outputs().throatCommand, run.plant.Design().throatMax);
    TS_ASSERT_LESS_THAN_EQUALS(run.plant.Design().throatMin, run.plant.Outputs().throatCommand);
  }

  void testFullDemandLiftEquilibriumSurvivesBothSpoolPerturbationsForSixtySeconds()
  {
    // Synthetic engine, not F135 identification: this full-demand point is
    // admitted under the N2 limiter. Engineering gates are <.001 percentage
    // point spool ripple and <.01% design-thrust ripple in the final 30 s,
    // <.01 percentage point final speed error, and independent ledger bounds.
    for (const Algorithm algorithm : {Algorithm::Reduced, Algorithm::Component}) {
      for (const double perturbation : {0.0, -0.005, 0.005}) {
        Run run(true);
        run.plant.SelectAlgorithm(algorithm);
        run.in.clutchCommand = run.in.doorCommand = 1.0;
        run.in.rollPostCommand = {{1.0, 1.0}};
        run.in.liftSplit = 0.42;
        TS_ASSERT(run.StartRunning(1.0));
        const PlantOutputs equilibrium = run.plant.Outputs();
        PlantState displaced = run.plant.State();
        displaced.omegaLP *= 1.0 + perturbation;
        displaced.omegaHP *= 1.0 + perturbation;
        displaced.omegaLF *= 1.0 + perturbation;
        TS_ASSERT(run.plant.RestoreState(displaced));
        std::array<double, 4> low{{1e99, 1e99, 1e99, 1e99}}, high{{0, 0, 0, 0}};
        for (int i = 0; i < 7200; ++i) {
          const bool accepted = run.Step();
          TS_ASSERT(accepted);
          if (!accepted) break;
          const PlantOutputs& out = run.plant.Outputs();
          const NumericalStatus& st = out.status;
          TS_ASSERT_EQUALS(st.stepAlgorithm, static_cast<int>(algorithm));
          TS_ASSERT_EQUALS(st.algorithm, static_cast<int>(algorithm));
          TS_ASSERT_EQUALS(st.sequence, static_cast<double>(i + 1));
          TS_ASSERT_EQUALS(st.iterations, st.stepIterations + st.publicationIterations);
          TS_ASSERT_LESS_THAN_EQUALS(st.iterations, run.plant.settings.iterationCap);
          if (i >= 3600) {
            const double values[4] = {out.n1Percent, out.n2Percent, out.nozzle.grossThrust,
                                      out.liftFan.grossThrust};
            for (int j = 0; j < 4; ++j) {
              low[j] = std::min(low[j], values[j]); high[j] = std::max(high[j], values[j]);
            }
          }
        }
        TS_ASSERT(!run.failed);
        TS_ASSERT_LESS_THAN(high[0] - low[0], 1e-3);
        TS_ASSERT_LESS_THAN(high[1] - low[1], 1e-3);
        TS_ASSERT_LESS_THAN(high[2] - low[2], 1e-4 * run.plant.Design().thrust);
        TS_ASSERT_LESS_THAN(high[3] - low[3], 1e-4 * run.plant.Design().thrust);
        TS_ASSERT_DELTA(run.plant.Outputs().n1Percent, equilibrium.n1Percent, 0.01);
        TS_ASSERT_DELTA(run.plant.Outputs().n2Percent, equilibrium.n2Percent, 0.01);
        TS_ASSERT_LESS_THAN(RelativeError(run.plant.Outputs().nozzle.grossThrust,
                                         equilibrium.nozzle.grossThrust), 1e-4);
        TS_ASSERT_DELTA(run.plant.Outputs().liftFan.grossThrust
                         / run.plant.Outputs().nozzle.grossThrust, run.in.liftSplit, 1e-3);
        TS_ASSERT_LESS_THAN_EQUALS(run.worstEnergy, 1e-5);
        TS_ASSERT_LESS_THAN_EQUALS(run.worstMass, 1e-6);
      }
    }
  }
};
