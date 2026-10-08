#!/usr/bin/env python3
"""Generate the coupled turbine plant's gas property data and its manifest.

One source of truth for the species polynomials: this script writes both the
NASA 7-coefficient arrays used by the reference (thermally perfect) gas model
and the Hermite tables used by the reduced algorithm, into
src/models/propulsion/plant/TurbinePlantGasData.inc. The manifest beside it
records provenance, domain, interpolation, held-out errors and resident bytes.

The polynomial coefficients are the GRI-Mech 3.0 thermodynamic fits
(thermo30.dat; Smith, Golden, Frenklach et al., http://combustion.berkeley.edu/
gri-mech/version30/text30.html), which reproduce the NASA Glenn/Burcat
7-coefficient form of McBride, Gordon and Reno, NASA TM-4513 (1993).
They are factual property data; checks below compare them with NIST-JANAF
heat capacities and test the 1000 K joins, which catch transcription errors.

Usage (from the JSBSim root):
    python3 utils/turbine_plant/generate_gas_data.py [--check]

--check regenerates in memory and fails if the committed files differ.
The output is deterministic: no timestamps, no randomness without a seed.
"""
import argparse
import hashlib
import json
import math
import os
import random
import sys

SCHEMA = 1
GENERATOR_REVISION = 1
RU = 8.31446261815324  # J/mol/K, CODATA 2018 exact
T_REF = 298.15
T_MIN, T_MAX, T_STEP = 200.0, 3500.0, 10.0
SEED = 20261007

# Molar masses, kg/mol (IUPAC conventional atomic weights).
M_C, M_H, M_O, M_N, M_AR = 12.0107e-3, 1.00794e-3, 15.9994e-3, 14.0067e-3, 39.948e-3
MOLAR = {
    "N2": 2 * M_N, "O2": 2 * M_O, "AR": M_AR,
    "CO2": M_C + 2 * M_O, "H2O": 2 * M_H + M_O,
}

# GRI-Mech 3.0 thermo30.dat: (Tmid, high a1..a7, low a1..a7).
NASA7 = {
    "O2": (1000.0,
           [3.28253784e+00, 1.48308754e-03, -7.57966669e-07, 2.09470555e-10,
            -2.16717794e-14, -1.08845772e+03, 5.45323129e+00],
           [3.78245636e+00, -2.99673416e-03, 9.84730201e-06, -9.68129509e-09,
            3.24372837e-12, -1.06394356e+03, 3.65767573e+00]),
    "N2": (1000.0,
           [2.92664000e+00, 1.48797680e-03, -5.68476000e-07, 1.00970380e-10,
            -6.75335100e-15, -9.22797700e+02, 5.98052800e+00],
           [3.29867700e+00, 1.40824040e-03, -3.96322200e-06, 5.64151500e-09,
            -2.44485400e-12, -1.02089990e+03, 3.95037200e+00]),
    "CO2": (1000.0,
            [3.85746029e+00, 4.41437026e-03, -2.21481404e-06, 5.23490188e-10,
             -4.72084164e-14, -4.87591660e+04, 2.27163806e+00],
            [2.35677352e+00, 8.98459677e-03, -7.12356269e-06, 2.45919022e-09,
             -1.43699548e-13, -4.83719697e+04, 9.90105222e+00]),
    "H2O": (1000.0,
            [3.03399249e+00, 2.17691804e-03, -1.64072518e-07, -9.70419870e-11,
             1.68200992e-14, -3.00042971e+04, 4.96677010e+00],
            [4.19864056e+00, -2.03643410e-03, 6.52040211e-06, -5.48797062e-09,
             1.77197817e-12, -3.02937267e+04, -8.49032208e-01]),
    "AR": (1000.0,
           [2.50000000e+00, 0.0, 0.0, 0.0, 0.0, -7.45375000e+02, 4.36600000e+00],
           [2.50000000e+00, 0.0, 0.0, 0.0, 0.0, -7.45375000e+02, 4.36600000e+00]),
}

# NIST-JANAF ideal-gas heat capacities, J/mol/K, used only as a transcription
# check: 0.6 % at 298.15 and 1000 K, 1.5 % at 2000 K. The fits are not JANAF
# interpolants; the GRI-Mech H2O fit exceeds the tabulated 2000 K value by
# about 1.1 % (about 0.1 % of products-mixture cp). The 1000 K joins, checked
# to 2e-3 and observed near 1e-9, are the decisive transcription test.
JANAF_CP = {
    "N2": {298.15: 29.124, 1000.0: 32.698, 2000.0: 35.988},
    "O2": {298.15: 29.376, 1000.0: 34.870, 2000.0: 37.741},
    "CO2": {298.15: 37.135, 1000.0: 54.308, 2000.0: 60.350},
    "H2O": {298.15: 33.590, 1000.0: 41.268, 2000.0: 51.180},
    "AR": {298.15: 20.786, 1000.0: 20.786, 2000.0: 20.786},
}

# U.S. Standard Atmosphere 1976 dry-air mole fractions of the species modeled;
# minor constituents are omitted and the rest renormalized.
AIR_MOLE = {"N2": 0.78084, "O2": 0.209476, "AR": 0.00934, "CO2": 0.000314}


def coefficients(name, T):
    tmid, high, low = NASA7[name]
    return high if T >= tmid else low


def cp_mol(name, T):
    a = coefficients(name, T)
    return RU * (a[0] + a[1] * T + a[2] * T**2 + a[3] * T**3 + a[4] * T**4)


def h_mol(name, T):
    a = coefficients(name, T)
    return RU * (a[0] * T + a[1] * T**2 / 2 + a[2] * T**3 / 3 + a[3] * T**4 / 4
                 + a[4] * T**5 / 5 + a[5])


def s_mol(name, T):
    a = coefficients(name, T)
    return RU * (a[0] * math.log(T) + a[1] * T + a[2] * T**2 / 2 + a[3] * T**3 / 3
                 + a[4] * T**4 / 4 + a[6])


def air_mass_fractions():
    total = sum(AIR_MOLE.values())
    moles = {k: v / total for k, v in AIR_MOLE.items()}
    mass = {k: moles[k] * MOLAR[k] for k in moles}
    m = sum(mass.values())
    return {k: v / m for k, v in mass.items()}, moles, m


def product_mass_fractions(x, y):
    _, moles, m_air_mol = air_mass_fractions()
    n_o2 = x + y / 4.0
    n_air = n_o2 / moles["O2"]
    prod = {
        "CO2": (x + n_air * moles["CO2"]) * MOLAR["CO2"],
        "H2O": (y / 2.0) * MOLAR["H2O"],
        "N2": n_air * moles["N2"] * MOLAR["N2"],
        "AR": n_air * moles["AR"] * MOLAR["AR"],
    }
    total = sum(prod.values())
    fuel_mass = x * M_C + y * M_H
    far = fuel_mass / (n_air * m_air_mol)
    assert abs(total - (fuel_mass + n_air * m_air_mol)) < 1e-12
    return {k: v / total for k, v in prod.items()}, far


def mixture_poly(mass_fractions):
    """Mass-weighted NASA coefficients per unit mass (J/kg/K basis)."""
    high = [0.0] * 7
    low = [0.0] * 7
    for name, w in mass_fractions.items():
        scale = w * RU / MOLAR[name]
        _, hi, lo = NASA7[name]
        for i in range(7):
            high[i] += scale * hi[i]
            low[i] += scale * lo[i]
    return low, high


def poly_eval(low, high, T, kind):
    a = high if T >= 1000.0 else low
    if kind == "cp":
        return a[0] + a[1] * T + a[2] * T**2 + a[3] * T**3 + a[4] * T**4
    if kind == "dcp":
        return a[1] + 2 * a[2] * T + 3 * a[3] * T**2 + 4 * a[4] * T**3
    if kind == "h":
        return (a[0] * T + a[1] * T**2 / 2 + a[2] * T**3 / 3 + a[3] * T**4 / 4
                + a[4] * T**5 / 5 + a[5])
    if kind == "phi":
        return (a[0] * math.log(T) + a[1] * T + a[2] * T**2 / 2 + a[3] * T**3 / 3
                + a[4] * T**4 / 4 + a[6])
    raise ValueError(kind)


def hermite(t0, step, values, derivs, T):
    u = (T - t0) / step
    i = min(max(int(math.floor(u)), 0), len(values) - 2)
    s = u - i
    h00 = (1 + 2 * s) * (1 - s) ** 2
    h10 = s * (1 - s) ** 2
    h01 = s * s * (3 - 2 * s)
    h11 = s * s * (s - 1)
    return (h00 * values[i] + h10 * step * derivs[i]
            + h01 * values[i + 1] + h11 * step * derivs[i + 1])


def build(x=12.0, y=23.0):
    air, _, _ = air_mass_fractions()
    products, far = product_mass_fractions(x, y)
    species = {"air": air, "products": products}
    polys = {name: mixture_poly(w) for name, w in species.items()}

    checks = {"joins1000K": {}, "janafCp": {}}
    for name in NASA7:
        lo_v = [cp_mol_branch(name, 1000.0, "low"), h_mol_branch(name, 1000.0, "low"),
                s_mol_branch(name, 1000.0, "low")]
        hi_v = [cp_mol_branch(name, 1000.0, "high"), h_mol_branch(name, 1000.0, "high"),
                s_mol_branch(name, 1000.0, "high")]
        rel = [abs(a - b) / max(abs(b), 1.0) for a, b in zip(lo_v, hi_v)]
        checks["joins1000K"][name] = {"cpRelative": rel[0], "hRelative": rel[1], "sRelative": rel[2]}
        if max(rel) > 2e-3:
            raise SystemExit(f"{name}: 1000 K join mismatch {rel}; check transcription")
        for T, ref in JANAF_CP[name].items():
            err = cp_mol(name, T) / ref - 1.0
            checks["janafCp"].setdefault(name, {})[str(T)] = err
            if abs(err) > (1.5e-2 if T > 1500.0 else 6e-3):
                raise SystemExit(f"{name}: cp({T}) differs from JANAF by {err:.4%}")

    n = int(round((T_MAX - T_MIN) / T_STEP)) + 1
    grid = [T_MIN + i * T_STEP for i in range(n)]
    tables = {}
    for name, (low, high) in polys.items():
        h298 = poly_eval(low, high, T_REF, "h")
        tables[name] = {
            "h": [poly_eval(low, high, T, "h") - h298 for T in grid],
            "cp": [poly_eval(low, high, T, "cp") for T in grid],
            "dcp": [poly_eval(low, high, T, "dcp") for T in grid],
            "phi": [poly_eval(low, high, T, "phi") for T in grid],
        }

    # Held-out validation: cell midpoints plus seeded random interior points.
    rng = random.Random(SEED)
    held = [T_MIN + (i + 0.5) * T_STEP for i in range(n - 1)]
    held += [rng.uniform(T_MIN, T_MAX) for _ in range(4000)]
    errors = {}
    for name, (low, high) in polys.items():
        tab = tables[name]
        h298 = poly_eval(low, high, T_REF, "h")
        e = {"hAbsJPerKg": 0.0, "cpAbsJPerKgK": 0.0, "phiAbsJPerKgK": 0.0}
        for T in held:
            hp = poly_eval(low, high, T, "h") - h298
            cpp = poly_eval(low, high, T, "cp")
            php = poly_eval(low, high, T, "phi")
            ht = hermite(T_MIN, T_STEP, tab["h"], tab["cp"], T)
            cpt = hermite(T_MIN, T_STEP, tab["cp"], tab["dcp"], T)
            pht = hermite(T_MIN, T_STEP, tab["phi"], [c / t for c, t in zip(tab["cp"], grid)], T)
            e["hAbsJPerKg"] = max(e["hAbsJPerKg"], abs(ht - hp))
            e["cpAbsJPerKgK"] = max(e["cpAbsJPerKgK"], abs(cpt - cpp))
            e["phiAbsJPerKgK"] = max(e["phiAbsJPerKgK"], abs(pht - php))
        errors[name] = e
    return {"species": species, "far": far, "polys": polys, "grid": grid,
            "tables": tables, "errors": errors, "checks": checks,
            "heldOutCount": len(held)}


def cp_mol_branch(name, T, branch):
    tmid, high, low = NASA7[name]
    a = high if branch == "high" else low
    return RU * (a[0] + a[1] * T + a[2] * T**2 + a[3] * T**3 + a[4] * T**4)


def h_mol_branch(name, T, branch):
    tmid, high, low = NASA7[name]
    a = high if branch == "high" else low
    return RU * (a[0] * T + a[1] * T**2 / 2 + a[2] * T**3 / 3 + a[3] * T**4 / 4
                 + a[4] * T**5 / 5 + a[5])


def s_mol_branch(name, T, branch):
    tmid, high, low = NASA7[name]
    a = high if branch == "high" else low
    return RU * (a[0] * math.log(T) + a[1] * T + a[2] * T**2 / 2 + a[3] * T**3 / 3
                 + a[4] * T**4 / 4 + a[6])


def fmt(values):
    return ",\n  ".join(", ".join(f"{v:.17g}" for v in values[i:i + 4])
                        for i in range(0, len(values), 4))


def render_inc(data):
    out = ["// Generated by utils/turbine_plant/generate_gas_data.py; do not edit.",
           f"// Schema {SCHEMA}, generator revision {GENERATOR_REVISION}.",
           "// Per-unit-mass NASA 7-coefficient sums for the air and stoichiometric",
           "// products pseudo-species (C12H23 surrogate), and Hermite tables.",
           f"static constexpr int kGasDataSchema = {SCHEMA};",
           f"static constexpr double kTableMinK = {T_MIN:.17g};",
           f"static constexpr double kTableStepK = {T_STEP:.17g};",
           f"static constexpr int kTableCount = {len(data['grid'])};"]
    for idx, name in enumerate(["air", "products"]):
        low, high = data["polys"][name]
        out.append(f"static constexpr double kPoly{idx}Low[7] = {{{fmt(low)}}};")
        out.append(f"static constexpr double kPoly{idx}High[7] = {{{fmt(high)}}};")
        for q in ("h", "cp", "dcp", "phi"):
            out.append(f"static constexpr double kTable{idx}_{q}[{len(data['grid'])}] = {{\n  "
                       + fmt(data["tables"][name][q]) + "};")
    return "\n".join(out) + "\n"


def render_manifest(data, inc_text, script_hash):
    grid = data["grid"]
    resident = 2 * 4 * len(grid) * 8
    return {
        "schema": SCHEMA,
        "artifact": "TurbinePlantGasData.inc",
        "artifactSha256": hashlib.sha256(inc_text.encode()).hexdigest(),
        "generator": "utils/turbine_plant/generate_gas_data.py",
        "generatorRevision": GENERATOR_REVISION,
        "generatorSha256": script_hash,
        "reproduce": "python3 utils/turbine_plant/generate_gas_data.py",
        "seed": SEED,
        "sources": [
            {"name": "GRI-Mech 3.0 thermodynamic data (thermo30.dat)",
             "url": "http://combustion.berkeley.edu/gri-mech/version30/text30.html",
             "use": "NASA 7-coefficient fits for N2, O2, Ar, CO2, H2O"},
            {"name": "McBride, Gordon, Reno, NASA TM-4513 (1993)",
             "url": "https://ntrs.nasa.gov/citations/19940013151",
             "use": "Polynomial form and reference state"},
            {"name": "NIST-JANAF Thermochemical Tables",
             "url": "https://janaf.nist.gov/",
             "use": "Heat-capacity transcription checks only"},
            {"name": "U.S. Standard Atmosphere 1976",
             "url": "https://ntrs.nasa.gov/citations/19770009539",
             "use": "Dry-air composition of the modeled species"},
        ],
        "units": {"T": "K", "h": "J/kg sensible, datum 298.15 K", "cp": "J/kg/K",
                  "phi": "J/kg/K standard-state entropy function"},
        "pseudoSpecies": {"air": data["species"]["air"],
                          "products": data["species"]["products"],
                          "fuel": "C12H23 surrogate, constant cp, configured per plant"},
        "stoichiometricFuelAirRatio": data["far"],
        "domain": {"minK": T_MIN, "maxK": T_MAX, "outOfDomain":
                   "inversions clamp to the domain and count a domain event; "
                   "no extrapolation"},
        "interpolation": "cubic Hermite on a uniform grid; derivatives are the analytic "
                         "polynomial derivatives (cp, dcp/dT, cp/T); 1000 K is a node",
        "gridStepK": T_STEP,
        "nodes": len(grid),
        "validation": {"heldOutPoints": data["heldOutCount"],
                       "design": "cell midpoints plus seeded uniform random points; "
                                 "no node is a validation point",
                       "maxAbsoluteError": data["errors"]},
        "selfChecks": data["checks"],
        "limits": ["complete-combustion products; dissociation and minor species absent",
                   "ideal gas; no real-gas or condensed-phase effects",
                   "fits valid 200-3500 K (N2 and Ar fits from 300 K are used to 200 K)"],
        "residentBytes": resident,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    out_dir = os.path.join(root, "src", "models", "propulsion", "plant")
    with open(os.path.abspath(__file__), "rb") as f:
        script_hash = hashlib.sha256(f.read()).hexdigest()
    data = build()
    inc = render_inc(data)
    manifest = json.dumps(render_manifest(data, inc, script_hash), indent=2, sort_keys=True) + "\n"
    targets = {os.path.join(out_dir, "TurbinePlantGasData.inc"): inc,
               os.path.join(out_dir, "TurbinePlantGasData.json"): manifest}
    if args.check:
        stale = [p for p, text in targets.items()
                 if not os.path.exists(p) or open(p).read() != text]
        if stale:
            print("stale generated files:", *stale, sep="\n  ")
            return 1
        print("generated gas data is current")
        return 0
    for path, text in targets.items():
        with open(path, "w") as f:
            f.write(text)
    print(json.dumps(data["errors"], indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
