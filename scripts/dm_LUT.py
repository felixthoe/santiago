#!/usr/bin/env python3
"""
Hybrid-LUT-Modell mit linearer Interpolation.
- Referenz: Pinocchio
- Variable Terme werden per LUT linear interpoliert
- Feste Terme:
    M[0,0] = 55.0
    M[2,2] = 0.263
- Validierung auf Trainingsdaten und zufälligen Testdaten

Ausgaben auf 3 Dezimalstellen gerundet.
"""

import math
import numpy as np
import pinocchio as pin
from scipy.interpolate import RegularGridInterpolator
from pathlib import Path
import os

# ============================================================
# KONFIGURATION
# ============================================================
URDF_PATH = "/home/uwxvp/ws_Aufstehhilfe/santiago/urdf/robot.urdf"


cwd = os.getcwd()

SAVE_PATH = os.path.join(os.path.dirname(cwd),
                    'model_data',
                    'aufstehhilfe_model_LUTs.npz')

JOINT_Z   = "column_outside_column_inside_joint"
JOINT_PHI = "upper_cylinder_upper_motor_joint"
JOINT_THETA = "upper_motor_box_arm_joint"

G_VAL = 9.81

# Ursprüngliche Trainingskonfigurationen
TRAIN_CONFIGS = [
    ("C1_Zero",      0.0, 0.0, 0.0,       0.0, 0.0, 0.0),
    ("C2_Mid",       0.25, 0.4, 0.3,      0.05, 0.2, -0.15),
    ("C3_Extreme",   0.5, math.pi/2, math.pi/2, 0.1, math.pi/2, math.pi/2),
    ("C4_NegTheta",  0.2, -0.5, -0.8,     0.08, -0.3, 0.25),
    ("C5_HighSpeed", 0.4, 0.0, 1.2,       0.15, math.pi, -math.pi),
]

# Grenzen passend zu den Trainingsdaten
BOUNDS = {
    "z":      (0.0, 0.5),
    "phi":    (-math.pi, math.pi),
    "theta":  (-math.pi, math.pi),
    "zd":     (-0.15, 0.15),
    "phid":   (-math.pi, math.pi),
    "thetad": (-math.pi, math.pi),
}

# Feste Matrixelemente
M00_const = 55.0
M22_const = 0.263

# Anzahl LUT-Stützstellen pro Dimension
N_LUT = 10

# Anzahl Zufallstests
N_TEST = 1000
SEED_TEST = 42

# ============================================================
# MODELL LADEN
# ============================================================
model = pin.buildModelFromUrdf(URDF_PATH)
data = model.createData()

jid_z = model.getJointId(JOINT_Z)
jid_phi = model.getJointId(JOINT_PHI)
jid_theta = model.getJointId(JOINT_THETA)

iq_z = model.idx_qs[jid_z] if jid_z > 0 else -1
iq_phi = model.idx_qs[jid_phi] if jid_phi > 0 else -1
iq_theta = model.idx_qs[jid_theta] if jid_theta > 0 else -1

iv_z = model.idx_vs[jid_z] if jid_z > 0 else -1
iv_phi = model.idx_vs[jid_phi] if jid_phi > 0 else -1
iv_theta = model.idx_vs[jid_theta] if jid_theta > 0 else -1

def eval_pinocchio(z, phi, theta, zd, phid, thetad):
    q = pin.neutral(model)
    v = np.zeros(model.nv)

    if iq_z >= 0:
        q[iq_z] = z
    if iq_phi >= 0:
        q[iq_phi] = phi
    if iq_theta >= 0:
        q[iq_theta] = theta

    if iv_z >= 0:
        v[iv_z] = zd
    if iv_phi >= 0:
        v[iv_phi] = phid
    if iv_theta >= 0:
        v[iv_theta] = thetad

    pin.crba(model, data, q)
    M_full = data.M.copy()
    M_full = np.triu(M_full) + np.triu(M_full, 1).T

    pin.nonLinearEffects(model, data, q, v)
    b_full = data.nle.copy()

    pin.computeGeneralizedGravity(model, data, q)
    G_full = data.g.copy()

    Cqd_full = b_full - G_full

    sel = [i for i in [iv_z, iv_phi, iv_theta] if i >= 0]
    return M_full[np.ix_(sel, sel)], Cqd_full[sel], G_full[sel]

# ============================================================
# HILFSFUNKTIONEN
# ============================================================
def make_random_tests(n_test, seed=42):
    np.random.seed(seed)
    tests = []
    for i in range(n_test):
        z      = np.random.uniform(*BOUNDS["z"])
        phi    = np.random.uniform(*BOUNDS["phi"])
        theta  = np.random.uniform(*BOUNDS["theta"])
        zd     = np.random.uniform(*BOUNDS["zd"])
        phid   = np.random.uniform(*BOUNDS["phid"])
        thetad = np.random.uniform(*BOUNDS["thetad"])
        tests.append((f"Test_{i+1}", z, phi, theta, zd, phid, thetad))
    return tests

# ============================================================
# TRAININGSDATEN SAMMELN
# ============================================================
print("Sammle Pinocchio-Daten für Trainingskonfigurationen...")
train_data = []
for name, z, phi, th, zd, phid, thd in TRAIN_CONFIGS:
    M, Cqd, G = eval_pinocchio(z, phi, th, zd, phid, thd)
    train_data.append({
        "name": name,
        "z": z, "phi": phi, "theta": th,
        "zd": zd, "phid": phid, "thetad": thd,
        "M": M, "Cqd": Cqd, "G": G
    })

# Mittelwert für das variable Diagonalelement M[1,1] nur zur Info
M11_mean = float(np.mean([d["M"][1,1] for d in train_data]))

print(f"\nM[0,0]_const = {M00_const:.3f}")
print(f"M[2,2]_const = {M22_const:.3f}")
print(f"M[1,1]_mean  = {M11_mean:.3f}")

# ============================================================
# LUT AUFBAUEN
# ============================================================
print("\n" + "=" * 80)
print(f"BAUE LUT ({N_LUT} PUNKTE PRO DIMENSION, lineare Interpolation)")
print("=" * 80)

grid_names = ["z", "phi", "theta", "zd", "phid", "thetad"]
grid_vecs = [np.linspace(BOUNDS[name][0], BOUNDS[name][1], N_LUT) for name in grid_names]
shape_6d = tuple([N_LUT] * 6)

# Variable M-Terme
M11_vals = np.zeros(shape_6d)
M01_vals = np.zeros(shape_6d)
M02_vals = np.zeros(shape_6d)
M12_vals = np.zeros(shape_6d)

# Cqd-Terme
C0_vals = np.zeros(shape_6d)
C1_vals = np.zeros(shape_6d)
C2_vals = np.zeros(shape_6d)

# G-Terme
G0_vals = np.zeros(shape_6d)
G1_vals = np.zeros(shape_6d)
G2_vals = np.zeros(shape_6d)

total = N_LUT ** 6
count = 0

for i_z, z in enumerate(grid_vecs[0]):
    for i_phi, phi in enumerate(grid_vecs[1]):
        for i_th, th in enumerate(grid_vecs[2]):
            for i_zd, zd in enumerate(grid_vecs[3]):
                for i_phid, phid in enumerate(grid_vecs[4]):
                    for i_thd, thd in enumerate(grid_vecs[5]):
                        M, Cqd, G = eval_pinocchio(z, phi, th, zd, phid, thd)

                        M11_vals[i_z, i_phi, i_th, i_zd, i_phid, i_thd] = M[1,1]
                        M01_vals[i_z, i_phi, i_th, i_zd, i_phid, i_thd] = M[0,1]
                        M02_vals[i_z, i_phi, i_th, i_zd, i_phid, i_thd] = M[0,2]
                        M12_vals[i_z, i_phi, i_th, i_zd, i_phid, i_thd] = M[1,2]

                        C0_vals[i_z, i_phi, i_th, i_zd, i_phid, i_thd] = Cqd[0]
                        C1_vals[i_z, i_phi, i_th, i_zd, i_phid, i_thd] = Cqd[1]
                        C2_vals[i_z, i_phi, i_th, i_zd, i_phid, i_thd] = Cqd[2]

                        G0_vals[i_z, i_phi, i_th, i_zd, i_phid, i_thd] = G[0]
                        G1_vals[i_z, i_phi, i_th, i_zd, i_phid, i_thd] = G[1]
                        G2_vals[i_z, i_phi, i_th, i_zd, i_phid, i_thd] = G[2]

                        count += 1
                        if count % 1000 == 0 or count == total:
                            print(f"  Fortschritt: {count}/{total}")

interp_M11 = RegularGridInterpolator(grid_vecs, M11_vals, method="linear")
interp_M01 = RegularGridInterpolator(grid_vecs, M01_vals, method="linear")
interp_M02 = RegularGridInterpolator(grid_vecs, M02_vals, method="linear")
interp_M12 = RegularGridInterpolator(grid_vecs, M12_vals, method="linear")

interp_C0 = RegularGridInterpolator(grid_vecs, C0_vals, method="linear")
interp_C1 = RegularGridInterpolator(grid_vecs, C1_vals, method="linear")
interp_C2 = RegularGridInterpolator(grid_vecs, C2_vals, method="linear")

interp_G0 = RegularGridInterpolator(grid_vecs, G0_vals, method="linear")
interp_G1 = RegularGridInterpolator(grid_vecs, G1_vals, method="linear")
interp_G2 = RegularGridInterpolator(grid_vecs, G2_vals, method="linear")

def eval_lut(z, phi, theta, zd, phid, thetad):
    pt = np.array([z, phi, theta, zd, phid, thetad], dtype=float)

    M11 = float(interp_M11(pt))
    M01 = float(interp_M01(pt))
    M02 = float(interp_M02(pt))
    M12 = float(interp_M12(pt))

    C0 = float(interp_C0(pt))
    C1 = float(interp_C1(pt))
    C2 = float(interp_C2(pt))

    G0 = float(interp_G0(pt))
    G1 = float(interp_G1(pt))
    G2 = float(interp_G2(pt))

    M = np.array([
        [M00_const, M01,       M02],
        [M01,       M11,       M12],
        [M02,       M12, M22_const],
    ])

    Cqd = np.array([C0, C1, C2])
    G = np.array([G0, G1, G2])

    return M, Cqd, G

print("LUT erfolgreich erstellt.")

# ============================================================
# VALIDIERUNG AUF TRAININGSDATEN
# ============================================================
print("\n" + "=" * 80)
print("VALIDIERUNG AUF TRAININGSDATEN")
print("=" * 80)

for d in train_data:
    M_lut, Cqd_lut, G_lut = eval_lut(
        d["z"], d["phi"], d["theta"], d["zd"], d["phid"], d["thetad"]
    )

    err_G0 = abs(G_lut[0] - d["G"][0])
    err_M  = np.max(np.abs(M_lut - d["M"]))
    err_C  = np.max(np.abs(Cqd_lut - d["Cqd"]))

    print(f"\n{d['name']}:")
    print(f"  z={d['z']:.2f}, φ={d['phi']:.2f}, θ={d['theta']:.2f}")
    print(f"  zd={d['zd']:.2f}, φd={d['phid']:.2f}, θd={d['thetad']:.2f}")
    print(f"  |ΔG0| = {err_G0:.2f} N")
    print(f"  max|ΔM| = {err_M:.3f}")
    print(f"  max|ΔC| = {err_C:.3f}")

# ============================================================
# VALIDIERUNG AUF 5 UNABHÄNGIGEN TESTKONFIGURATIONEN
# ============================================================
TEST_INDEPENDENT = make_random_tests(N_TEST, seed=SEED_TEST)

print("\n" + "=" * 80)
print(f"VALIDIERUNG (ABSOLUTE FEHLER, {N_TEST} TESTPUNKTE)")
print("=" * 80)

max_err_G0 = 0.0
max_err_M = 0.0
max_err_C = 0.0

for name, z, phi, th, zd, phid, thd in TEST_INDEPENDENT:
    M_pin, Cqd_pin, G_pin = eval_pinocchio(z, phi, th, zd, phid, thd)
    M_lut, Cqd_lut, G_lut = eval_lut(z, phi, th, zd, phid, thd)

    err_G0 = abs(G_lut[0] - G_pin[0])
    err_M  = np.max(np.abs(M_lut - M_pin))
    err_C  = np.max(np.abs(Cqd_lut - Cqd_pin))

    max_err_G0 = max(max_err_G0, err_G0)
    max_err_M  = max(max_err_M, err_M)
    max_err_C  = max(max_err_C, err_C)

    print(f"\n{name}:")
    print(f"  z={z:.2f}, φ={phi:.2f}, θ={th:.2f}")
    print(f"  zd={zd:.2f}, φd={phid:.2f}, θd={thd:.2f}")
    print(f"  |ΔG0| = {err_G0:.2f} N")
    print(f"  max|ΔM| = {err_M:.3f}")
    print(f"  max|ΔC| = {err_C:.3f}")

    print("  M_lut =")
    print(np.array2string(M_lut, precision=3, suppress_small=True))
    print("  M_pin =")
    print(np.array2string(M_pin, precision=3, suppress_small=True))

    print("  Cqd_lut =")
    print(np.array2string(Cqd_lut, precision=3, suppress_small=True))
    print("  Cqd_pin =")
    print(np.array2string(Cqd_pin, precision=3, suppress_small=True))

    print("  G_lut =")
    print(np.array2string(G_lut, precision=3, suppress_small=True))
    print("  G_pin =")
    print(np.array2string(G_pin, precision=3, suppress_small=True))

print("\n" + "=" * 80)
print("ZUSAMMENFASSUNG (WORST CASE)")
print("=" * 80)
print(f"Max |ΔG0| : {max_err_G0:.2f} N")
print(f"Max |ΔM|  : {max_err_M:.3f}")
print(f"Max |ΔC|  : {max_err_C:.3f}")

# ============================================================
# Arrays (LUTs) speichern
# ============================================================

np.savez(
    SAVE_PATH,
    grid_vecs=np.array(grid_vecs),
    M11=M11_vals,
    M01=M01_vals,
    M02=M02_vals,
    M12=M12_vals,
    C0=C0_vals,
    C1=C1_vals,
    C2=C2_vals,
    G0=G0_vals,
    G1=G1_vals,
    G2=G2_vals,
)

print(f"LUTs saved to: {SAVE_PATH}")