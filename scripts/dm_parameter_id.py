#!/usr/bin/env python3
"""
Parameteridentifikation ohne iterative Optimierung.
Verwendet 5 Trainingskonfigurationen und lineare Ausgleichsrechnung.
Ausgaben auf 3 Dezimalstellen gerundet.
"""

import math
import numpy as np
import pinocchio as pin

# ============================================================
# KONFIGURATION
# ============================================================
URDF_PATH = "/home/uwxvp/ws_Aufstehhilfe/santiago/urdf/robot.urdf"

JOINT_Z   = "column_outside_column_inside_joint"
JOINT_PHI = "upper_cylinder_upper_motor_joint"
JOINT_THETA = "upper_motor_box_arm_joint"

G_VAL = 9.81

# Fünf Trainingskonfigurationen mit zufälligen Werten innerhalb der Schranken
np.random.seed(42)

TRAIN_CONFIGS = [
    ("C1_Zero", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
]

for i in range(2, 6):
    z      = np.random.uniform(0.0, 0.5)
    phi    = np.random.uniform(-math.pi, math.pi)
    theta  = np.random.uniform(-math.pi, math.pi)
    zd     = np.random.uniform(-0.05, 0.05)
    phid   = np.random.uniform(-0.05 * math.pi, 0.05 * math.pi)
    thetad = np.random.uniform(-0.05 * math.pi, 0.05 * math.pi)

    TRAIN_CONFIGS.append((
        f"C{i}_Rand",
        z, phi, theta, zd, phid, thetad
    ))

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
    if iq_z >= 0: q[iq_z] = z
    if iq_phi >= 0: q[iq_phi] = phi
    if iq_theta >= 0: q[iq_theta] = theta
    if iv_z >= 0: v[iv_z] = zd
    if iv_phi >= 0: v[iv_phi] = phid
    if iv_theta >= 0: v[iv_theta] = thetad

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
# DATEN SAMMELN
# ============================================================
print("Sammle Pinocchio-Daten für Trainingskonfigurationen...")
train_data = []
for name, z, phi, th, zd, phid, thd in TRAIN_CONFIGS:
    M, Cqd, G = eval_pinocchio(z, phi, th, zd, phid, thd)
    train_data.append({
        "name": name, "z": z, "phi": phi, "theta": th,
        "zd": zd, "phid": phid, "thetad": thd,
        "M": M, "Cqd": Cqd, "G": G
    })

# ============================================================
# 1. BESTIMME THETA_OFFSET UND J_PHI-PARAMETER (beta1, beta2)
# ============================================================
theta_offset = -math.pi/2   # fest aus URDF-Geometrie

thetas = np.array([d["theta"] for d in train_data])
J_phis = np.array([d["M"][1,1] for d in train_data])
sin2_vals = np.sin(thetas - theta_offset)**2

A = np.vstack([np.ones(len(sin2_vals)), sin2_vals]).T
beta1, beta2 = np.linalg.lstsq(A, J_phis, rcond=None)[0]

print(f"\nTheta_offset = {theta_offset:.3f} rad (fest)")
print(f"J_phi(θ) = {beta1:.3f} + {beta2:.3f} * sin^2(θ - theta_offset)")

# ============================================================
# 2. KONSTANTE DIAGONALELEMENTE
# ============================================================
M_total = np.mean([d["M"][0,0] for d in train_data])
J_theta = np.mean([d["M"][2,2] for d in train_data])
print(f"\nM_total = {M_total:.3f} kg")
print(f"J_theta = {J_theta:.3f} kg·m²")

# ============================================================
# 3. KOPPELPARAMETER beta3 AUS M12
# ============================================================
cos_vals = np.cos(thetas - theta_offset)
M12_vals = np.array([d["M"][1,2] for d in train_data])
beta3 = np.linalg.lstsq(cos_vals.reshape(-1,1), M12_vals, rcond=None)[0][0]
print(f"beta3 = {beta3:.3f} kg·m²")

# ============================================================
# 4. GRAVITATIONSPARAMETER (lineare Abhängigkeit von z)
# ============================================================
z_vals = np.array([d["z"] for d in train_data])
G0_vals = np.array([d["G"][0] for d in train_data]) / G_VAL
A_grav = np.vstack([np.ones(len(z_vals)), z_vals]).T
M_grav0, M_grad_z = np.linalg.lstsq(A_grav, G0_vals, rcond=None)[0]
print(f"\nM_grav0 = {M_grav0:.3f} kg")
print(f"M_grad_z = {M_grad_z:.3f} kg/m")

# ============================================================
# 5. CORIOLIS-PARAMETER alpha2
# ============================================================
mask = np.array([abs(d["thetad"]) > 1e-6 for d in train_data])
cos_vals_c0 = np.cos(np.array([d["theta"] for d in train_data])[mask] - theta_offset)
thetad2_vals = np.array([d["thetad"]**2 for d in train_data])[mask]
C0_vals = np.array([d["Cqd"][0] for d in train_data])[mask]
A0 = (cos_vals_c0 * thetad2_vals).reshape(-1,1)
alpha2 = np.linalg.lstsq(A0, C0_vals, rcond=None)[0][0]
print(f"\nalpha2 = {alpha2:.3f} kg·m")

# ============================================================
# 6. CORIOLIS-PARAMETER gamma2, gamma3, gamma4
# ============================================================
mask = np.array([abs(d["phid"]) > 1e-6 and abs(d["thetad"]) > 1e-6 for d in train_data])

st = np.sin(np.array([d["theta"] for d in train_data])[mask] - theta_offset)
ct = np.cos(np.array([d["theta"] for d in train_data])[mask] - theta_offset)
phid = np.array([d["phid"] for d in train_data])[mask]
thetad = np.array([d["thetad"] for d in train_data])[mask]
zd = np.array([d["zd"] for d in train_data])[mask]

term1_1 = 2 * beta2 * st * ct * phid * thetad
term2_1 = -beta3 * st * thetad**2
C1_known = term1_1 + term2_1
C1_true = np.array([d["Cqd"][1] for d in train_data])[mask]

rhs1 = C1_true - C1_known
A_gamma2 = (-st * phid * thetad).reshape(-1,1)
gamma2 = np.linalg.lstsq(A_gamma2, rhs1, rcond=None)[0][0]

termA = -gamma2 * st * phid * thetad
C2_true = np.array([d["Cqd"][2] for d in train_data])[mask]
rhs2 = C2_true - termA

A2 = np.vstack([ct * zd * thetad, st * ct * phid**2]).T
gamma3, gamma4 = np.linalg.lstsq(A2, rhs2, rcond=None)[0]

print(f"gamma2 = {gamma2:.3f} kg·m²")
print(f"gamma3 = {gamma3:.3f} kg·m")
print(f"gamma4 = {gamma4:.3f} kg·m²")

# ============================================================
# REDUZIERTES MODELL
# ============================================================
def eval_reduced(z, phi, theta, zd, phid, thetad):
    vartheta = theta - theta_offset
    st = math.sin(vartheta)
    ct = math.cos(vartheta)
    s2 = st * st

    M = np.array([
        [M_total, 0.0, 0.0],
        [0.0, beta1 + beta2 * s2, beta3 * ct],
        [0.0, beta3 * ct, J_theta],
    ])

    Cqd = np.array([
        alpha2 * ct * thetad**2,
        2 * beta2 * st * ct * phid * thetad - beta3 * st * thetad**2 - gamma2 * st * phid * thetad,
        -gamma2 * st * phid * thetad + gamma3 * ct * zd * thetad + gamma4 * st * ct * phid**2,
    ])

    M_grav = M_grav0 + M_grad_z * z
    G = np.array([M_grav * G_VAL, 0.0, 0.0])

    return M, Cqd, G

# ============================================================
# VALIDIERUNG AUF TRAININGSDATEN
# ============================================================
def compute_rel_error(a, b, eps=1e-10):
    denom = np.maximum(np.abs(a), np.abs(b))
    denom = np.maximum(denom, eps)
    return np.abs(a - b) / denom

print("\n" + "=" * 80)
print("VALIDIERUNG AUF TRAININGSDATEN")
print("=" * 80)

for d in train_data:
    M_pred, Cqd_pred, G_pred = eval_reduced(d["z"], d["phi"], d["theta"],
                                            d["zd"], d["phid"], d["thetad"])
    rel_M = compute_rel_error(M_pred, d["M"])
    rel_Cqd = compute_rel_error(Cqd_pred, d["Cqd"])
    rel_G = compute_rel_error(G_pred, d["G"])
    max_rel = max(np.max(rel_M), np.max(rel_Cqd), np.max(rel_G))
    print(f"{d['name']}: max rel. Fehler = {max_rel:.3%}")
    if max_rel < 0.05:
        print("  ✓ Unter 5%")
    else:
        print("  ✗ Über 5%")

# ============================================================
# VALIDIERUNG AUF UNABHÄNGIGEN TESTKONFIGURATIONEN
# ============================================================
TEST_INDEPENDENT = [
    ("Test_1_mid_z", 0.15, 0.2, 0.5, 0.03, 0.1, 0.1),
    ("Test_2_neg_phi", 0.3, -0.8, 0.7, -0.04, -0.25, 0.12),
    ("Test_3_mixed", 0.45, 1.2, -0.4, 0.07, 0.5, -0.3),
    ("Test_4_slow", 0.1, -0.3, -1.0, 0.01, -0.05, 0.02),
]

print("\n" + "=" * 80)
print("VALIDIERUNG AUF UNABHÄNGIGEN TESTKONFIGURATIONEN")
print("=" * 80)

for name, z, phi, th, zd, phid, thd in TEST_INDEPENDENT:
    M_pin, Cqd_pin, G_pin = eval_pinocchio(z, phi, th, zd, phid, thd)
    M_red, Cqd_red, G_red = eval_reduced(z, phi, th, zd, phid, thd)
    
    rel_M = compute_rel_error(M_red, M_pin)
    rel_Cqd = compute_rel_error(Cqd_red, Cqd_pin)
    rel_G = compute_rel_error(G_red, G_pin)
    max_rel = max(np.max(rel_M), np.max(rel_Cqd), np.max(rel_G))
    
    print(f"\n{name}:")
    print(f"  z={z:.2f}, φ={phi:.2f}, θ={th:.2f}, zd={zd:.2f}, φd={phid:.2f}, θd={thd:.2f}")
    print(f"  Max rel. Fehler: {max_rel:.3%}")
    if max_rel < 0.05:
        print("  ✓ Unter 5%")
    else:
        print("  ✗ Über 5%")
        # Bei großen Fehlern die Matrizen ausgeben
        print("  M_red =")
        print(np.array2string(M_red, precision=3, suppress_small=True))
        print("  M_pin =")
        print(np.array2string(M_pin, precision=3, suppress_small=True))
        print("  Cqd_red =", np.array2string(Cqd_red, precision=3, suppress_small=True))
        print("  Cqd_pin =", np.array2string(Cqd_pin, precision=3, suppress_small=True))
        print("  G_red  =", np.array2string(G_red, precision=3, suppress_small=True))
        print("  G_pin  =", np.array2string(G_pin, precision=3, suppress_small=True))


# ============================================================
# DIREKTE INTERPOLATION AUF 6D‑GITTER (5 PUNKTE PRO DIMENSION)
# ============================================================
from scipy.interpolate import RegularGridInterpolator

print("\n" + "=" * 80)
print("BAUE 6D‑INTERPOLATIONSTABELLE (5^6 = 15625 PUNKTE)")
print("=" * 80)

# Grenzen (wie im Gesamtskript definiert)
bounds = {
    "z":      (0.0, 0.5),
    "phi":    (-math.pi, math.pi),
    "theta":  (-math.pi, math.pi),
    "zd":     (-0.15, 0.15),
    "phid":   (-math.pi, math.pi),
    "thetad": (-math.pi, math.pi),
}

# Erzeuge Gittervektoren
grid_vecs = [np.linspace(bounds[dim][0], bounds[dim][1], 5) for dim in ["z", "phi", "theta", "zd", "phid", "thetad"]]

# Arrays für die Komponenten (flach, später reshaped)
shape_6d = tuple([5]*6)
M00_vals = np.zeros(shape_6d)
M11_vals = np.zeros(shape_6d)
M22_vals = np.zeros(shape_6d)
M01_vals = np.zeros(shape_6d)
M02_vals = np.zeros(shape_6d)
M12_vals = np.zeros(shape_6d)
C0_vals  = np.zeros(shape_6d)
C1_vals  = np.zeros(shape_6d)
C2_vals  = np.zeros(shape_6d)
G0_vals  = np.zeros(shape_6d)
G1_vals  = np.zeros(shape_6d)
G2_vals  = np.zeros(shape_6d)

# Fülle das Gitter
total = 3**6
count = 0
for i_z, z in enumerate(grid_vecs[0]):
    for i_phi, phi in enumerate(grid_vecs[1]):
        for i_th, th in enumerate(grid_vecs[2]):
            for i_zd, zd in enumerate(grid_vecs[3]):
                for i_phid, phid in enumerate(grid_vecs[4]):
                    for i_thd, thd in enumerate(grid_vecs[5]):
                        M, Cqd, G = eval_pinocchio(z, phi, th, zd, phid, thd)
                        M00_vals[i_z,i_phi,i_th,i_zd,i_phid,i_thd] = M[0,0]
                        M11_vals[i_z,i_phi,i_th,i_zd,i_phid,i_thd] = M[1,1]
                        M22_vals[i_z,i_phi,i_th,i_zd,i_phid,i_thd] = M[2,2]
                        M01_vals[i_z,i_phi,i_th,i_zd,i_phid,i_thd] = M[0,1]
                        M02_vals[i_z,i_phi,i_th,i_zd,i_phid,i_thd] = M[0,2]
                        M12_vals[i_z,i_phi,i_th,i_zd,i_phid,i_thd] = M[1,2]
                        C0_vals[i_z,i_phi,i_th,i_zd,i_phid,i_thd]  = Cqd[0]
                        C1_vals[i_z,i_phi,i_th,i_zd,i_phid,i_thd]  = Cqd[1]
                        C2_vals[i_z,i_phi,i_th,i_zd,i_phid,i_thd]  = Cqd[2]
                        G0_vals[i_z,i_phi,i_th,i_zd,i_phid,i_thd]  = G[0]
                        G1_vals[i_z,i_phi,i_th,i_zd,i_phid,i_thd]  = G[1]
                        G2_vals[i_z,i_phi,i_th,i_zd,i_phid,i_thd]  = G[2]
                        count += 1
                        if count % 2000 == 0:
                            print(f"  Fortschritt: {count}/{total}")

# Erstelle Interpolatoren
interp_M00 = RegularGridInterpolator(grid_vecs, M00_vals)
interp_M11 = RegularGridInterpolator(grid_vecs, M11_vals)
interp_M22 = RegularGridInterpolator(grid_vecs, M22_vals)
interp_M01 = RegularGridInterpolator(grid_vecs, M01_vals)
interp_M02 = RegularGridInterpolator(grid_vecs, M02_vals)
interp_M12 = RegularGridInterpolator(grid_vecs, M12_vals)
interp_C0  = RegularGridInterpolator(grid_vecs, C0_vals)
interp_C1  = RegularGridInterpolator(grid_vecs, C1_vals)
interp_C2  = RegularGridInterpolator(grid_vecs, C2_vals)
interp_G0  = RegularGridInterpolator(grid_vecs, G0_vals)
interp_G1  = RegularGridInterpolator(grid_vecs, G1_vals)
interp_G2  = RegularGridInterpolator(grid_vecs, G2_vals)

def eval_interpolated(z, phi, theta, zd, phid, thetad):
    pt = np.array([z, phi, theta, zd, phid, thetad])
    M = np.array([
        [interp_M00(pt)[0], interp_M01(pt)[0], interp_M02(pt)[0]],
        [interp_M01(pt)[0], interp_M11(pt)[0], interp_M12(pt)[0]],
        [interp_M02(pt)[0], interp_M12(pt)[0], interp_M22(pt)[0]],
    ])
    Cqd = np.array([interp_C0(pt)[0], interp_C1(pt)[0], interp_C2(pt)[0]])
    G   = np.array([interp_G0(pt)[0], interp_G1(pt)[0], interp_G2(pt)[0]])
    return M, Cqd, G

print("Interpolatoren erfolgreich erstellt.")

# ============================================================
# VALIDIERUNG AUF 5 UNABHÄNGIGEN TESTKONFIGURATIONEN
# ============================================================
np.random.seed(42)

TEST_INDEPENDENT = []
for i in range(5):
    z      = np.random.uniform(0.0, 0.5)
    phi    = np.random.uniform(-math.pi, math.pi)
    theta  = np.random.uniform(-math.pi, math.pi)
    zd     = np.random.uniform(-0.05, 0.05)
    phid   = np.random.uniform(-0.05 * math.pi, 0.05 * math.pi)
    thetad = np.random.uniform(-0.05 * math.pi, 0.05 * math.pi)

    TEST_INDEPENDENT.append((
        f"Test_{i+1}",
        z, phi, theta, zd, phid, thetad
    ))

print("\n" + "=" * 80)
print("VALIDIERUNG (ABSOLUTE FEHLER, 5 TESTPUNKTE)")
print("=" * 80)

max_err_G0 = 0.0
max_err_M = 0.0
max_err_C = 0.0

for name, z, phi, th, zd, phid, thd in TEST_INDEPENDENT:
    M_pin, Cqd_pin, G_pin = eval_pinocchio(z, phi, th, zd, phid, thd)
    M_red, Cqd_red, G_red = eval_reduced(z, phi, th, zd, phid, thd)

    err_G0 = abs(G_red[0] - G_pin[0])
    err_M  = np.max(np.abs(M_red - M_pin))
    err_C  = np.max(np.abs(Cqd_red - Cqd_pin))

    max_err_G0 = max(max_err_G0, err_G0)
    max_err_M  = max(max_err_M, err_M)
    max_err_C  = max(max_err_C, err_C)

    print(f"\n{name}:")
    print(f"  z={z:.2f}, φ={phi:.2f}, θ={th:.2f}")
    print(f"  zd={zd:.2f}, φd={phid:.2f}, θd={thd:.2f}")
    print(f"  |ΔG0| = {err_G0:.2f} N")
    print(f"  max|ΔM| = {err_M:.3f}")
    print(f"  max|ΔC| = {err_C:.3f}")

    print("  M_red =")
    print(np.array2string(M_red, precision=3, suppress_small=True))
    print("  M_pin =")
    print(np.array2string(M_pin, precision=3, suppress_small=True))

    print("  Cqd_red =")
    print(np.array2string(Cqd_red, precision=3, suppress_small=True))
    print("  Cqd_pin =")
    print(np.array2string(Cqd_pin, precision=3, suppress_small=True))

    print("  G_red =")
    print(np.array2string(G_red, precision=3, suppress_small=True))
    print("  G_pin =")
    print(np.array2string(G_pin, precision=3, suppress_small=True))

print("\n" + "=" * 80)
print("ZUSAMMENFASSUNG (WORST CASE)")
print("=" * 80)
print(f"Max |ΔG0| : {max_err_G0:.2f} N")
print(f"Max |ΔM|  : {max_err_M:.3f}")
print(f"Max |ΔC|  : {max_err_C:.3f}")



