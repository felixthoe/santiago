import numpy as np
from scipy.interpolate import RegularGridInterpolator


class RobotModel:
    M00 = 55.0
    M22 = 0.263
    Kd = np.diag([0.1, 0.1, 0.1])
    Lambda = np.diag([1.0, 1.0, 1.0])

    def __init__(self, npz_path: str):
        data = np.load(npz_path, allow_pickle=True)
        grid_vecs = list(data["grid_vecs"])

        self._interp = {
            key: RegularGridInterpolator(grid_vecs, data[key], method="linear")
            for key in ("M01", "M11", "M02", "M12", "C0", "C1", "C2", "G0", "G1", "G2")
        }

    def eval(self, z, phi, theta, zd, phid, thetad):
        pt = np.array([z, phi, theta, zd, phid, thetad], dtype=float)
        f = self._interp

        M01 = float(f["M01"](pt))
        M02 = float(f["M02"](pt))
        M11 = float(f["M11"](pt))
        M12 = float(f["M12"](pt))

        M = np.array([
            [self.M00, M01,       M02     ],
            [M01,      M11,       M12     ],
            [M02,      M12,       self.M22],
        ])
        Cqd = np.array([float(f["C0"](pt)), float(f["C1"](pt)), float(f["C2"](pt))])
        G   = np.array([float(f["G0"](pt)), float(f["G1"](pt)), float(f["G2"](pt))])

        return M, Cqd, G

    def calculate_assistive_torque(self, M, Cqd, G, q, qd, q_ref, qd_ref, tau_h,
                                   Kd, Lambda, qdd=None, qdd_ref=None):
        # Tracking errors
        q_tilde  = q - q_ref
        qd_tilde = qd - qd_ref

        # Sliding variable e = q̃̇ + Λq̃  (Xu 2023 eq. 16)
        e = qd_tilde + Lambda @ q_tilde

        # Sliding variable derivative ė = q̃̈ + Λq̃̇
        if qdd is not None and qdd_ref is not None:
            # Full version: uses actual and reference acceleration
            qdd_tilde = qdd_ref - qdd
            ed = qdd_tilde + Lambda @ qd_tilde
        else:
            # Approximate version: assumes qdd ≈ qdd_ref → q̃̈ ≈ 0
            ed = Lambda @ qd_tilde

        # AAN control law (Xu 2023 eq. 17): 
        # Note: Cqd = C(q,q̇)·q̇ is already evaluated at current velocity from LUT!
        # f (friction) ignored for now
        tau_act = M @ ed + Cqd + G - tau_h - Kd @ e

        return tau_act


def main():
    model = RobotModel("/home/uwxvp/ws_Aufstehhilfe/santiago/model_data/aufstehhilfe_model_LUTs.npz")

    # Plausible state: mid-trajectory, partially stood up
    z, phi, theta           = 0.05, -0.4, -0.3
    zd, phid, thetad        = 0.003, -0.03, -0.025
    zdd, phidd, thetadd     = 0.0001, -0.001, -0.001

    M, Cqd, G = model.eval(z, phi, theta, zd, phid, thetad)
    print("M =\n", M)
    print("Cqd =", Cqd)
    print("G =", G)

    q      = np.array([z, phi, theta])
    qd     = np.array([zd, phid, thetad])
    qdd    = np.array([zdd, phidd, thetadd])
    q_ref  = np.array([0.052, -0.38, -0.28])
    qd_ref = np.array([0.003, -0.032, -0.026])
    qdd_ref= np.array([0.0001, -0.0011, -0.0011])
    tau_h  = np.array([-100.0, -0.01, -0.05])

    print("\n--- Without acceleration (approximate) ---")
    tau_act = model.calculate_assistive_torque(
        M, Cqd, G, q, qd, q_ref, qd_ref, tau_h,
        Kd=model.Kd, Lambda=model.Lambda)
    print("tau_act =", tau_act)

    print("\n--- With acceleration (full) ---")
    tau_act_full = model.calculate_assistive_torque(
        M, Cqd, G, q, qd, q_ref, qd_ref, tau_h,
        Kd=model.Kd, Lambda=model.Lambda,
        qdd=qdd, qdd_ref=qdd_ref)
    print("tau_act =", tau_act_full)


if __name__ == "__main__":
    main()
