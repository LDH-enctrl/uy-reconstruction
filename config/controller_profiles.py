"""Shared plant/controller presets used by all tutorial stages.

The `unstable_high` profile is a coordinate-transformed realization of
`unstable_low`. Therefore its ideal plant input/output trajectory is identical
when `xc0` is transformed consistently, while its internal realization has much
larger transient amplification and poorer observability conditioning.
"""
from __future__ import annotations

from dataclasses import dataclass
import numpy as np

A = np.array([
    [0.72, 0.08, 0.00, 0.00],
    [-0.05, 0.68, 0.06, 0.00],
    [0.00, -0.04, 0.62, 0.05],
    [0.00, 0.00, -0.03, 0.58],
], dtype=float)
B = np.array([0.08, 0.04, 0.02, 0.01], dtype=float)
C = np.array([0.70, -0.25, 0.15, 0.05], dtype=float)
XP0 = np.array([0.20, -0.10, 0.05, 0.08], dtype=float)
XC0_BASE = np.array([0.04, -0.03, 0.02, 0.01], dtype=float)


@dataclass(frozen=True)
class ControllerProfile:
    name: str
    label: str
    F: np.ndarray
    G: np.ndarray
    H: np.ndarray
    xc0: np.ndarray
    note: str


def _stable() -> ControllerProfile:
    return ControllerProfile(
        name="stable",
        label="Stable F",
        F=np.diag([-0.6, -0.2, 0.2, 0.6]).astype(float),
        G=np.array([0.03, -0.02, 0.025, -0.015], dtype=float),
        H=np.array([0.25, -0.20, 0.15, 0.10], dtype=float),
        xc0=XC0_BASE.copy(),
        note="Schur-stable controller realization used as the numerical baseline.",
    )


def _unstable_low() -> ControllerProfile:
    return ControllerProfile(
        name="unstable_low",
        label="Unstable F / low internal transient",
        F=np.diag([
            1.0460417098754866,
            -0.67744232979736663,
            -0.27798296156944574,
            0.23303382186836177,
        ]).astype(float),
        G=np.array([
            -0.7565539794458972,
            -0.68957818853917052,
            -0.9776385921687093,
            0.39964345096436504,
        ], dtype=float),
        H=np.array([
            1.1509092590273153,
            0.57057137227696975,
            1.7343969701278994,
            -0.38552603325660173,
        ], dtype=float),
        xc0=XC0_BASE.copy(),
        note=("Internally unstable but low-transient realization used in the "
              "long-horizon CKKS experiments."),
    )


def _unstable_high() -> ControllerProfile:
    low = _unstable_low()
    # Deliberately non-normal similarity transform.  This preserves the ideal
    # input/output controller behavior but makes the internal realization much
    # more sensitive to numerical perturbations.
    s = 8.0
    T = np.eye(4)
    T[0, 1] = s
    T[0, 2] = 0.4 * s
    T[1, 2] = 0.2 * s
    Ti = np.linalg.inv(T)
    return ControllerProfile(
        name="unstable_high",
        label="Unstable F / high internal transient",
        F=T @ low.F @ Ti,
        G=T @ low.G,
        H=low.H @ Ti,
        xc0=T @ low.xc0,
        note=("Similarity-transformed version of unstable_low. Ideal u/y/plant "
              "behavior is the same, but internal transient amplification and "
              "conditioning are intentionally much larger."),
    )


_PROFILES = {
    "stable": _stable(),
    "unstable_low": _unstable_low(),
    "unstable_high": _unstable_high(),
}


def normalize_name(name: str) -> str:
    key = name.strip().lower().replace("-", "_")
    aliases = {
        "low": "unstable_low",
        "high": "unstable_high",
        "unstable": "unstable_low",
    }
    return aliases.get(key, key)


def get_profile(name: str) -> ControllerProfile:
    key = normalize_name(name)
    if key not in _PROFILES:
        raise ValueError(f"unknown profile {name!r}; choose from {list(_PROFILES)}")
    return _PROFILES[key]


def profile_names() -> list[str]:
    return list(_PROFILES)


def observability_matrix(F: np.ndarray, H: np.ndarray, nu: int = 4) -> np.ndarray:
    return np.vstack([H @ np.linalg.matrix_power(F, i) for i in range(nu)])


def reconstruction_matrices(F: np.ndarray, G: np.ndarray, H: np.ndarray, nu: int = 4):
    if nu != 4:
        raise ValueError("current SISO 4-state tutorial expects nu=4")
    O = observability_matrix(F, H, nu)
    if np.linalg.matrix_rank(O) < 4:
        raise ValueError("observability matrix is not full rank")
    T = np.zeros((nu, nu))
    for i in range(nu):
        for j in range(i):
            T[i, j] = H @ np.linalg.matrix_power(F, i - 1 - j) @ G
    Mu = np.linalg.matrix_power(F, nu) @ np.linalg.inv(O)
    S = np.column_stack([
        np.linalg.matrix_power(F, nu - 1 - j) @ G for j in range(nu)
    ])
    My = S - Mu @ T
    return O, T, Mu, My


def closed_loop_matrix(profile: ControllerProfile) -> np.ndarray:
    return np.block([
        [A, np.outer(B, profile.H)],
        [np.outer(profile.G, C), profile.F],
    ])


def profile_metrics(profile: ControllerProfile, K: int = 4) -> dict[str, float]:
    O, _, Mu, My = reconstruction_matrices(profile.F, profile.G, profile.H, 4)
    acl = closed_loop_matrix(profile)
    rho_f = float(np.max(np.abs(np.linalg.eigvals(profile.F))))
    rho_acl = float(np.max(np.abs(np.linalg.eigvals(acl))))
    g_f = max(float(np.linalg.norm(np.linalg.matrix_power(profile.F, i), np.inf))
              for i in range(1, K + 1))
    g_hf = max(float(np.linalg.norm(profile.H @ np.linalg.matrix_power(profile.F, i), 1))
               for i in range(K))
    gamma_b = max(float(np.linalg.norm(profile.H @ np.linalg.matrix_power(profile.F, i) @ Mu, 1))
                  for i in range(K))
    return {
        "rho_F": rho_f,
        "rho_Acl": rho_acl,
        "cond_O4": float(np.linalg.cond(O)),
        "Mu_inf": float(np.linalg.norm(Mu, np.inf)),
        "My_inf": float(np.linalg.norm(My, np.inf)),
        "g_F_K": g_f,
        "g_HF_K": g_hf,
        "Gamma_B": gamma_b,
    }
