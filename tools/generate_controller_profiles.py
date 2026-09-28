#!/usr/bin/env python3
from __future__ import annotations
import json
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from config.controller_profiles import (  # noqa: E402
    A, B, C, XP0, get_profile, profile_names, reconstruction_matrices,
    closed_loop_matrix, profile_metrics,
)


def fmt(x: float) -> str:
    return f"{float(x):.17g}"


def vec(v) -> str:
    return "{" + ", ".join(fmt(x) for x in v) + "}"


def mat(m) -> str:
    rows = ", ".join("{{" + ", ".join(fmt(x) for x in row) + "}}" for row in m)
    return "{{" + rows + "}}"


def main():
    out_h = ROOT / "include" / "controller_profiles.h"
    out_j = ROOT / "config" / "controller_profiles.json"
    out_h.parent.mkdir(parents=True, exist_ok=True)

    payload = {"plant": {"A": A.tolist(), "B": B.tolist(), "C": C.tolist(), "xp0": XP0.tolist()}, "profiles": {}}

    lines = [
        "#pragma once",
        "",
        "#include <array>",
        "#include <stdexcept>",
        "#include <string>",
        "",
        "namespace tutorial_profiles {",
        "using Vec4 = std::array<double, 4>;",
        "using Mat4 = std::array<Vec4, 4>;",
        "",
        "struct ControllerProfile {",
        "    const char* name;",
        "    const char* label;",
        "    Mat4 A; Vec4 B; Vec4 C;",
        "    Mat4 F; Vec4 G; Vec4 H;",
        "    Vec4 xp0; Vec4 xc0;",
        "    Mat4 O4; Mat4 T4; Mat4 M_u; Mat4 M_y; Mat4 P; Mat4 q;",
        "    double rho_F; double rho_Acl; double cond_O4;",
        "    double M_u_inf; double M_y_inf; double g_F_4; double g_HF_4; double Gamma_B;",
        "};",
        "",
    ]

    cpp_names = {}
    for name in profile_names():
        p = get_profile(name)
        O, T, Mu, My = reconstruction_matrices(p.F, p.G, p.H, 4)
        m = profile_metrics(p, 4)
        ident = name.upper()
        cpp_names[name] = f"kProfile_{ident}"
        payload["profiles"][name] = {
            "label": p.label, "note": p.note,
            "F": p.F.tolist(), "G": p.G.tolist(), "H": p.H.tolist(), "xc0": p.xc0.tolist(),
            "O4": O.tolist(), "T4": T.tolist(), "M_u": Mu.tolist(), "M_y": My.tolist(),
            "metrics": m,
        }
        lines += [
            f"inline constexpr ControllerProfile kProfile_{ident} = {{",
            f"    \"{name}\", \"{p.label}\",",
            f"    {mat(A)}, {vec(B)}, {vec(C)},",
            f"    {mat(p.F)}, {vec(p.G)}, {vec(p.H)},",
            f"    {vec(XP0)}, {vec(p.xc0)},",
            f"    {mat(O)}, {mat(T)}, {mat(Mu)}, {mat(My)}, {mat(O)}, {mat(T)},",
            f"    {fmt(m['rho_F'])}, {fmt(m['rho_Acl'])}, {fmt(m['cond_O4'])},",
            f"    {fmt(m['Mu_inf'])}, {fmt(m['My_inf'])}, {fmt(m['g_F_K'])}, {fmt(m['g_HF_K'])}, {fmt(m['Gamma_B'])}",
            "};",
            "",
        ]

    lines += [
        "inline const ControllerProfile& GetControllerProfile(std::string name) {",
        "    for (char& c : name) if (c == '-') c = '_';",
        "    if (name == \"low\" || name == \"unstable\") name = \"unstable_low\";",
        "    if (name == \"high\") name = \"unstable_high\";",
        f"    if (name == \"stable\") return {cpp_names['stable']};",
        f"    if (name == \"unstable_low\") return {cpp_names['unstable_low']};",
        f"    if (name == \"unstable_high\") return {cpp_names['unstable_high']};",
        "    throw std::invalid_argument(\"unknown controller profile; choose stable, unstable_low, or unstable_high\");",
        "}",
        "",
        "}  // namespace tutorial_profiles",
        "",
    ]
    out_h.write_text("\n".join(lines), encoding="utf-8")
    out_j.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    print(f"wrote {out_h}")
    print(f"wrote {out_j}")
    print("\nprofile summary")
    for name in profile_names():
        m = profile_metrics(get_profile(name), 4)
        print(f"{name:15s} rhoF={m['rho_F']:.6f} rhoAcl={m['rho_Acl']:.6f} "
              f"gF4={m['g_F_K']:.3f} gHF4={m['g_HF_K']:.3f} condO4={m['cond_O4']:.2f}")


if __name__ == "__main__":
    main()
