#!/usr/bin/env python3
from __future__ import annotations
import sys
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from config.controller_profiles import A,B,C,XP0,get_profile,profile_names,profile_metrics,observability_matrix

def trajectory(name, samples=400):
    p=get_profile(name); xp=XP0.copy(); xc=p.xc0.copy(); us=[]; xs=[]
    for _ in range(samples):
        y=float(C@xp); u=float(p.H@xc); us.append(u); xs.append(xp.copy())
        xp=A@xp+B*u; xc=p.F@xc+p.G*y
    return np.asarray(us),np.asarray(xs)

for name in profile_names():
    p=get_profile(name);m=profile_metrics(p)
    rank=np.linalg.matrix_rank(observability_matrix(p.F,p.H))
    assert rank==4, (name,rank)
    assert m['rho_Acl']<1.0,(name,m['rho_Acl'])
    print(f"{name:15s} rhoF={m['rho_F']:.9f} rhoAcl={m['rho_Acl']:.9f} gF4={m['g_F_K']:.3f} gHF4={m['g_HF_K']:.3f} condO4={m['cond_O4']:.2f}")

u1,x1=trajectory('unstable_low');u2,x2=trajectory('unstable_high')
print(f"low/high max |u difference| = {np.max(np.abs(u1-u2)):.3e}")
print(f"low/high max plant difference = {np.max(np.abs(x1-x2)):.3e}")
assert np.max(np.abs(u1-u2))<1e-12
assert np.max(np.abs(x1-x2))<1e-12
print('profile validation: PASS')
