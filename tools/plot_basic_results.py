#!/usr/bin/env python3
from __future__ import annotations
import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def pick(df, *names):
    for name in names:
        if name in df.columns:
            return df[name].to_numpy(float)
    raise KeyError(f"none of {names} found")


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('results_dir')
    a=ap.parse_args(); d=Path(a.results_dir); df=pd.read_csv(d/'sample_trace.csv')
    k=pick(df,'global_sample','sample')
    u_ref=pick(df,'u_nominal','u_plain','u_reference')
    u=pick(df,'u_ckks','u_test')
    e=np.abs(u-u_ref)
    # Plant columns differ slightly between implementations.
    if 'plant_nominal_0' in df:
        pref='plant_nominal'; ptest='plant_ckks'
    elif 'plant_plain_0' in df:
        pref='plant_plain'; ptest='plant_ckks'
    else:
        pref='xp_reference'; ptest='xp_test'
    xr=np.column_stack([df[f'{pref}_{i}'].to_numpy(float) for i in range(4)])
    x=np.column_stack([df[f'{ptest}_{i}'].to_numpy(float) for i in range(4)])
    ep=np.max(np.abs(x-xr),axis=1)

    plt.figure();plt.plot(k,u_ref,label='reference');plt.plot(k,u,'--',label='encrypted/test');plt.xlabel('sample');plt.ylabel('u');plt.legend();plt.tight_layout();plt.savefig(d/'u.png',dpi=160);plt.close()
    plt.figure();
    for i in range(4):plt.plot(k,xr[:,i],label=f'ref x_p[{i}]');plt.plot(k,x[:,i],'--',label=f'test x_p[{i}]')
    plt.xlabel('sample');plt.ylabel('plant state');plt.legend(ncol=2,fontsize=8);plt.tight_layout();plt.savefig(d/'plant_state.png',dpi=160);plt.close()
    plt.figure();plt.semilogy(k,np.maximum(e,1e-18));plt.xlabel('sample');plt.ylabel('|u error|');plt.tight_layout();plt.savefig(d/'u_error.png',dpi=160);plt.close()
    plt.figure();plt.semilogy(k,np.maximum(ep,1e-18));plt.xlabel('sample');plt.ylabel('plant error inf-norm');plt.tight_layout();plt.savefig(d/'plant_state_error.png',dpi=160);plt.close()
    print(f'max |u error|={np.max(e):.6e}')
    print(f'max plant error inf={np.max(ep):.6e}')
    print(f'saved plots in {d}')

if __name__=='__main__': main()
