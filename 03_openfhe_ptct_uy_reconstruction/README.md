# 03 — OpenFHE u/y reconstruction, pt-ct coefficients

Signals/states are CKKS ciphertexts; controller coefficients are plaintext multipliers. This is the simplest encrypted baseline.

```bash
export OPENFHE_ROOT="$HOME/openfhe-development"
./scripts/build.sh
./scripts/run.sh stable 10
./scripts/run.sh unstable_low 10
./scripts/run.sh unstable_high 10
```

The run script saves raw CSV logs and generates `u.png`, `plant_state.png`, `u_error.png`, and `plant_state_error.png`.

The current OpenFHE implementation is fixed at \(K=\nu=4\) to preserve the validated baseline circuit. Use the plaintext stage to study arbitrary \(K\) first.
