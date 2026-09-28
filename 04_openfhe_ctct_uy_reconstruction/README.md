# 04 — OpenFHE u/y reconstruction, ct-ct coefficients

Both signals and controller/reconstruction coefficients are ciphertexts. Coefficients are encrypted once during setup and reused.

```bash
export OPENFHE_ROOT="$HOME/openfhe-development"
./scripts/build.sh
./scripts/run.sh stable 10
./scripts/run.sh unstable_low 10
./scripts/run.sh unstable_high 10
```

Recommended progression: 3 blocks -> 10 blocks -> 50/100 blocks.

Do not use controller-state error alone as the pass/fail criterion. Compare the actual control input and plant trajectory. The high-transient profile is intentionally numerically difficult even though its ideal input/output behavior is equivalent to `unstable_low`.
