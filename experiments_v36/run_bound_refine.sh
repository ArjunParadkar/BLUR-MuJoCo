#!/bin/bash
cd "$(dirname "$0")"; PY=../blur_mujoco/.venv/bin/python; export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
for v in 1.5 2.0 2.5; do
  $PY bound_gait.py --v $v --f 3.5 --opt 500 --seeds 3 --beta-max 0.45 --out results/bound_refine/v${v}_f3.5.json --render results/bound_refine/v${v}_f3.5.png > results/bound_refine/v${v}_f3.5.log 2>&1 &
done; wait; echo REFINE DONE
