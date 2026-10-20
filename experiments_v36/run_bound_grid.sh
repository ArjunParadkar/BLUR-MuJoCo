#!/bin/bash
cd "$(dirname "$0")"; PY=../blur_mujoco/.venv/bin/python; export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1
for v in 1.5 2.0 2.5; do for f in 3.0 3.5 4.0; do
  $PY bound_gait.py --v $v --f $f --opt 250 --seeds 2 --out results/bound/v${v}_f${f}.json --render results/bound/v${v}_f${f}.png > results/bound/v${v}_f${f}.log 2>&1 &
done; done; wait; echo GRID DONE
