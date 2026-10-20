#!/bin/bash
# Run the headless report on the original model and every variant. Logs + JSON land in experiments/results/.
cd "$(dirname "$0")"
PY=../blur_mujoco/.venv/bin/python
mkdir -p results
echo "== original (blur_mujoco/) =="; $PY replay_exp.py --report --out ../baseline/report_full.json --jobs 14 2>&1 | tee results/original.log
for v in base w15 w25 w40 w40_light w40_kp60; do
  echo "== $v =="; $PY replay_exp.py --report --model variants/$v --out results/$v.json --jobs 14 2>&1 | tee results/$v.log
done
echo ALL DONE
