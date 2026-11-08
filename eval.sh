#!/bin/zsh
# 40 random journey-1 runs, then 40 journey-2 runs on the same remembered route
cd ~/agentverse
rm -f results/eval_j1.jsonl results/eval_j2.jsonl
for s in $(seq 1 40); do venv/bin/python guide_sim.py --seed $s --fresh --headless --memory routes/eval.npz 2>/dev/null | grep '^{' >> results/eval_j1.jsonl; done
venv/bin/python guide_sim.py --seed 11 --fresh --headless --memory routes/eval.npz 2>/dev/null >/dev/null
cp routes/eval.npz routes/eval_j1.npz
for s in $(seq 1 40); do cp routes/eval_j1.npz routes/eval.npz; venv/bin/python guide_sim.py --seed $s --headless --memory routes/eval.npz 2>/dev/null | grep '^{' >> results/eval_j2.jsonl; done
