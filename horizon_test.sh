#!/bin/zsh
cd ~/agentverse
for h in 0 2 4; do
  out=results/eval_h$h.jsonl; rm -f $out
  for s in $(seq 1 20); do
    if [ $h = 0 ]; then extra="--horizon 0.01"; else extra="--horizon $h"; fi
    venv/bin/python guide_sim.py --seed $s --fresh --headless --memory routes/h$h.npz $=extra 2>/dev/null | grep '^{' >> $out
  done
done
