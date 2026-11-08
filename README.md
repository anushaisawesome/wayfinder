# Wayfinder

A robot guide for people losing their sight. You lead whenever you can; Wayfinder reads how you're doing through the handle, steps in only when you need it, and always says why.

AgentVerse 2026, Robotics track. Simulation in MuJoCo; everything runs on-device.

![Wayfinder](brand/wayfinder-logo.png)

## Why
- 250 people start losing their sight every day in the UK (RNIB); Guide Dogs made 553 new partnerships in 2025.
- 85% of English councils miss the 28-day target for sight loss rehab (RNIB, *Life on Pause*, 2025).
- Only 44% of blind rail passengers are reliably met by booked assistance (RNIB, 2026).
- Blind people memorise their regular routes; RNIB says changes to them must be communicated accessibly (*In My Way*, 2025).

## What it does
- **Perception-driven action:** a simulated depth camera labels every pixel as floor, obstacle or missing floor; a short-range sensor ring and the handle add close-range and user state.
- **You stay in charge:** on a route you know and when you're calm, you lead and it only watches out; when you're tense, it's busy or there's danger, it takes over and explains.
- **Safety first:** an independent safety layer refuses any move towards a drop, keeps personal space, and stops when unsure (no safe route, camera blind, handle released).
- **Crowds:** constant-velocity trajectory prediction 2 s ahead, uncertainty that grows with time and erratic walkers, only avoiding where people will be when it gets there.
- **Route memory:** remembers regular routes and says what has changed since last time.

## Edge budget
One CPU core, 40 ms control cycle (25 Hz), no GPU, no network. Typical decision step under 1 ms; worst-case A* re-plan about 13 ms.

## Results (randomised crowds and events)
| Run set | Arrived | Robot-caused contacts | Closest to the drop |
|---|---|---|---|
| Journey 1, 40 runs | 39/40 | 0 | 0.78 m |
| Journey 2 (remembered route, user leading), 40 runs | 39/40 | 0 | 0.60 m |

Prediction horizon, same 20 crowds: 2 s gave 0 contacts vs 1 with no prediction; 4 s was equally safe but stopped 25% more often. Raw results in `results/`.

## Run it
```
python3 -m venv venv && venv/bin/pip install -r requirements.txt
venv/bin/python guide_sim.py --fresh --people 5   # journey 1 (forgets the route first)
venv/bin/python guide_sim.py --people 5           # journey 2: remembers, hands control back
venv/bin/python guide_sim.py --show               # live window
venv/bin/python guide_sim.py --headless --seed 4  # safety metrics only
./eval.sh                                         # the 80-run evaluation
```
Demo video: `runs/Wayfinder demo - journey 2.mp4`. One-page summary: `onepager/Wayfinder - technical summary.pdf`.

## Limits
Simulation only (perfect sensors, no wheel slip); handle tension is modelled, not yet measured from real hands; constant-velocity prediction; indoors only; not yet tested with blind users.

## References
- Schöller et al., *What the Constant Velocity Model Can Teach Us About Pedestrian Motion Prediction*, RA-L 2020.
- Kayukawa et al., guiding blind users through crowds with a suitcase robot (CaBot), IMWUT 2020.
- Trautman & Krause, *Unfreezing the robot*, IROS 2010.
- RNIB *Life on Pause* (2025), *In My Way* (2025); Guide Dogs Annual Report 2025; RAIB Eden Park report (2021).
