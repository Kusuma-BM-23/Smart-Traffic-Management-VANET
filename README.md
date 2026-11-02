# Smart Traffic Management Using VANET

## Overview
A Python discrete-time simulation of a four-way road intersection. Vehicles send periodic V2I beacon messages to a Roadside Unit (RSU). The RSU estimates traffic demand, and an adaptive signal controller allocates green time to the North-South and East-West directions.

## Features
- Four road approaches with stochastic vehicle arrivals.
- Vehicle movement and queue formation.
- V2I beacon communication with configurable range and packet loss.
- RSU traffic estimation.
- Adaptive green-time allocation.
- Fixed-time baseline for comparison.
- Queue, signal, packet, and performance CSV files.
- Performance comparison plots and an illustrative traffic animation.

## Requirements
- Python 3.10 or newer
- NumPy
- Matplotlib
- Pillow (only needed to save GIF animation)

## Windows setup (PowerShell)
```powershell
cd Smart_Traffic_VANET
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
python main.py
```

If `py -3.11` is unavailable, use `python -m venv .venv`.

## Run without animation
```powershell
python main.py --no-animation
```

## Change simulation duration and random seed
```powershell
python main.py --duration 900 --seed 7
```

## Output files
All generated results are saved in `outputs/`:
- `summary.json`: summary metrics and model assumptions.
- `performance_comparison.csv`: adaptive vs fixed-time metrics.
- `adaptive_queue_history.csv`, `fixed_queue_history.csv`: queue length by second.
- `adaptive_signal_history.csv`: phase and green allocations.
- `adaptive_rsu_snapshots.csv`: queue estimates and signal state.
- `performance_comparison.png`: metric comparison charts.
- `queue_length_comparison.png`: queue length over time.
- `vanet_architecture.png`: system architecture diagram.
- `intersection_layout.png`: intersection and RSU coverage diagram.
- `traffic_animation.gif`: illustrative animation (if Pillow is installed).

## Model description
Time advances in one-second steps. Each approach receives Poisson-distributed vehicle arrivals. Vehicles move toward the stop line subject to a minimum spacing. A vehicle can depart only when its direction has a green signal and the saturation headway has elapsed.

Vehicles within the RSU communication range send periodic beacon packets. Packets may be lost according to the configured packet-loss probability. The RSU uses the successfully received per-approach beacon counts as its latest traffic-demand estimate. At the start of each signal cycle, the adaptive controller distributes a 54-second green budget proportionally between North-South and East-West demand, subject to minimum and maximum green limits. Amber periods are modelled as three seconds each.

The fixed-time baseline uses 27 seconds of green for each direction group. Both runs use the same random seed and traffic arrival rates to make the comparison reproducible.

## Important scope and assumptions
This is an educational, discrete-time VANET abstraction. It is not a packet-level network simulator and does not implement IEEE 802.11p/DSRC, GPS error, realistic car-following, lane changing, or a calibrated traffic-flow model. The animation is illustrative; quantitative results come from the simulation engine. Results are synthetic and must not be presented as real-road measurements.

## Suggested experiments
1. Change `communication_range` from 75 m to 150 m and 200 m.
2. Change `packet_loss` from 0.08 to 0.20.
3. Increase East arrival rate from 0.42 to 0.60 vehicles/second.
4. Compare adaptive and fixed-time average waiting time and queue length.
5. Change simulation duration and repeat with several random seeds.

## Syllabus mapping (22ISE773)
- Unit 1: wireless communication concepts and wireless channel assumptions.
- Unit 3: traffic monitoring, vehicle movement, V2V/V2I communication concepts.
- Unit 4: data dissemination concepts (periodic beacon information).
- Unit 5: VANET applications, infrastructure use, vehicular network simulation.
- Unit 2 is not implemented as a detailed MAC protocol; packet loss is an abstract communication parameter.
