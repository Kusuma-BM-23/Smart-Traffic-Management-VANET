# Project Report
## Smart Traffic Management at Road Intersections Using Vehicular Ad Hoc Networks (VANET)

**Course:** Ad Hoc Network (22ISE773)  
**Department:** Information Science and Engineering  
**Platform:** Python  
**Project type:** Simulation

## Abstract
Traffic congestion at road intersections increases travel time, fuel consumption, and driver frustration. Conventional fixed-time traffic signals allocate the same green duration regardless of the current traffic demand. This project presents an educational VANET-based smart traffic management simulation. Vehicles periodically transmit traffic beacons to a Roadside Unit (RSU) using a simplified Vehicle-to-Infrastructure (V2I) communication model. The RSU estimates traffic demand on four approaches and an adaptive controller allocates green time proportionally between North-South and East-West traffic. A fixed-time signal model is also simulated as a baseline. The system evaluates average vehicle waiting time, average queue length, throughput, packet delivery ratio, and communication delay.

## 1. Introduction
A Vehicular Ad Hoc Network is a wireless network in which vehicles communicate with other vehicles or roadside infrastructure. VANET applications include traffic monitoring, road-safety alerts, congestion management, and traveller information. In this project, an RSU is installed near a four-way intersection. Vehicles send periodic beacon messages containing their approach and traffic presence. The RSU estimates traffic demand and informs the signal controller.

## 2. Problem Statement
Fixed-time traffic signals do not continuously adapt to changing traffic density. As a result, one road may remain congested while another road receives green time despite having fewer vehicles. A traffic management approach is required that uses information collected from connected vehicles to adjust signal timing according to observed demand.

## 3. Objectives
1. Model vehicle arrivals and movement at a four-way intersection.
2. Model V2I beacon communication between vehicles and an RSU.
3. Estimate traffic demand from successfully received messages.
4. Allocate green time according to estimated traffic demand.
5. Compare adaptive signal control with fixed-time signal control.
6. Evaluate traffic and communication performance metrics.

## 4. System Architecture
The architecture consists of four main stages:
1. **Vehicles:** generate traffic and maintain their approach and distance from the stop line.
2. **V2I communication:** vehicles in RSU range periodically transmit beacon messages.
3. **RSU:** receives messages subject to a configurable packet-loss probability and estimates queue demand.
4. **Adaptive controller:** assigns green time to North-South and East-West phases based on the estimated demand.

See `outputs/vanet_architecture.png`.

## 5. Intersection Model
The road network contains four approaches: North, South, East, and West. North and South share one signal phase; East and West share another. The RSU is placed at the intersection. The communication coverage is represented by a configurable circular range.

See `outputs/intersection_layout.png`.

## 6. Methodology
### 6.1 Vehicle generation
Vehicles arrive independently on each approach using a Poisson arrival model. The default arrival rates are:
- North: 0.14 vehicles/second
- South: 0.10 vehicles/second
- East: 0.40 vehicles/second
- West: 0.20 vehicles/second

These are illustrative synthetic traffic loads, not measured traffic counts.

### 6.2 Vehicle movement and queueing
The simulation advances in one-second time steps. Vehicles approach the stop line at a modelled speed of 10 m/s, while maintaining an 8 m minimum gap. Vehicles wait at the stop line until their signal group is green.

### 6.3 V2I communication
Every five seconds, vehicles within 150 m of the RSU attempt to transmit a beacon. The default packet-loss probability is 0.08. Successfully received beacons are counted by approach and used as the RSU's latest traffic-demand estimate. A simplified 0.05-second processing/transmission delay is assigned to each successfully received beacon.

### 6.4 Adaptive signal control
A signal cycle has a 54-second green-time budget and two 3-second amber periods. At the beginning of each cycle, the controller calculates the North-South and East-West demand estimates. Green time is allocated proportionally to the estimated demand, subject to a minimum of 10 seconds per phase and a maximum of 44 seconds. If no demand estimate is available, the green time is split equally.

### 6.5 Baseline
The fixed-time baseline allocates 27 seconds of green time to each direction group. It uses the same traffic-generation parameters and random seed as the adaptive simulation.

## 7. Performance Metrics
- **Average waiting time:** mean time spent in the system by vehicles that depart during the simulation.
- **Average queue length:** mean total number of vehicles waiting across all four approaches at each simulation second.
- **Throughput:** number of vehicles served per simulation minute.
- **Packet Delivery Ratio (PDR):** successfully received beacon packets divided by attempted beacon packets.
- **Average V2I delay:** modelled delay per successfully received beacon.

## 8. Implementation
The project is implemented in Python using NumPy for stochastic arrivals and numerical calculations, Matplotlib for charts and animation, and the Python standard library for CSV and JSON output. The main source file is `main.py`.

## 9. Results
Run the simulation using:
```bash
python main.py
```
The program writes actual results from the selected duration and random seed to `outputs/summary.json` and `outputs/performance_comparison.csv`. Insert the values from those generated files into the result table below after running the project.

| Metric | Adaptive VANET | Fixed-time |
|---|---:|---:|
| Vehicles arrived | 505 | 505 |
| Vehicles served | 432 | 421 |
| Average waiting time (s) | 49.73 | 48.38 |
| Average queue length | 44.48 | 47.33 |
| Maximum queue length | 74 | 87 |
| Packet delivery ratio | 92.43% | 92.58% |
| Average V2I delay (s) | 0.050 | 0.050 |

**Interpretation:** Adaptive control is expected to provide an advantage when traffic demand is unbalanced because it can allocate more green time to the busier direction group. The exact outcome depends on arrival rates, packet losses, simulation duration, and random seed. Do not claim improvement unless the generated results demonstrate it.

## 10. Conclusion
The project demonstrates how VANET-based V2I communication can support traffic-aware signal timing. Vehicles transmit periodic beacons to an RSU, which estimates traffic demand and adjusts green time. The comparison with a fixed-time baseline provides a way to evaluate whether adaptive control improves waiting time, queue length, or throughput under the selected synthetic traffic conditions.

## 11. Limitations and Future Enhancement
The model is an educational abstraction. It does not implement a standards-compliant wireless MAC/PHY, IEEE 802.11p, realistic radio propagation, GPS inaccuracies, lane changing, detailed car-following, or a real traffic simulator. Future work can integrate SUMO with Veins/OMNeT++ or an NS-3 VANET model, add multiple RSUs, implement a more detailed MAC protocol, and test calibrated traffic traces.

## 12. Syllabus Mapping
- **Unit 1:** wireless communication concepts; communication assumptions are abstracted.
- **Unit 2:** MAC protocol design concepts motivate wireless access, but a detailed MAC protocol is not implemented.
- **Unit 3:** traffic monitoring, vehicle movement, and V2I communication.
- **Unit 4:** dissemination of vehicle traffic information; routing protocols are not implemented.
- **Unit 5:** VANET applications, infrastructure use, and vehicular network simulation.

The syllabus lists simulation of a VANET application as a course assessment component. This project addresses that application through traffic monitoring and adaptive intersection control.
