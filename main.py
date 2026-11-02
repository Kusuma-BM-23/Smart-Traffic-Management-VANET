"""
Smart Traffic Management Using VANET
A discrete-time four-way intersection simulation with V2I communication.

Run:
    python main.py
    python main.py --no-animation
    python main.py --duration 900 --seed 7

Outputs are written to ./outputs:
    adaptive_metrics.csv
    fixed_metrics.csv
    performance_comparison.png
    traffic_animation.gif (when animation is enabled and Pillow is installed)
    summary.json
"""
import argparse
import csv
import json
import math
import random
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

APPROACHES = ("North", "South", "East", "West")
GROUPS = {"NS": ("North", "South"), "EW": ("East", "West")}

@dataclass
class Vehicle:
    vehicle_id: int
    approach: str
    distance: float          # metres from stop line; 0 is the stop line
    arrival_time: int
    last_beacon_time: int = -999

class VANETIntersection:
    def __init__(self, duration=600, seed=42, adaptive=True,
                 communication_range=150.0, packet_loss=0.08):
        self.duration = duration
        self.rng = random.Random(seed)
        self.np_rng = np.random.default_rng(seed)
        self.adaptive = adaptive
        self.communication_range = communication_range
        self.packet_loss = packet_loss
        self.road_length = 200.0
        self.vehicle_speed = 10.0       # m/s while approaching
        self.min_gap = 8.0               # metres between queued vehicles
        self.saturation_headway = 2.0    # one vehicle per lane every 2 seconds
        self.beacon_interval = 5
        self.cycle_green_budget = 54     # 60-second cycle minus two 3-sec amber periods
        self.min_green = 10
        self.max_green = 44

        # Average arrivals per second per approach; varied to create uneven demand.
        self.arrival_rates = {
            "North": 0.14, "South": 0.10,
            "East": 0.40, "West": 0.20
        }
        self.queues = {a: [] for a in APPROACHES}
        self.next_id = 1
        self.time = 0
        self.phase = "NS_GREEN"
        self.phase_remaining = 1
        self.ns_green = 27
        self.ew_green = 27
        self.cycle_number = 0
        self.last_departure = {a: -999 for a in APPROACHES}
        self.rsu_estimates = {a: 0 for a in APPROACHES}
        self.packets_sent = 0
        self.packets_received = 0
        self.total_packet_delay = 0.0
        self.total_waiting = 0.0
        self.total_departed = 0
        self.total_arrived = 0
        self.queue_history = []
        self.signal_history = []
        self.wait_samples = []
        self.throughput_history = []
        self.pdr_history = []
        self.delay_history = []
        self.snapshot_history = []

    def _current_green_group(self):
        if self.phase == "NS_GREEN":
            return "NS"
        if self.phase == "EW_GREEN":
            return "EW"
        return None

    def _allocate_green(self):
        if not self.adaptive:
            self.ns_green = 27
            self.ew_green = 27
            return
        ns = self.rsu_estimates["North"] + self.rsu_estimates["South"]
        ew = self.rsu_estimates["East"] + self.rsu_estimates["West"]
        total = ns + ew
        if total == 0:
            ns_green = self.cycle_green_budget // 2
        else:
            ns_green = round(self.cycle_green_budget * ns / total)
        ns_green = max(self.min_green, min(self.max_green, ns_green))
        ew_green = self.cycle_green_budget - ns_green
        # Keep both phases feasible even under extreme demand.
        if ew_green < self.min_green:
            ew_green = self.min_green
            ns_green = self.cycle_green_budget - ew_green
        self.ns_green, self.ew_green = ns_green, ew_green

    def _start_cycle(self):
        self.cycle_number += 1
        self._allocate_green()
        self.phase = "NS_GREEN"
        self.phase_remaining = self.ns_green

    def _advance_signal(self):
        self.phase_remaining -= 1
        if self.phase_remaining > 0:
            return
        if self.phase == "NS_GREEN":
            self.phase = "NS_AMBER"
            self.phase_remaining = 3
        elif self.phase == "NS_AMBER":
            self.phase = "EW_GREEN"
            self.phase_remaining = self.ew_green
        elif self.phase == "EW_GREEN":
            self.phase = "EW_AMBER"
            self.phase_remaining = 3
        else:
            self._start_cycle()

    def _arrivals(self):
        for approach in APPROACHES:
            count = int(self.np_rng.poisson(self.arrival_rates[approach]))
            for _ in range(count):
                self.queues[approach].append(
                    Vehicle(self.next_id, approach, self.road_length,
                            self.time)
                )
                self.next_id += 1
                self.total_arrived += 1

    def _move_vehicles(self):
        # Move vehicles toward the stop line without violating minimum spacing.
        for approach, queue in self.queues.items():
            queue.sort(key=lambda v: v.distance)
            for idx, vehicle in enumerate(queue):
                if idx == 0:
                    max_move = vehicle.distance
                else:
                    leader = queue[idx - 1]
                    max_move = max(0.0, vehicle.distance - leader.distance - self.min_gap)
                vehicle.distance -= min(self.vehicle_speed, max_move)
                vehicle.distance = max(0.0, vehicle.distance)

    def _communicate_with_rsu(self):
        # Every vehicle in RSU range sends a periodic beacon. The RSU counts
        # successfully received beacons per approach for the current update.
        if self.time % self.beacon_interval != 0:
            return
        received_by_approach = {a: 0 for a in APPROACHES}
        for approach, queue in self.queues.items():
            for vehicle in queue:
                if vehicle.distance <= self.communication_range:
                    self.packets_sent += 1
                    vehicle.last_beacon_time = self.time
                    if self.rng.random() >= self.packet_loss:
                        self.packets_received += 1
                        received_by_approach[approach] += 1
                        self.total_packet_delay += 0.05  # modelled V2I processing/transmission delay
        # Retain the most recent received estimate; scale by known sampling period.
        for approach in APPROACHES:
            self.rsu_estimates[approach] = received_by_approach[approach]

    def _serve_traffic(self):
        green_group = self._current_green_group()
        if green_group is None:
            return
        for approach in GROUPS[green_group]:
            queue = self.queues[approach]
            if queue and self.time - self.last_departure[approach] >= self.saturation_headway:
                # Queue is ordered nearest-to-stop-line first.
                queue.sort(key=lambda v: v.distance)
                vehicle = queue.pop(0)
                wait = max(0, self.time - vehicle.arrival_time)
                self.wait_samples.append(wait)
                self.total_waiting += wait
                self.total_departed += 1
                self.last_departure[approach] = self.time

    def step(self):
        if self.time == 0:
            self._start_cycle()
        self._arrivals()
        self._move_vehicles()
        self._communicate_with_rsu()
        self._serve_traffic()

        total_queue = sum(len(q) for q in self.queues.values())
        self.queue_history.append(total_queue)
        self.signal_history.append((self.time, self.phase, self.ns_green, self.ew_green))
        self.throughput_history.append(self.total_departed)
        pdr = self.packets_received / self.packets_sent if self.packets_sent else 0.0
        self.pdr_history.append(pdr)
        avg_delay = self.total_packet_delay / self.packets_received if self.packets_received else 0.0
        self.delay_history.append(avg_delay)
        if self.time % 5 == 0:
            self.snapshot_history.append({
                "time": self.time,
                "queues": {a: len(self.queues[a]) for a in APPROACHES},
                "estimates": dict(self.rsu_estimates),
                "phase": self.phase,
                "ns_green": self.ns_green,
                "ew_green": self.ew_green
            })
        self._advance_signal()
        self.time += 1

    def run(self):
        for _ in range(self.duration):
            self.step()
        avg_wait = float(np.mean(self.wait_samples)) if self.wait_samples else 0.0
        pdr = self.packets_received / self.packets_sent if self.packets_sent else 0.0
        avg_packet_delay = self.total_packet_delay / self.packets_received if self.packets_received else 0.0
        return {
            "mode": "Adaptive VANET" if self.adaptive else "Fixed-time",
            "duration_seconds": self.duration,
            "vehicles_arrived": self.total_arrived,
            "vehicles_served": self.total_departed,
            "vehicles_remaining": sum(len(q) for q in self.queues.values()),
            "average_wait_seconds": avg_wait,
            "average_queue_length": float(np.mean(self.queue_history)) if self.queue_history else 0.0,
            "maximum_queue_length": int(max(self.queue_history)) if self.queue_history else 0,
            "packet_delivery_ratio": pdr,
            "packets_sent": self.packets_sent,
            "packets_received": self.packets_received,
            "average_v2i_delay_seconds": avg_packet_delay,
            "throughput_vehicles_per_minute": self.total_departed / (self.duration / 60.0),
            "final_ns_green_seconds": self.ns_green,
            "final_ew_green_seconds": self.ew_green,
        }

def save_csv(path, rows):
    if not rows:
        return
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

def run_comparison(duration, seed, outdir):
    adaptive = VANETIntersection(duration, seed, adaptive=True)
    adaptive_result = adaptive.run()
    fixed = VANETIntersection(duration, seed, adaptive=False)
    fixed_result = fixed.run()

    save_csv(outdir / "adaptive_queue_history.csv",
             [{"second": i, "queue_length": q} for i, q in enumerate(adaptive.queue_history)])
    save_csv(outdir / "fixed_queue_history.csv",
             [{"second": i, "queue_length": q} for i, q in enumerate(fixed.queue_history)])
    save_csv(outdir / "adaptive_signal_history.csv",
             [{"second": t, "phase": p, "ns_green": n, "ew_green": e}
              for t, p, n, e in adaptive.signal_history])
    save_csv(outdir / "adaptive_rsu_snapshots.csv", adaptive.snapshot_history)

    metrics = [adaptive_result, fixed_result]
    save_csv(outdir / "performance_comparison.csv", metrics)
    summary = {"adaptive": adaptive_result, "fixed_time": fixed_result,
               "assumptions": {
                   "arrival_rates_vehicles_per_second": adaptive.arrival_rates,
                   "communication_range_metres": adaptive.communication_range,
                   "packet_loss_probability": adaptive.packet_loss,
                   "beacon_interval_seconds": adaptive.beacon_interval,
                   "simulation_step_seconds": 1,
                   "note": "Synthetic simulation results, not field measurements."
               }}
    (outdir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return adaptive, fixed, metrics

def make_performance_plot(adaptive, fixed, metrics, outdir):
    names = ["Avg wait (s)", "Avg queue", "Vehicles served", "PDR (%)"]
    a, f = metrics
    av = [a["average_wait_seconds"], a["average_queue_length"],
          a["vehicles_served"], a["packet_delivery_ratio"] * 100]
    fv = [f["average_wait_seconds"], f["average_queue_length"],
          f["vehicles_served"], f["packet_delivery_ratio"] * 100]
    fig, axes = plt.subplots(2, 2, figsize=(11, 7))
    for ax, name, x, y in zip(axes.flat, names, av, fv):
        ax.bar(["Adaptive VANET", "Fixed-time"], [x, y])
        ax.set_title(name)
        ax.grid(axis="y", alpha=0.25)
        for i, val in enumerate([x, y]):
            ax.text(i, val, f"{val:.2f}", ha="center", va="bottom")
    fig.suptitle("Smart Traffic Management: Performance Comparison")
    fig.tight_layout()
    fig.savefig(outdir / "performance_comparison.png", dpi=160)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(adaptive.queue_history, label="Adaptive VANET")
    ax.plot(fixed.queue_history, label="Fixed-time", alpha=0.8)
    ax.set_xlabel("Simulation time (seconds)")
    ax.set_ylabel("Total queued vehicles")
    ax.set_title("Queue length over time")
    ax.legend()
    ax.grid(alpha=0.25)
    fig.tight_layout()
    fig.savefig(outdir / "queue_length_comparison.png", dpi=160)
    plt.close(fig)

def make_architecture_diagram(outdir):
    fig, ax = plt.subplots(figsize=(11, 5))
    ax.set_xlim(0, 11); ax.set_ylim(0, 5); ax.axis("off")
    boxes = [
        (0.3, 2.0, 2.0, 1.2, "Vehicles\nGPS / speed / queue"),
        (3.0, 2.0, 2.0, 1.2, "V2I wireless\nbeacon messages"),
        (5.7, 2.0, 2.0, 1.2, "Roadside Unit\ntraffic estimation"),
        (8.4, 2.0, 2.2, 1.2, "Adaptive signal\ncontroller"),
    ]
    for x, y, w, h, label in boxes:
        ax.add_patch(plt.Rectangle((x,y),w,h,fill=False,linewidth=2))
        ax.text(x+w/2,y+h/2,label,ha="center",va="center",fontsize=10)
    for x1, x2 in [(2.3,3.0),(5.0,5.7),(7.7,8.4)]:
        ax.annotate("", xy=(x2,2.6), xytext=(x1,2.6),
                    arrowprops=dict(arrowstyle="->",lw=2))
    ax.text(5.5, 4.25, "SMART TRAFFIC MANAGEMENT USING VANET",
            ha="center", fontsize=14, weight="bold")
    ax.text(6.8, 1.1, "Queue estimates determine green-time allocation",
            ha="center", fontsize=10)
    ax.annotate("", xy=(9.5,1.5), xytext=(9.5,2.0),
                arrowprops=dict(arrowstyle="->",lw=1.5))
    fig.tight_layout()
    fig.savefig(outdir / "vanet_architecture.png", dpi=180, bbox_inches="tight")
    plt.close(fig)

def make_intersection_diagram(outdir):
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(-100,100); ax.set_ylim(-100,100); ax.set_aspect("equal"); ax.axis("off")
    ax.add_patch(plt.Rectangle((-18,-100),36,200,facecolor="#d9d9d9",edgecolor="black"))
    ax.add_patch(plt.Rectangle((-100,-18),200,36,facecolor="#d9d9d9",edgecolor="black"))
    ax.add_patch(plt.Rectangle((-18,-18),36,36,facecolor="#bcbcbc",edgecolor="black"))
    ax.add_patch(plt.Circle((0,0),75,fill=False,linestyle="--",linewidth=1.5))
    ax.text(0,0,"INTERSECTION",ha="center",va="center",fontsize=9)
    ax.text(0,88,"North",ha="center",fontsize=11)
    ax.text(0,-94,"South",ha="center",fontsize=11)
    ax.text(87,0,"East",ha="center",fontsize=11)
    ax.text(-87,0,"West",ha="center",fontsize=11)
    ax.text(48,53,"RSU\ncommunication\ncoverage",ha="center",fontsize=9)
    ax.text(0,-7,"VANET",ha="center",fontsize=8)
    fig.tight_layout()
    fig.savefig(outdir / "intersection_layout.png", dpi=180, bbox_inches="tight")
    plt.close(fig)


def animate_simulation(sim, outdir):
    """
    Create a more realistic top-down intersection animation.
    This is a visual demonstration of the traffic scenario; the quantitative
    metrics continue to come from VANETIntersection.run().
    """
    from matplotlib.patches import Rectangle, Circle, Polygon, FancyBboxPatch

    fig, ax = plt.subplots(figsize=(9, 9))
    fig.patch.set_facecolor("#e8eee8")
    ax.set_facecolor("#7da36b")
    ax.set_xlim(-110, 110)
    ax.set_ylim(-110, 110)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title("SMART TRAFFIC MANAGEMENT USING VANET",
                 fontsize=15, weight="bold", color="#18352a", pad=14)

    # Main roads and shoulders.
    road_color = "#343a40"
    ax.add_patch(Rectangle((-24, -110), 48, 220, color=road_color, zorder=1))
    ax.add_patch(Rectangle((-110, -24), 220, 48, color=road_color, zorder=1))
    # Road edge lines and dashed lane dividers.
    for x in (-24, 24):
        ax.plot([x, x], [-110, 110], color="#f5f1d5", lw=1.5, zorder=2)
    for y in (-24, 24):
        ax.plot([-110, 110], [y, y], color="#f5f1d5", lw=1.5, zorder=2)
    ax.plot([0, 0], [-110, 110], color="white", lw=1.2,
            ls=(0, (8, 8)), alpha=0.85, zorder=2)
    ax.plot([-110, 110], [0, 0], color="white", lw=1.2,
            ls=(0, (8, 8)), alpha=0.85, zorder=2)

    # Sidewalks / road borders.
    for x in (-29, 29):
        ax.plot([x, x], [-110, 110], color="#c7c7bd", lw=3, zorder=0)
    for y in (-29, 29):
        ax.plot([-110, 110], [y, y], color="#c7c7bd", lw=3, zorder=0)

    # Zebra crossings on all four approaches.
    for offset in range(-18, 19, 6):
        ax.add_patch(Rectangle((offset-1.8, 29), 3.6, 9,
                               color="#f7f7f2", zorder=3))
        ax.add_patch(Rectangle((offset-1.8, -38), 3.6, 9,
                               color="#f7f7f2", zorder=3))
        ax.add_patch(Rectangle((29, offset-1.8), 9, 3.6,
                               color="#f7f7f2", zorder=3))
        ax.add_patch(Rectangle((-38, offset-1.8), 9, 3.6,
                               color="#f7f7f2", zorder=3))

    # RSU communication coverage.
    ax.add_patch(Circle((0, 0), sim.communication_range,
                        fill=False, linestyle="--", linewidth=1.5,
                        edgecolor="#65d6ff", alpha=0.7, zorder=2))
    # Small RSU cabinet in the middle island.
    ax.add_patch(FancyBboxPatch((-8, -8), 16, 16,
                                boxstyle="round,pad=0.6",
                                facecolor="#f2c14e", edgecolor="#5a4510",
                                linewidth=1.5, zorder=7))
    ax.text(0, 0, "RSU", ha="center", va="center",
            fontsize=8, weight="bold", color="#302400", zorder=8)

    # Direction labels.
    ax.text(0, 101, "NORTH", ha="center", va="center",
            fontsize=10, weight="bold", color="#18352a")
    ax.text(0, -101, "SOUTH", ha="center", va="center",
            fontsize=10, weight="bold", color="#18352a")
    ax.text(101, 0, "EAST", ha="center", va="center",
            fontsize=10, weight="bold", color="#18352a", rotation=90)
    ax.text(-101, 0, "WEST", ha="center", va="center",
            fontsize=10, weight="bold", color="#18352a", rotation=90)

    # Four traffic signal heads, each with red / amber / green lamps.
    signal_centres = {
        "North": (-34, 43), "South": (34, -43),
        "East": (43, 34), "West": (-43, -34)
    }
    signal_lamps = {}
    for approach, (sx, sy) in signal_centres.items():
        ax.add_patch(FancyBboxPatch((sx-4, sy-8), 8, 16,
                                    boxstyle="round,pad=0.5",
                                    facecolor="#171a1d", edgecolor="#dddddd",
                                    linewidth=0.8, zorder=9))
        lamps = []
        for j, c in enumerate(("#ed4545", "#f3c644", "#48c774")):
            lamp = Circle((sx, sy+5-j*5), 1.65, facecolor="#454545",
                          edgecolor="#111111", linewidth=0.5, zorder=10)
            ax.add_patch(lamp)
            lamps.append(lamp)
        signal_lamps[approach] = lamps

    # Car body polygons. Each vehicle is a small top-view car with windows.
    car_specs = []
    car_colors = ["#e4572e", "#2e86de", "#f1c40f", "#8e44ad",
                  "#16a085", "#ecf0f1", "#d35400", "#e84393"]
    for i in range(28):
        approach = APPROACHES[i % 4]
        lane = -8 if i % 8 < 4 else 8
        rank = i // 4
        distance = 43 + rank * 17
        color = car_colors[i % len(car_colors)]
        car_specs.append((approach, lane, distance, color, i))

    car_patches = []
    window_patches = []
    for approach, lane, distance, color, idx in car_specs:
        body = Polygon([[0,0],[0,0],[0,0],[0,0]], closed=True,
                       facecolor=color, edgecolor="#202020",
                       linewidth=0.8, zorder=6)
        ax.add_patch(body)
        car_patches.append(body)
        win = Polygon([[0,0],[0,0],[0,0],[0,0]], closed=True,
                      facecolor="#bfe5f2", edgecolor="#263238",
                      linewidth=0.45, zorder=7)
        ax.add_patch(win)
        window_patches.append(win)

    info = ax.text(-104, 91, "", fontsize=9, color="white",
                   weight="bold", va="top",
                   bbox=dict(boxstyle="round,pad=0.4",
                             facecolor="#20352b", alpha=0.85,
                             edgecolor="none"), zorder=12)
    legend_text = ax.text(104, -91, "Blue dashed circle: RSU range",
                          fontsize=8, color="white", ha="right",
                          bbox=dict(boxstyle="round,pad=0.3",
                                    facecolor="#20352b", alpha=0.8,
                                    edgecolor="none"), zorder=12)

    def car_polygons(approach, lane, d):
        # Local car coordinates: length along +y, width along x.
        body_local = np.array([
            [-3.2, -6.0], [3.2, -6.0], [3.8, -4.0],
            [3.8, 4.0], [2.7, 6.0], [-2.7, 6.0],
            [-3.8, 4.0], [-3.8, -4.0]
        ])
        window_local = np.array([
            [-2.5, -2.8], [2.5, -2.8], [2.5, 3.7], [-2.5, 3.7]
        ])
        if approach == "North":  # heading south
            angle, cx, cy = 180, lane, d
        elif approach == "South":  # heading north
            angle, cx, cy = 0, lane, -d
        elif approach == "East":  # heading west
            angle, cx, cy = 90, d, lane
        else:  # West, heading east
            angle, cx, cy = -90, -d, lane
        theta = math.radians(angle)
        rot = np.array([[math.cos(theta), -math.sin(theta)],
                        [math.sin(theta), math.cos(theta)]])
        centre = np.array([cx, cy])
        return body_local @ rot.T + centre, window_local @ rot.T + centre

    def update(frame):
        phase = "NS_GREEN" if (frame % 66) < 30 else (
                "NS_AMBER" if (frame % 66) < 33 else (
                "EW_GREEN" if (frame % 66) < 63 else "EW_AMBER"))
        ns_green = phase == "NS_GREEN"
        ew_green = phase == "EW_GREEN"

        # Update signal lamps for each approach.
        for approach, lamps in signal_lamps.items():
            green = ns_green if approach in ("North", "South") else ew_green
            amber = phase.endswith("AMBER") and (
                (phase.startswith("NS") and approach in ("North", "South")) or
                (phase.startswith("EW") and approach in ("East", "West")))
            active = 2 if green else (1 if amber else 0)
            for j, lamp in enumerate(lamps):
                lamp.set_facecolor(("#ed4545", "#f3c644", "#48c774")[j]
                                   if j == active else "#454545")

        # Cars approach the stop line, form queues during red, and move through
        # the junction during green. Positions are illustrative for the GIF.
        for k, (approach, lane, initial_d, color, idx) in enumerate(car_specs):
            phase_offset = (frame * 1.15 + (idx // 4) * 2.0) % 105
            distance = 43 + ((initial_d - 43 + phase_offset) % 66)
            is_green = ns_green if approach in ("North", "South") else ew_green
            # Hold vehicles close to the stop line during red.
            if not is_green and distance < 48:
                distance = 48 + (idx % 3) * 12
            # During green, let the leading cars pass the junction.
            elif is_green and distance < 43:
                distance = 43 + (idx % 4) * 13
            body_xy, win_xy = car_polygons(approach, lane, distance)
            car_patches[k].set_xy(body_xy)
            window_patches[k].set_xy(win_xy)

        phase_label = "NORTH / SOUTH GREEN" if ns_green else (
            "EAST / WEST GREEN" if ew_green else phase.replace("_", " "))
        info.set_text(f"Time: {frame:03d} s   |   Signal: {phase_label}   |   RSU range: {sim.communication_range:.0f} m")
        return car_patches + window_patches + [info] + [
            lamp for lamps in signal_lamps.values() for lamp in lamps
        ]

    anim = FuncAnimation(fig, update, frames=120, interval=100,
                         blit=False, repeat=True)
    try:
        anim.save(outdir / "traffic_animation.gif", writer="pillow", fps=10)
    except Exception as exc:
        print("GIF not saved:", exc)
    plt.close(fig)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=int, default=600)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--no-animation", action="store_true")
    args = parser.parse_args()
    outdir = Path(__file__).resolve().parent / "outputs"
    outdir.mkdir(exist_ok=True)
    adaptive, fixed, metrics = run_comparison(args.duration, args.seed, outdir)
    make_performance_plot(adaptive, fixed, metrics, outdir)
    make_architecture_diagram(outdir)
    make_intersection_diagram(outdir)
    if not args.no_animation:
        animate_simulation(adaptive, outdir)
    print("\n=== SMART TRAFFIC VANET SIMULATION RESULTS ===")
    for result in metrics:
        print("\n" + result["mode"])
        for key, value in result.items():
            if key != "mode":
                print(f"{key}: {value:.3f}" if isinstance(value, float) else f"{key}: {value}")
    print(f"\nOutputs saved in: {outdir}")

if __name__ == "__main__":
    main()
