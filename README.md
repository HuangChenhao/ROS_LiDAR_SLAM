# ROSMaster R2 — LiDAR SLAM Toolbox

A collection of Mac-side scripts for operating, diagnosing, and collecting SLAM data from a **Yahboom ROSMaster R2** robot over SSH.

> 中文说明请见 [README_zh.md](README_zh.md)

---

## Hardware

| Component | Details |
|-----------|---------|
| Robot | Yahboom ROSMaster R2 |
| SBC | Raspberry Pi (IP: `192.168.0.110`, user: `pi`) |
| LiDAR | RPLidar (via `/dev/rplidar`) |
| Depth Camera | Astra (via `/dev/astradepth`, `/dev/astrauvc`) |
| Motor Controller | `/dev/myserial` |
| ROS | Melodic, running inside Docker container `rosmaster_slam` |

---

## Repository Structure

```
rosmaster/
├── robot.sh              # Core SSH wrapper — run any command on the robot
├── setup_ssh.sh          # One-time passwordless SSH setup
├── daemon.sh             # Command daemon: watches for trigger, executes remote commands
├── cmd.sh                # Send a command block to the robot
├── run.sh                # Quick run helper
├── remote_exec.sh        # Remote execution helper
├── diagnose.sh           # Full ROS/system diagnostics
├── diagnose2.sh          # Display, Docker X11, and pygame diagnostics
├── fix_and_start.sh      # Fix device mappings and restart ROS/Docker
├── full_check.sh         # Comprehensive system check (devices, Docker, ROS, network)
├── stop_buzzer.sh        # Silence the robot's buzzer
├── sync_data.sh          # Sync SLAM maps, bags, and trajectory from robot to Mac
├── pull_robot_files.sh   # Pull robot export files
├── remote_commands.sh    # Remote command file (written by daemon workflow)
└── visualize_trajectory.py  # Overlay recorded trajectory onto the SLAM map
```

---

## Quick Start

### 1. Passwordless SSH Setup (run once)

```bash
bash ~/Claude/Projects/rosmaster/setup_ssh.sh
```

This copies the SSH key to `~/.ssh/` and adds a `rosmaster` host alias to `~/.ssh/config`. You'll need to enter the robot password (`yahboom`) once.

### 2. Run a Command on the Robot

```bash
bash ~/Claude/Projects/rosmaster/robot.sh "rosnode list"
```

### 3. Sync SLAM Data to Mac

```bash
bash ~/Claude/Projects/rosmaster/sync_data.sh
```

Syncs to `~/Documents/rosmaster_r2/`:

```
rosmaster_r2/
├── maps/
│   ├── overall_map.pgm       # SLAM occupancy grid
│   ├── overall_map.yaml      # Map metadata (resolution, origin)
│   ├── trajectory.csv        # Recorded robot trajectory (x, y, theta)
│   ├── slices/               # Incremental PCD slices
│   └── map_with_trajectory.png  # Visualization output
├── bags/                     # ROS bag files
└── logs/                     # Service and recording logs
```

### 4. Visualize Trajectory

After syncing, generate a trajectory overlay image:

```bash
python3 ~/Claude/Projects/rosmaster/visualize_trajectory.py ~/Documents/rosmaster_r2/maps
```

Outputs `map_with_trajectory.png` (and `.ppm` fallback). Red dots = trajectory, green = start, blue = end.

---

## Diagnostic Scripts

| Script | Purpose |
|--------|---------|
| `diagnose.sh` | ROS nodes/topics, processes, systemd services, GPIO, I2C |
| `diagnose2.sh` | Display environment, Docker X11 mounts, pygame install |
| `full_check.sh` | Devices, Docker inspect, ROS setup, disk/memory/load |
| `fix_and_start.sh` | Reload udev rules, recreate Docker container, restart `rosmaster-slam.service` |
| `stop_buzzer.sh` | Kill buzzer via ROS topic or GPIO |

All diagnostic scripts save output to `*_output.txt` files (excluded from git).

---

## Daemon Workflow

For repeated command execution without re-establishing SSH each time:

```bash
# Terminal 1 — start the daemon
bash ~/Claude/Projects/rosmaster/daemon.sh

# Terminal 2 — write commands to remote_commands.sh, then trigger
touch ~/Claude/Projects/rosmaster/.trigger
# daemon detects .trigger, runs remote_commands.sh on the robot, writes result to cmd_output.txt
```

---

## Requirements

- macOS with `bash`, `ssh`, `scp`
- Python 3 with `numpy`, `pyyaml` (for `visualize_trajectory.py`)
- Optional: `Pillow` for PNG output (`pip3 install Pillow`)
- Robot reachable at `192.168.0.110` on the same LAN

---

## Security Note

The `.ssh/` directory containing the private key is excluded from this repository via `.gitignore`. Never commit SSH private keys.
