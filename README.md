# Episode1 MuJoCo Upper Computer

An open-source upper-computer application for the Episode1 six-axis robotic
arm. It provides a browser-based control panel, offline forward/inverse
kinematics, trajectory planning, local 3D preview, generated motion scripts,
and an independent MuJoCo simulation process.

一个用于 Episode1 六轴机械臂的上位机程序，包含网页控制台、离线正逆运动学、
轨迹规划、本地 3D 预览、运动脚本生成，以及独立运行的 MuJoCo 仿真程序。

## Features

- **Two web interfaces**: `/v2/base` for joint and gripper control, `/v2/viz`
  for 3D visualization and trajectory programming.
- **Process isolation**: the upper-computer server and MuJoCo simulator run as
  separate processes and communicate through a TCP protocol with CAN semantics.
- **Motion planning**: FK/IK, reachability checks, self-collision checks,
  interpolation, playback, and Python trajectory script generation.
- **Simulation and hardware abstraction**: the same control path is designed to
  support both the MuJoCo bridge and a real Episode1 arm.
- **Local-first**: the service binds to `127.0.0.1`; no cloud account or remote
  backend is required.

## Requirements

- macOS 13 or newer, or Windows 10/11
- Python 3.9-3.13 (tested with Python 3.12)
- MuJoCo 3.17.0, Mink 1.4.0, and NumPy

The Windows launcher is included, but the complete Windows runtime is not yet
end-to-end verified. Linux is also not yet verified.

## Quick Start

```bash
git clone https://github.com/zyj613503-hub/episode1-mujoco-controller.git
cd episode1-mujoco-controller
python3.12 -m venv .venv
.venv/bin/python -m pip install -U pip
.venv/bin/python -m pip install -r requirements.txt
.venv/bin/python 运行器/server/app.py
```

Then open <http://127.0.0.1:8770/v2/base>.

On macOS, after installing Python, you can also:

1. Double-click `1-安装依赖.command` to create `.venv` and install dependencies.
2. Double-click `2-启动上位机.command` to start the server and open the
   control panel.

On Windows, create the environment from Command Prompt in this folder:

```bat
py -3.12 -m venv .venv
.venv\Scripts\python.exe -m pip install -U pip
.venv\Scripts\python.exe -m pip install -r requirements.txt
```

Then double-click `2-启动上位机.bat` to start the server and open the control
panel. Closing its terminal window stops the service.

To use a different port:

```bash
UPPER_PORT=8780 .venv/bin/python 运行器/server/app.py
```

On Windows, run `set UPPER_PORT=8780` before starting
`2-启动上位机.bat` from Command Prompt.

## Usage

- **Basic control** (`/v2/base`): start the MuJoCo bridge, adjust six joint
  targets and the gripper, apply or recover the home pose, and use emergency
  stop.
- **3D programming** (`/v2/viz`): inspect the local 3D model, enter TCP poses,
  solve IK, edit trajectory waypoints, preview motion, and generate runnable
  Python scripts.

See [安装和使用说明.md](安装和使用说明.md) for the complete Chinese guide,
coordinate conventions, hardware notes, and troubleshooting.

## Architecture

```text
Browser (/v2/base, /v2/viz)
          |
          | HTTP
          v
运行器/server/app.py + v2.py
          |
          | TCP 47999, CAN-style messages
          v
bridge/sim_bridge.py
          |
          | MuJoCo
          v
model/scene_episode1.xml
```

- `运行器/`: web UI, HTTP API, scene helpers, and server-side application code.
- `robot/`: conventions, forward/inverse kinematics, planning, script
  generation, and simulation/hardware adapters.
- `bridge/`: independent MuJoCo simulation process.
- `model/`: MuJoCo XML models and mesh assets.
- `scripts/`: generated or example trajectory scripts.

## Safety and Known Limitations

- Real-hardware control has not been verified end-to-end on a physical arm.
- Start with small, slow motions when testing real hardware and keep an
  emergency stop available.
- The macOS workflow has been fully validated. The Windows launcher is
  provided, but the Windows runtime still needs end-to-end hardware and
  simulation verification.
- The MuJoCo simulation is more ideal than real joint tracking and does not
  represent final hardware latency or calibration.

## Third-Party Components

This project includes Three.js r147, distributed under the MIT License. Its
license header is preserved in
`运行器/web/v2/vendor/three.min.js`. Episode1 model and CAD assets may also
carry upstream manufacturer terms; review those terms before commercial
redistribution.

## License

Source code and documentation in this repository are released under the
[MIT License](LICENSE).
