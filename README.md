# irobot
### *Intelligent. Modular. Build-Free.*

**irobot** is a Python library of robot drivers developed at the **iHuman Lab**. Each driver exposes hardware access through a clean, importable Python class. Install irobot, import the driver for your robot, and build your application on top — no framework lock-in required.

Drivers work standalone in Python or through optional ROS 2 components. Sawyer provides a direct Humble node and an optional ros_sugar application adapter. Custom Sawyer interfaces require an explicit CMake build; the core driver does not.

---

## 🤖 Supported Robots

This branch contains the Sawyer driver.

| Robot | Description | Driver Docs |
|-------|-------------|-------------|
| Sawyer | Direct Python connection for the right arm and Intera electric gripper (in development) | [irobot/src/robots/sawyer/README.md](irobot/src/robots/sawyer/README.md) |

---

## Prerequisites

- Python 3.11 or higher
- Network access to Sawyer (see the driver README for connection setup)
- **ROS2 + ros_sugar** — only required if you are using ROS2-based components (optional)
- **Sawyer (`sawyer_api`):** direct Python control through the irobot driver, with an optional ROS 2 Humble adapter using the same backend. No workstation ROS 1 runtime or bridge is required. See the driver implementation guide and qualification checklist.

---

## 📦 Installation

Clone the repository and install in editable mode:

```bash
git clone https://github.com/iHumanLab/irobot.git
cd irobot
python3 -m pip install -e '.[sawyer]'
```

See the Sawyer README for connection settings and the optional ROS 2 runtime.

---

## 🚀 Quick Start

Import the controller and configuration class for your robot, configure it, and call the driver's methods:

```python
from irobot import SawyerConfig, SawyerController

config = SawyerConfig(ros_master_uri='http://ROBOT_IP:11311', ros_ip='PC_IP')
with SawyerController(config) as robot:
    print(robot.get_status())
    # Preview without moving; requires usable robot feedback:
    # preview = robot.move_to_neutral()
```

The development `main.py` runs the Sawyer neutral example: `python main.py` previews and `python main.py --execute` moves the arm. Configure the direct Sawyer connection as described in its driver guide.

Sawyer development references: [files and functions](irobot/src/robots/sawyer/IMPLEMENTATION_GUIDE.md)
and [unfinished work](irobot/src/robots/sawyer/UNFINISHED_WORK.md).

See the [API reference](API_REFERENCE.md) for existing Python methods,
configuration fields, ROS 2 endpoints, and custom interface schemas.

The [Sawyer architecture proposal](SAWYER_ARCHITECTURE_PROPOSAL.md) describes
layered modules, irobot template compatibility, and suggested template improvements.

Each robot exports its classes at the top level of the `irobot` package. See the [Supported Robots](#-supported-robots) table above for links to robot-specific documentation.

---

## 📁 Repository Structure

```
irobot/
├── main.py                    ← demo launcher (development use only)
│
└── irobot/src/
    └── robots/                ← one folder per supported robot
        └── <robot_name>/
            ├── core/          ← connection, controller, and supporting layers
            ├── examples/      ← usage examples and ROS2 component templates
            ├── config.py      ← all hardware parameters in one dataclass
            └── README.md      ← setup guide and full API documentation
```

Each robot folder is self-contained: the driver, its configuration, its examples, and its hardware setup guide all live together. When you want to use a robot, start with its `README.md`.

---

## 🤖 Adding a New Robot

Contributions are welcome. Follow these steps to add a new robot driver:

1. Create a folder at `irobot/src/robots/<robot_name>/`
2. Add `config.py` — a Python dataclass holding all hardware parameters (addresses, timeouts, rates). This is the only place parameters should live.
3. Add `core/base.py` — own the connection/session and expose measured state, model queries, and device access. Construction prepares the driver; explicit `connect()` / `disconnect()` manage hardware resources. Support context-manager cleanup.
4. Add `core/controller.py` — public operations built on the base class. Coordinate planning and execution through supporting modules when needed.
5. Add `core/logging.py` (optional) — a mixin for onboard sensor logging, if applicable.
6. Add `examples/` — at least one runnable Python example and, if applicable, a ROS2 component template.
7. Add a `README.md` — hardware setup instructions (drivers, udev rules, pairing) plus configuration and API documentation.
8. Add `__init__.py` files throughout and export the actual robot-specific classes from `irobot/__init__.py`. Keep ROS imports in optional adapters.

For a larger driver, use `core/planning/`, `core/execution/`,
`core/hardware/`, and `core/communication/`. Small drivers can keep a few
focused files. Keep settings and operation options in `config.py`; examples
should call the public controller API.

| Part | Responsibility |
| --- | --- |
| `base.py` | Connection, state, model, and device access |
| `controller.py` | Public operations and coordination |
| `planning/` (optional) | Paths and timing |
| `execution/` (optional) | Tracking and operation cleanup |
| ROS adapters (optional) | Request conversion and telemetry |

---

## 📌 Design Conventions

- **One `config.py` per robot** — all hardware parameters belong there and nowhere else.
- **`robots/` is hardware-only** — drivers provide hardware access through SDKs or native protocols. No experiment logic, application state, or project-specific constants belong here.
- **`examples/` travels with the robot** — examples are co-located with the robot they demonstrate and ship as part of the driver.

---

*iHuman Lab — Intelligent Human-Machine Nexus*
