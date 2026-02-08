# Installation Guide

This project uses **uv** for fast, reliable Python dependency management.

## Prerequisites

- [uv](https://docs.astral.sh/uv/)

## Quick Start

```bash
git clone https://github.com/thomasfermi/Algorithms-for-Automated-Driving.git
cd Algorithms-for-Automated-Driving

# Create virtual environment and install
uv sync

# Use the package
uv run python -c "from aad.exercises.lane_detection import CameraGeometry; print('✓ Works!')"
```

All dependencies including Carla and Jupyter Book are included by default.

**Note:** The Carla Python package is just the client API. To use the simulator, you still need to download and run the [Carla simulator server in version 0.9.16](https://github.com/carla-simulator/carla/releases/tag/0.9.16).

To build the Jupyter Book:

```bash
uv run jupyter-book build book/
```

## Development Workflow

### Activate the Virtual Environment

```bash
source .venv/bin/activate  # Linux/macOS
.venv\Scripts\activate      # Windows
```

Or use `uv run` to run commands without activating:

```bash
uv run python script.py
uv run jupyter lab
uv run pytest
```

### Add New Dependencies

Edit `pyproject.toml` and run:

```bash
uv sync  # or uv sync --all-extras if using extras
```

The `uv.lock` file is committed to git for reproducible installs.

## Google Colab

To install in Google Colab (from a mounted repo):

```python
import subprocess
import sys

# Install aad package (base install)
subprocess.check_call([
    sys.executable, "-m", "pip", "install", "-e",
    "/content/drive/MyDrive/path-to-repo"
])

# Or with carla
subprocess.check_call([
    sys.executable, "-m", "pip", "install", "-e",
    "/content/drive/MyDrive/path-to-repo[carla]"
])

# Then use as normal
from aad.exercises.lane_detection import CameraGeometry
```

## Project Structure

```
aad/
├── exercises/          # Student exercises
│   ├── lane_detection/
│   ├── camera_calibration/
│   └── control/
├── solutions/          # Solution code
│   ├── lane_detection/
│   ├── camera_calibration/
│   └── control/
├── tests/              # Test scripts and notebooks
│   ├── lane_detection/
│   ├── camera_calibration/
│   └── control/
└── util/               # Shared utilities
    ├── carla_util.py
    ├── geometry_util.py
    └── seg_data_util.py

book/                  # Jupyter Book source
├── CameraCalibration/
├── LaneDetection/
├── Control/
└── Appendix/
```

## Import Examples

```python
# From exercises
from aad.exercises.lane_detection import CameraGeometry, LaneDetector
from aad.exercises.control import pure_pursuit

# From solutions
from aad.solutions.lane_detection import LaneDetector as SolutionLaneDetector
from aad.solutions.control import PurePursuitPlusPID

# From utilities
from aad.util.carla_util import carla_vec_to_np_array, CarlaSyncMode
from aad.util.geometry_util import rotation_matrix
```

## Troubleshooting

### "No module named 'aad'"

- Ensure you ran `uv sync` in the repo root
- Or activated the venv: `source .venv/bin/activate`

### "No module named 'carla'"

- Ensure `uv sync` was run in the repo root
- Carla server must be running separately if using the simulator

### "No module named 'jupyter_book'"

- Ensure `uv sync` was run in the repo root

### Dependency conflicts

- Delete `.venv` and `uv.lock`, then run `uv sync` again
- Report issues on the [GitHub repository](https://github.com/thomasfermi/Algorithms-for-Automated-Driving)

## More Info

See [REFACTORING_PLAN.md](REFACTORING_PLAN.md) for details on the migration from conda to uv, and [book/Appendix/CarlaInstallation.md](book/Appendix/CarlaInstallation.md) for Carla-specific setup.
