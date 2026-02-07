# Carla Installation

Carla is an **optional** component. You only need it if you plan to run simulations using the Carla simulator.

## Installation

The Carla Python API is installed as an optional dependency:

```bash
uv sync --extra carla
```

This installs Carla 0.9.16.

## Important: Carla Server

The Python package (`carla`) is just the client API. You must download and run the **Carla simulator server separately**:

1. Download from [GitHub releases](https://github.com/carla-simulator/carla/releases) (version 0.9.16)
2. Extract and run the simulator executable
3. The Python API will connect to this running server

Example on Linux:
```bash
# Download and extract
wget https://github.com/carla-simulator/carla/releases/download/0.9.16/CARLA_0.9.16.tar.gz
tar xzf CARLA_0.9.16.tar.gz
cd CARLA_0.9.16

# Run simulator (stays running)
./CarlaUE4.sh
```

In another terminal, run your Python scripts that import carla.

## Troubleshooting

**"No module named 'carla'"**
- Install with: `uv sync --extra carla`

**"Failed to connect to Carla server"**
- Ensure Carla simulator is running (see above)
- Default connection: `localhost:2000`
