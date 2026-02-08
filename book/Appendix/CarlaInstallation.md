# Carla Installation

Carla is an **optional** component. You only need it if you plan to run simulations using the Carla simulator.

## Installation

The Carla Python API is installed by default as part of `uv sync`:

```bash
uv sync
```

This installs Carla 0.9.16 automatically.

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
