# Carla Installation

Carla is an **optional** component. You only need it if you plan to run simulations using the Carla simulator.

## Installation

The Carla Python API is installed by default as part of `uv sync`. But the Python package (`carla`) is just the client API. You must download and run the **Carla simulator server separately**:

Example on Linux:
* Visit [carla release 0.9.16 on github](https://github.com/carla-simulator/carla/releases/tag/0.9.16) and download "CARLA_0.9.16.tar.gz"
* Extract the archive to a folder on your machine
* With your terminal go into the folder and run `./CarlaUE4.sh`
* In another terminal, run your Python scripts that import carla. Example: From root of this repo run `uv run python -m aad.tests.control.carla_sim`
