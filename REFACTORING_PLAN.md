# Refactoring Complete: uv + Package Structure (Feb 7, 2025)

## Overview

Migrated from conda/pixi-based setup to modern **uv** package manager with consolidated `pyproject.toml` and proper `aad` package structure.

## Key Changes

| Component | Before | After |
|-----------|--------|-------|
| **Package structure** | `/code/` folder with relative imports | `/aad/` package with absolute imports |
| **Dependencies** | Separate `environment.yml` in code/, book/ | Single `pyproject.toml` at repo root |
| **Dependency manager** | Pixi (conda-based) | uv (PyPI-based) |
| **Python version** | 3.7 (EOL) | 3.10 (carla 0.9.16 support) |
| **Carla** | Manual .pth file, 0.9.10 | `pip install` via optional extra, 0.9.16 |
| **Book build** | jupyter-book 0.13.2 | jupyter-book 0.14.0 with sphinx 5.0+ |

## Installation

```bash
git clone https://github.com/thomasfermi/Algorithms-for-Automated-Driving.git
cd Algorithms-for-Automated-Driving

# Base installation
uv sync

# With Carla (optional)
uv sync --extra carla

# With book tools (optional)
uv sync --extra book

# Everything
uv sync --all-extras
```

## Completion Checklist

**Package & Imports**
- [x] Renamed `/code/` → `/aad/` at repo root
- [x] Converted all relative imports to absolute (`from aad.*)
- [x] Removed all `sys.path.append()` hacks from notebooks (10 notebooks)
- [x] All subdirectories have `__init__.py`

**Dependencies**
- [x] Created root `pyproject.toml` with setuptools config
- [x] Pinned Python 3.10.* for carla 0.9.16 compatibility
- [x] Added optional extras: `[carla]`, `[book]`
- [x] Generated `uv.lock` for reproducible installs

**Testing & Documentation**
- [x] All extras tested independently and together
- [x] Updated `book/Appendix/ExerciseSetup.md` (uv-focused)
- [x] Updated `book/Appendix/CarlaInstallation.md` (simplified)
- [x] Book builds successfully with all notebooks executing

## Known Issues

**HTML Rendering**: Local build differs slightly from live site (styling of inline code). Root cause TBD—likely theme or CSS differences. Content and structure are correct.

## Files Changed

- `pyproject.toml` - Created (new)
- `uv.lock` - Generated (new)
- `/aad/` - Renamed from `/code/`
- Book appendices - Updated for uv/modern setup
- 7 Python files - Imports fixed
- 10 Notebooks - `sys.path` removed
