# Refactoring Complete: uv + Package Structure + Jupyter Book 1.x (Feb 7, 2025)

## Overview

Migrated from conda/pixi-based setup to modern **uv** package manager with consolidated `pyproject.toml` and proper `aad` package structure. Additionally upgraded to Jupyter Book 1.0.4.post1 with modernized book configuration.

## Key Changes

| Component | Before | After |
|-----------|--------|-------|
| **Package structure** | `/code/` folder with relative imports | `/aad/` package with absolute imports |
| **Dependencies** | Separate `environment.yml` in code/, book/ | Single `pyproject.toml` at repo root |
| **Dependency manager** | Pixi (conda-based) | uv (PyPI-based) |
| **Python version** | 3.7 (EOL) | 3.10 (carla 0.9.16 support) |
| **Carla** | Manual .pth file, 0.9.10 | `pip install` via optional extra, 0.9.16 |
| **Book build** | jupyter-book 0.13.2 | jupyter-book 1.0.4.post1 with sphinx 7.0+ |
| **Book theme** | Sphinx Book Theme 0.4 | Sphinx Book Theme 1.1.4 |
| **Analytics** | Google Analytics tracking | Disabled |
| **Book display** | Light/dark mode toggle | Light mode only |

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

## Building the Book

```bash
uv run jupyter-book build book
```

Output HTML will be in `book/_build/html/`

## Completion Checklist

**Package & Imports**
- [x] Renamed `/code/` → `/aad/` at repo root
- [x] Converted all relative imports to absolute (`from aad.*)
- [x] Removed all `sys.path.append()` hacks from notebooks (10 notebooks)
- [x] All subdirectories have `__init__.py`

**Dependencies (Phase 1 - uv migration)**
- [x] Created root `pyproject.toml` with setuptools config
- [x] Pinned Python 3.10.* for carla 0.9.16 compatibility
- [x] Added optional extras: `[carla]`, `[book]`
- [x] Generated `uv.lock` for reproducible installs

**Modernization (Phase 2 - Jupyter Book 1.x upgrade)**
- [x] Upgraded jupyter-book: 0.14.0 → 1.0.4.post1
- [x] Upgraded sphinx: 5.0+ → 7.0+ (required by JB 1.0.4)
- [x] Updated myst-parser: 0.18.1 → 3.0.1
- [x] Updated myst-nb: 0.17.2 → 1.3.0
- [x] Removed Google Analytics tracking code
- [x] Enforced light mode only (`dark_mode_enabled: false`)
- [x] Updated all `code/` paths to `aad/` in book content
- [x] Added `aad.egg-info/` to `.gitignore`

**Testing & Documentation**
- [x] All extras tested independently and together
- [x] Updated `book/Appendix/ExerciseSetup.md` (uv-focused)
- [x] Updated `book/Appendix/CarlaInstallation.md` (simplified)
- [x] Book builds successfully with all notebooks executing
- [x] Jupyter Book 1.0.4.post1 build verified with no breaking changes

## Known Issues

**HTML Rendering**: Local build differs slightly from live site (styling of inline code). Root cause TBD—likely theme or CSS differences. Content and structure are correct.

## Git Workflow

All refactoring work completed in February 2025 has been organized into a clean feature branch:

- **`master`** - Original state (preserved as-is)
- **`modernize`** - All 2025 changes squashed into 1 commit (69 files changed)
- **`backup-2026`** - Safety backup of modernize branch

To merge modernize into master when ready:
```bash
git checkout master
git merge modernize
git push origin master
```

## Files Changed (Phase 1 + 2)

- `pyproject.toml` - Created (new, updated with JB 1.x deps)
- `uv.lock` - Generated (new, includes all transitive deps)
- `/aad/` - Renamed from `/code/`
- `book/_config.yml` - Updated for JB 1.x, removed analytics, light mode only
- `book/Appendix/*.md` - Updated for uv/modern setup
- Book content `.md` and `.ipynb` files - Paths updated from `code/` to `aad/`
- `.gitignore` - Added `aad.egg-info/`
- 7 Python files - Imports fixed (Phase 1)
- 10 Notebooks - `sys.path` removed (Phase 1)
