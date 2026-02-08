# Refactoring Complete: uv + Package Structure + Jupyter Book 1.x (Feb 7, 2026)

## TODO for Mario

Before merging `modernize` into `master`:

1. **Test all notebooks** - Run each notebook in `aad/tests/` to verify they execute without errors
2. **Read the book carefully** - Build with `uv run jupyter-book build book` and review all content, especially updated paths and examples
3. **Try the Carla simulator** - If Carla is available, test `uv run python -m aad.tests.control.carla_sim` and `uv run python -m aad.tests.camera_calibration.carla_sim`
4. **Verify exercise notebooks** - Open a few exercise notebooks in Jupyter Lab locally and confirm absolute imports work

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

# Installation (all dependencies included)
uv sync
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
- [x] Updated test notebooks with improved Colab setup (6 notebooks)
- [x] Modernized all `code.tests` → `aad.tests` references
- [x] Updated all commands to use `uv run python`
- [x] Removed stray syntax errors from notebooks

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
- [x] Enforced light mode only (custom JS + CSS workaround)
- [x] Updated all `code/` paths to `aad/` in book content
- [x] Added `aad.egg-info/` to `.gitignore`
- [x] Updated GitHub workflow to use `uv` and `astral-sh/setup-uv@v3`

**Testing & Documentation**
- [x] All extras tested independently and together
- [x] Updated `book/Appendix/ExerciseSetup.md` (uv-focused, removed manual aad install)
- [x] Updated `book/Appendix/CarlaInstallation.md` (simplified with direct GitHub link)
- [x] Book builds successfully with all notebooks executing
- [x] Jupyter Book 1.0.4.post1 build verified with no breaking changes
- [x] Test notebooks automatically detect Colab environment
- [x] All absolute imports verified in test notebooks

## Known Issues & Workarounds

**Theme Persistence Bug (Sphinx Book Theme 1.1.4)**: In Firefox, dark/light mode toggle state did not persist across page navigation. This was caused by the theme JavaScript reading localStorage on every page load, overriding the hardcoded HTML `data-theme` attribute.

**Workaround Implemented**:
1. Created `book/_static/force-light-mode.js` - Clears localStorage and forces light mode on every page load
2. Created `book/_static/hide-theme-toggle.css` - Hides the theme switcher button from the UI entirely
3. Added `recursive_update: true` to Sphinx config - Ensures config options apply correctly
4. Added documentation to `book/_config.yml` explaining the three-part fix

**Result**: Light mode is now enforced consistently across all pages and browsers. Users cannot enable dark mode (button is hidden and localStorage is cleared).

**HTML Rendering**: Local build differs slightly from live site (styling of inline code). Root cause TBD—likely theme or CSS differences. Content and structure are correct.

## Git Workflow

All refactoring work completed in February 2026 has been organized into a clean feature branch:

- **`master`** - Original state (preserved as-is)
- **`modernize`** - All 2026 changes (multiple commits, including Phase 4 improvements)
- **`backup-2026`** - Safety backup of modernize branch

To merge modernize into master when ready:
```bash
git checkout master
git merge modernize
git push origin master
```

## Files Changed (Phase 1 + 2 + 3 + 4)

### Phase 1 & 2 (Core Modernization)
- `pyproject.toml` - Created (new, with uv + JB 1.x deps)
- `uv.lock` - Generated (new, includes all transitive deps)
- `/aad/` - Renamed from `/code/`
- `book/_config.yml` - Updated for JB 1.x, removed analytics, added theme workaround
- `book/Appendix/*.md` - Updated for uv/modern setup
- Book content `.md` and `.ipynb` files - Paths updated from `code/` to `aad/`
- `.gitignore` - Added `aad.egg-info/`
- 7 Python files - Imports fixed
- 10 Notebooks - `sys.path` removed

### Phase 3 (Theme & Workflow Fixes)
- `book/_static/force-light-mode.js` - Custom script to enforce light mode
- `book/_static/hide-theme-toggle.css` - CSS to hide theme toggle button
- `.github/workflows/book.yml` - Updated to use `uv` and `astral-sh/setup-uv@v3`

### Phase 4 (Colab & Test Improvements)
- 6 Test notebooks in `aad/tests/` - Added Colab detection, mount, and aad package install cells
- Test module paths - Updated from `code.tests` → `aad.tests`
- Commands updated - All to use `uv run python`
- Stray syntax errors removed from notebooks


# newest user findings
a) search any .py and .ipynb files for string "code/", to find any remaining references to the old code directory structure. Fix it to say "aad/" instead
b) Search in the book directory for any remaining references to the old code directory structure. Fix it to say "aad/" instead. this might be in markdown files also
c) search in the book directory for any commands that say "run python" and replace it with "uv run python". I guess also look in the comments of all .py and .ipynb files, where there might be comments that say "please run python" and replace it with "please run uv run python"

d) in carla_sim.py it looks like it is night. can you make it day? maybe there is some problem since we upgraded to carla 0.9.16 (by the way also search in the whole code base for 0.9.* and see if we incorrectly talk about the wrong carla version)

e) Since our main dependency pytorch is so huge, we can make the optional dependencies required. This will make the installation process more straightforward and won't make a big difference anyway. When you do this change search for "uv sync --extra" and change the command to "uv sync" (since we are not using extra dependencies anymore after this change)

progress:
a) [x] Fixed - Searched .py and .ipynb files; found outdated paths in 2 Colab notebooks (lane_segmentation.ipynb in exercises and solutions)
b) [x] Fixed - Verified no "code/" refs in book .md files; checked .ipynb files (no issues found)
c) [x] Fixed - Updated PurePursuit.md (2 instances of `python -m aad...` → `uv run python -m aad...`); verified all other commands already use `uv run`
d) [x] Fixed - Created get_weather_clear_noon() function in carla_util.py; updated 3 files (control/carla_sim.py, camera_calibration/carla_sim.py, collect_data.py) to use daytime preset
e) [x] Fixed - Moved carla and book from optional extras to required dependencies in pyproject.toml; updated all documentation (INSTALLATION.md, CarlaInstallation.md, REFACTORING_PLAN.md) to remove `--extra` flags

## Phase 5: Colab-Optimized Notebooks (In Progress - Phase 5a Complete)

**Problem**: Colab has conflicting Python versions (3.12 vs required 3.10), no `uv` support, and pre-installed optimized PyTorch. Current approach of `uv sync` in Colab fails.

**Solution**: Create separate Colab-optimized notebook copies (`*_colab.ipynb`) that avoid dependency management entirely.

### Strategy

1. For each `.ipynb` in the repo, create a "sister" notebook with `_colab` suffix
   - Example: `inverse_perspective_mapping.ipynb` → `inverse_perspective_mapping_colab.ipynb`

2. Initial population: Copy from `master` branch (original working notebooks)
   - Avoids conflicts between modernized local notebooks and Colab setup
   - Starting point is known-working Colab environment

3. **Phase 5a: COMPLETE** - Minimal adaptation with 1 notebook pilot
   - ✅ Created `aad/tests/lane_detection/inverse_perspective_mapping_colab.ipynb` from master branch
   - ✅ Updated import paths: `code` → `aad`
   - ✅ Enforces Colab-only execution (throws error if run locally)
   - ✅ Simplified setup: Colab detection → Drive mount → sys.path injection
   - ✅ Removed `uv sync` and `pip install` entirely
   - ✅ Uses relative `sys.path` approach: `sys.path.append(str(Path('../../')))`  to import exercises/solutions modules
   - ✅ Tested successfully in Colab (user confirmed working)

4. **Phase 5b: COMPLETE** - Roll out to all notebooks
   - ✅ Applied pattern to remaining 7 test, exercise, and solution notebooks
   - ✅ Created `*_colab.ipynb` versions for all `.ipynb` files in `aad/tests/`, `aad/exercises/`, `aad/solutions/`

5. Maintain two parallel versions:
   - **Local**: Modern approach (uv, absolute imports, `code` → `aad`)
   - **Colab**: Lightweight approach (Colab-only check, Drive mount, sys.path, no uv/pip)

### Phase 5a Pilot Results

**Phase 5a Pilot (Tested)**:
- `aad/tests/lane_detection/inverse_perspective_mapping_colab.ipynb`
  - Works in Colab with modern `aad/` package structure
  - Avoids Python version conflicts
  - No dependency installation needed
  - Tested and confirmed working

**Phase 5b Rollout (All Remaining Notebooks)**:
Tests (5):
- `aad/tests/lane_detection/lane_boundary_projection_colab.ipynb`
- `aad/tests/lane_detection/lane_detector_colab.ipynb`
- `aad/tests/camera_calibration/calibrated_lane_detector_colab.ipynb`
- `aad/tests/control/control_colab.ipynb`
- `aad/tests/control/target_point_colab.ipynb`

Exercises (1):
- `aad/exercises/lane_detection/lane_segmentation_colab.ipynb`

Solutions (1):
- `aad/solutions/lane_detection/lane_segmentation_colab.ipynb`

**Key implementation details**:
- Cell 1: Colab environment check (raises exception if run locally)
- Cell 2: Google Drive mount to `/content/drive`
- Cell 3+: sys.path append for relative imports (exercises, solutions)
- No `%autoload` magic (causes module loading issues in Colab)

**Exact Changes for Phase 5b Rollout**:

To create `*_colab.ipynb` notebooks from modernized versions, apply these transformations using Python:

```python
import json
from pathlib import Path

def create_colab_notebook(input_notebook_path, output_notebook_path, relative_module_path):
    """
    Convert a modernized notebook to Colab-optimized version.
    
    Args:
        input_notebook_path: Path to original modernized .ipynb
        output_notebook_path: Path to output *_colab.ipynb
        relative_module_path: Relative path for imports, e.g., '../../' for tests
    """
    with open(input_notebook_path) as f:
        nb = json.load(f)
    
    # CHANGE 1: Remove first N cells until we find content after autoreload
    # Keep markdown cells, remove autoreload/import cells
    cells_to_keep = []
    skip_until_markdown = False
    
    for i, cell in enumerate(nb['cells']):
        # Skip code cells that have autoreload, uv sync, or pip install
        if cell['cell_type'] == 'code':
            source = ''.join(cell['source'])
            if any(x in source for x in ['%autoload', '%load_ext', 'uv sync', 'pip install', 'subprocess']):
                skip_until_markdown = True
                continue
        
        # Keep markdown cells (these are section headers)
        if cell['cell_type'] == 'markdown':
            skip_until_markdown = False
        
        if not skip_until_markdown or cell['cell_type'] == 'markdown':
            cells_to_keep.append(cell)
    
    # CHANGE 2: Insert Colab setup cells at the beginning (after first markdown title)
    colab_setup_cells = [
        {
            "cell_type": "code",
            "source": ["if not 'google.colab' in str(get_ipython()):\n    raise Exception(\"You should only run this notebook in colab!\")"],
            "metadata": {},
            "execution_count": None,
            "outputs": []
        },
        {
            "cell_type": "code",
            "source": ["from google.colab import drive\ndrive.mount('/content/drive')\n%cd drive/MyDrive/aad/" + relative_module_path.split('/')[-2].split('.')[-1] + "\n"],
            "metadata": {},
            "execution_count": None,
            "outputs": []
        }
    ]
    
    # Insert after title (first markdown cell)
    if cells_to_keep and cells_to_keep[0]['cell_type'] == 'markdown':
        cells_to_keep = cells_to_keep[:1] + colab_setup_cells + cells_to_keep[1:]
    else:
        cells_to_keep = colab_setup_cells + cells_to_keep
    
    # CHANGE 3: Update all import statements
    for cell in cells_to_keep:
        if cell['cell_type'] == 'code':
            source = ''.join(cell['source'])
            # Add sys.path for exercises/solutions imports
            if 'from exercises.' in source or 'from solutions.' in source:
                if 'sys.path.append' not in source:
                    source = f"import sys\nfrom pathlib import Path\nsys.path.append(str(Path('{relative_module_path}')))\n" + source
            cell['source'] = [source]
    
    # Save
    nb['cells'] = cells_to_keep
    with open(output_notebook_path, 'w') as f:
        json.dump(nb, f, indent=1)
```

**Manual adjustments needed (from Phase 5a pilot)**:
1. **Remove `%autoload` and `%load_ext`** - These cause `ModuleNotFoundError` in Colab
2. **Update `cd` paths** - Change from `code/` to `aad/` in path navigation
3. **Update command execution** - Change `!python -m code.` to `!python -m aad.`
4. **Add sys.path injection** - Before importing exercises/solutions modules, add:
   ```python
   import sys
   from pathlib import Path
   sys.path.append(str(Path('../../')))  # Adjust relative path as needed
   ```
5. **Keep relative imports** - Unlike modernized notebooks (which use absolute `aad.exercises`), Colab versions use relative imports (`from exercises.`, `from solutions.`) with sys.path

**Example: Converting `lane_boundary_projection.ipynb` to `lane_boundary_projection_colab.ipynb`**:
- Remove cells with `%autoload` and `%load_ext autoreload`
- Replace first setup cell with Colab check + Drive mount (pointing to `/content/drive/MyDrive/aad/tests/lane_detection`)
- Keep remaining content unchanged (import statements already use relative imports from master branch)
- Verify unit test commands use `python -m aad.` format

## Phase 5c: Testing & Documentation (Next Steps)

**Remaining work**:
1. **User testing** - Verify a few `*_colab.ipynb` notebooks work in Colab (spot-check 2-3)
2. **Update documentation** - Add Colab instructions to `book/Appendix/ExerciseSetup.md` with links to `*_colab.ipynb` versions
3. **GitHub links** - Add note in README.md directing Colab users to the `*_colab.ipynb` versions with "Open in Colab" buttons
4. **Maintain both versions** - When adding new notebooks, create both local and `*_colab.ipynb` versions

**Testing checklist**:
- [ ] Load `inverse_perspective_mapping_colab.ipynb` in Colab (already tested)
- [ ] Test 1-2 more from different categories (e.g., `control_colab.ipynb`, `lane_segmentation_colab.ipynb`)
- [ ] Verify all imports resolve correctly
- [ ] Verify unit tests execute (cells with `!python -m aad.tests...`)
