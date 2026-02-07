# Exercise Setup

You can work on the exercises on your local machine, or in the cloud using Google Colab. Choose the tab that matches your setup. For local work, we recommend [uv](https://docs.astral.sh/uv/) for fast, reliable dependency management.

## Downloading the exercises

Clone the repository or download it as a zip:

```bash
git clone https://github.com/thomasfermi/Algorithms-for-Automated-Driving.git
```

Or visit [the GitHub repo](https://github.com/thomasfermi/Algorithms-for-Automated-Driving), click "Code", and download the zip.

````{tab} Local installation
Nothing more to do.
````

````{tab} Google Colab
Open [Google Drive](https://drive.google.com/drive/my-drive). Create a new folder called "aad". Upload the repo contents to this folder (you can skip the `book` folder).
````

## Python environment

`````{tab} Local installation

### Using uv (recommended)

If you don't have uv, install it:
```bash
pip install uv
```

Then set up the environment:
```bash
cd Algorithms-for-Automated-Driving
uv sync
```

Activate the virtual environment or use `uv run`:
```bash
# Option 1: Activate the venv
source .venv/bin/activate  # Linux/macOS
.venv\Scripts\activate      # Windows

# Option 2: Run commands directly with uv
uv run jupyter lab
```

### Using conda (legacy)

If you prefer conda/mamba:
```bash
# With conda
conda create -n aad python=3.10
conda activate aad
pip install -e .

# Or with mamba (faster)
mamba create -n aad python=3.10
mamba activate aad
pip install -e .
```

`````

`````{tab} Google Colab
Most libraries are pre-installed. If you need something, just import it—Colab will suggest installation if needed.

For the aad package, install in your first cell:
```python
import subprocess
import sys
subprocess.check_call([sys.executable, "-m", "pip", "install", "-e", "/content/drive/MyDrive/path-to-aad"])
```
`````

## Navigating the exercises

The repository structure is:

```
Algorithms-for-Automated-Driving/
├── aad/
│   ├── exercises/     (write your code here)
│   ├── solutions/     (don't peek!)
│   ├── tests/         (run these to test your work)
│   └── util/          (shared utilities)
├── book/              (book source, you can delete)
└── data/              (datasets)
```

Work on exercises by editing files in `aad/exercises/`. Test your code using notebooks in `aad/tests/`.

````{tab} Local installation

### Editing code

We recommend [Visual Studio Code](https://code.visualstudio.com/), which has good Jupyter notebook support.

Open the repo folder:
```bash
code Algorithms-for-Automated-Driving
```

Start Jupyter Lab to edit notebooks:
```bash
uv run jupyter lab
```

Then navigate to the exercise notebook specified in the book. In VS Code, select the `.venv` Python interpreter when opening notebooks.

````

````{tab} Google Colab

Open [Google Drive](https://drive.google.com/drive/my-drive), navigate to your `aad` folder, and double-click a `.ipynb` file. At the top, click "Open with Google Colaboratory".

The first cells mount your Google Drive. After that, you can edit Python files via the folder icon in the left sidebar, and save with Ctrl+S.

````

## Getting help

Questions? Ask on [GitHub Discussions](https://github.com/thomasfermi/Algorithms-for-Automated-Driving/discussions) or [Discord](https://discord.gg/57YEzkCFHN).
