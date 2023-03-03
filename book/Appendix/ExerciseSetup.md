# Exercise Setup

You can work on the exercises on your local machine, or in the cloud using Google Colab. Dependent on your choice, please select the corresponding tab in the following tutorial. If you know how to work with [anaconda](https://www.anaconda.com/products/individual) and are ok with an anaconda environment taking more than 1GB of disk space on your machine, I would recommend you to use your local machine. However, there is one deep learning exercise where you temporarily might want to switch to Colab, if you do not own a GPU. 

## Downloading the exercises

If you know how [git](https://git-scm.com/) works, please clone this [book's github repo](https://github.com/thomasfermi/Algorithms-for-Automated-Driving).
```bash
git clone https://github.com/thomasfermi/Algorithms-for-Automated-Driving.git
```
Otherwise visit this [book's github repo](https://github.com/thomasfermi/Algorithms-for-Automated-Driving) and click on the green button that says "Code". In the pop-up menu, please select "Download zip". Extract the zip to a directory of your choice.

````{tab} Local installation
Nothing more to do.
````

````{tab} Google Colab
Open [Google Drive](https://drive.google.com/drive/my-drive). In the top left navigation you can see "My Drive". Right click "My Drive" and select "New folder". Name this folder "aad". You will see the folder appear. Double-click it. Now open a file explorer on your computer and navigate to the folder "Algorithms-for-Automated-Driving" that you have downloaded from github. Select all folders except the "book" folder and drag and drop them into the empty "aad" folder in your Google Drive. 
````



## Python environment


`````{tab} Local installation
If you do not have anaconda, please [download and install it](https://www.anaconda.com/products/individual).
Please create a conda environment called `aad` (Algorithms for Automated Driving) for this course using the environment.yml file within "Algorithms-for-Automated-Driving/code"
````bash
cd Algorithms-for-Automated-Driving/code
conda env create -f environment.yml
````

````{admonition} Tip: Use mamba!
:class: tip, dropdown
You may find that creating a conda environment takes a lot of time. I recommend to install mamba:
```bash
conda install mamba -n base -c conda-forge
```
Installing mamba takes some time, but afterwards setting up environments like the one for this book is way faster. Just write `mamba` instead of `conda`:
```bash
mamba env create -f environment.yml
``` 
````

Be sure to activate that environment to work with it
```bash
conda activate aad
```
If you are working on Windows, consider [adding anaconda to your PowerShell](https://www.scivision.dev/conda-powershell-python/).
`````


`````{tab} Google Colab
When you run code in Google Colab, you will have most of the libraries you need already installed. Just import whatever you need. If it is missing, you will get an error message that explains how to install it. 
`````


## Navigating the exercises

Within the `Algorithms-for-Automated-Driving` folder you will find a subfolder `book` containing the source code which created this book (not too interesting for you right now, you can even delete it if you want), a folder `data`, and a folder `code`. Within the `code` folder you have subfolders `exercises`, `solutions`, `tests`, and `util`. You will complete exercises by writing code in the `exercises` folder and testing it with code from the `tests` folder. You should *not* look into the `solutions` directory, unless you are desperate and really can't solve the exercise on your own.


````{tab} Local installation
To edit the source code, I recommend to use [Visual Studio Code](https://code.visualstudio.com/), since it has nice integration for jupyter notebooks. You can open the `code` folder with Visual Studio code and then easily navigate between the `tests` and the `exercises`. An alternative to Visual Studio code is jupyter lab, which you can start from a terminal:
```bash
conda activate aad
cd Algorithms-for-Automated-Driving
jupyter lab
```
In the book's exercise sections, I typically tell you to start working on the exercise by opening some jupyter notebook (.ipynb file).
When you open the .ipynb file with VS code be sure to select the "aad" conda environment as your python kernel.
Once you opened the notebook, read through it cell by cell. Execute each cell by pressing ctrl+enter. Typically the first section of the notebook is for setting up Google Colab. This won't do anything on your machine. You can also delete these Colab-specific cells if you want.
````

````{tab} Google Colab
In the book's exercise sections, I typically tell you to start working on the exercise by opening some jupyter notebook (.ipynb file).
Open [Google Drive](https://drive.google.com/drive/my-drive) and navigate to the .ipynb file specified in the book. Double-click the .ipynb file and then at the very top select "Open with Google Colaboratory". If you do not see this option, click "Connect more apps" and search for "colab". Once you opened the notebook, read through it cell by cell. Execute each cell, either by pressing ctrl+enter or by clicking the run button on the cell. The first few cells will mount your Google Drive in Colab. Once you completed this, you can click on the folder icon in the left navigation, and then for example on "drive", "My Drive", "aad", "code", "exercises", "lane_detection", "camera_geometry.py". This way you can work on python scripts. Be sure to press ctrl+s to save your work. It will be synchronized with your Google Drive.
````

## Getting help
If you have a question about the exercises, feel free to ask it on  [github discussions](https://github.com/thomasfermi/Algorithms-for-Automated-Driving/discussions) or on the [discord server](https://discord.gg/57YEzkCFHN). 