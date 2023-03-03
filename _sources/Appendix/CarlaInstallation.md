# Carla Installation
For some parts of this course you **can** use the Carla simulator.
It is most convenient to install Carla on your local machine. However, it might not be powerful enough, since Carla is quite ressource hungry. If you find that Carla is not running well on your machine, you can try running it on Colab. Personally, I found that running Carla through Colab was an unpleasant experience. 

````{tab} Local installation
You can get Carla at [the Carla github repo](https://github.com/carla-simulator/carla/blob/master/Docs/download.md). Download version 0.9.10 (or newer if no breaking API changes will be introduced in the future) and move it to a location where you want to keep it.
The Carla simulation can be controlled via a python API. In your Carla folder you will find a subfolder `PythonAPI` which contains the python package as well as some examples.
I recommend that you use an anaconda environment called `aad` for this course (see [Exercise Setup](ExerciseSetup.md)). An easy way to *install* the carla python package into your anaconda enviroment is the following:
* Go to your anaconda installation folder and then into the `site-packages` subfolder of your environment. The path may be something like `~/anaconda3/envs/aad/lib/pythonX.X/site-packages/` or `C:\Users\{YOUR_USERNAME}\anaconda3\envs\aad\Lib\site-packages`
* Create a file `carla.pth` and open it with a text editor
* Paste in the path to the carla egg file, then save. The carla egg file is located in `{PATH_TO_YOUR_CARLA_FOLDER}/PythonAPI/carla/dist/`. Hence, I pasted `C:\Users\mario\Documents\Carla_0910\PythonAPI\carla\dist\carla-0.9.10-py3.7-win-amd64.egg` into the `carla.pth` file.
Do not move the Carla folder afterwards, since it will break this link and hence the anaconda installation.
````

````{tab} Running on Colab
If you want to run Carla on [Google Colab](https://colab.research.google.com/), check out [Michael Bossello's carla-colab repository](https://github.com/MichaelBosello/carla-colab). When you follow this link, you will see a nice image of the Carla simulator and above there is a button "Open in Colab". Click that button. Then go through the notebook step by step and follow the instructions. Note that if you move your python code to the remote machine, and execute it, the `import carla` statements will not work. Add the following lines before the `import carla` statement
```python
import sys
sys.path.append("/home/colab/carla/PythonAPI/carla/dist/")
```
This will let python know where to look for the carla python package.
````