Pure Pursuit
============================

## Algorithm

In this section we want to control the front wheel angle $\delta$, such that the vehicle follows a given path. This is known as **lateral vehicle control**.
In the pure pursuit method a target point (TP) on the desired path is identified, which is a **look-ahead distance** $l_d$ away from the vehicle. The angle $\delta$ is chosen such that the vehicle will reach the target point according to the kinematic bicycle model. 
The look-ahead distance is a parameter, and is typically chosen to depend on the speed $v$ via $l_d = K_{dd} v$, where the constant $K_{dd}$ needs to be tuned. We can also enforce a minimal and maximal look-ahead distance, so as to avoid undesirable behavior at very high and very low speeds. 

Let us draw the bicycle model and a given path we should follow. We also draw a circle of radius $l_d$ around the center of the rear wheel. The intersection of this circle with the path is our target point TP. According to the kinematic bicycle model, the vehicle will move along the orange arc, which is determined by the front wheel angle $\delta$. We want to choose $\delta$, such that the orange vehicle trajectory will move to the target point.

```{figure} tikz/PurePursuit/PurePursuitWrongDelta.svg
---
width: 70%
name: PurePursuitWrongDelta
---
Bicycle model should follow a path. With the current front wheel angle $\delta$, it will not reach the target point TP.
```

Since the above drawing is generated programmatically (using [tikz](https://en.wikipedia.org/wiki/PGF/TikZ)), we can change the value of $\delta$ until the vehicle trajectory goes through the target point:

````{tab} δ = 25°
```{figure} tikz/PurePursuit/PurePursuit_delta_25.svg
---
width: 90%
name: PurePursuit_delta_25
---
```
````

````{tab} δ = 20°
```{figure} tikz/PurePursuit/PurePursuit_delta_20.svg
---
width: 90%
name: PurePursuit_delta_20
---
```
````

````{tab} δ = 15°
```{figure} tikz/PurePursuit/PurePursuit_delta_15.svg
---
width: 90%
name: PurePursuit_delta_15
---
```
````

````{tab} δ = 11.3°
```{figure} tikz/PurePursuit/PurePursuit_delta_11p3.svg
---
width: 90%
name: PurePursuit_delta_11p3
---
```
````

But there is a more elegant solution than just trying out a bunch of different $\delta$ values. We can actually compute the optimal $\delta$ based on the magenta triangle in the sketch below

```{figure} tikz/PurePursuit/PurePursuitLawOfSines.svg
---
width: 90%
name: PurePursuitLawOfSines
---
The magenta triangle helps us to establish a formula for $\delta$.
```

First, we note that the distance from the instantaneous center of rotation (ICR) to the target point (TP) is equal to $R$, since TP lies on the orange circle of radius $R$ around ICR. Hence, the magenta triangle is [isosceles](https://en.wikipedia.org/wiki/Isosceles_triangle) and $\gamma_2=\gamma_3$. From the figure we can see that $\gamma_3+\alpha=90°$. Hence $\gamma_2=\gamma_3=90°-\alpha$. Since the sum of all angles in a triangle equals $180°$, we have 

$$180°=\gamma_1+\gamma_2+\gamma_3 = \gamma_1 + (90°-\alpha) + (90°-\alpha)$$

which yields $\gamma_1=2\alpha$. According to the [law of sines](https://en.wikipedia.org/wiki/Law_of_sines)

$$ \frac{l_d}{\sin(\gamma_1)} = \frac{R}{\sin(\gamma_2)} $$

Here, we used that the distance between the rear wheel and the target point TP is $l_d$. If we substitute $\gamma_1=2\alpha$ and $\gamma_2=90°-\alpha$ into the above formula, we obtain

$$ \frac{l_d}{\sin(2 \alpha)} = \frac{R}{\sin(90° - \alpha)} $$

Due to the [trigonometric addition formulas](https://mathworld.wolfram.com/TrigonometricAdditionFormulas.html), we have $\sin(90° - \alpha) = \cos(\alpha)$ and $\sin(2\alpha)=\sin(\alpha+\alpha)=2\sin(\alpha) \cos(\alpha)$. Hence, we can further simplify the formula above to find

$$ \frac{l_d}{2\sin(\alpha) \cos(\alpha)} = \frac{R}{\cos(\alpha)} $$

which yields $R=l_d/(2 \sin(\alpha))$. For the kinematic bicycle model we have previously derived a formula for the wheel angle $\delta$ as a function of $R$. It was $\delta = \arctan(L/R)$, where $L$ is the wheel base, i.e., the distance between the wheels. Combining this with the newly found formula for $R$ we finally obtain

$$ \delta = \arctan \left(\frac{2 L \sin(\alpha)}{l_d}\right) $$ (eq-pp)

This is the angle $\delta$ we need to pick to reach the target point! We can write down the pure pursuit algorithm now:

```{admonition} Pure pursuit algorithm
For each instant in time:
* Compute the look ahead distance $l_d$ as `l_d = np.clip(K_dd * speed, min_ld, max_ld)`. The function `np.clip` is documented [here](https://numpy.org/doc/stable/reference/generated/numpy.clip.html). `K_dd`, `min_ld`, and `max_ld` are parameters that you can tune.
* Find the target point TP as the intersection of the desired path with a circle of radius $l_d$ around the rear wheel. 
* Using the target point coordinates `(x_tp,y_tp)`, determine $\alpha$ as `alpha=arctan2(y_tp,x_tp)`
* Use equation {eq}`eq-pp` to compute the pure pursuit front wheel angle $\delta$
* **Act**: Turn your steering wheel to set the front wheel angle to $\delta$
```


## Exercise
In this exercise you will implement both pure pursuit and PID.

If you did not do the chapter on lane detection, you probably did not set up your python environment, and you did not download the exercise code. In this case, please visit [the appendix](../Appendix/ExerciseSetup.md) to do this now.

To start working, open `code/tests/control/target_point.ipynb` and follow the instructions. Next, open `code/tests/control/control.ipynb` and follow the instructions. This exercise uses a simplistic vehicle simulator within the Jupyter Notebook to test your code. If you completed these exercises successfully, you **can** also run your controller in a Carla simulation:

* Start Carla by executing the file `CarlaUE4.exe` (Windows) or `CarlaUE4.sh` (Linux) in your Carla folder (If you did not download Carla yet, see [the appendix](../Appendix/CarlaInstallation.md)). 
* Execute `python -m code.tests.control.carla_sim --ex` from the parent directory of `code` and witness your control algorithm in action! If you omit the `--ex` flag, you will see the sample solution.
* By default, the center of the lane is queried from Carla's HD map and given as reference path to your controller. But, if you run `python -m code.tests.control.carla_sim --ex --ld` your `LaneDetector` will be used: The average of the left and right lane boundary, i.e., $(y_l(x)+y_r(x))/2$ will be given to your controller as the reference path. Note that there is a "TODO" item in `carla_sim.py` regarding the correct call to your `LaneDetector` constructor. You should work on this to make sure the simulation runs without error. Running the Carla simulation and your `LaneDetector` at the same time will eat up a lot of hardware resources. Hence, the simulation will probably run with only a few frames per second on your machine, unless it is very powerful.
