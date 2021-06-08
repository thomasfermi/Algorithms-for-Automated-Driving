# Discussion

## Limitations and outlook
The lateral control we implemented in this chapter is a good starting point, but real highway lane-keeping systems are more sophisticated.
In the Carla simulations you might have observed that the vehicle is sliding/drifting when driving curves at high speed.
In this case, we are outside of the validity regime of the kinematic bicycle model, which is the basis our pure pursuit controller. Hence, it is no wonder that our lateral control is performing suboptimal here.

Control algorithms based on the **dynamic** bicycle model are more promising in this situation. You can learn about the dynamic bicycle model in the [lectures on Vehicle Dynamics and Control by Prof. Georg Schildbach on youtube](https://www.youtube.com/playlist?list=PLW3FM5Kyc2_4PGkumkAHNXzWtgHhaYe1d). A very sophisticated control method that can work with the dynamic bicycle model is Model Predictive Control (MPC), for which I recommend [this youtube playlist by MathWorks](https://www.youtube.com/playlist?list=PLn8PRpmsu08ozoeoXgxPSBKLyd4YEHww8) as an introduction. I plan to write a [chapter on MPC in the future](../Appendix/NextChapters.md).

Finally, I recommend Ref. {cite}`snider2009automatic`. It discusses several controls methods for automated driving, and has a nice empirical performance comparison table at the end of section 5.


## Control in a real ADAS system: openpilot
```{margin}
Take my discussion of openpilot with a grain of salt here. Their documentation is very limited, so my discussion is based on what I could piece together by reading their code.
```
It is interesting to look at a real-world control system for automated driving: [openpilot](https://github.com/commaai/openpilot/). 
The relevant code is available on [github](https://github.com/commaai/openpilot/) in `openpilot/selfdrive/controls/`.
As you can see [here](https://github.com/commaai/openpilot/blob/master/selfdrive/controls/lib/vehicle_model.py), the software is using the dynamic bicycle model. Model Predictive control is used to [plan](https://github.com/commaai/openpilot/blob/059cf6b43e579b8634090a0ecac4fb1c6c7a205e/selfdrive/controls/plannerd.py) a trajectory. 
Lateral control is based on this plan, and dependent on the vehicle a different algorithm is used. 
You will find lateral control with PID, [LQR](https://en.wikipedia.org/wiki/Linear%E2%80%93quadratic_regulator) or [INDI](http://www.aerostudents.com/courses/advanced-flight-control/nonlinearDynamicInversion.pdf) in the [source code](https://github.com/commaai/openpilot/blob/ee99b59bade8d3b5057a5b3f22ad8b3edd102c78/selfdrive/controls/controlsd.py). [Longitudinal control](https://github.com/commaai/openpilot/blob/254814cc793dc4668ea9fd25f092b0712fb5b8a0/selfdrive/controls/lib/longcontrol.py) is done using a [PIController](https://github.com/commaai/openpilot/blob/f575a9ec12990ac2a764a5f416795d1c618f4609/selfdrive/controls/lib/pid.py). 

## References

```{bibliography} 
:filter: docname in docnames
```