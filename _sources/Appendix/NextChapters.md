# Future Chapters

I would like to add the following chapters to this book in the future

* **Model Predictive Control** The pure pursuit controller does not work well when driving curves at high speeds. In this case the assumption of zero slip for the kinematic bicycle model does not apply. In this chapter we will design a model predictive controller based on the dynamic bicycle model, which accounts for nonzero side slip angles. 
* **Camera Calibration** How do we estimate the camera height above the road, as well as the camera roll, pitch, and yaw angle? In the chapter on Lane Detection, we got these parameters directly from the simulation. Of course, we cannot do this in the real world. In this chapter we will implement a camera calibration module to estimate the camera extrinsics.
* **HD Map Localization** Carla has a very nice API to access a high definition (HD) map of the road. How can we use our detected lane boundaries, a GPS sensor, a yaw rate sensor, and a speedometer to estimate our position on the HD map? This is relevant for navigation, and can also be used for improved vehicle control.

If you have some additional wishes for future chapters, please raise an issue on the [book's github repo](https://github.com/thomasfermi/Algorithms-for-Automated-Driving). If you want to motivate me to continue working on this book, please star the [book's github repo](https://github.com/thomasfermi/Algorithms-for-Automated-Driving) 😉. 