Overview
============================

```{figure} images/control_carla.gif
---
scale: 100%
name: ControlCarla
---
Vehicle being controlled by pure pursuit and PID.
```

In this module, we are going to control a vehicle in the Carla simulator. 
Our algorithm's input will be the current vehicle speed, as well as the desired speed and desired trajectory. 
The algorithm's output will be the actuator signals: gas pedal, and steering wheel.

Our approach will be a PID controller for the gas pedal (longitudinal control) and a method called pure pursuit for steering (lateral control).

We will begin by learning about [PID control](./PID.ipynb). Subsequently, we introduce a mathematical model that describes how the vehicle will move as a function of the steering wheel angle, the so-called [Kinematic Bicycle Model](./BicycleModel.md). Using that model we introduce the [Pure Pursuit](./PurePursuit.md) method for lateral control. In the final exercise, you will implement what you learned to control a vehicle in Carla. If your computer cannot run Carla, don't worry: you can still use the simplistic simulator I created for this course.

```{note}
You can work through this chapter, even if you did not work through the chapter on Lane Detection at all. However, you miss out on the fun of piping together your lane-detection module with your control module. 
```

```{tip}
What is covered in this chapter is somewhat close to the content of weeks 4-7 in the [Coursera course "Introduction to Self-Driving Cars"](https://www.coursera.org/learn/intro-self-driving-cars). If you like, you can audit that course for free on Coursera and watch the videos.
```