# Discussion

## Limitations

While this method works great if your autonomous veehicle software stack uses lane detection in image space and would be used in areas with lane markings. But what if your ADAS software dosen't predicts lanes in image space ?  Maybe it predicts lanes in world space as [openpilot](https://github.com/commaai/openpilot), or maybe it dosent't predicts lanes at all and makes predictions [end-to-end](https://developer.nvidia.com/blog/deep-learning-self-driving-cars/). In either approaches, this method of camera calibration isn't going to work. For a way around this, we can use visual odometery (VO) based camera calibration. 

## VO based camera calibration

In this approach, visual odmetery is performed to find the motion of the camera. This approach also needs to be performed when the car is aligned with the lane line i.e. going straight and fast. Since the output of VO would be motion of the camera, this information can be used to find out the orientation of the camera with respect to the car. See how openpilot performs this in the [calibrationd](https://github.com/commaai/openpilot/blob/master/selfdrive/locationd/calibrationd.py#L148) module. The vehicle forwards axis is more or less identical to the direction of the translation vector, since the vehicle is driving straight. But having the vehicle forwards axis with respect to the camera reference frame means that you can estimate how the optical axis (the z-axis) of the camera is tilted with respect to the vehicle forwards direction. Hence you get the extrinsic rotation matrix! To get started with visual odometery, the fastest way is to get started is [PySlam](https://github.com/luigifreda/pyslam). It has lot's of methods for visual odometery including deeplearning based ones. 

## Further reading

 - A great paper to get started with more advanced methods is [Online Extrinsic Camera Calibration for Temporally Consistent IPM Using Lane Boundary Observations with a Lane Width Prior](https://arxiv.org/abs/2008.03722).
 - For Visual Odometery, [PySlam](https://github.com/luigifreda/pyslam) and the resources mentioned in the repo. 
 - VO [Blog post](http://avisingh599.github.io/vision/monocular-vo/) by Avi Singh.
 - [Minimal python](https://github.com/yoshimasa1700/mono_vo_python/) implementaion of VO.
 - [Lectures](https://www.youtube.com/watch?v=BuRCJ2fegcc) by Cyrill Stachniss. 
