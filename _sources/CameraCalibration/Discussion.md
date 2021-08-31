<!-- #region -->
# Discussion

## Limitations

The method we presented just assumed that the roll is zero. Also we did not estimate the height $h$ of the camera. In the real world you could estimate the height using a tape measure and will probably only make an error of around 5 percent. Assuming a roll of zero does not seem to lead to practical problems, since this is done in the [source code](https://github.com/commaai/openpilot/blob/d74def61f88937302f7423eea67895d5f4c596b5/selfdrive/locationd/calibrationd.py#L5) of openpilot, which is known to perfrom really well. As a bonus exercise you can run experiments with `code/tests/camera_calibration/carla_sim.py` where you change the roll of the camera or you slightly modify the height. Investigate how this affects the control of the vehicle. Regarding estimation of height and roll we also recommend to have a look at [this paper](https://arxiv.org/abs/2008.03722).

Another limitation: The method we discussed in this chapter only works if your autonomous vehicle software stack uses lane detection in image space and if it is used in areas with good lane markings. But what if your software doesn't predict lanes in image space?  Maybe it predicts lanes in world space as [openpilot](https://github.com/commaai/openpilot), or maybe it doesn't predict lanes at all and makes predictions [end-to-end](https://developer.nvidia.com/blog/deep-learning-self-driving-cars/). In either approaches, this method of camera calibration isn't going to work. As an alternative, we can use visual odometery (VO) based camera calibration. 



## Alternative: VO-based camera calibration

In this approach, visual odometery is performed to find the motion of the camera. This approach also needs to be performed when the car is aligned with the lane line, i.e. when it is going straight and fast. Since the output of VO would be motion of the camera, this information can be used to find out the orientation of the camera with respect to the car. See how openpilot performs this in the [calibrationd](https://github.com/commaai/openpilot/blob/master/selfdrive/locationd/calibrationd.py#L148) module. The vehicle forwards axis is more or less identical to the direction of the translation vector, since the vehicle is driving straight. But having the vehicle forwards axis with respect to the camera reference frame means that you can estimate how the optical axis (the z-axis) of the camera is tilted with respect to the vehicle forwards direction. Hence you get the extrinsic rotation matrix! However, you also  need to assume the roll is zero in this approach. To get started with visual odometery, the fastest way is to get started is [PySlam](https://github.com/luigifreda/pyslam). It has lots of methods for visual odometery including novel approaches based on deep learning.

## Further reading

 - A great paper to get started with more advanced methods is Ref. {cite}`lee2020online`: [Online Extrinsic Camera Calibration for Temporally Consistent IPM Using Lane Boundary Observations with a Lane Width Prior](https://arxiv.org/abs/2008.03722). This paper also discusses estimation of roll and height.
 - Visual Odometery: [PySlam](https://github.com/luigifreda/pyslam) and the resources mentioned in the repo. 
 - VO [Blog post](http://avisingh599.github.io/vision/monocular-vo/) by Avi Singh.
 - [Minimal python](https://github.com/yoshimasa1700/mono_vo_python/) implementation of VO.
 - [VO Lectures with exercises](https://web.archive.org/web/20200709104300/http://rpg.ifi.uzh.ch/teaching.html) by David Scaramuzza


## References
The formalism of how to compute the camera orientation from the vanishing point was adapted from Ref. {cite}`ShiCourseraCalibration`. The idea to use lane boundaries to determine the vanishing point can be found in the paper by Lee et al. {cite}`lee2020online` and within the references of that paper.

```{bibliography}
:filter: docname in docnames
```


<!-- #endregion -->
