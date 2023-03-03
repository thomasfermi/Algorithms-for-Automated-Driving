# Discussion

## Limitations of the presented approach
* The second stage of the pipeline is implemented on CPU. For better performance everything should run on GPU or even dedicated hardware.
* We only detect the ego lane boundaries. But it is obvious how to extend our approach to include the left and right lanes.
* The semantic segmentation approach will have problems when there are lane changes. A cleaner solution would be *instance segmentation*.
* We created the lane boundary labels automatically using Carla's high definition map. Creating labels for a camera installed in a real car is way more challenging. The options I know of are 
    1. You can have humans manually label each image separately. This approach is done for example for NVIDIA's PilotNet (Ref. {cite}`bojarski2020nvidia`).
    2. Similar to our virtual approach, you can create labels using a high definition map of a real highway (Ref. {cite}`llamas2019`). The challenge is to perfectly localize the vehicle within the map in order to get good labels. Furthermore, to train a lane detection system that works on different highways across the world, you will need high definition maps of lots of different highways.
    3. If you have some system that detects lane markings on a short distance reliably, you can combine that with visual-inertial odometry to create good labels. Examples of such short-distance lane-detection systems would be a [lidar](https://en.wikipedia.org/wiki/Lidar), or a camera-based lane-detection system that works well for the first say 5 meters, but is not so good further away. Once you logged some seconds or minutes of driving, you can use [visual-inertial odometry](https://en.wikipedia.org/wiki/Visual_odometry#Visual_inertial_odometry) to obtain the vehicle trajectory and then stitch together a local map of lane boundaries. Subsequently, you can project those mapped lane boundaries into each image you have logged.
* The inverse perspective mapping step relies on very good calibration parameters, i.e., on knowing the position and orientation of the camera with respect to the road. Since we are running simulations here, we exactly know those parameters. In the real world you need to calibrate your camera. Getting the camera intrinsics is typically no problem [if you have a chess boad](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html). Obtaining the camera extrinsics (orientation and height) is more challenging and might become [another chapter of this book](../Appendix/NextChapters.md) at some point.
* We are assuming that the road is flat, which is obviously not true everywhere. We are also neglecting that the car dips or "nose-dives" a bit when breaking. In this case, the vehicle's forwards axis is not parallel to the road, which is something we assumed in our derivations.


## Comparison to literature

As our approach to lane detection in this chapter is heavily inspired by the baseline described in Ref. {cite}`gansbeke2019endtoend`, we want to list the differences
* We are using an approach known as *inverse perspective mapping* to transform from pixel to road coordinates, since this allows us to fit a lane boundary model in meters. Describing the lane boundary in meters rather than pixels is necessary if we want to use the lane boundary model for control algorithms (see next chapter). Ref. {cite}`gansbeke2019endtoend` also transforms to a bird's eye view, but they use a homography for that. The resulting coordinates are not in meters. Note that this is not the aim of the paper, and hence should not be seen as a criticism.
* For the image segmentation we are using an off-the-shelf neural network from the great pytorch library [segmentation models pytorch](https://github.com/qubvel/segmentation_models.pytorch).
* Our pipeline is similar to the **baseline model** in Ref. {cite}`gansbeke2019endtoend`, not their actual model. Their actual model is an end-to-end neural network which fuses the two-step pipeline of the baseline model into one single neural network. This is advantageous, since it increases the accuracy, and speed of execution. Of course, creating an end-to-end network is also possible for our slightly modified approach, but we keep this as an exercise for the reader 😉.

## Comparison to a real ADAS system: openpilot

```{margin}
Take my discussion of openpilot with a grain of salt here. Their documentation is very limited, so my discussion is based on what I could piece together by reading their code.
```
It is interesting to see how a real world lane-detection system works. Luckily, there is one ADAS company that open sources their software: comma.ai. As you can read in the [source code of their product openpilot](https://github.com/commaai/openpilot) their lane-detection system is designed roughly as follows
* Perform online [calibration](https://github.com/commaai/openpilot/blob/0b849d5a4e417d73e4b821b909839f379d70e75d/selfdrive/locationd/calibrationd.py) to estimate camera extrinsics
* Apply homography (warpPerspective) to the camera image in order to compute the image that you would get from a camera with *default* extrinsics. In the openpilot documentation this is referred to as [calibrated frame](https://github.com/commaai/openpilot/tree/master/common/transformations).
* Train a neural net with the default-perspective images. The output of the neural network is the path the vehicle should take (somewhat close to the center between the lane boundaries). I am not totally sure, but based on their [medium article](https://medium.com/@comma_ai/towards-a-superhuman-driving-agent-1f7391e2e8ec) I think they create labels like this: Take recorded videos and estimate vehicle trajectory using visual odometry. Then for each image frame, transform this trajectory into the vehicle reference frame at that point in time and use this as a label.

## Further Reading
If you want to read some more about lane detection, I recommend the following ressources:

```{glossary}
[Papers with code](https://paperswithcode.com/task/lane-detection/)
  Here you can see papers grouped by the datasets they tackled, and also ranked by stars on github.
[awesome-lane-detection](https://github.com/amusi/awesome-lane-detection)
  This github repo lists papers, code, blogs/tutorials and datasets connected to lane detection.
```


## References

```{bibliography}
:filter: docname in docnames
```