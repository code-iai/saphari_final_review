# Saphari Tool detector

## Maintainer

- [Thiemo Wiedemeyer](https://ai.uni-bremen.de/team/thiemo_wiedemeyer) <<wiedemeyer@cs.uni-bremen.de>>, [Institute for Artificial Intelligence](http://ai.uni-bremen.de/), University of Bremen

## Introduction

This package contains code to detect the surgery tools used in the saphari project.

## Installation

Just `git clone git@github.com:code-iai/saphari_final_review.git` into your catkin workspace source folder and run `catkin_make`. If you only need the perception, add `CATKIN_IGNORE` files into all other folders except `saphari_tool_detector`.

## Calibrating a Camera

Use http://wiki.ros.org/camera_calibration for calibrating the camera. A tutorial is given [here](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). Make sure that the focus of the lens is set to the distance from which it later should detect the tools.

1. Print a pattern like [chess5x7x0.03.pdf](https://github.com/code-iai/iai_kinect2/blob/master/kinect2_calibration/patterns/chess5x7x0.03.pdf) and glue it to a flat board.
   *Note: you have to make sure that the printout is exact, for example the boxes are exactly 30mm wide and tall*

2. Open [saphari_tool_detector/launch/camera.launch](https://github.com/code-iai/saphari_final_review/blob/master/saphari_tool_detector/launch/camera.launch#L6) and change `camera_serial` to the serial number of your camera, set `calibrated` to `0` and roslaunch it.

3. Rund the calibration tool:
   `rosrun camera_calibration cameracalibrator.py --size 7x5 --square 0.03 image:=/camera/image_mono`

4. Show the pattern to the camera and move/rotate around until the bars are green and mostly filled.


5. Click on calibrate and wait.

6. Copy the terminal output, beginning with `# oST version 5.0 parameters` into a text file named `calib_<camera_serial_number>.ini` and store it in `saphari_final_review/saphari_tool_detector/calib/`

7. Convert the ini file to a yaml file using `rosrun camera_calibration_parsers convert calib_<camera_serial_number>.ini calib_<camera_serial_number>.yaml`

8. Open [saphari_tool_detector/launch/camera.launch](https://github.com/code-iai/saphari_final_review/blob/master/saphari_tool_detector/launch/camera.launch#L6) again and set `calibrated` to `1`.

## Training

For the training it is needed to move the camera/arm to the position from where it should perceive the objects. The position of the camera should be perpendicular / orthogonal to the image plane with a fixed distance.

1. Launch the camera launch file `roslaunch saphari_tools camera.launch`.

2. Start the training for a tool with `rosrun saphari_tool_detector train_tool _id:=<tool_id> _name:=<tool_name>`. Replace `tool_id` and `tool_name` with the name and id of the tool.

3. Align the tool to one of the axis and press `SPACE` to continue.
   ![Training step 3](https://github.com/code-iai/saphari_final_review/blob/master/saphari_tool_detector/doc/training_step1.jpg)

4. Draw a minimal bounding box with a few pixel border around the object and press `SPACE` to continue.

   *Tipp: For example click somewhere left top of the object and draw the bounding box so that the bottom and right side of the bounding box fits the object, then release the mouse button and click again. Now draw a bounding box that fits all sides.*

   ![Training step 4](https://github.com/code-iai/saphari_final_review/blob/master/saphari_tool_detector/doc/training_step2.jpg)

5. Select the reference point of the object by clicking somewhere on the image, a small circle should be drawn. Then move the mouse around and a line should be drawn, fit this line to the desired orientation axis of the object pointing to the front of the tool and click again to summit. Press `SPACE` to continue.

   ![Training step 5](https://github.com/code-iai/saphari_final_review/blob/master/saphari_tool_detector/doc/training_step3.jpg)

6. Capture 8 images of different rotations. Make sure to move the tool befor capturing the first image. Press `SPACE` to continue.

   ![Training step 6](https://github.com/code-iai/saphari_final_review/blob/master/saphari_tool_detector/doc/training_step4.jpg)

7. Continue with step 2. until all tools are trained.

8. Learn meaningful confidence values by executing `rosrun saphari_tool_detector train_confidence` after all tools are trained.

## Usage

1. Launch the camera launch file `roslaunch saphari_tools camera.launch`.

2. Launch the detector `roslaunch saphari_tools detector.launch`.

3. Call the service with an empty request message: `rosservice call /tool_detector/detect_tools 0 0 0 0`. The 4 parameters are `x`, `y`, `width` and `height` of a region of interest for the detection, `0 0 0 0` will result in the full image being used.

The service will response with a list of results. Each result is a `Tool.msg` and contains the `id`, `name` and `pose` of the detected objects. In parallel a debug image is published under `/tool_detector/debug_image`

## Parameters of the detector

- `topic`: Topic for the rectified colored camera image
- `table_frame`: TF frame for the table
- `camera_frame`: TF frame for the camera
- `data_path`: Path to the data folder containing the trained models
- `threshold_low`: Lower threshold for the edge detection
- `threshold_high`: Higher threshold for the edge detection
- `threshold_hough`: Minimum votes for an object to be detected
- `max_overlap`: Maximum allowed overlap between bounding boxes of detected objects. Objects with less confidence will be discarded
- `min_confidence`: Minimum confidence of detected objects.
- `fake_perception`: Instead of running the perception it will publish some static results.
- `publish_tf`: Publish poses of the object in TF under `/tool_<id>_<name>`.
- `static_tf`: Enable a static TF publisher for the camera and table frame.
