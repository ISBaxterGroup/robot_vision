# robot_vision
Robot vision programs
## rs_controller
### Publication 
This node publishies following images to ROS-Topic.
- Color-image :             rs_camera/image_color
- Depth-image :             rs_camera/image_depth
- Color-alligned-to-depth : rs_camera/image_color_aligned_to_depth
- Depth-alligned-to-color : rs_camera/image_depth_aligned_to_color
### Service
Provide deproject_service.
Convert 2D position on Color-image to 3D position on Color-camera-coordinates.
- Service name : deproject
- request : 2D position on Color-image.
- response : 3D position on Color-camera-coordinates.

## tf_camera_chess
This node publishies "tf" from Color-camera-coordinates to Chessboard-coordinates.
### Requirements
This program requests camera parameter file.
- robot_vision/yaml/ost.yaml
The yaml file is generated by ros_calibration.launch
It is necessary to manually rewrite the generated file to the OpenCV Mat style.
