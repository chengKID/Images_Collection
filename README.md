# Automatically collect the images (Optional: with different exposure times)

Usually, the camera changes its exposure time automatically. When we want to analyze the porformance of the camera under different illumination conditions, we need to collect images with different exposure times. This tool help us more efficiently creat the dataset.

## Installation

### Linux Ubuntu 16.04, ROS Kinetic

Install OpenCV, C++

A few updates to the instructions above were needed.

```bash
$> cd ~
$> git clone https://github.com:chengKID/Images_Collection.git
$> cd Images_Collection
$> catkin_make
```

## Optional
You can also set the camera's configure with respect to your own camera to collect images from stero camera.

