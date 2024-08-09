# IRP5-Shopping-Cart-GROUP2
This project involves the development of an intelligent shopping cart designed to autonomously follow a user and stop when obstacles are detected, enhancing both convenience and safety during shopping. The cart leverages a RealSense camera, the YOLO (You Only Look Once) algorithm for person detection, and a ROS-based framework for data processing and cart control. Obstacle avoidance is managed through external bumper sensors triggered by a push button. This repository includes the relevant files and packages

1. person_detector package: This package handles the acquisition and processing of images using a RealSense camera. It captures images in real-time, converts them to grayscale, and saves them for further analysis. The package also provides functionality to publish the processed images to a ROS topic for use in tasks such as object tracking.

2. push_button_utils: This package contains the edited push_button.py node which stops the cart when the button is pressed.  It continuously monitors the button, publishing its state (pressed or released) to a ROS topic.

3. person_detector_package: This package is responsible for detecting and tracking individuals using the YOLO (You Only Look Once) algorithm. It processes the images captured by the camera and identifies the presence of a person, enabling the autonomous shopping cart to follow the user accurately.
