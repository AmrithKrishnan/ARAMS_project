The scope of this project is to drive a car autonomoulsy in a simulated environment. This was developed as part of the coursework for Autonomous Mobile Robotic System - Master Mechatronics at FH Aachen.  

Prerequisites: ROS@, arams-city files (virtual city - accessible via ili.fh-aachen.de) , nav2 assosciated files and other ros2 dependencies
The following is a brief description of the packages / nodes used:

Structure:

    PKG img_proc

        NODE crop_raw_img : takes raw img from /prius/front_camera/image_raw and crops the top right corner part for yolo (traffic ight detection). publishes to the topic /traffic_light_roi

        NODE traffic_light_detect: gets the image of traffic light from yolo on topic /traffic_light_cropped, does processing to find the color of traffic light and sends status over the topic /status_message. The processed image is also sent over the topic /output_opencv to view on rviz

    PKG my_robot_nav

        NODE prius_cmd_vel_mirror: mirrors twist messages from /cmd_vel to /prius/cmd_vel. Used for debugging and testing nav2

        NODE prius_cmd_vel_traffic_light: based on traffic light color from /status_message, the /prius/cmd_vel is controlled. Red light-stop, yellow-slow speed, green-drive

        NODE auto_explorer: generates and sends 3 random navigation goals to prius based on a fixed set of 12 predefined coordinates, allowing the robot to navigate to these positions in sequence.

        CONFIG nav2_params.yaml: navigation parameters file

        LAUNCH localization.launch.yaml: localization related launch file

        LAUNCH navigation_launch.py: for navigation server

        LAUNCH robot_nav.launch.py: launch file for navigation, it also links navigation_launch.py and the param file nav2_params.yaml

        MAPS my_map.pgm and my_map.yaml: map files

        MAPS my_map1.pgm and my_map1.yaml: alternate map files

    PKG yolo

        NODE yolo_node: detects traffic lights on the image topic /traffic_light_roi, crops the image as per its bounding box and sends over the topic /traffic_light_cropped. also detects fire trucks and sends '1' over topic /truck_status when detected.

Run instructions:

launch arams_city and spawn prius. Launch localization.launch.yaml and robot_nav.lauch.py . Then, run all three nodes from PKG img_proc and PKG yolo. Run node prius_cmd_vel_traffic_light from PKG my_robot_nav. Run auto_explorer from PKG my_robot_nav for fully autonomous navigation, including sending goals.

To test only nav2 (without traffic light following behavior), just launch localization.launch.yaml and robot_nav.launch.py. (and of course launch arams_city and spawn prius before hand)
