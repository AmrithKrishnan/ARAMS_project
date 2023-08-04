Structure:

    PKG img_proc

        NODE crop_raw_img : takes raw img from /prius/front_camera/image_raw and crops the top right corner part for yolo (traffic ight detection). publishes to the topic /traffic_light_roi

        NODE traffic_light_detect: gets the image of traffic light from yolo on topic /traffic_light_cropped, does processing to find the color and sends status over the topic /status_message. The processed image is also sent over the topic /output_opencv

    PKG my_robot_nav

        NODE prius_cmd_vel_mirror: mirrors twist messages from /cmd_vel to /prius/cmd_vel. Used for debugging and testing nav2

        NODE prius_cmd_vel_traffic_light: based on traffic light color from /status_message, the /prius/cmd_vel is controlled. Red-stop, yellow-slow, green-drive

        NODE auto_explorer: generates and sends 3 random navigation goals to prius based on a fixed set of 12 predefined coordinates, allowing the robot to navigate to these positions in a loop.

        LAUNCH localization.launch.yaml: localization related launch file

        LAUNCH navigation_launch.py: for navigation server

        LAUNCH robot_nav.launch.py: launch file for navigation, it also links navigation_launch.py and the param file nav2_params.yaml

        CONFIG nav2_params.yaml: navigation parameters file

        MAPS my_map.pgm and my_map.yaml: map files

        MAPS my_map1.pgm and my_map1.yaml: alternate map files

    PKG yolo

        NODE yolo_node: detects traffic lights, crops the image as per its bounding box and sends over /traffic_light_cropped. also detects fire trucks and send '1' over topic /truck_status if detected.

Run instructions:

launch arams_city and spawn prius. Launch localization.launch.yaml and robot_nav.lauch.py . Then, run all three nodes from PKG img_proc and PKG yolo. Run node prius_cmd_vel_traffic_light from PKG my_robot_nav.

To test only nav2 (without traffic light following behavior), just launch localization.launch.yaml and robot_nav.launch.py. (and of course launch arams_city and spawn prius before hand)