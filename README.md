# Autonomous Car Control in Simulated Environment (ROS2)

This project aims to drive a car autonomously in a simulated environment. It was developed as part of the coursework for the Autonomous Mobile Robotic System at FH Aachen, Master Mechatronics program.

## Prerequisites

To use this project, ensure you have the following:

- ROS2 installation
- 'arams-city' files for the virtual city (accessible via ili.fh-aachen.de)
- Navigation2 (nav2) and its associated ROS2 dependencies

## Project Structure

### Package: img_proc

#### Nodes:

- **crop_raw_img:** Obtains raw images from `/prius/front_camera/image_raw`, crops the top right corner for YOLO (traffic light detection), and publishes to `/traffic_light_roi`.
  
- **traffic_light_detect:** Receives the cropped image from YOLO via the topic `/traffic_light_cropped`, processes to identify traffic light colors, and sends status over `/status_message`. The processed image is also available for visualization on RViz through the topic `/output_opencv`.

### Package: my_robot_nav

#### Nodes:

- **prius_cmd_vel_mirror:** Mirrors twist messages from `/cmd_vel` to `/prius/cmd_vel`. Useful for debugging and testing nav2.

- **prius_cmd_vel_traffic_light:** Controls `/prius/cmd_vel` based on the traffic light color received from `/status_message`. Red light means stop, yellow indicates slow speed, and green allows the car to proceed.
  
- **auto_explorer:** Generates and sends three random navigation goals to the Prius using a set of 12 predefined coordinates. This allows the robot to navigate to these positions sequentially.

#### Configuration and Launch Files:

- `nav2_params.yaml`: Navigation parameters file.
- `localization.launch.yaml`: Localization-related launch file.
- `navigation_launch.py`: Launch file for the navigation server.
- `robot_nav.launch.py`: Launch file that links navigation_launch.py and the parameter file nav2_params.yaml.

#### Maps:

- `my_map.pgm` and `my_map.yaml`: Primary map files.
- `my_map1.pgm` and `my_map1.yaml`: Alternate map files.

### Package: yolo

#### Nodes:

- **yolo_node:** Detects traffic lights on the image topic `/traffic_light_roi`, crops the image as per its bounding box, and sends it over the topic `/traffic_light_cropped`. Additionally, it detects fire trucks and sends '1' over topic `/truck_status` when detected.

## Run Instructions

Follow these steps to run the project:

1. Launch 'arams_city' and spawn the Prius.
2. Launch `localization.launch.yaml` and `robot_nav.launch.py`.
3. Run all three nodes from `img_proc` and `yolo` packages.
4. Execute the `prius_cmd_vel_traffic_light` node from the `my_robot_nav` package.
5. For fully autonomous navigation, including sending goals, run the `auto_explorer` node from the `my_robot_nav` package.

### Testing Navigation without Traffic Light Behavior

To test only the nav2 functionality without the traffic light following behavior, follow these steps:

1. Launch `localization.launch.yaml` and `robot_nav.launch.py`.
2. (Ensure 'arams_city' is launched and the Prius is spawned.)

Feel free to modify these instructions according to your specific setup or any changes in the project's structure or dependencies.
