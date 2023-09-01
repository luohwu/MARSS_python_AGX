
# Example for visualizing optical tracking data    
    
This Example contains an application for streaming and visualizing optical tracking data. The application comprises two essential components: a Unity client and a Python server. These components communicate via ROS2, facilitated by the [Unity-Technologies/ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector), which serves as a TCP-based communication framework. In this setup, the Python server runs on a laptop, acquiring tracking data from an optical tracking camera (SpryTrack) and streaming it to the Unity client, which operates on a HoloLens2 device.  
  

# Requirements

Before running this application, ensure that your PC meets the following requirements:

1.  **Install ROS2**:
    
    -   It is recommended to run the application on a Linux system, as installing ROS2 is simpler on Linux.
    -   For Linux: You can install ROS2 directly by following the instructions in the [Ubuntu (Debian) — ROS 2 Documentation: Foxy documentation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html). Alternatively, you can use the Docker image as described in [Running ROS 2 nodes in Docker [community-contributed] — ROS 2 Documentation: Foxy documentation](https://docs.ros.org/en/foxy/How-To-Guides/Run-2-nodes-in-single-or-separate-docker-containers.html?highlight=docker).
    -   For Windows: The easiest way to install ROS2 on Windows is detailed in [ROS 2 Binary Installation - ROS on Windows (ms-iot.github.io)](https://ms-iot.github.io/ROSOnWindows/GettingStarted/SetupRos2.html). It is not necessary to install Visual Studio of the same version as in the guide. You can also refer to the official [guide](https://docs.ros.org/en/foxy/Installation/Windows-Install-Binary.html) for installing ROS2 on Windows, which requires some hands-on skills.
    -   To verify that your ROS2 environment is ready, try importing `rclpy` in Python. If successful, your environment is set up.
2.  **Install SpryTrack SDK**:
    
    -   Locate and install the SpryTrack SDK, found in the `SpryTrackSDK installer` folder, by following the provided how-to-use manual. While the manual contains comprehensive information about the camera, focus solely on the installation section for this project's needs.

# Usage

Once you have the required environment set up, running the Python server is straightforward:

1.  **Connect SpryTrack**:
    
    -   Connect the SpryTrack camera to your laptop via USB-C.
2.  **Run Python Server**:
    
    -   Execute the `server.py` script.
    -   The script will automatically track all optical markers and publish corresponding tracking data to ROS topics. For example, tracking data for `Pelvis` will be published to the topic `\Pelvis`.


# Question
Please don't hesitate to contact Luohong.wu@balgrist.ch if you have any questions.