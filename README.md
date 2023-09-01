
# Speech Recognition and Image Streaming Example

Welcome to the Speech Recognition and Image Streaming example application. This repository contains a demonstration of how to integrate speech recognition and image streaming using two fundamental components: a Unity client and a Python server. These components communicate through ROS2, utilizing the [Unity-Technologies/ROS-TCP-Connector](https://github.com/Unity-Technologies/ROS-TCP-Connector), a TCP-based communication framework. In this configuration, the Python server operates on Clara AGX, streaming images to HoloLens2 and transcribing speech from HoloLens2 to text. The transcribed text can then be analyzed using Chat-GPT. The Unity client, running on HoloLens2, can visualize images from Clara AGX and record and transmit user speech to Clara AGX.

## Usage

Follow these steps to set up and run the Speech Recognition and Image Streaming example:

1.  **Login to Clara AGX**
    
    -   Access Clara AGX and log in using the password `rocs`.
2.  **Clone the Repository**
    
    -   Clone this repository into the `/media/m2/` directory on Clara AGX. You can use the following command:
        

        
        `git clone [repository URL] /media/m2/` 
        
3.  **Build the Docker Image**
    
    -   Navigate to the repository folder on Clara AGX.
    -   Build the Docker image by executing the `build_image.sh` script:
        

        
        `./build_image.sh` 
        
4.  **Run the Docker Image**
    
    -   Start the Docker container with the image you built by running the `run_image.sh` script:

        
        `./run_image.sh` 
        
5.  **Run the Python Script**
    
    -   Execute the `example.py` Python script to initialize the Python server on Clara AGX.
6.  **Start the Unity Application on HoloLens2**
    
    -   On your HoloLens2 device, launch the Unity application.
    -   You should be able to visualize the streamed images from Clara AGX.
    -   Additionally, you can record and send audio from HoloLens2 to Clara AGX as part of the speech recognition functionality.

Explore the capabilities of this example to integrate speech recognition and image streaming. Feel free to adapt and extend this project to suit your specific needs.

## Questions
Should you have any questions, please don't hesitate to contact Luohong.wu@balgrist.ch	