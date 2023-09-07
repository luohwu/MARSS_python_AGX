#!/usr/bin/env python
import sys
import os

import sensor_msgs.msg
import std_msgs.msg

sys.path.append(os.path.join(os.path.dirname(__file__)))

import ctypes
import  cv2
import numpy as np

dir_path = os.path.dirname(os.path.realpath(__file__))
libcast = ctypes.CDLL(os.path.join(dir_path, "cast.dll"), ctypes.RTLD_GLOBAL)  # load the libcast.so shared library
pyclariuscast = ctypes.cdll.LoadLibrary(
    os.path.join(dir_path, "pyclariuscast.pyd"))  # load the pyclariuscast.so shared library

import pyclariuscast
from PIL import Image
import sensor_msgs
import rclpy
from Tools.cv2ROS import cv2_to_imgmsg
printStream = True

from rclpy.node import Node
class ClariusRosNode(Node):
    def __init__(self):

        super().__init__('ClariusRosNode')

        self.US_publisher = self.create_publisher(sensor_msgs.msg.Image, 'US_images', 0)


clariusNode=None
## called when a new processed image is streamed
# @param image the scan-converted image data
# @param width width of the image in pixels
# @param height height of the image in pixels
# @param sz full size of image
# @param micronsPerPixel microns per pixel
# @param timestamp the image timestamp in nanoseconds
# @param angle acquisition angle for volumetric data
# @param imu inertial data tagged with the frame
def newProcessedImage(image, width, height, sz, micronsPerPixel, timestamp, angle, imu):
    bpp = sz / (width * height)
    if printStream:
        print(
            "image: {0}, {1}x{2} @ {3} bpp, {4:.2f} um/px, imu: {5} pts".format(
                timestamp, width, height, bpp, micronsPerPixel, len(imu)
            ),
            end="\r",
        )
    if bpp == 4:
        img = Image.frombytes("RGBA", (width, height), image)
    else:
        img = Image.frombytes("L", (width, height), image)
    img=img.convert('L')
    # global image_in_opencv
    image_in_opencv=cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR)
    global clariusNode
    clariusNode.US_publisher.publish(cv2_to_imgmsg(image_in_opencv))
    # cv2.imshow('image',image_in_opencv)
    # cv2.waitKey(1)
    print(f"new Image of {width} x {height}")
    # img.save("processed_image.png")
    return


## called when a new raw image is streamed
# @param image the raw pre scan-converted image data, uncompressed 8-bit or jpeg compressed
# @param lines number of lines in the data
# @param samples number of samples in the data
# @param bps bits per sample
# @param axial microns per sample
# @param lateral microns per line
# @param timestamp the image timestamp in nanoseconds
# @param jpg jpeg compression size if the data is in jpeg format
# @param rf flag for if the image received is radiofrequency data
# @param angle acquisition angle for volumetric data
def newRawImage(image, lines, samples, bps, axial, lateral, timestamp, jpg, rf, angle):
    # check the rf flag for radiofrequency data vs raw grayscale
    # raw grayscale data is non scan-converted and in polar co-ordinates
    # print(
    #    "raw image: {0}, {1}x{2} @ {3} bps, {4:.2f} um/s, {5:.2f} um/l, rf: {6}".format(
    #        timestamp, lines, samples, bps, axial, lateral, rf
    #    ), end = "\r"
    # )
    # if jpg == 0:
    #    img = Image.frombytes("L", (samples, lines), image, "raw")
    # else:
    #    # note! this probably won't work unless a proper decoder is written
    #    img = Image.frombytes("L", (samples, lines), image, "jpg")
    # img.save("raw_image.jpg")
    return


## called when a new spectrum image is streamed
# @param image the spectral image
# @param lines number of lines in the spectrum
# @param samples number of samples per line
# @param bps bits per sample
# @param period line repetition period of spectrum
# @param micronsPerSample microns per sample for an m spectrum
# @param velocityPerSample velocity per sample for a pw spectrum
# @param pw flag that is true for a pw spectrum, false for an m spectrum
def newSpectrumImage(image, lines, samples, bps, period, micronsPerSample, velocityPerSample, pw):
    return


## called when freeze state changes
# @param frozen the freeze state
def freezeFn(frozen):
    if frozen:
        print("\nimaging frozen")
    else:
        print("imaging running")
    return


## called when a button is pressed
# @param button the button that was pressed
# @param clicks number of clicks performed
def buttonsFn(button, clicks):
    print("button pressed: {0}, clicks: {1}".format(button, clicks))
    return


## main function
def startClariusStreaming():


    # uncomment to get documentation for pyclariuscast module
    # print(help(pyclariuscast))

    rclpy.init()
    global clariusNode
    clariusNode=ClariusRosNode()
    # get home path
    path = os.path.expanduser("~/")

    ip="10.5.3.39"
    port=5828

    # Because we transfer US images by TCP, due to TCP's performance, lower resolution should increase frame rate on HL2
    image_width=640//4
    image_height=480//4
    # initialize
    cast = pyclariuscast.Caster(newProcessedImage, newRawImage, newSpectrumImage, freezeFn, buttonsFn)
    ret = cast.init(path, image_width, image_height)
    if ret:
        print("initialization succeeded")
        ret = cast.connect(ip, port, "research")
        if ret:
            print("connected to {0} on port {1}".format(ip, port))
        else:
            print("connection failed")
            if sys.platform.startswith("linux"):
                # unload the shared library before destroying the cast object
                ctypes.CDLL("libc.so.6").dlclose(libcast_handle)
            cast.destroy()
            return
    else:
        print("initialization failed")
        return

    # input loop
    key = ""
    while key != "q" and key != "Q":
        key = input("press (q)->quit (a)->action (s)->stream (p)->param change: ")
        if key == "a" or key == "A":
            key = input("(f)->freeze (i)->capture image (c)->capture cine, (d/D)->depth, (g/G)->gain: ")
            if key == "f" or key == "F":
                cast.userFunction(1, 0)
            elif key == "i" or key == "I":
                cast.userFunction(2, 0)
            elif key == "c" or key == "C":
                cast.userFunction(3, 0)
            elif key == "d":
                cast.userFunction(4, 0)
            elif key == "D":
                cast.userFunction(5, 0)
            elif key == "g":
                cast.userFunction(6, 0)
            elif key == "G":
                cast.userFunction(7, 0)
        elif key == "d" or key == "D":
            ret = cast.disconnect()
            if ret:
                print("successful disconnect")
            else:
                print("disconnection failed")
        elif key == "s" or key == "S":
            global printStream
            printStream = not printStream
        elif key == "p" or key == "P":
            inp = input("enter: {parameter name} {value [float/true/false]}").split()
            if len(inp) != 2:
                print("please format as: {parameter name} {value [float/true/false]}")
            else:
                if inp[1] == "true" or inp[1] == "false":
                    cast.enableParam(inp[0], 1 if inp[1] == "true" else 0)
                else:
                    cast.setParam(inp[0], float(inp[1]))

    cast.destroy()


if __name__ == "__main__":
    startClariusStreaming()