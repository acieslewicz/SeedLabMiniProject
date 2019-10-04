"""
Mini Project OpenCV and communication integration
========

Detects aruco markers, localizes the marker within 4 quadrants and then transmits this 
quadrant position to an Arduino for motor control. It also handles reading position from
and Ardunio and displays the desired and actual position values on a 2x16 LCD display.

Look how easy it is to use:

    Import as a module in order to access the functions inside
    Run as main program in order to start Aruco detection and Arduino communication automatically
    : "python3 MiniProject.py"
	
Credits
--------
Alexander Cieslewicz and Austin Seamount

Features
--------

- Shows an image using imshow and handles closing it
- Configures cameras for consistent image quality
- Scales an image to have its size maintaining its aspect ration
- Converts an image to grayscale
- Detects any aruco markers within an image and returns the corners in pixel positions
- Detects aruco markers localizes them with a quadrant and communicates with an arduino in order
  to complete motor control and positioning showing these valeus on an LCD screen

Dependencies
------------

- Numpy
- Picamera
- PiRGBArray
- time
- openCV2
- Aruco
- MiniProjectCom module


Support
-------
If there are any issues email us at: acieslewicz@mymail.mines.edu or seamount@mymail.mines.edu
"""

from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
from cv2 import aruco
import MiniProjectCom

def show_image(image, window_name):
    """Display a cv2 image and handle closing the imshow window"""
    cv2.imshow(window_name, image)
    cv2.waitKey(0)
    cv2.destroyWindow(window_name)

def configure_camera(resolution=(1280, 720), framerate = 24, iso_mode=None):
    """Create a camera object with set resolution, framerate, and iso. Also calibrates the AWB
    using a manual method.
    
    Key word arguments:
    resolution -- The camera image and video resolution default 1280x720
    framerate -- The camera framerate default 24
    isomode -- The camera iso whether passed as (b)right or (d)ark. If no parameter is passed
    the configuration method will prompt the user for a value
    """
    
    iso = 200
    #Determining the camera iso based on user input
    while iso_mode is None:
        iso_mode = input("Is is (b)right or (d)ark?")
        if iso_mode == "b":
            iso = 200
        elif iso_mode == "d":
            iso = 1200
        else:
            iso_mode = None

    camera = PiCamera(resolution=resolution, framerate=framerate)
    
    #Configure camera iso and exposure for consistent quality
    camera.iso = iso
    time.sleep(2)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    
    #Preconfigure the camera AWB setting to static values
    gains = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = gains
    
    output = PiRGBArray(camera)
    new_awb_gain = [0,0]
    
    #Determining rg bg ratios from a calibration photo in order to determine white balance gains
    #PiCamera AWB using rg bg ration for setting balance.
    for i in range(10):
        camera.capture(output, format='bgr')
        calibration_image = output.array
        b = np.mean(calibration_image[:,:,0])
        g = np.mean(calibration_image[:,:,1])
        r = np.mean(calibration_image[:,:,2])
        new_awb_gain[0] += r/g
        new_awb_gain[1] += b/g
        output.truncate(0)
        print("rg ratio:",r/g)
        print("bg ratio:",b/g)
    
    new_awb_gain = [x/ 10 for x in new_awb_gain]
    print("New awb gain:", new_awb_gain)
    
    camera.awb_gains = new_awb_gain

    # allow the camera to warmup
    time.sleep(0.1)
    
    return camera

def resize_image(image, verbose=False):
    """Resizes an image maintaining the aspect ratio based off the internal function ratios"""
    window_name = 'Image'
    
    resize_ratiox = 0.5
    resize_ratioy = 0.5
    resized_image = cv2.resize(image, None, fx=resize_ratiox, fy=resize_ratioy, interpolation=cv2.INTER_CUBIC)
    
    if verbose == True:
        show_image(resized_image, window_name)
        
    return resized_image

def image_to_grayscale(image, verbose=False):
    """Converts and image to grayscale and returns the grayscale image"""
    window_name = 'Image'
    
    gray_scale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if verbose == True:
        show_image(gray_scale_image, window_name)
        
    return gray_scale_image

def detect_markers(image, verbose=False):
    """Detects Aruco markers and returns the Aruco marker pixel based corner positions"""
    #Detect the aruco markers
    gs_image = image_to_grayscale(image)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gs_image, aruco_dict, parameters=parameters)

    #Print marker ids if in verbose mode
    if verbose==True:
        if ids is None:
            print("No markers found")
        else:
            for id in ids:
                print("Marker", id, "found")

    #Return the appropriate values based on whether markers were detected or not
    if corners is None:
        return None
    else:
        return corners
    
    return

def analyze_video_stream():
    """This method has many functions. It handles captures a video stream, detects markers within
    the stream and determines the image quadrant of these markers. The determined position is then
    sent to the Arduino and displayed on an LCD while the actual Arduino motor position is read by
    the function and also displayed on the LCD. This function handles the camera calibration and 
    configuration. As such it can be called directly without prior configration.
    """
    #TODO: Refactor into smaller methods that can be individually called.

    #Set the default Arduino address for i2c communication
    address = 0x04

    #Define lookup for quadrant to acutual radial position
    roundPos = 2
    quadrant_2_position = (0, round(np.pi/2, roundPos), round(np.pi, roundPos), round(3*np.pi/2,roundPos))

    #Create and Configure the camera object
    resolution=(1280,720)
    camera = configure_camera(resolution=resolution)
    rawCapture = PiRGBArray(camera)
    
    #Configure I2C Com and LCD
    bus, i2c, lcd = MiniProjectCom.configure_communication()
    MiniProjectCom.write_messages(lcd, "D. Pos:" + " None")
    
    oldQuadrant = None
    
    #Capture and Analyze each frame in a continous PiCamera Capture
    for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
        image = frame.array
        corners = detect_markers(image)
        centers = list()

        #Detect Aruco centers by taking the average of the top left and right Aruco corners
        #for horizontal centers and top left and bottom left for vertical centers
        for cornerset in corners:
            for corner in cornerset:
                point = list()
                point.append((corner[0][0]+corner[1][0])/2)
                point.append((corner[0][1]+corner[3][1])/2)
                centers.append(point)
                
        #Initialize the size of data to recieve and ensures that the starting data value is Null 
        #in order to recieve the Arduino's actual position
        size = 5
        data = None
        try:
            #Reads data from the Arduino in blocks of 5
            data = bus.read_i2c_block_data(address, 0, size)
        except:
            print("Error Reading Data")
            
        #Process any actual position data if it is received
        if data is not None:
            #Recieve each character from the recieve buffer and append it to a string
            actualPosition = ""
            for character in data:
                actualPosition = actualPosition + (chr(character))
            #Prevents the system from displaying negative numbers
            actualPositionFloat = float(actualPosition[:-1])
            if actualPositionFloat < 0:
                actualPositionFloat = round(2*np.pi + actualPositionFloat, 2)
            if actualPositionFloat == 0:
                actualPositionFloat = 0.00
            
            #Show the position on the LCD
            actualPosition = str(actualPositionFloat)
            MiniProjectCom.write_messages(lcd, "","A. Pos:" + actualPosition)

        #Detect quadrant location by comparing the Aruco center to the image quadrant bounds 
        quadrant = None
        if len(centers) != 0:
            if centers[0][0] < resolution[0]/2 and centers[0][1] > resolution[1]/2:
                quadrant = 4
            elif centers[0][0] > resolution[0]/2 and centers[0][1] > resolution[1]/2:
                quadrant = 3
            elif centers[0][0] > resolution[0]/2 and centers[0][1] < resolution[1]/2:
                quadrant = 2
            else:
                quadrant = 1
        
        #As long as the quadrant is detected show the value on the LCD and send the
        #value to the Arduino
        if quadrant is not None:
            if quadrant != oldQuadrant:
                lcd.clear()
                MiniProjectCom.write_messages(lcd, "D. Pos:" + str(quadrant_2_position[quadrant - 1]))
             #Sends quandrant information to the arduino
            bus.write_byte(address, quadrant) 
            oldQuadrant = quadrant
            
        cv2.aruco.drawDetectedMarkers(image, corners)

        #Flips the image since the camera reads it in backwards
        image = cv2.flip(image, 1)

        #Show the image stream
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
        
        #Exit and turn off devices if 'q' is pressed
        if key == ord("q"):
            cv2.destroyAllWindows()
            MiniProjectCom.turn_lcd_off(lcd)
            break
 
if __name__ == '__main__':
    analyze_video_stream()
