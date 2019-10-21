#Importing Hardware Inteface Dependencies
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd
from picamera.array import PiRGBArray
from picamera import PiCamera
import smbus
import board
import busio

#Import Computer Vision Dependencies
import cv2
from cv2 import aruco
import numpy as np
import time

class CommuniVision:
    roundPos = 2
    quadrant_2_position = (0, round(np.pi/2, roundPos), round(np.pi, roundPos), round(3*np.pi/2,roundPos))

    #LCD Power On and Off
    def turnOnLCD(self):
        self.lcd.clear()
        self.lcd.color = [0,100,0]
        time.sleep(1)
    
    def turnOffLCD(self):
        self.lcd.clear()
        self.lcd.color = [0,0,0]
        time.sleep(1)

    def __init__(self, resolution=[1280, 720], framerate=27, isoMode="", address=0x04, lcdDimensions=[16, 2]):

        # Defining Communication Variables
        self.address = address
        self.lcmDimensions = lcdDimensions

        self.camera = PiCamera(resolution=resolution, framerate=framerate)
        self.resolution = resolution

        #TODO: Need to move these params to the init function so that they can be customized
        self.sensor_dimensions = [3.76, 2.74]
        self.distortion = np.load('camera_distortion.npy')
        self.intrinsic_params = np.load('camera_intrinsic.npy')
        self.focal_lengths = [self.intrinsic_params[0][0] * self.sensor_dimensions[0] /resolution[0], self.intrinsic_params[1][1] * self.sensor_dimensions[1] /resolution[1]]

        #Configure camera iso and exposure for consistent quality
        if isoMode == "dark":
            self.camera.iso = 1200
        elif isoMode == "bright":
            self.camera.iso = 200
        else:
            #print("Unrecognized iso mode. Setting iso to 700.")
            self.camera.iso = 700

        self.camera.shutter_speed = self.camera.exposure_speed
        self.camera.exposure_mode = 'off'
        
        time.sleep(2)

        #Preconfigure the camera AWB setting to static values
        gains = self.camera.awb_gains
        self.camera.awb_mode = 'off'
        self.camera.awb_gains = gains

        time.sleep(0.1)

        #Configure i2c Settings
        self.bus = smbus.SMBus(1)
        #Set SDA and SCL to the connection on the I2c
        self.i2c = busio.I2C(board.SCL, board.SDA)
        
        # Initialise the LCD object
        self.lcd = character_lcd.Character_LCD_RGB_I2C(self.i2c, lcdDimensions[0], lcdDimensions[1])
        self.turnOnLCD()
    
    #Write Lines to the LCD
    def writeTopLine(self, topLine=""):
        message = str(topLine)
        #self.lcd.clear()
        self.lcd.message = message
        return

    def writeBotLine(self, botLine=""):
        message = "\n" + str(botLine)
        #self.lcd.clear()
        self.lcd.message = message
        return

    def readPosition(self, messageSize=5):
        data = None
        try:
            #Reads floats up to messageSize - 1 digits from Arduino
            data = bus.read_i2c_block_data(self.address, 0, messageSize)
        except:
            #print("Error Reading Data")
            data = None
            
        actualPositionFloat = None
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
            self.writeBotLine("A. pos: " + actualPosition)
        
        return actualPositionFloat

    def sendValueandDisplay(self, quadrant):
        if quadrant is not None:
            self.lcd.clear()
            self.writeTopLine("D. Pos:" + str(self.quadrant_2_position[quadrant-1]))
             #Sends quandrant information to the arduino
            try:
                self.bus.write_byte(self.address, quadrant)
            except:
                pass
        return

#Image Processing
def resizeImage(image, resizeRatio=0.5):
    """Resizes an image maintaining the aspect ratio based off the internal function ratios"""
    return cv2.resize(image, None, fx=resizeRatio, fy=resizeRatio, interpolation=cv2.INTER_CUBIC)

def imageToGrayscale(image):
    """Converts and image to grayscale and returns the grayscale image"""
    return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

#Aruco Marker Detection
def detectMarkers(image, arucoDictionary=aruco.DICT_6X6_250):
    """Detects Aruco markers and returns the Aruco corners and ids"""
    gs_image = imageToGrayscale(image)
    aruco_dict = aruco.Dictionary_get(arucoDictionary)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedCorners = aruco.detectMarkers(gs_image, aruco_dict, parameters=parameters)

    return corners, ids

def arucoCenters(corners):
    centers = list()

    #Detect Aruco centers by taking the average of the top left and right Aruco corners
    #for horizontal centers and top left and bottom left for vertical centers
    if len(corners) > 0:
        for cornerset in corners:
            point = list()
            point.append((cornerset[0][0][0]+cornerset[0][1][0]+cornerset[0][2][0]+cornerset[0][3][0])/4)
            point.append((cornerset[0][0][1]+cornerset[0][1][1]+cornerset[0][2][1]+cornerset[0][3][1])/4)
            centers.append(point)

    return centers  
    
def determineQuadrant(arucoCenters, resolution):
    quadrants = list()
    for center in arucoCenters:
        if center[0] < resolution[0]/2 and center[1] > resolution[1]/2:
            quadrants.append(4)
        elif center[0] > resolution[0]/2 and center[1] > resolution[1]/2:
            quadrants.append(3)
        elif center[0] > resolution[0]/2 and center[1] < resolution[1]/2:
            quadrants.append(2)
        else:
            quadrants.append(1)

    return quadrants

def arucoMarkerDimension(arucoCorners):
    width_1 = np.linalg.norm((arucoCorners[0][0][0] - arucoCorners[0][0][1]))
    height_1 = np.linalg.norm((arucoCorners[0][0][1] - arucoCorners[0][0][2]))
    
    width_2 = np.linalg.norm((arucoCorners[0][0][2] - arucoCorners[0][0][3]))
    height_2 = np.linalg.norm((arucoCorners[0][0][3] - arucoCorners[0][0][0]))

    return (width_1+width_2)/2, (height_1+height_2)/2

#Calculate distance using focal length and aruco marker dimensions
def calculateDistance(focalLengths, arucoCorners, arucoMarkerDim_w, imageDim, sensorDim):
    #Determine the dimensions of the Aruco Marker
    markerWidth, markerHeight = arucoMarkerDimension(arucoCorners)

    #Using the width and heigh determine the distance to the marker in mm
    distance_height = (focalLengths[0]*arucoMarkerDim_w[0])/(markerWidth*sensorDim[0]/imageDim[0])
    distance_width = (focalLengths[1]*arucoMarkerDim_w[1])/(markerHeight*sensorDim[1]/imageDim[1])
    distanceToMarker = (distance_height + distance_width)/2

    return distanceToMarker