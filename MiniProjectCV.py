from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import time
import cv2
from cv2 import aruco

def show_image(image, window_name):
    """Display a cv2 image and handle closing the imshow window"""
    cv2.imshow(window_name, image)
    cv2.waitKey(0)
    cv2.destroyWindow(window_name)

def configure_camera(resolution=(1280, 720), framerate = 24, iso_mode=None):
    """Create a camera object with set resolution, framerate, and iso"""
    
    iso = 200
    #Determining the camera iso
    while iso_mode is None:
        iso_mode = input("Is is (b)right or (d)ark?")
        if iso_mode == "b":
            iso = 200
        elif iso_mode == "d":
            iso = 1200
        else:
            iso_mode = None
    
    #Create camera object
    camera = PiCamera(resolution=resolution, framerate=framerate)
    
    # camera configuration for consistent photos
    camera.iso = iso
    time.sleep(2)
    camera.shutter_speed = camera.exposure_speed
    camera.exposure_mode = 'off'
    
    #Calibrate AWB Gains
    gains = camera.awb_gains
    camera.awb_mode = 'off'
    camera.awb_gains = gains
    
    output = PiRGBArray(camera)
    new_awb_gain = [0,0]
    
    for i in range(3):
        camera.capture(output, format='bgr')
        calibration_image = output.array
        b = np.mean(calibration_image[:,:,0])
        g = np.mean(calibration_image[:,:,1])
        r = np.mean(calibration_image[:,:,2])
        new_awb_gain[0] += r/g
        new_awb_gain[1] += b/g
        output.truncate(0)
        print("r",r/g)
        print("g",b/g)
    
    new_awb_gain = [x/ 10 for x in new_awb_gain]
    print(new_awb_gain)
    
    camera.awb_gains = new_awb_gain

    # allow the camera to warmup
    time.sleep(0.1)
    
    return camera

def resize_image(image, verbose=False):
    window_name = 'Image'
    
    resize_ratiox = 0.5
    resize_ratioy = 0.5
    resized_image = cv2.resize(image, None, fx=resize_ratiox, fy=resize_ratioy, interpolation=cv2.INTER_CUBIC)
    
    if verbose == True:
        show_image(resized_image, window_name)
        
    return resized_image

def image_to_grayscale(image, verbose=False):
    window_name = 'Image'
    
    gray_scale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if verbose == True:
        show_image(gray_scale_image, window_name)
        
    return gray_scale_image

def detect_markers(image, verbose=False):
    resized_gs_image = image_to_grayscale(image)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = aruco.detectMarkers(resized_gs_image, aruco_dict, parameters=parameters)
    if verbose==True:
        if ids is None:
            print("No markers found")
        else:
            for id in ids:
                print("Marker", id, "found")

    if corners is None:
        return None
    else:
        return corners
    
    return

def capture_video_stream():
    resolution=(1280,720)
    camera = configure_camera(resolution=resolution)
    rawCapture = PiRGBArray(camera)
    
    #Capture Frames
    for frame in camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
        image = frame.array
        corners = detect_markers(image)
        centers = list()

        #Detect Centers
        for cornerset in corners:
            for corner in cornerset:
                point = list()
                point.append((corner[0][0]+corner[1][0])/2)
                point.append((corner[0][1]+corner[3][1])/2)
                centers.append(point)

        #Detect quadrant location
        quadrant = None
        if len(centers) != 0:
            if centers[0][0] < resolution[0]/2 and centers[0][1] > resolution[1]/2:
                quadrant = 3
            elif centers[0][0] > resolution[0]/2 and centers[0][1] > resolution[1]/2:
                quadrant = 2
            elif centers[0][0] > resolution[0]/2 and centers[0][1] < resolution[1]/2:
                quadrant = 1
            else:
                quadrant = 0
        if quadrant is not None:
            print(quadrant)

        cv2.aruco.drawDetectedMarkers(image, corners)
        
        image = cv2.flip(image, 1)
        cv2.imshow("Frame", image)
        detect_markers(image)
        key = cv2.waitKey(1) & 0xFF
        
        rawCapture.truncate(0)
        
        if key == ord("q"):
            cv2.destroyAllWindows()
            break
 
if __name__ == '__main__':
    capture_video_stream()