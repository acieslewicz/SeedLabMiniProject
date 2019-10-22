import CommuniVision
import cv2
import numpy as np
from picamera.array import PiRGBArray

if __name__ == "__main__":
    oldQuadrant = None  

    comandCamera = CommuniVision.CommuniVision()
    rawCapture = PiRGBArray(comandCamera.camera)
    
    comandCamera.writeTopLine("D. Pos:")
    comandCamera.writeBotLine("A. Pos:")
    
    count = 0
    for image in comandCamera.camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
        frame = image.array
        corners = CommuniVision.detectMarkers(frame)
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners[0], 3.0, comandCamera.intrinsic_params, comandCamera.distortion)

        print(tvecs)
        if tvecs is not None:
            print("Distance:",tvecs[0][0][2], "cm")
            angle = -np.arctan(tvecs[0][0][0]/tvecs[0][0][2])
            print("Angle:", angle)
            
        

        #Show the image stream
        cv2.imshow("Video", frame)
        key = cv2.waitKey(1) & 0xFF
        rawCapture.truncate(0)
        
        #Exit and turn off devices if 'q' is pressed
        if key == ord("q"):
            cv2.destroyAllWindows()
            comandCamera.turnOffLCD()
            break
    pass

# if __name__ == "__main__":
#     oldQuadrant = None  

#     comandCamera = CommuniVision.CommuniVision()
#     rawCapture = PiRGBArray(comandCamera.camera)
    
#     comandCamera.writeTopLine("D. Pos:")
#     comandCamera.writeBotLine("A. Pos:")
    
#     count = 0
#     for image in comandCamera.camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
#         frame = image.array
#         print(count)
#         cv2.imwrite('test' + str(count) + '.jpg',frame)
#         count = count + 1
#         corners = CommuniVision.detectMarkers(frame)
#         centers = CommuniVision.arucoCenters(corners[0])
#         quadrants = CommuniVision.determineQuadrant(centers, comandCamera.resolution)
        
#         if len(quadrants) > 0:
#             if quadrants[0] != oldQuadrant:
#                 comandCamera.sendValueandDisplay(quadrants[0])
#                 oldQuadrant = quadrants[0]

#         currentPosition = comandCamera.readPosition()
#         if currentPosition is None:
#             comandCamera.writeBotLine("A. Pos:")

#         #Show the image stream
#         cv2.imshow("Video", frame)
#         key = cv2.waitKey(1) & 0xFF
#         rawCapture.truncate(0)
        
#         #Exit and turn off devices if 'q' is pressed
#         if key == ord("q"):
#             cv2.destroyAllWindows()
#             comandCamera.turnOffLCD()
#             break
#     pass