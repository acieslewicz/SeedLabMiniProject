import CommuniVision
import cv2
from picamera.array import PiRGBArray

if __name__ == "__main__":
    oldQuadrant = None  

    comandCamera = CommuniVision.CommuniVision(isoMode="dark")
    rawCapture = PiRGBArray(comandCamera.camera)
    
    comandCamera.writeTopLine("D. Pos:")
    comandCamera.writeBotLine("A. Pos:")
    
    for image in comandCamera.camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
        frame = image.array
        corners = CommuniVision.detectMarkers(frame)
        centers = CommuniVision.arucoCenters(corners[0])
        quadrants = CommuniVision.determineQuadrant(centers, comandCamera.resolution)
        
        if len(quadrants) > 0:
            if quadrants[0] != oldQuadrant:
                comandCamera.sendValueandDisplay(quadrants[0])
                oldQuadrant = quadrants[0]

        currentPosition = comandCamera.readPosition()
        if currentPosition is None:
            comandCamera.writeBotLine("A. Pos:")

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