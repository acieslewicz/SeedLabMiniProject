import CommuniVision

if __name__ == "__main__":
    roundPos = 2
    quadrant_2_position = (0, round(np.pi/2, roundPos), round(np.pi, roundPos), round(3*np.pi/2,roundPos))
    oldQuadrant = None  
    oldPosition = None

    comandCamera = CommuniVision.CommuniVision()
    rawCapture = PiRGBArray(comandCamera.camera)
    
    for image in comandCamera.camera.capture_continuous(rawCapture, format='bgr', use_video_port=True):
        frame = image.array
        corners = CommuniVision.detectMarkers(frame)
        centers = CommuniVision.arucoCenters(corners)
        quadrants = CommuniVision.determineQuadrant(centers, comandCamera.resolution)
        
        if len(quadrants) > 0:
            if oldQuadrant is None or quadrants[0] != oldQuadrant:
                comandCamera.sendValueandDisplay(quadrants[0])
                oldQuadrant = quadrants[0]

        comandCamera.readPosition()

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