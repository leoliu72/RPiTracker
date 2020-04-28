import cv2

class Detection():
    def __init__(self):
        
        # Define HSV thresholds for detection
        self.lowerGreen = (29,86,6)
        self.upperGreen = (64,255,255)
        
    def update(self, frame):
        image = frame.array # read image
                    
        blur = cv2.GaussianBlur(image.copy(), (7,7), 0)
        ##    blur = cv2.medianBlur(blur, 7)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lowerGreen, self.upperGreen)
        ##    cv2.imshow('Before morpho', mask)

        ##    # More filtering at the cost of speed. Removes more noise, but just HSV masking
        ##    # seems to be sufficient for now
        ##    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
        ##    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        ##    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        ####    cv2.imshow('After morpho', mask)

        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if len(contours) > 0 and max([cv2.contourArea(x) for x in contours]) > 100:
            c = max(contours, key = cv2.contourArea)
            M = cv2.moments(c) # find moments
            
            cx = int(M['m10']/M['m00']) # x coordinate of center
            cy = int(M['m01']/M['m00']) # y coordinate of center
            ((x,y), radius) = cv2.minEnclosingCircle(c)
            center = (cx, cy)
            
            cv2.circle(image, (int(x),int(y)), int(radius), (0,0,255), 2) # draw circle around object
            cv2.circle(image, center, 5, (255,0,0), 2) # draw center dot
            cv2.imshow('Detected Ball', image)
            
            # return (x, y) center coordinates of the ball
            return(center)