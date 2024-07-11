import cv2

def check_cameras():
    # Iterate over camera indices until no more cameras are found
    index = 0
    while True:
        # Try to capture from the camera index
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        
        # Check if camera is accessible
        if index > 10:
            break
        
        # Read a frame from the camera (this should always succeed if the camera is opened)
        ret, frame = cap.read()
        
        # Display information about the camera
        print(f"Camera {index}: {'Open' if ret else 'Closed'}")
        
        # Release the capture object
        cap.release()
        
        # Move to the next camera index
        index += 1

if __name__ == "__main__":
    print("Checking for open cameras...")
    check_cameras()
