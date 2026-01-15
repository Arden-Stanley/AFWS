import cv2
from ultralytics import YOLO
import torch

def send_signal():
    print(f"Sending high signal now..")

def pi_camera_active():
    data = cv2.VideoCapture(0)
    model = YOLO("runs/detect/train21/weights/best.pt")
    while data.isOpened:
        verify, frame = data.read() #verify to call later to validity, frame for 1 frame per 4.5ms
        if not verify:
            break # breaks out of script if it can't call back a frame 
        #print(f"Video capture has been opened successfully")
        current_results = model(frame, conf =.25) #used to predict with our trained model 
        single_frame = current_results[0].plot() #plots the data in the terminal 

        cv2.imshow("yolo", single_frame) #Displays the single image in the window 
    
        k= cv2.waitKey(1) & 0xFF
        if k == ord('q'): # press q to quit 
            break
    
    data.release()
    data.destroyAllWindows()



def test():
    img = "img.jpg"
    model = YOLO("runs/detect/train21/weights/best.pt") #loads in our new model for yolov8
    weed_results = model.predict(f"../test/{img}")
    result_example = weed_results[0]
    weed_results[0].show()
    print(f"The amount of bounding boxes in this picture: {len(result_example.boxes)}")
    # Run pi camera and get results per image 
    # Get coordinates of bounding box, move target to location and spray 

def test_face():
    # open the window
    # rasberry pi is going to find face, send command to arduino to light up led 
    
def main():
    pi_camera_active()




if __name__ == "__main__":
    main()