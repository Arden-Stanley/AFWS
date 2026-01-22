import cv2
from ultralytics import YOLO
import torch
import serial
import time


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

# This will be used to test the face 

def save_to_file(x1,y1):
    with open("test.txt", "a") as info:
        info.write(f"{x1}, {y1} \n")


def arduinoSignal():
    ser.write('0').encode('utf-8')
    print(f"Sent: {data_to_send.strip()}")






def test_face(isDetected):


    #TO DO:

    # CAPTURE THIS DATA AND STORE IN CSV
    ## Get the normal values as well 

    # ANNOTATE A FACE AND TRAIN IT WITH train.afs.py 
    # TEST THE FACE AND RUN AN ARDUINO SCRIPT TO LIGHT UP LED

    value = 27
    select = input("Would you like to use pretrained or custom trained model? 1 - Pretrain | 2 - Custom")
    
    if int(select) == 2:
        face_model = YOLO(f"runs/detect/train{value}/weights/best.pt")
    else:
        face_model = YOLO("yolov8n.pt") #Original class, we don't have the faces yet 

    face_camera = cv2.VideoCapture(0)

    while face_camera.isOpened():
        verify, frame = face_camera.read()
        if not verify:
            break

        current_results = face_model(frame, conf=.50, verbose = False)
        isDetected = False # save coordinates to file
        ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        ser.flush()
        time.sleep(2) #Serial communication for PI and Arduino
        # This will take each invidiual frame that has been predicted with face_model and draw bounding boxes around them 
        for result in current_results:
            boxes = result.boxes
            for box in boxes:
               
                x1, y1, x2, y2 = map(int, box.xyxy[0]) # Takes the cooridnates out of a tensor and then changes the value from float to int for cv2
                confidence = box.conf[0].item() # confidence for the identified class
                
                #TEST CASES:
                hand_was_detected = False
                if y1 > 250 and not hand_was_detected #Test to see if it writes to file if bounding box passes 250 for y coordinate
                    hand_was_detected = True
                    save_to_file(x1,y1)
                    print("Test")
                    arduinoSignal(isDetected)
                    print(f"Weed detected, moving AFS 5 feet forward...")
                elif y1 <= 249
                    hand_was_detected = False
                class_id = int(box.cls[0]) #Class id to use with the class_name 
                class_name = face_model.names[class_id]
                label = f'{class_name}, {confidence}, {x1}, {y1}' # THis is the label that is above the bounding box

                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,100,0), 2) #This creates our own custom bounding box, using this instead of .plot
                cv2.putText(frame, label, (x1,y1-10), cv2.FONT_HERSHEY_SIMPLEX, .5, (0,255,0), 2) #Puts the label above the box 

        cv2.imshow("yolo", frame)

        k= cv2.waitKey(1) & 0xFF
        if k == ord('q'): # press q to quit 
            break
    face_camera.release()
    face_camera.destroyAllWindows()

def main():
    #pi_camera_active()
    isDetected = False

    test_face(isDetected)




if __name__ == "__main__":
    main()



