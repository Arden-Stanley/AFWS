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

# This will be used to test the face 
def test_face():


    #TO DO:

    # CAPTURE THIS DATA AND STORE IN CSV
    ## Get the normal values as well 

    # ANNOTATE A FACE AND TRAIN IT WITH train.afs.py 
    # TEST THE FACE AND RUN AN ARDUINO SCRIPT TO LIGHT UP LED


    face_model = YOLO("runs/detect/train26/weights/best.pt")
    #face_model = YOLO("yolov8n.pt") #Original class, we don't have the faces yet 
    face_camera = cv2.VideoCapture(0)

    while face_camera.isOpened():
        verify, frame = face_camera.read()
        if not verify:
            break

        current_results = face_model(frame, conf=.50)

        # This will take each invidiual frame that has been predicted with face_model and draw bounding boxes around them 
        for result in current_results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0]) # Takes the cooridnates out of a tensor and then changes the value from float to int for cv2
                confidence = box.conf[0].item() # confidence for the identified class

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
    test_face()




if __name__ == "__main__":
    main()