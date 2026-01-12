from ultralytics import YOLO
import torch

def send_signal():
    print(f"Sending high signal now..")

def pi_camera_active():


def main():
    img = "img.jpg"
    model = YOLO("runs/detect/train21/weights/best.pt") #loads in our new model for yolov8
    weed_results = model.predict(f"../test/{img}")
    result_example = weed_results[0]
    weed_results[0].show()
    print(f"The amount of bounding boxes in this picture: {len(result_example.boxes)}")
    # Run pi camera and get results per image 
    # Get coordinates of bounding box, move target to location and spray 




if __name__ == "__main__":
    main()