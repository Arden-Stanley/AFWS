from ultralytics import YOLO
import torch

    
def main():
    model = YOLO("yolov8n.pt") #loads in the pretrained model for yolov8

    #We are going to train the model off of weed.v1i.yolov8

    folder = "facetest2"
    print(f"We are training the model from {folder} directory")
    model.train(
        data = "./" + folder + "/data.yaml", # We are getting the data from weed.v1i.yolov8
        epochs = 55, #more epochs, longer it takes to run 
        workers = 8, #more workers, faster
        imgsz = 640, #size to adjust image
        batch = 16, 
        device = 0 #device 0 graphics card
    )

    perfomance = model.val()


if __name__ == "__main__":
    main()