from ultralytics import YOLO
import torch


def main():
    img = "img.jpg"
    model = YOLO("runs/detect/train21/weights/best.pt") #loads in our new model for yolov8
    weed_results = model.predict(f"../test/{img}")
    result_example = weed_results[0]
    weed_results[0].show()
    print(f"The amount of bounding boxes in this picture: {len(result_example.boxes)}")




if __name__ == "__main__":
    main()