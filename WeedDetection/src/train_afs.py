from ultralytics import YOLO
import torch


model = YOLO("yolov8n.pt")  

# Train on your weed dataset
model.train(
    data="data.yaml", 
    epochs=100,
    imgsz=640,
    batch=16,
    device=0,      
    workers=8
)
