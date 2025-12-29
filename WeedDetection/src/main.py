import cv2 as cv 
import random
from ultralytics import YOLO

#current relative path to picture
WEED_IMG_PATH = "../test/weed_test.jpeg" 
CAR_IMG_PATH = "../test/car_test.jpg"

#this is a basic model, not to be used in final model
model = YOLO("yolov8s.pt")

#for obtaining a pseudorandom color for image classification
def get_color(id):
    random.seed(id)
    return tuple(random.randint(0, 255) for _ in range(3))

#reading image data into buffer
image = cv.imread(WEED_IMG_PATH)

#tracking image data
results = model.track(image)

for result in results:
    class_names = result.names
    for box in result.boxes:
        if box.conf[0] > 0.4:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            
            cls = int(box.cls[0])
            class_name = class_names[cls]

            conf = float(box.conf[0])

            color = get_color(cls)

            cv.rectangle(image, (x1, y1), (x2, y2), color, 2)


cv.imshow("test", image)
cv.waitKey(0)
cv.destroyAllWindows()