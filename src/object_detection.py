import cv2
import numpy as np
import torch

def image_object_detection(img):
    # Load the pre-trained YOLOv5 model
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)


    # Perform inference
    results = model(img)
    results.show()

    # # Results
    # results.print()

    # # Rendering the results
    # results.render()
    # for img in results.imgs:
    #     img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    #     cv2.imshow('YOLOv5 Object Detection', img_rgb)
    #     cv2.waitKey(0)
    #     cv2.destroyAllWindows()