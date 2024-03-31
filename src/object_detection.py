import cv2
import numpy as np
import torch

# def image_object_detection(img):
#     # Load the pre-trained YOLOv5 model
#     model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)


#     # Perform inference
#     results = model(img)
#     results.show()

def image_object_detection(img):
    # Load the pre-trained YOLOv5 model
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

    # Perform inference
    results = model(img)

    # Extract detected class names and confidence scores
    detected_classes = results.names  # List of all class names in the YOLO model
    detections = results.xyxy[0]  # Detection results

    # Define the objects of interest
    objects_of_interest = ['cup', 'bottle']  # Example: looking for cups and bottles

    # Filter detections for our objects of interest
    filtered_detections = []
    for *xyxy, conf, cls in detections:
        class_name = detected_classes[int(cls)]
        if class_name in objects_of_interest:
            filtered_detections.append((xyxy, conf, class_name))
            print(f"Detected {class_name} with confidence {conf:.2f}")

    # Draw filtered detections on the image
    for xyxy, conf, class_name in filtered_detections:
        color = (255, 0, 0)  # Blue color for the bounding box
        cv2.rectangle(img, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), color, 2)
        cv2.putText(img, f"{class_name} {conf:.2f}", (int(xyxy[0]), int(xyxy[1]-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

    # Show the image
    cv2.imshow('Filtered YOLOv5 Object Detection', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()