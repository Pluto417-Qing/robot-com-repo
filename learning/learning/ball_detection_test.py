import cv2
import numpy as np
import os
import joblib
from ball_detection import extract_features

# Load the trained model and scaler
scaler = joblib.load("../models/scaler.pkl")
classifier = joblib.load("../models/classifier.pkl")


def predict(image, bboxes):
    """
    Predict the class of each bounding box in the image.
    """
    predictions = []
    for bbox in bboxes:
        feature = extract_features(image, bbox)
        feature = scaler.transform([feature])
        pred = classifier.predict(feature)
        predictions.append(pred[0])
    return predictions


# Load test image and labels
test_image_dir = "../data/images/valid"
test_label_dir = "../data/labels/valid"

for filename in os.listdir(test_image_dir):
    if filename.endswith(".jpg") or filename.endswith(".png"):
        image_path = os.path.join(test_image_dir, filename)
        label_path = os.path.join(
            test_label_dir, filename.replace(".jpg", ".txt").replace(".png", ".txt")
        )

        # Load image
        image = cv2.imread(image_path)
        if image is None:
            continue

        # Load bounding boxes
        bboxes = []
        with open(label_path, "r") as f:
            for line in f:
                _, x_center, y_center, width, height = map(float, line.strip().split())
                bboxes.append((x_center, y_center, width, height))

        # Predict classes
        predictions = predict(image, bboxes)

        # Draw predictions on the image
        h, w, _ = image.shape
        for bbox, pred in zip(bboxes, predictions):
            x_center, y_center, width, height = bbox
            x1 = int((x_center - width / 2) * w)
            y1 = int((y_center - height / 2) * h)
            x2 = int((x_center + width / 2) * w)
            y2 = int((y_center + height / 2) * h)

            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                image,
                str(pred),
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        # Save the result
        output_path = f"../output/images/valid/{filename}"
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        cv2.imwrite(output_path, image)
        print(f"Processed and saved: {output_path}")
