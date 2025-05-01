# --- ball_detection.py ---#
# use opencv to detect the ball


import cv2
import numpy as np
import os
from sklearn.preprocessing import StandardScaler
from sklearn.svm import SVC
import joblib


def extract_features(image, bbox):
    """
    Extract features from the image within the bounding box.
    Features include color histogram and shape information.
    """
    x_center, y_center, width, height = bbox
    h, w, _ = image.shape
    x1 = int((x_center - width / 2) * w)
    y1 = int((y_center - height / 2) * h)
    x2 = int((x_center + width / 2) * w)
    y2 = int((y_center + height / 2) * h)

    # Crop the region of interest (ROI)
    roi = image[y1:y2, x1:x2]

    # Resize ROI to a fixed size
    roi = cv2.resize(roi, (64, 64))

    # Compute color histogram
    hist = cv2.calcHist([roi], [0, 1, 2], None, [8, 8, 8], [0, 256, 0, 256, 0, 256])
    hist = cv2.normalize(hist, hist).flatten()

    return hist


def load_data(image_dir, label_dir):
    """
    Load images and corresponding labels, and extract features.
    """
    features = []
    labels = []

    for filename in os.listdir(image_dir):
        if filename.endswith(".jpg") or filename.endswith(".png"):
            image_path = os.path.join(image_dir, filename)
            label_path = os.path.join(
                label_dir, filename.replace(".jpg", ".txt").replace(".png", ".txt")
            )

            # Load image
            image = cv2.imread(image_path)
            if image is None:
                continue

            # Load labels
            with open(label_path, "r") as f:
                for line in f:
                    class_id, x_center, y_center, width, height = map(
                        float, line.strip().split()
                    )
                    bbox = (x_center, y_center, width, height)

                    # Extract features
                    feature = extract_features(image, bbox)
                    features.append(feature)
                    labels.append(int(class_id))

    return np.array(features), np.array(labels)


# Load training data
image_dir = "../data/images/train"
label_dir = "../data/labels/train"
features, labels = load_data(image_dir, label_dir)

# Normalize features
scaler = StandardScaler()
features = scaler.fit_transform(features)

# Train a classifier (e.g., SVM)
classifier = SVC(kernel="linear", probability=True)
classifier.fit(features, labels)

# Save the model and scaler
joblib.dump(scaler, "../models/scaler.pkl")
joblib.dump(classifier, "../models/classifier.pkl")
print("Model trained and saved.")
