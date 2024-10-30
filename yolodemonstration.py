import cv2
import numpy as np
from ultralytics import YOLO

model = YOLO('best.pt')
model.predict(source=0, show=True, conf=0.5)