import cv2
from PyQt5.QtGui import *
import numpy as np

def covert_cv2qt_rgba(cv_img):
    cvtd_img = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2RGBA)
    h, w,c = cvtd_img.shape
    qimg = QImage(cvtd_img.data, w, h, w*c, QImage.Format_RGBA8888)
    
    pixmap01 = QPixmap.fromImage(qimg)
    return pixmap01

def convert_cv2qt(cv_img):
    cvtd_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
    h, w,c = cvtd_img.shape
    qimg = QImage(cvtd_img.data, w, h, w*c, QImage.Format_RGB888)
    
    pixmap01 = QPixmap.fromImage(qimg)
    return pixmap01

def intensity_to_color(intensities):
    normalized = (intensities - intensities.min()) / (intensities.max() - intensities.min())
    colors = np.zeros((len(intensities), 4))
    colors[:, 0] = normalized * 255  # R
    colors[:, 1] = (1 - normalized) * 255  # G
    colors[:, 2] = 0  # B
    colors[:, 3] = 255  # Alpha
    return colors