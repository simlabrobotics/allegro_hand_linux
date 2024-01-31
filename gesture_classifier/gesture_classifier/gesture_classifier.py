import rclpy
from rclpy.node import Node
import mediapipe as mp
import numpy as np
import cv2
from .submodules.model import KeyPointClassifier


class GestureClassifier(Node): 
    
    def __init__(self):
        super().__init__('gesture_classifier')
        
        self.kpclf = KeyPointClassifier()
        self.gestures = {
            1: "rock",
            2: "paper",
            3: "scissors",
        }
        
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands
        self.cap = cv2.VideoCapture(0)
        self.hands = self.mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5)
        
        self.timer = self.create_timer(0.1, self.timer_cb)
        
    def timer_cb(self):
        self.get_logger().info('In timer callback')
        
        

def main(args=None):
    rclpy.init(args=args)
    gesture_classifier = GestureClassifier()
    rclpy.spin(gesture_classifier)
    rclpy.shutdown()