#!/usr/bin/env python3

import cv2
import mediapipe as mp
import numpy as np

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float64MultiArray


class MediaPipeNode(Node):

    def __init__(self):

        super().__init__('mediapipe_node')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/hand/landmarks',
            10
        )

        self.cap = cv2.VideoCapture(0)

        self.mp_hands = mp.solutions.hands

        self.hands = self.mp_hands.Hands(

            static_image_mode=False,

            max_num_hands=1,

            min_detection_confidence=0.5,

            min_tracking_confidence=0.5

        )

        self.mp_draw = mp.solutions.drawing_utils

        self.timer = self.create_timer(
            0.03,
            self.update
        )

        self.get_logger().info(
            'MediaPipe Vision Node Started'
        )

    def update(self):

        success, frame = self.cap.read()

        if not success:
            return

        frame = cv2.flip(frame, 1)

        rgb = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )

        results = self.hands.process(rgb)

        msg = Float64MultiArray()

        landmark_data = []

        if results.multi_hand_landmarks:

            hand_landmarks = results.multi_hand_landmarks[0]

            for landmark in hand_landmarks.landmark:

                landmark_data.extend([

                    landmark.x,
                    landmark.y,
                    landmark.z

                ])

            self.mp_draw.draw_landmarks(

                frame,

                hand_landmarks,

                self.mp_hands.HAND_CONNECTIONS

            )

        msg.data = landmark_data

        self.publisher_.publish(msg)

        cv2.imshow(
            "Robot Hand Vision",
            frame
        )

        cv2.waitKey(1)

    def destroy_node(self):

        self.cap.release()

        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = MediaPipeNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()