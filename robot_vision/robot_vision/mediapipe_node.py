#!/usr/bin/env python3

# Copyright 2026 Mezael Docoy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import cv2
import mediapipe as mp

import rclpy

from rclpy.node import Node

from robot_vision_msgs.msg import HandLandmarks


class MediaPipeNode(Node):

    def __init__(self):

        super().__init__('mediapipe_node')

        self.publisher_ = self.create_publisher(
            HandLandmarks,
            '/hand/landmarks',
            10,
        )

        self.cap = cv2.VideoCapture(0)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.set(cv2.CAP_PROP_FPS, 10)

        self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

        self.mp_hands = mp.solutions.hands

        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        self.mp_draw = mp.solutions.drawing_utils

        self.window_name = "Robot Hand Vision"
        cv2.namedWindow(
            self.window_name,
            cv2.WINDOW_NORMAL,
        )
        cv2.resizeWindow(
            self.window_name,
            960,
            720,
        )

        self.timer = self.create_timer(0.03, self.update)

        self.get_logger().info('MediaPipe Vision Node Started')

    def update(self):

        success, frame = self.cap.read()

        if not success:
            return

        frame_stamp = self.get_clock().now().to_msg()

        frame = cv2.flip(frame, 1)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = self.hands.process(rgb)

        msg = HandLandmarks()
        msg.header.stamp = frame_stamp
        msg.header.frame_id = 'camera_frame'

        landmark_data = [0.0] * 63
        hand_detected = False

        if (
            results.multi_hand_landmarks
            and results.multi_handedness
        ):
            detected_count = min(
                len(results.multi_hand_landmarks),
                len(results.multi_handedness),
            )

            for index in range(detected_count):
                hand_landmarks = (
                    results.multi_hand_landmarks[index]
                )
                handedness = (
                    results.multi_handedness[index]
                    .classification[0]
                    .label
                )

                if handedness != "Right":
                    continue

                detected_landmarks = []

                for landmark in hand_landmarks.landmark:
                    detected_landmarks.extend([
                        landmark.x,
                        landmark.y,
                        landmark.z,
                    ])

                if len(detected_landmarks) != 63:
                    continue

                self.mp_draw.draw_landmarks(
                    frame,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                )

                landmark_data = detected_landmarks
                hand_detected = True
                break

        msg.landmarks = landmark_data
        msg.hand_detected = hand_detected

        self.publisher_.publish(msg)

        cv2.imshow(self.window_name, frame)

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
