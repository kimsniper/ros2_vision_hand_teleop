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

        topic = self.declare_parameter(
            'landmarks_topic',
            '/hand/landmarks',
        ).value

        camera_index = self.declare_parameter(
            'camera_index',
            0,
        ).value

        frame_width = self.declare_parameter(
            'frame_width',
            1280,
        ).value

        frame_height = self.declare_parameter(
            'frame_height',
            720,
        ).value

        camera_fps = self.declare_parameter(
            'camera_fps',
            10.0,
        ).value

        max_num_hands = self.declare_parameter(
            'max_num_hands',
            2,
        ).value

        min_detection_confidence = self.declare_parameter(
            'min_detection_confidence',
            0.5,
        ).value

        min_tracking_confidence = self.declare_parameter(
            'min_tracking_confidence',
            0.5,
        ).value

        self.publisher_ = self.create_publisher(
            HandLandmarks,
            topic,
            10,
        )

        self.cap = cv2.VideoCapture(camera_index)

        if not self.cap.isOpened():
            raise RuntimeError(
                'Could not open camera index {}'.format(
                    camera_index
                )
            )

        self.cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            frame_width,
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            frame_height,
        )

        self.cap.set(
            cv2.CAP_PROP_FPS,
            camera_fps,
        )

        self.mp_hands = mp.solutions.hands

        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_num_hands,
            min_detection_confidence=(
                min_detection_confidence
            ),
            min_tracking_confidence=(
                min_tracking_confidence
            ),
        )

        self.frame_count = 0
        self.read_failure_count = 0
        self.right_hand_count = 0
        self.no_right_hand_count = 0

        self.get_logger().info(
            'Camera opened: requested={}x{} at {:.1f} FPS, '
            'reported={}x{} at {:.1f} FPS'.format(
                frame_width,
                frame_height,
                camera_fps,
                int(
                    self.cap.get(
                        cv2.CAP_PROP_FRAME_WIDTH
                    )
                ),
                int(
                    self.cap.get(
                        cv2.CAP_PROP_FRAME_HEIGHT
                    )
                ),
                self.cap.get(
                    cv2.CAP_PROP_FPS
                ),
            )
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

        self.frame_count += 1

        if not success:
            self.read_failure_count += 1

            if self.read_failure_count % 10 == 1:
                self.get_logger().warning(
                    'Camera read failed: {} of {} frames'.format(
                        self.read_failure_count,
                        self.frame_count,
                    )
                )

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

            right_candidates = []

            for index in range(detected_count):
                classification = (
                    results.multi_handedness[index]
                    .classification[0]
                )

                if classification.label != 'Right':
                    continue

                right_candidates.append((
                    float(classification.score),
                    results.multi_hand_landmarks[index],
                ))

            if right_candidates:
                _, hand_landmarks = max(
                    right_candidates,
                    key=lambda candidate: candidate[0],
                )

                detected_landmarks = []

                for landmark in hand_landmarks.landmark:
                    detected_landmarks.extend([
                        landmark.x,
                        landmark.y,
                        landmark.z,
                    ])

                if len(detected_landmarks) == 63:
                    self.mp_draw.draw_landmarks(
                        frame,
                        hand_landmarks,
                        self.mp_hands.HAND_CONNECTIONS,
                    )

                    landmark_data = detected_landmarks
                    hand_detected = True

        msg.landmarks = landmark_data
        msg.hand_detected = hand_detected

        if hand_detected:
            self.right_hand_count += 1
        else:
            self.no_right_hand_count += 1

        self.publisher_.publish(msg)

        cv2.imshow(self.window_name, frame)

        cv2.waitKey(1)

    def destroy_node(self):

        total_results = (
            self.right_hand_count
            + self.no_right_hand_count
        )

        detection_ratio = (
            self.right_hand_count / total_results
            if total_results > 0
            else 0.0
        )

        self.get_logger().info(
            'Vision summary: frames={}, read_failures={}, '
            'right_hand={}, no_right_hand={}, '
            'right_hand_duty_cycle={:.1f}%'.format(
                self.frame_count,
                self.read_failure_count,
                self.right_hand_count,
                self.no_right_hand_count,
                detection_ratio * 100.0,
            )
        )

        self.hands.close()
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
