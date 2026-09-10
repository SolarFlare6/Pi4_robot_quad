import cv2
import numpy as np
import servo_driver_imp
import time

CAM_X = 0
CAM_Y = 1

CAM_X_CENTER = 90
CAM_Y_CENTER = 90

CAM_X_MIN = 20
CAM_X_MAX = 160
CAM_Y_MIN = 30
CAM_Y_MAX = 150

SERVO_STEP = 2

DEADBAND_X = 60
DEADBAND_Y = 50

SMOOTHING = 0.25

MIN_HAND_AREA = 2500
MAX_HAND_AREA = 200000

servo_driver_imp.set_servo_range(CAM_X, 180)
servo_driver_imp.set_servo_range(CAM_Y, 180)

camera_x = CAM_X_CENTER
camera_y = CAM_Y_CENTER

target_x = camera_x
target_y = camera_y

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("OpenCV hand tracking started.")
print("Move your hand to control the camera.")
print("Press Q to quit.")

kernel = np.ones((5, 5), np.uint8)

while True:
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read camera frame.")
        break

    frame = cv2.flip(frame, 1)

    height, width, _ = frame.shape

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower_skin = np.array([0, 20, 40], dtype=np.uint8)
    upper_skin = np.array([25, 255, 255], dtype=np.uint8)

    skin_mask = cv2.inRange(hsv, lower_skin, upper_skin)

    skin_mask = cv2.erode(skin_mask, kernel, iterations=1)
    skin_mask = cv2.dilate(skin_mask, kernel, iterations=2)
    skin_mask = cv2.GaussianBlur(skin_mask, (5, 5), 0)

    contours, _ = cv2.findContours(
        skin_mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    hand_found = False

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        if MIN_HAND_AREA < area < MAX_HAND_AREA:
            x, y, w, h = cv2.boundingRect(largest_contour)

            aspect_ratio = w / float(h)

            if 0.3 < aspect_ratio < 2.5:
                moments = cv2.moments(largest_contour)

                if moments["m00"] != 0:
                    hand_x = int(moments["m10"] / moments["m00"])
                    hand_y = int(moments["m01"] / moments["m00"])

                    hand_found = True

                    cv2.drawContours(
                        frame,
                        [largest_contour],
                        -1,
                        (0, 255, 0),
                        2
                    )

                    cv2.rectangle(
                        frame,
                        (x, y),
                        (x + w, y + h),
                        (0, 255, 0),
                        2
                    )

                    cv2.circle(
                        frame,
                        (hand_x, hand_y),
                        10,
                        (0, 255, 0),
                        -1
                    )

                    center_x = width // 2
                    center_y = height // 2

                    error_x = hand_x - center_x
                    error_y = hand_y - center_y

                    if abs(error_x) > DEADBAND_X:
                        if error_x > 0:
                            target_x -= SERVO_STEP
                        else:
                            target_x += SERVO_STEP

                    if abs(error_y) > DEADBAND_Y:
                        if error_y > 0:
                            target_y += SERVO_STEP
                        else:
                            target_y -= SERVO_STEP

                    target_x = max(
                        CAM_X_MIN,
                        min(CAM_X_MAX, target_x)
                    )

                    target_y = max(
                        CAM_Y_MIN,
                        min(CAM_Y_MAX, target_y)
                    )

                    camera_x += (
                        target_x - camera_x
                    ) * SMOOTHING

                    camera_y += (
                        target_y - camera_y
                    ) * SMOOTHING

                    servo_driver_imp.driver_set_servo_angle_with_index(
                        CAM_X,
                        camera_x
                    )

                    servo_driver_imp.driver_set_servo_angle_with_index(
                        CAM_Y,
                        camera_y
                    )

    center_x = width // 2
    center_y = height // 2

    cv2.line(
        frame,
        (center_x - 20, center_y),
        (center_x + 20, center_y),
        (255, 255, 255),
        1
    )

    cv2.line(
        frame,
        (center_x, center_y - 20),
        (center_x, center_y + 20),
        (255, 255, 255),
        1
    )

    if hand_found:
        cv2.putText(
            frame,
            "Hand detected",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )
    else:
        cv2.putText(
            frame,
            "No hand detected",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2
        )

    cv2.putText(
        frame,
        "X: {:.1f}  Y: {:.1f}".format(camera_x, camera_y),
        (20, height - 20),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2
    )

    cv2.imshow("Robot Hand Tracking", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()

servo_driver_imp.driver_set_servo_angle_with_index(
    CAM_X,
    CAM_X_CENTER
)

servo_driver_imp.driver_set_servo_angle_with_index(
    CAM_Y,
    CAM_Y_CENTER
)

print("Camera returned to center.")
servo_driver_imp.cleanup_servos()
