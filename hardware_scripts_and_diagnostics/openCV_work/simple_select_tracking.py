import cv2

TRACKER_TYPE = "MIL"

if TRACKER_TYPE == "MIL":
    tracker = cv2.TrackerMIL_create()
else:
    print("Invalid tracker type.")
    exit()

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

tracking = False
bbox = None

print("Object tracker started.")
print("Press S to select an object.")
print("Press Q to quit.")

while True:
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read camera frame.")
        break

    frame = cv2.flip(frame, 1)

    height, width, _ = frame.shape

    if tracking:
        success, bbox = tracker.update(frame)

        if success:
            x, y, w, h = [int(v) for v in bbox]

            center_x = x + w // 2
            center_y = y + h // 2

            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                2
            )

            cv2.circle(
                frame,
                (center_x, center_y),
                8,
                (0, 255, 0),
                -1
            )

            print(
                "X: {}  Y: {}".format(center_x, center_y),
                end="\r"
            )

            cv2.putText(
                frame,
                "X: {} Y: {}".format(center_x, center_y),
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )

        else:
            tracking = False
            bbox = None
            print("\nObject lost.")

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

    cv2.imshow("Object Tracking", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("s"):
        bbox = cv2.selectROI(
            "Object Tracking",
            frame,
            False,
            False
        )

        if bbox[2] > 0 and bbox[3] > 0:
            tracker = cv2.TrackerMIL_create()
            tracker.init(frame, bbox)
            tracking = True

            print("\nTracking started.")

    elif key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
