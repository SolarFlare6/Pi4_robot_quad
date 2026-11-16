import cv2
from ultralytics import YOLO

MODEL = "yolo11n.pt"
model = YOLO(MODEL)

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

print("Object detection and tracking started.")
print("Press Q to quit.")

while True:
    ret, frame = cap.read()

    if not ret:
        print("Error: Could not read camera frame.")
        break

    frame = cv2.flip(frame, 1)

    results = model.track(
        frame,
        persist=True,
        verbose=False
    )

    annotated_frame = results[0].plot()

    if results[0].boxes is not None:
        boxes = results[0].boxes

        for i in range(len(boxes)):
            box = boxes[i]

            if box.id is not None:
                object_id = int(box.id[0])
            else:
                object_id = -1

            class_id = int(box.cls[0])
            confidence = float(box.conf[0])

            class_name = model.names[class_id]

            x1, y1, x2, y2 = box.xyxy[0].tolist()

            x1 = int(x1)
            y1 = int(y1)
            x2 = int(x2)
            y2 = int(y2)

            object_x = (x1 + x2) // 2
            object_y = (y1 + y2) // 2

            print(
                "ID: {} | Object: {} | Confidence: {:.2f} | X: {} | Y: {}".format(
                    object_id,
                    class_name,
                    confidence,
                    object_x,
                    object_y
                )
            )

            cv2.circle(
                annotated_frame,
                (object_x, object_y),
                6,
                (0, 255, 0),
                -1
            )

            cv2.putText(
                annotated_frame,
                "ID {}: {} ({:.2f})".format(
                    object_id,
                    class_name,
                    confidence
                ),
                (object_x - 60, object_y - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

    height, width, _ = frame.shape

    center_x = width // 2
    center_y = height // 2

    cv2.line(
        annotated_frame,
        (center_x - 20, center_y),
        (center_x + 20, center_y),
        (255, 255, 255),
        1
    )

    cv2.line(
        annotated_frame,
        (center_x, center_y - 20),
        (center_x, center_y + 20),
        (255, 255, 255),
        1
    )

    cv2.imshow("Robot Object Tracking", annotated_frame)

    key = cv2.waitKey(1) & 0xFF

    if key == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
