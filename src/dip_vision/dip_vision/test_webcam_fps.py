import cv2
import time

def test_webcam_fps(camera_index=0):
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print("❌ Cannot open webcam")
        return

    print("✅ Webcam opened successfully")
    prev_time = time.time()
    frame_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("❌ Can't receive frame (stream end?). Exiting...")
            break

        frame_count += 1
        curr_time = time.time()
        elapsed = curr_time - prev_time

        if elapsed >= 1.0:
            fps = frame_count / elapsed
            print(f"📷 FPS: {fps:.2f}")
            prev_time = curr_time
            frame_count = 0

        # Display the FPS on the frame
        cv2.putText(frame, f"FPS: {fps:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        cv2.imshow('Webcam Test', frame)
        if cv2.waitKey(1) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_webcam_fps()
