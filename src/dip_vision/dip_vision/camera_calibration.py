import cv2
import numpy as np
import time

# === Calibration Settings ===
checkerboard_size = (9, 6)     # inner corners (not squares)
square_size = 0.025            # in meters (adjust to your printed checkerboard)
max_samples = 20               # number of valid frames to collect

# === Prepare object points ===
objp = np.zeros((np.prod(checkerboard_size), 3), np.float32)
objp[:, :2] = np.indices(checkerboard_size).T.reshape(-1, 2)
objp *= square_size

objpoints = []  # 3D real world points
imgpoints = []  # 2D image points

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("❌ Failed to open webcam.")
    exit()

print("📷 Show the checkerboard to the camera. Auto-capturing...")
last_capture_time = 0

while len(objpoints) < max_samples:
    ret, frame = cap.read()
    if not ret:
        print("❌ Frame read failed.")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

    if found:
        # Refine corner accuracy
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        cv2.drawChessboardCorners(frame, checkerboard_size, corners2, found)

        # Time buffer between captures
        if time.time() - last_capture_time > 1.0:
            objpoints.append(objp.copy())
            imgpoints.append(corners2)
            print(f"✅ Captured {len(objpoints)} / {max_samples}")
            last_capture_time = time.time()

    cv2.putText(frame, f"Captured: {len(objpoints)} / {max_samples}",
                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if found else (0, 0, 255), 2)
    cv2.imshow("Live Calibration", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("❌ Quit before calibration.")
        cap.release()
        cv2.destroyAllWindows()
        exit()

# === Calibration ===
ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

cap.release()
cv2.destroyAllWindows()

print("\n🎯 Calibration completed!")
print("Camera Matrix:\n", cameraMatrix)
print("Distortion Coefficients:\n", distCoeffs)

# Save results
np.savez("calibration_data.npz", cameraMatrix=cameraMatrix, distCoeffs=distCoeffs)
print("\n💾 Saved to calibration_data.npz")
