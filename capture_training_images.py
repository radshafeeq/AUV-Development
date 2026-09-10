#!/usr/bin/env python3
"""
AUV Camera Training Image Capture Tool
--------------------------------------
Press SPACEBAR to capture an image from the live AUV / laptop camera.
Images are automatically saved to 'dataset_images/' ready for labeling.
"""

import os
import sys
import time
import cv2
import auv_yolo_tracking

SAVE_DIR = "dataset_images"
os.makedirs(SAVE_DIR, exist_ok=True)

def main():
    print("=" * 60)
    print("      AUV DATASET IMAGE CAPTURE TOOL FOR CUSTOM YOLO       ")
    print("=" * 60)
    print(" - Press SPACEBAR to capture & save a training image.")
    print(" - Press 'q' to quit.")
    print("-" * 60)

    # Connect to BlueOS 1.4.5 RTSP stream or fallback
    rtsp_url = "rtsp://192.168.2.2:8554/video_udp_stream_0"
    print(f"[Video] Connecting to camera stream: {rtsp_url}...")
    grabber = auv_yolo_tracking.RTSPFrameGrabber(rtsp_url, width=1280, height=720)

    if not grabber.isOpened():
        print("[Video Warning] RTSP stream absent. Falling back to GStreamer UDP 5600...")
        grabber = auv_yolo_tracking.GStreamerFrameGrabber(port=5600, width=1280, height=720)

    if not grabber.isOpened():
        print("[Video Warning] Network stream absent. Falling back to local camera index 0...")
        grabber = auv_yolo_tracking.FallbackWebcamGrabber(0)

    if not grabber.isOpened():
        print("[Video Error] Failed to open camera stream.")
        return

    WINDOW_NAME = "AUV Training Image Collector"
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, 1280, 720)

    count = len(os.listdir(SAVE_DIR))
    print(f"[System] Ready! Currently {count} images saved in '{SAVE_DIR}/'")

    while True:
        ret, frame = grabber.read()
        if not ret or frame is None:
            time.sleep(0.05)
            continue

        display_frame = frame.copy()
        cv2.putText(display_frame, f"Saved Images: {count}  |  Press SPACE to Snap  |  Press Q to Exit", 
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        cv2.imshow(WINDOW_NAME, display_frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord(' '):  # SPACEBAR
            count += 1
            filename = os.path.join(SAVE_DIR, f"mechatronics_{count:04d}.jpg")
            cv2.imwrite(filename, frame)
            print(f"[Captured] Saved: {filename}")
            
            # Flash visual feedback
            cv2.rectangle(display_frame, (0, 0), (display_frame.shape[1], display_frame.shape[0]), (255, 255, 255), -1)
            cv2.imshow(WINDOW_NAME, display_frame)
            cv2.waitKey(50)

        elif key == ord('q'):
            break

    grabber.release()
    cv2.destroyAllWindows()
    print(f"[System] Done! Total {count} images saved in '{SAVE_DIR}/'.")

if __name__ == "__main__":
    main()
