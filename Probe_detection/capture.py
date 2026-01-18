import pyrealsense2 as rs
import numpy as np
import cv2
from datetime import datetime

# Configure streams
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

print("Press 'a' to capture photo, 'q' to quit")

try:
    while True:
        # Get frames
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            continue
        
        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data())
        
        # Display
        cv2.imshow('RealSense D435i', color_image)
        
        # Key controls
        key = cv2.waitKey(1)
        if key == ord('a'):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{timestamp}.png"
            cv2.imwrite(filename, color_image)
            print(f"Saved: {filename}")
        elif key == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
