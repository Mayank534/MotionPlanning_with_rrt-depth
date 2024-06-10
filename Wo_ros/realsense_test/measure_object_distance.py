import cv2
from realsense_camera import RealsenseCamera
from mask_rcnn import MaskRCNN
import numpy as np
import velocity_calculation as vc
import vp  # Import your velocity planning code

# Initialize camera and MaskRCNN
rs = RealsenseCamera()
mrcnn = MaskRCNN()

# Function to resize the frames to fit within the screen
def resize_frame(frame, max_height=600):
    height, width = frame.shape[:2]
    if height > max_height:
        scale_factor = max_height / height
        new_width = int(width * scale_factor)
        return cv2.resize(frame, (new_width, max_height))
    return frame

def custom_round(depth_cm):
    if depth_cm is None:
        return None

    # Convert depth to millimeters
    depth_mm = depth_cm * 100

    # Determine the rounded value
    rounded_value = int(depth_mm)
    if depth_mm - rounded_value >= 0.5:
        rounded_value += 1

    # Convert back to centimeters
    rounded_depth_cm = rounded_value / 100

    return rounded_depth_cm

def measure_object_distance(depth_frame, centers):
    if not centers:
        return None, None, None  # Return None for all values if no objects are detected

    # Assuming the first detected person
    cx, cy = centers[0]
    depth_mm = depth_frame[cy, cx]
    depth_cm = depth_mm / 1000.0  # Convert depth to centimeters

    # Custom rounding
    rounded_depth = custom_round(depth_cm)

    return rounded_depth, cx, cy

while True:
    # Get frame in real time from Realsense camera
    ret, bgr_frame, depth_frame = rs.get_frame_stream()

    # Resize frames to fit within the screen
    bgr_frame_resized = resize_frame(bgr_frame)
    depth_frame_resized = resize_frame(depth_frame)

    # Get object mask
    boxes, classes, contours, centers = mrcnn.detect_objects_mask(bgr_frame_resized)

    # Draw object mask
    bgr_frame_resized = mrcnn.draw_object_mask(bgr_frame_resized)

    # Show depth info of the objects
    mrcnn.draw_object_info(bgr_frame_resized, depth_frame_resized)

    # Measure the distance to the first detected person
    depth_m, cx, cy = measure_object_distance(depth_frame_resized, centers)

    if depth_m is not None:
        frame_height, frame_width, _ = bgr_frame_resized.shape

        # Calculate the angle with respect to the center of the screen
        angle_rad = np.arctan2(cy - frame_height / 2, cx - frame_width / 2)

        # Call the velocity calculation function
        vc.calculate_velocity_and_heading(depth_m, angle_rad)

    # Print the values
    print("Depth (m):", depth_m)
    print("Center X:", cx)
    print("Center Y:", cy)
    start_x=0
    start_y=0
    # Call the velocity planning main function
    vp.main(cx, cy, start_x, start_y)
    start_x=cx
    start_y=cy
    cv2.imshow("depth frame", depth_frame_resized)
    cv2.imshow("Bgr frame", bgr_frame_resized)

    key = cv2.waitKey(1)
    if key == 27:
        break

rs.release()
cv2.destroyAllWindows()
