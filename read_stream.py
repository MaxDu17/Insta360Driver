import cv2
import numpy as np

def h264_stream_to_numpy_array(stream_path):
    # Open the video stream (could be a file or RTSP URL)
    cap = cv2.VideoCapture(stream_path)

    if not cap.isOpened():
        print("Error: Could not open the video stream.")
        return None

    frames = []

    while True:
        # Read a frame from the stream
        ret, frame = cap.read()
        
        if ret:
            cv2.imshow("test", frame)
            cv2.waitKey(1)
                
        # # If we couldn't read a frame, end of stream
        # if not ret:
        #     break
        
        # # Convert the frame to a NumPy array (it is already a NumPy array in OpenCV)
        # frames.append(frame)
    
    # Release the video stream
    cap.release()

    # # Convert list of frames to a single NumPy array (if desired)
    # frames_array = np.array(frames)

    # return frames_array

# Example usage
stream_path = '01.h264'  # Path to the H.264 video stream or file
frames_array = h264_stream_to_numpy_array(stream_path)

# Optional: Print the shape of the resulting NumPy array (frames, height, width, channels)
if frames_array is not None:
    print(f"Shape of the NumPy array: {frames_array.shape}")
