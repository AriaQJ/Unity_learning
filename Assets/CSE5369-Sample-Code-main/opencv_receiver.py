import os
os.environ["OPENCV_IO_ENABLE_OPENEXR"] = "1"

import threading
import struct
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
import socket
import traceback
import math

INTERRUPT = False
THREAD_LOCK = threading.Lock()
OUTGOING_BUFFER = None
CHUNK_SIZE = 1400

# image defaults
IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 720
EOM_MARKER = b'<<EOM>>'

# image for later use
RGB_FRAME = None
DEPTH_FRAME = None

# thread safety lock
THREAD_LOCK = threading.Lock()

def rgb_func(ipaddr='127.0.0.1', port=65400):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind((ipaddr, port))
    udp_socket.settimeout(0.1)
    while not INTERRUPT:
        try:
            frame_data = bytearray()
            # read until saw at least two EOM markers
            while frame_data.count(EOM_MARKER) < 2:
                try:
                    data, _ = udp_socket.recvfrom(CHUNK_SIZE)
                    frame_data.extend(data)
                except socket.timeout:
                    pass
            # get the content between the two EOM markers
            start_idx = frame_data.find(EOM_MARKER)
            end_idx = frame_data.find(EOM_MARKER, start_idx + 1)
            frame_data = frame_data[start_idx + len(EOM_MARKER):end_idx]
            # decode JPEG image
            nparr = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            # display the image
            #cv2.imshow('RGB Image', frame)
            #cv2.waitKey(1)
            #print(frame)
            # save the image
            with THREAD_LOCK:
                global RGB_FRAME
                RGB_FRAME = frame
        except:
            # print(traceback.format_exc())
            # probably garbled frame, ignore
            pass
        
def depth_func(ipaddr='127.0.0.1', port=65401):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind((ipaddr, port))
    udp_socket.settimeout(0.1)
    while not INTERRUPT:
        try:
            frame_data = bytearray()
            # read until saw at least two EOM markers
            while frame_data.count(EOM_MARKER) < 2:
                try:
                    data, _ = udp_socket.recvfrom(CHUNK_SIZE)
                    frame_data.extend(data)
                except socket.timeout:
                    pass
            # get the content between the two EOM markers
            start_idx = frame_data.find(EOM_MARKER)
            end_idx = frame_data.find(EOM_MARKER, start_idx + 1)
            frame_data = frame_data[start_idx + len(EOM_MARKER):end_idx]
            # decode EXR ZIP image
            frame = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(frame, cv2.IMREAD_UNCHANGED)
            frame = frame[:, :, 2]
            # print(f'Minimum depth: {np.min(frame)}, Maximum depth: {np.max(frame)}')
            frame = cv2.normalize(frame, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            # display the image
            #cv2.imshow('Depth Image', frame)
            #cv2.waitKey(1)
            #print(frame)
            # save the image
            with THREAD_LOCK:
                global DEPTH_FRAME
                DEPTH_FRAME = frame
        except:
            # print(traceback.format_exc())
            # probably garbled frame, ignore
            pass

def send_thread(ipaddr='127.0.0.1', bind_port=65403, destination_port=65402):
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    udp_socket.bind((ipaddr, bind_port))
    udp_socket.settimeout(0.1)

    while not INTERRUPT:
        try:
            rgb_image = None
            depth_image = None
            
            try:
                with THREAD_LOCK:
                    if RGB_FRAME is not None: rgb_image = RGB_FRAME.copy()
                    if DEPTH_FRAME is not None: depth_image = DEPTH_FRAME.copy()
            except:
                pass
            
# ---------------------------------------------------------------------------- #
#                         ADD YOUR CODE BELOW THIS LINE                        #
# ---------------------------------------------------------------------------- #
            if rgb_image is not None and depth_image is not None:
                # Do image processing here

                # Convert RGB image to HSV color space
                hsv = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

                # Define color range for segmentation (e.g., red color)
                # Note: Adjust these values based on your scene
                lower_red1 = np.array([0, 70, 50])
                upper_red1 = np.array([10, 255, 255])
                mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

                lower_red2 = np.array([170, 70, 50])
                upper_red2 = np.array([180, 255, 255])
                mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

                mask = mask1 | mask2  # Combine masks

                # Find contours in the mask
                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

                detected_objects = []

                for cnt in contours:
                    # Calculate area and remove small elements
                    area = cv2.contourArea(cnt)
                    if area > 500:
                        # Compute bounding box
                        x, y, w, h = cv2.boundingRect(cnt)
                        # Draw bounding box on the RGB image
                        cv2.rectangle(rgb_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        # Compute centroid
                        M = cv2.moments(cnt)
                        if M['m00'] != 0:
                            cX = int(M['m10'] / M['m00'])
                            cY = int(M['m01'] / M['m00'])
                            # Draw a circle at the centroid
                            cv2.circle(rgb_image, (cX, cY), 5, (255, 0, 0), -1)  # Blue color

                            # Get depth value at centroid
                            Z = depth_image[cY, cX]  # depth_image should be in meters
                            # Handle invalid depth values
                            if np.isnan(Z) or Z <= 0:
                                continue
                            # Camera intrinsics (Adjust based on your camera parameters)
                            fx = 800  # Focal length in pixels along x
                            fy = 800  # Focal length in pixels along y
                            cx = IMAGE_WIDTH / 2  # Principal point x-coordinate
                            cy = IMAGE_HEIGHT / 2  # Principal point y-coordinate
                            # Compute camera coordinates
                            Xc = (cX - cx) * Z / fx
                            Yc = (cY - cy) * Z / fy
                            Zc = Z
                            # Adjust for coordinate system differences
                            Yc = -Yc  # Invert Y-axis to match Unity's coordinate system

                            # Compute rotation matrices
                            theta_x = math.radians(15)  # Rotation around X-axis
                            theta_y = math.radians(0)  # Rotation around Y-axis
                            theta_z = math.radians(0)  # Rotation around Z-axis

                            # Rotation matrices
                            Rx = np.array([
                                [1, 0, 0],
                                [0, math.cos(theta_x), -math.sin(theta_x)],
                                [0, math.sin(theta_x), math.cos(theta_x)]
                            ])

                            Ry = np.array([
                                [math.cos(theta_y), 0, math.sin(theta_y)],
                                [0, 1, 0],
                                [-math.sin(theta_y), 0, math.cos(theta_y)]
                            ])

                            Rz = np.array([
                                [math.cos(theta_z), -math.sin(theta_z), 0],
                                [math.sin(theta_z), math.cos(theta_z), 0],
                                [0, 0, 1]
                            ])

                            # Combined rotation matrix
                            R = Rz @ Ry @ Rx  # Note the order of rotations

                            # Camera position in Unity
                            t = np.array([0.0, 1.01, -1.85])  # [x, y, z]

                            # Camera coordinates as a vector
                            Pc = np.array([Xc, Yc, Zc])  # [Xc, Yc, Zc]

                            # Compute world coordinates
                            Pw = R @ Pc + t  # [Xw, Yw, Zw]

                            # Extract world coordinates
                            Xw, Yw, Zw = Pw.tolist()

                            # Append to list
                            detected_objects.append({'pixel': (cX, cY), 'world': (Xw, Yw, Zw)})
                # Prepare outgoing message
                outgoing_message = str(detected_objects).encode()
                print(detected_objects)
                # Display the image with bounding boxes and centroids
                cv2.imshow('Detected Objects', rgb_image)
                cv2.waitKey(1)

            else:
                # No image data available
                outgoing_message = 'No image data'.encode()
            
# ---------------------------------------------------------------------------- #
#                         ADD YOUR CODE ABOVE THIS LINE                        #
# ---------------------------------------------------------------------------- #

            udp_socket.sendto(outgoing_message, (ipaddr, destination_port))
        except Exception:
            # print(traceback.format_exc())
            # probably garbled frame, ignore
            pass

if __name__ == '__main__':
    # start rgb thread
    rbg_thread = threading.Thread(target=rgb_func)
    rbg_thread.start()
    # start depth thread
    depth_thread = threading.Thread(target=depth_func)
    depth_thread.start()
    # start sending thread
    send_thread = threading.Thread(target=send_thread)
    send_thread.start()
    # wait for threads to finish
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        INTERRUPT = True
        rbg_thread.join()
        send_thread.join()
        exit(0)
        
    

    