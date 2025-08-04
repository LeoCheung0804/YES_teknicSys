from typing import Tuple
from ultralytics import YOLO
from camera import WebcamVideoStream
import cv2
import socket
import json
import threading
from utils import get_center, draw_point, draw_contour, get_simple_mask, get_dist, draw_text, get_orientation

def get_current_position() -> dict:
    """Get the current position of the brick in the camera frame"""
    if saved is not None:
        return saved
    return None

def run_udp_server():
    """Run UDP server to handle position requests"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"UDP server listening on {UDP_IP}:{UDP_PORT}")
    
    while True:
        data, addr = sock.recvfrom(1024)
        if data.decode() == "get_brick_position":
            position_data = get_current_position()
            if position_data:
                response = json.dumps(position_data)
                sock.sendto(response.encode(), addr)
            else:
                sock.sendto(b"No position data available", addr)

if __name__ == "__main__":
    UDP_IP = "0.0.0.0"
    UDP_PORT = 5000
    saved = None

    try: 
        model = YOLO('yoloe-11l-seg.engine')
        # model = YOLO('yoloe-11s-seg_openvino_model/')
    except Exception as e:
        print(f"Error loading model: {e}, trying to export from YOLOE model.")
        model = YOLO('yoloe-11s-seg.pt')
        names = ['rectangular red brick']
        model.set_classes(names, model.get_text_pe(names))  # Set the classes you want to detect
        model.export(format='openvino', half=True)

        # reload the model after exporting
        model = YOLO('yoloe-11s-seg_openvino_model/')

        # model.export(format='engine')
        # model = YOLO('yoloe-11l-seg.engine')

    # open camera
    cap = WebcamVideoStream(src=0, flip_horizontal=False).start()
    udp_thread = threading.Thread(target=run_udp_server, daemon=True)
    udp_thread.start()

    while True:
        # read a frame
        img = cap.read()
        # Perform inference
        results = next(model.predict(img, stream=True, verbose=False))
        temp = None
        min_dist = None
        for mask in results.masks.xy if len(results) > 0 else []:
            mask = get_simple_mask(mask)
            
            center: Tuple[int, int] | None = get_center(mask)
            w, h = img.shape[1], img.shape[0]
            dist = get_dist(mask, 5900)
            angle = get_orientation(mask)
            if center is not None:
                # Draw the center point on the image
                img = draw_point(img, center)
                img = draw_text(img, f"{angle:.3f}", center)
                center_offset = ((center[0] - w // 2) * dist / 480, (center[1] - h // 2) * dist / 480)
                center_dist = (center_offset[0] ** 2 + center_offset[1] ** 2)
                if min_dist is None or center_dist < min_dist ** 2:
                    min_dist = center_dist ** 0.5
                    temp = {
                        "x": center_offset[0],
                        "y": center_offset[1],
                        "z": dist,
                        "angle": angle
                    }
                img = draw_text(img, f"{center_offset[0]:.3f}, {center_offset[1]:.3f}, {dist:.3f}", (center[0], center[1] + 20))
            
            if temp is not None:
                saved = temp
            # longest_edge = get_longest_edge(mask)
            img = draw_point(img, (w // 2, h // 2))  # Draw center of the image
            # img = draw_text(img, f"Longest Edge: {longest_edge:.2f}", (10, 30))
            img = draw_contour(img, mask)

        # Display the image
        cv2.imshow('YOLOe Tracking', img)

        # Exit on 'q' key press
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
    cap.stop()
