from threading import Thread
import cv2


class WebcamVideoStream:
    def __init__(self, src=0, flip_horizontal=False):
        self.stream = cv2.VideoCapture(src)
        if not self.stream.isOpened():
            raise ValueError(f"Cannot open camera with source {src}")
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.stream.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 2592)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 1944)
        self.stream_res = (
            self.stream.get(cv2.CAP_PROP_FRAME_WIDTH),
            self.stream.get(cv2.CAP_PROP_FRAME_HEIGHT),
        )
        print(f"Camera opened with source {src}")
        print(f"Camera resolution set to {self.stream_res[0]}x{self.stream_res[1]}")
        self.flip_horizontal = flip_horizontal
        self.stopped = False

    def start(self):
        Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            ret, img = self.stream.read()
            if ret:
                self.frame = img

    def read(self, raw=False):
        while not hasattr(self, "frame"):
            pass
        frame = self.frame
        
        # Apply horizontal flip if enabled
        if self.flip_horizontal:
            frame = cv2.flip(frame, 1)
            
        if raw:
            return frame
        h, w = frame.shape[:2]
        side = min(w, h)
        left = (w - side) // 2
        top = (h - side) // 2
        cropped_frame = frame[top:top + side, left:left + side]
        return cv2.resize(cropped_frame, (640, 640))

    def set_flip_horizontal(self, flip=True):
        """Toggle or set horizontal flip"""
        self.flip_horizontal = flip
        print(f"Horizontal flip {'enabled' if flip else 'disabled'}")

    def stop(self):
        self.stopped = True
        self.stream.release()
