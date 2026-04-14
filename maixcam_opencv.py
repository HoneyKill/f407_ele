import cv2
import numpy as np
import struct
import time

try:
    import serial
except ImportError:
    serial = None


# ---------- Config ----------
CAMERA_INDEX = 0
FRAME_WIDTH = 320
FRAME_HEIGHT = 240
SERIAL_PORT = "COM3"        # Linux example: "/dev/ttyUSB0"
SERIAL_BAUDRATE = 1_000_000
RED_DOT_REACHED_THRESHOLD = 10
WINDOW_NAME = "opencv_tracking"
# ---------------------------


VOFA_HEADER = bytes([0x00, 0xFF, 0xFA])
VOFA_FRAME_SIZE = 8
VOFA_CMD_CALIB_REQ = 0xE0
VOFA_CMD_CALIB_PX_X = 0xE1
VOFA_CMD_CALIB_PX_Y = 0xE2


def create_vofa_frame(command, data_value):
    cmd_byte = command.to_bytes(1, "big")
    data_bytes = struct.pack("<f", float(data_value))
    return VOFA_HEADER + cmd_byte + data_bytes


class DummySerial:
    def read(self, _size=0):
        return b""

    def write(self, _data):
        return 0

    @property
    def in_waiting(self):
        return 0

    def close(self):
        pass


class VofaReceiver:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf.extend(data)
        results = []
        while len(self.buf) >= VOFA_FRAME_SIZE:
            idx = self.buf.find(VOFA_HEADER)
            if idx < 0:
                self.buf = self.buf[-2:] if len(self.buf) >= 2 else self.buf
                break
            if idx > 0:
                self.buf = self.buf[idx:]
            if len(self.buf) < VOFA_FRAME_SIZE:
                break
            cmd = self.buf[3]
            value = struct.unpack("<f", self.buf[4:8])[0]
            results.append((cmd, value))
            self.buf = self.buf[VOFA_FRAME_SIZE:]
        return results


def open_serial():
    if serial is None:
        print("pyserial not found, serial disabled.")
        return DummySerial()
    try:
        dev = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=0)
        print(f"Serial opened: {SERIAL_PORT} @ {SERIAL_BAUDRATE}")
        return dev
    except Exception as err:
        print(f"Serial open failed ({SERIAL_PORT}): {err}. Run without serial.")
        return DummySerial()


def is_contour_on_border(contour, image_shape, tolerance=5):
    height, width = image_shape[:2]
    x, y, w, h = cv2.boundingRect(contour)
    return (
        x <= tolerance
        or y <= tolerance
        or x + w >= width - tolerance
        or y + h >= height - tolerance
    )


def detect_red_dot(img_raw_cv):
    gray = cv2.cvtColor(img_raw_cv, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(img_raw_cv, cv2.COLOR_BGR2HSV)

    # Method 1: bright spot detection
    max_val = int(np.max(gray))
    if max_val >= 150:
        bright_thresh = max(100, max_val - 30)
        _, bright_mask = cv2.threshold(gray, bright_thresh, 255, cv2.THRESH_BINARY)
        bright_count = cv2.countNonZero(bright_mask)
        if 3 <= bright_count <= 500:
            coords = np.where(bright_mask > 0)
            cy = int(np.mean(coords[0]))
            cx = int(np.mean(coords[1]))
            return cx, cy

    # Method 2: HSV color detection (kept same logic as your original)
    mask_red = cv2.inRange(hsv, np.array([100, 50, 60]), np.array([130, 255, 255]))
    red_count = cv2.countNonZero(mask_red)
    if 3 <= red_count <= 500:
        coords = np.where(mask_red > 0)
        cy = int(np.mean(coords[0]))
        cx = int(np.mean(coords[1]))
        return cx, cy

    return None


def extract_corner_points(contour):
    perimeter = cv2.arcLength(contour, True)
    epsilon = 0.02 * perimeter
    approx = cv2.approxPolyDP(contour, epsilon, True)
    return approx.reshape(-1, 2)


def main():
    serial_dev = open_serial()
    vofa_rx = VofaReceiver()
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    if not cap.isOpened():
        print("Camera open failed.")
        return

    current_corner_index = 0
    corner_points = []
    visited_corners = []
    tracking_mode = "DETECT_CONTOUR"
    calib_request_pending = False

    while True:
        ok, img_raw = cap.read()
        if not ok:
            print("Camera frame read failed.")
            break

        # Poll STM32 commands
        rx_len = getattr(serial_dev, "in_waiting", 0)
        if rx_len:
            data = serial_dev.read(rx_len)
            for cmd, value in vofa_rx.feed(data):
                if cmd == VOFA_CMD_CALIB_REQ:
                    calib_request_pending = True
                    print(f"Calib request from STM32: type={value}")

        # Handle calibration request
        if calib_request_pending:
            calib_request_pending = False
            pos = detect_red_dot(img_raw)
            if pos is not None:
                px_x, px_y = float(pos[0]), float(pos[1])
            else:
                px_x, px_y = -1.0, -1.0
            serial_dev.write(create_vofa_frame(VOFA_CMD_CALIB_PX_X, px_x))
            serial_dev.write(create_vofa_frame(VOFA_CMD_CALIB_PX_Y, px_y))
            print(f"Calib reply: px=({px_x}, {px_y})")

        if tracking_mode == "DETECT_CONTOUR":
            img_gray = cv2.cvtColor(img_raw, cv2.COLOR_BGR2GRAY)
            img_blur = cv2.GaussianBlur(img_gray, (7, 7), 0)
            img_dilated = cv2.dilate(img_blur, kernel, iterations=1)
            img_eroded = cv2.erode(img_dilated, kernel, iterations=1)
            edged = cv2.Canny(img_eroded, 100, 200)
            contours, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

            filtered_contours = [
                contour
                for contour in contours
                if cv2.contourArea(contour) > 1000
                and not is_contour_on_border(contour, img_raw.shape, tolerance=5)
            ]

            if filtered_contours:
                areas = [cv2.contourArea(contour) for contour in filtered_contours]
                max_contour = filtered_contours[int(np.argmax(areas))]
                if cv2.contourArea(max_contour, oriented=True) > 0:
                    max_contour = max_contour[::-1]

                cv2.drawContours(img_raw, [max_contour], -1, (0, 255, 0), 2)
                corner_points = extract_corner_points(max_contour)
                visited_corners = [False] * len(corner_points)
                print(f"Detected {len(corner_points)} corner points: {corner_points}")

                for corner in corner_points:
                    x, y = int(corner[0]), int(corner[1])
                    cv2.circle(img_raw, (x, y), 5, (255, 0, 0), -1)
                    print(create_vofa_frame(0xC1, y))
                    cv2.imshow(WINDOW_NAME, img_raw)
                    cv2.waitKey(1)
                    time.sleep(0.01)

                tracking_mode = "TRACK_RED_DOT"
                current_corner_index = 0
                print("Switching to TRACK_RED_DOT mode")
            else:
                print(create_vofa_frame(0xCC, 0))

        elif tracking_mode == "TRACK_RED_DOT":
            for i in range(len(corner_points)):
                if not visited_corners[i]:
                    x, y = int(corner_points[i][0]), int(corner_points[i][1])
                    radius = 8 if i == current_corner_index else 4
                    cv2.circle(img_raw, (x, y), radius, (255, 0, 0), -1)

            red_dot_pos = detect_red_dot(img_raw)
            if red_dot_pos is not None and len(corner_points) > 0:
                red_x, red_y = red_dot_pos

                is_false_detection = False
                for corner in corner_points:
                    corner_x, corner_y = int(corner[0]), int(corner[1])
                    if abs(red_x - corner_x) < 5 and abs(red_y - corner_y) < 5:
                        is_false_detection = True
                        break

                if is_false_detection:
                    serial_dev.write(create_vofa_frame(0xD0, 1000.0))
                    serial_dev.write(create_vofa_frame(0xD1, 1000.0))
                    print("False dot detection, sending default vector (1000, 1000)")
                else:
                    cv2.circle(img_raw, (red_x, red_y), 10, (0, 255, 0), 2)
                    cv2.circle(img_raw, (red_x, red_y), 3, (0, 0, 255), -1)

                    target_corner = corner_points[current_corner_index]
                    target_x, target_y = target_corner[0], target_corner[1]
                    cv2.line(
                        img_raw,
                        (red_x, red_y),
                        (int(target_x), int(target_y)),
                        (0, 255, 255),
                        2,
                    )

                    vector_x = target_x - red_x
                    vector_y = target_y - red_y
                    serial_dev.write(create_vofa_frame(0xD0, float(vector_x)))
                    serial_dev.write(create_vofa_frame(0xD1, float(vector_y)))
                    print(f"Vector to corner {current_corner_index}: ({vector_x}, {vector_y})")

                    distance = (vector_x ** 2 + vector_y ** 2) ** 0.5
                    if distance < RED_DOT_REACHED_THRESHOLD:
                        visited_corners[current_corner_index] = True
                        next_corner_found = False
                        for i in range(current_corner_index + 1, len(corner_points)):
                            if not visited_corners[i]:
                                current_corner_index = i
                                next_corner_found = True
                                break
                        if not next_corner_found:
                            tracking_mode = "DETECT_CONTOUR"
                            current_corner_index = 0
                            corner_points = []
                            visited_corners = []
            else:
                serial_dev.write(create_vofa_frame(0xD0, 1000.0))
                serial_dev.write(create_vofa_frame(0xD1, 1000.0))
                print("No dot detected, sending default vector (1000, 1000)")

        cv2.imshow(WINDOW_NAME, img_raw)
        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord("q"):
            break

    cap.release()
    serial_dev.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
