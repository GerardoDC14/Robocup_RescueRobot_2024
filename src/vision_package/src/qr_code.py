from pyzbar import pyzbar
import cv2

class QRCodeDetector:
    def detect_and_decode(self, frame):
        """Detect and decode QR codes in the provided frame."""
        qr_codes = pyzbar.decode(frame)
        for qr in qr_codes:
            (x, y, w, h) = qr.rect
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            qr_data = qr.data.decode("utf-8")
            text = f"{qr_data} ({qr.type})"
            cv2.putText(frame, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        return frame
