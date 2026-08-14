
import sys

import cv2

pipeline = (
    "udpsrc address=192.168.2.1 port=5600 "
    'caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264,payload=96" '
    "! rtpjitterbuffer "
    "! rtph264depay "
    "! h264parse "
    "! avdec_h264 "
    "! videoconvert "
    "! appsink sync=false drop=true max-buffers=1"
)

cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)

print("Después", flush=True)
print("isOpened =", cap.isOpened(), flush=True)

while True:
    ret, frame = cap.read()
    print("ret =", ret, flush=True)

    if ret:
        cv2.imshow("BlueROV", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()
print("Finalizado.", flush=True)
