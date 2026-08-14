import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

import numpy as np
import cv2

class VideoPublisherCompressed(Node):
    def __init__(self):
        super().__init__('video_publisher_compressed')
        # Publicador en topic compressed
        self.pub = self.create_publisher(CompressedImage, '/camera/image/compressed', 10)

        # Inicializa GStreamer
        Gst.init(None)

        # Pipeline GStreamer usando CPU (avdec_h264)
        pipeline_str = (
            "udpsrc port=5601 caps=\"application/x-rtp, media=video, "
            "clock-rate=90000, encoding-name=H264, payload=96\" ! "
            "rtph264depay ! avdec_h264 ! videoconvert ! videoscale ! "
            "video/x-raw,width=640,height=480,format=BGR ! "
            "appsink name=sink max-buffers=1 drop=true"
        )

        self.pipeline = Gst.parse_launch(pipeline_str)
        self.appsink = self.pipeline.get_by_name("sink")
        self.appsink.set_property("emit-signals", True)
        self.appsink.connect("new-sample", self.on_new_sample)

        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info(
            "Pipeline GStreamer iniciado y publicando en /camera/image/compressed usando CPU"
        )

    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK

        buf = sample.get_buffer()
        caps = sample.get_caps()
        width = caps.get_structure(0).get_value('width')
        height = caps.get_structure(0).get_value('height')

        success, map_info = buf.map(Gst.MapFlags.READ)
        if not success:
            return Gst.FlowReturn.OK

        # Convertimos buffer a numpy array
        frame = np.frombuffer(map_info.data, dtype=np.uint8)
        frame = frame.reshape((height, width, 3))
        buf.unmap(map_info)

        # Creamos mensaje CompressedImage
        msg = CompressedImage()
        msg.format = 'jpeg'
        msg.data = cv2.imencode('.jpg', frame)[1].tobytes()
        self.pub.publish(msg)

        return Gst.FlowReturn.OK

def main(args=None):
    rclpy.init(args=args)
    node = VideoPublisherCompressed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
