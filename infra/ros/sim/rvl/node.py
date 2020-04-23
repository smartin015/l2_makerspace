from sensor_msgs.msg import Image
import rclpy
import rvl
import socket
import struct

class Server(rclpy.Node):
    PACK_FMT = "ii%dB"
    DEFAULT_UDP_DEST = ("127.0.0.1", 4242)

    def __init__(self):
        super().__init__('rvl_streamer')
        self.get_logger().info("Init")
        self.dest = self.DEFAULT_UDP_DEST # TODO make configurable/fetch from github
        self.sock = socket.socket(socket.AF_INIT, socket.SOCK_DGRAM)
        self.sub = self.create_subscription(Image, 'depth_raw', self.handle_image)

    def handle_image(self, img):
        rvl.Clear() # TODO don't clear all, instead do delta
        rvl.plain = img.data
        rvl.CompressRVL()
        packed = struct.pack(self.PACK_FMT % (len(rvl.encoded), 20, nvecs, *rvl.encoded))
        self.sock.sendto(packed, self.dest)
        self.get_logger().info(str(len(packed)
        
def main(args=None):
    rclpy.init(args=args)
    server = Server()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
