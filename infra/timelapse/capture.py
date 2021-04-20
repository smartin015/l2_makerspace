import cv2
import argparse
from datetime import datetime
import sys

def do_capture(args):
    print("Capturing", args.url)
    capture=cv2.VideoCapture(args.url)
    i = 0
    while True:
        if args.nframes is not None and i >= args.nframes:
            print("Done")
            break
        i += 1

        frame=capture.read()
        if frame is None:
            print('Camera not found')
            break
        else:
            if args.display:
                cv2.imshow("Display", frame[1])
            path = "%s_%s.%s" % (args.prefix, "{:%Y_%m_%d_%H_%M_%S_%f}".format(datetime.now()), args.fmt)
            print(path)
            cv2.imwrite(path, frame[1])
            if cv2.waitKey(22) & 0xFF == ord('q'):
                break

    capture.release()
    cv2.DestroyAllWindows()
    print("Exit")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="""Fetches MJPEG from web and saves to images""")
    parser.add_argument('--url', default="http://192.168.1.106:8081/", help="URL to grab")
    parser.add_argument('--nframes', default=None, help="Number of frames to grab")
    parser.add_argument('--prefix', default='out', help="Starting name of file")
    parser.add_argument('--fmt', default='png', help="File format")
    parser.add_argument('--display', default=False, type=bool, help="Display output in a window")
    args = parser.parse_args(sys.argv[1:])
    do_capture(args)
