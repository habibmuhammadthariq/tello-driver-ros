import traceback
import av
import cv2
import rospy
from video_stream_modul import StandaloneVideoStream
import numpy as np

stream = StandaloneVideoStream()

def callback(msg):
    stream.add_frame(msg.data)

def main():
    container = av.open(stream)

    for frame in container.decode(video=0):
        image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)

        cv2.imshow('Image', image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    rospy.init_node('to_grey');
    rospy.Subscriber("/tello/image_raw/h264", CompressedImage, callback)

    try:
        main()
    except BaseException:
        traceback.print_exec()
    finally:
        stream.close()