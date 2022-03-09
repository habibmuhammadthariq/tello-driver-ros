#!/usr/bin/env python2
import traceback
import av
import cv2
import numpy
from video_stream_modul import StandaloneVideoStream
import rospy
from sensor_msgs.msg import CompressedImage


stream = StandaloneVideoStream()


def callback(msg):
    #rospy.loginfo('frame: %d bytes' % len(msg.data))
    stream.add_frame(msg.data)


def main():
    rospy.init_node('h264_listener')
    rospy.Subscriber("/tello/image_raw/h264", CompressedImage, callback)
    container = av.open(stream)
    rospy.loginfo('main: opened')
    for frame in container.decode(video=0):
        image = cv2.cvtColor(numpy.array(
            frame.to_image()), cv2.COLOR_RGB2BGR)
        cv2.imshow('Frame', image)
        cv2.waitKey(1)


if __name__ == '__main__':
    try:
        main()
    except BaseException:
        traceback.print_exc()
    finally:
        stream.close()
        cv2.destroyAllWindows()
