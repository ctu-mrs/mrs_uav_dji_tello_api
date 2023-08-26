#!/usr/bin/python3

import sys
import traceback
import tellopy
import av
import cv2
import numpy
import matplotlib.pyplot as plt

def main():

    drone = tellopy.Tello()

    try:
        drone.connect()
        drone.wait_for_connection(60.0)

        retry = 3
        container = None
        while container is None and 0 < retry:
            retry -= 1
            try:
                container = av.open(drone.get_video_stream())
            except av.AVError as ave:
                print(ave)
                print('retry...')

        plt.ion()
        plt.show()

        while True:
            for frame in container.decode(video=0):

                print("decoding image")
                image = cv2.cvtColor(numpy.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                print("showing image")
                plt.imshow(image)
                plt.draw()
                plt.pause(0.0001)

    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
    finally:
        drone.quit()

if __name__ == '__main__':
    main()
