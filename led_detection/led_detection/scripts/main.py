from realsense import RealSense
import cv2
rs = RealSense()

try:
    while True:
        images, wait_flag = rs.update()

        if wait_flag:
            # Show images
            cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense', images)
            cv2.setWindowTitle('RealSense', 'RealSense FPS:' + str(rs.fps.get()))
            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break

        flag_first = 0

finally:
    # Stop streaming
    rs.pipeline.stop()