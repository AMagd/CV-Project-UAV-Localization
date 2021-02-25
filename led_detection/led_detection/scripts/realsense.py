import time 
import pyrealsense2 as rs
import numpy as np
import cv2
import json
import time 

class RealSense():
    def __init__(self, json_set = 'realsense_settings.json'):
        '''
        json_set - name of json file with realsense settings
        '''

        # load settings from json
        jsonObj = json.load(open(json_set))
        json_string= str(jsonObj).replace("'", '\"')

    # Configure streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        freq= int(jsonObj['stream-fps'])

        print("W: ", int(jsonObj['stream-width']))
        print("H: ", int(jsonObj['stream-height']))
        print("FPS: ", freq)
        config.enable_stream(rs.stream.depth, int(jsonObj['stream-width']), int(jsonObj['stream-height']), rs.format.z16, freq)

        config.enable_stream(rs.stream.color, int(jsonObj['stream-width']), int(jsonObj['stream-height']), rs.format.bgr8, freq)
        config.enable_stream(rs.stream.infrared, 1, int(jsonObj['stream-width']), int(jsonObj['stream-height']), rs.format.y8, freq)
        config.enable_stream(rs.stream.infrared, 2, int(jsonObj['stream-width']), int(jsonObj['stream-height']), rs.format.y8, freq)

        cfg = self.pipeline.start(config)
        dev = cfg.get_device()
        advnc_mode = rs.rs400_advanced_mode(dev)
        advnc_mode.load_json(json_string)

        align_to = rs.stream.infrared
        self.align = rs.align(align_to)

        frames = self.pipeline.wait_for_frames() # get first frames

        # Align the depth frame to infrared1 frame to get intrinsics
        frames = self.align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        ir1_frame = frames.get_infrared_frame(1) 
        ir2_frame = frames.get_infrared_frame(2) 
        self.depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        infrared_intrin_1 = ir1_frame.profile.as_video_stream_profile().intrinsics
        infrared_intrin_2 = ir2_frame.profile.as_video_stream_profile().intrinsics


        # Set up the detector with default parameters for detecting leds
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()

        # Change thresholds
        params.minThreshold = 0
        params.maxThreshold = 255
        # Filter by Area.
        params.filterByArea = False
        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.1
        # Filter by Convexity
        params.filterByConvexity = True
        params.minConvexity = 0.87
        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        self.detector = cv2.SimpleBlobDetector_create(params)

        self.fps = FPS()
        self.centroids = {}
        self.adequate_dist_to_correspont = 10 #in pixels (correspond centroids while moving and getting new frames)
        self.length_of_blink_queue = 20 # the length or array we put blinks in (elements are 0 or 1)

    def update(self, ):
        # Wait for a frames
        frames = self.pipeline.wait_for_frames()
        # Align the depth frame to infrared1 frame
        frames = self.align.process(frames)

        depth_frame = frames.get_depth_frame() # depth
        color_frame = frames.get_color_frame() # RGB
        ir1_frame = frames.get_infrared_frame(1) # Left IR Camera
        # ir2_frame = frames.get_infrared_frame(2) # Right IR camera
        if not depth_frame or not ir1_frame or not color_frame:
            wait_flag = 0 # if frame isn't gotten
        else: 
            wait_flag = 1

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            ir1_image = np.asanyarray(ir1_frame.get_data())
            # ir2_image = np.asanyarray(ir2_frame.get_data())

            ret, thresh1 = cv2.threshold(ir1_image,100,255,cv2.THRESH_BINARY_INV) # isolate leds from the background


            keypoints = self.detector.detect(thresh1) # detect all the leds
            if keypoints:
                pts = list(map(lambda x: (int(keypoints[x].pt[0]), int(keypoints[x].pt[1])), range(0,len(keypoints)))) # get positions for each centroid
                for pt in pts:
                    if not self.centroids:
                        self.centroids[0] = [pt, '1'] # it didn't exist but it blinked (new centroid)
                    else:
                        distances = list(map(lambda x: np.sqrt((self.centroids[x][0][0]-pt[0])**2+(self.centroids[x][0][1]-pt[1])**2), self.centroids)) # calculate distances between each centroid and current pt to try to find the correspondence

                        if np.min(distances) <= self.adequate_dist_to_correspont:
                            # update the centroid
                            self.centroids[np.argmin(distances)][0] = pt
                            if len(self.centroids[np.argmin(distances)][1]) <= self.length_of_blink_queue:
                                self.centroids[np.argmin(distances)][1] += '1'
                            else:
                                self.centroids[np.argmin(distances)][1] = '1'
                        else:
                            # it's a new centroid
                            self.centroids[len(self.centroids)] = [pt, '1'] 

            else:
                # no blinks at all in the frame. But in each of centroids zeros
                for centroid in self.centroids:
                    if len(self.centroids[centroid][1]) <= self.length_of_blink_queue:
                        self.centroids[centroid][1] += '0'
                    else:
                        self.centroids[centroid][1] = '0'

            # look through the centroids' queues in order to find needed binary codes
            for centroid in self.centroids:
                if '11111' in self.centroids[centroid][1]:
                    cv2.putText(ir1_image,'31',self.centroids[centroid][0], cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255)  )
                elif '1001000' in self.centroids[centroid][1]:
                    cv2.putText(ir1_image,'72',self.centroids[centroid][0], cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255) )

            images = ir1_image

            self.fps.update()
        return images, wait_flag
    
class FPS():
    def __init__(self):
        self.fps = 0
        self.i = 1
        self.start_time = time.time()
    def update(self):
        stop_time = time.time()
        fps_frame = 1/(stop_time-self.start_time)
        self.fps = (self.i-1)*self.fps/self.i + fps_frame/self.i
        self.i +=1
        self.start_time = stop_time
    def get(self):
        return round(self.fps)