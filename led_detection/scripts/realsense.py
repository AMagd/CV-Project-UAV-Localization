import time
import pyrealsense2 as rs
import numpy as np
import cv2
import json
import time
import imutils
import os
import math


class RealSense():
    def __init__(self, json_set='realsense_settings.json'):
        '''
        json_set - name of json file with realsense settings
        '''

        self.radius = 0.15  # length from red LEDs to the center of the charging station (in meters)
        self.finalPoint = np.array([0, 0, 0])  # final location in 3D
        self.nGroupsDetected = 0  # number of detected groups

        self.first_two_groups_counter = 0
        self.second_two_groups_counter = 0

        self.path_to_script = os.path.dirname(os.path.realpath(__file__))
        # load settings from json
        jsonObj = json.load(open(os.path.join(self.path_to_script, json_set)))
        json_string = str(jsonObj).replace("'", '\"')

        # Configure streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        freq = int(jsonObj['stream-fps'])

        print("W: ", int(jsonObj['stream-width']))
        print("H: ", int(jsonObj['stream-height']))
        print("FPS: ", freq)
        config.enable_stream(rs.stream.depth, int(jsonObj['stream-width']), int(jsonObj['stream-height']),
                             rs.format.z16, freq)

        config.enable_stream(rs.stream.color, int(jsonObj['stream-width']), int(jsonObj['stream-height']),
                             rs.format.bgr8, freq)
        config.enable_stream(rs.stream.infrared, 1, int(jsonObj['stream-width']), int(jsonObj['stream-height']), rs.format.y8, freq)
        # config.enable_stream(rs.stream.infrared, 2, int(jsonObj['stream-width']), int(jsonObj['stream-height']), rs.format.y8, freq)

        cfg = self.pipeline.start(config)
        dev = cfg.get_device()
        advnc_mode = rs.rs400_advanced_mode(dev)
        advnc_mode.load_json(json_string)

        align_to = rs.stream.color
        self.align = rs.align(align_to)

        frames = self.pipeline.wait_for_frames()  # get first frames

        # Align the depth frame to infrared1 frame to get intrinsics
        frames = self.align.process(frames)
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        depth_sensor = cfg.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        # ir1_frame = frames.get_infrared_frame(1)
        # ir2_frame = frames.get_infrared_frame(2)
        self.depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
        self.color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        # print(self.color_intrin)
        self.camera_matrix = np.array([[610.772, 0, 323.325],[0, 609.271, 242.082],[0, 0, 1]])
        self.dist = np.array([0, 0, 0, 0, 0], dtype = np.float32)

        phi = math.radians(135)
        theta = math.acos(0.01/0.05)
        gamma = phi - theta
        x_value = 0.05*math.cos(gamma)
        y_value = 0.05*math.sin(gamma)
        self.object1 = np.array([[0, 0, 0], [0.05, 0, 0], [0, 0.05, 0]], dtype=np.float64)
        self.object2 = np.array([[0, 0, 0], [x_value, y_value, 0], [-y_value, -x_value, 0]], dtype=np.float64)

        self.depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)
        self.axis = np.float32([[0.05,0,0], [0,0.05,0], [0,0,0.05]]).reshape(-1,3)

        # infrared_intrin_1 = ir1_frame.profile.as_video_stream_profile().intrinsics
        # infrared_intrin_2 = ir2_frame.profile.as_video_stream_profile().intrinsics

        self.fps = FPS()
        self.centroids = {}
        self.adequate_dist_to_correspont = 0.03  # in m (correspond centroids while moving and getting new frames)
        # self.length_of_blink_queue = 22  # the length or array we put blinks in (elements are 0 or 1)

        # # self.ledsIDs = list(np.random.randint(0,1023,10))
        # self.ledsIDs = [431, 41, 218, 768, 93, 629, 925, 212, 421, 155]
        # self.ledsIDs = list(map(lambda x: np.binary_repr(x, 10), self.ledsIDs))

        triangle_side = 0.06  # the length of the triangle side (m)
        self.distance_to_group = np.sqrt(2 * triangle_side ** 2)  # max distance between diods in the group

    def draw(self, img, corners, imgpts):
        corner = tuple(corners[0].ravel())
        img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
        img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
        return img

    def update(self, hBar):
        self.first_two_groups_counter = 0
        self.second_two_groups_counter = 0

        self.finalPoint = np.array([0, 0, 0])  # init final point for landing (we take average)
        self.nGroupsDetected = 0  # # of groups detected
        # Wait for a frames
        frames = self.pipeline.wait_for_frames()
        # Align the depth frame to infrared1 frame
        frames = self.align.process(frames)

        depth_frame = frames.get_depth_frame()  # depth
        color_frame = frames.get_color_frame()  # RGB
        ir1_frame = frames.get_infrared_frame(1) # Left IR Camera
        # ir2_frame = frames.get_infrared_frame(2) # Right IR camera
        if not depth_frame or not color_frame:
            wait_flag = 0  # if frame isn't gotten
        else:
            wait_flag = 1

            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            ir1_image = np.asanyarray(ir1_frame.get_data())
            # ir2_image = np.asanyarray(ir2_frame.get_data())

            labeled_image = self.red_white_leds_detect(depth_frame, color_image, hBar) # the method for getting landing point
            # self.blinking_leds_detect(depth_frame, color_image) # the method for identifying blinking patterns(doesn't work now)

            try: # CV part (can be commented out)
                self.finalPoint = self.finalPoint / self.nGroupsDetected
                finalPointPixel = rs.rs2_project_point_to_pixel(self.depth_intrin, self.finalPoint)
                finalPointPixel = [int(x) for x in finalPointPixel]
                final_x = finalPointPixel[0]
                final_y = finalPointPixel[1]
                cv2.circle(labeled_image, (final_x, final_y), 5, (0, 0, 255), -1)
                cv2.putText(labeled_image, "Landing Point", (final_x-20, final_y+20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            except ValueError:
                pass

            images = [labeled_image, ir1_image]

            self.fps.update()
        return images, wait_flag, self.finalPoint

    def ledsOnly(self, bgr_color, hsv_image, color_image, labeledImage, depth_frame, hBar=20):
        if bgr_color == [0, 0, 255]:
            color_label = "Red LED"
            y_detection_label = 40
        elif bgr_color == [255, 0, 0]:
            color_label = "blue LED"
            y_detection_label = 20
        else:
            color_label = "Unknown Color"
            y_detection_label = 20

        ######################### 1. Original Image Channels ##########################
        color = np.uint8([[bgr_color]])
        hsv_color = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        lower_color = np.array([hsv_color[0][0][0] - hBar, 50, 50])
        upper_color = np.array([hsv_color[0][0][0] + hBar, 255, 255])
        color_mask = cv2.inRange(hsv_image, lower_color, upper_color)
        color_LEDs = cv2.bitwise_and(color_image, color_image,
                                     mask=color_mask)  # colored image containing specified LEDs only

        ######################### 2. Reducing Image Channels ##########################

        gray_image = cv2.cvtColor(color_LEDs, cv2.COLOR_BGR2GRAY)  # convert image to gray scale
        _, bw_image = cv2.threshold(gray_image, 0, 255, cv2.THRESH_BINARY)  # convert image to binary color

        kernel = np.ones((5, 5), np.uint8)
        closing = cv2.morphologyEx(bw_image, cv2.MORPH_CLOSE, kernel,
                                   iterations=5)  # bw image containing specified LEDs only

        ######################### 3. Contouring Connected Components ##########################

        connectivity = 8
        n_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(closing, connectivity, cv2.CV_32S)
        # labeledImage = color_image.copy() # labeledImage will be used to contain a colored image with labels on the leds (can be returend by the function for testing purposes)

        leds_center = []
        leds_center3D = []
        for (i, label) in enumerate(range(1, n_labels)):
            x = stats[label, cv2.CC_STAT_LEFT]
            y = stats[label, cv2.CC_STAT_TOP]
            w = stats[label, cv2.CC_STAT_WIDTH]
            h = stats[label, cv2.CC_STAT_HEIGHT]
            cent_x, cent_y = int(centroids[label, 0]), int(centroids[label, 1])
            cv2.circle(labeledImage, (cent_x, cent_y), 3, (255, 0, 0), -1)
            cv2.rectangle(labeledImage, (x, y), (x + w, y + h), (255, 0, 0), 2)
            led_label = f"{color_label} {i + 1}"
            cv2.putText(labeledImage, led_label, (x + w + 10, y + (h // 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                        (255, 255, 255), 1)

            # rs.rs2_deproject_pixel_to_point()
            # dist = depth_frame.get_distance(cent_x, cent_y)
            leds_center.append([cent_x, cent_y])

            leds_center3D = []
            for led_center in leds_center:
                cX = led_center[0]
                cY = led_center[1]
                # getting 3D position
                depth = depth_frame.get_distance(cX, cY)
                # if depth == 0:
                # continue
                # pts.append((cX, cY))
                pt_3D = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [cX, cY], depth)
                # pt_3D = [int(x*1000) for x in pt_3D]
                leds_center3D.append(pt_3D)

        cv2.putText(labeledImage, f"Detected {n_labels - 1} {color_label}s", (20, y_detection_label),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        # print(f"labels are: {labels}")
        return leds_center3D

    def group_leds_by_dist(self, red_leds, blue_leds):
        '''
        groupds red and blue leds in groups.
        returns:
        grouped_leds (dict).
        grouped_leds[i] = 'red_led': pos, 'blue_leds': [pos, pos, ...]
        '''
        grouped_leds = {}
        for red_led in red_leds:
            distances = list(map(lambda x: np.linalg.norm(np.array(red_led) - x), blue_leds))
            distances_fited = list(filter(lambda x: x <= self.distance_to_group, distances))
            dist_indeces = list(map(lambda x: distances.index(x), distances_fited))
            grouped_leds[red_leds.index(red_led)] = {'red_led': red_led,
                                                     'blue_leds': list(map(lambda x: blue_leds[x], dist_indeces))}
        return grouped_leds

    def draw_vectors(self, image, blue_pts, red_pts):
        if len(blue_pts) >= 2 and red_pts:
            # getting 3D coordinates of each led
            gPixels1 = rs.rs2_project_point_to_pixel(self.depth_intrin, blue_pts[0])
            gPixels2 = rs.rs2_project_point_to_pixel(self.depth_intrin, blue_pts[1])
            rPixel = rs.rs2_project_point_to_pixel(self.depth_intrin, red_pts)

            # getting 3D coordinates of the center between the two points
            gCenterPoint = np.array(np.array(blue_pts[1])) + (np.array(blue_pts[0]) - np.array(blue_pts[1])) / 2
            gCenterPointPixels = rs.rs2_project_point_to_pixel(self.depth_intrin, gCenterPoint)

            # first getting the vector that connects the red LED with the middle point between the blue LEDs, then scale it to have the same size as self.radius
            finalVec = gCenterPoint - np.array(red_pts)
            final_vec_length = np.linalg.norm(finalVec)
            scaling_factor = self.radius/final_vec_length
            finalVec = finalVec*scaling_factor
            finalVec = red_pts - finalVec
            finalVecPixels = rs.rs2_project_point_to_pixel(self.depth_intrin, finalVec)

            distinct_feature = round(final_vec_length*100,2) # distance from the middle point between blue leds to the red led (should be more than 2.5 for two groups and less than 2.5 for the other two groups)

            # image_cord = np.array([rPixel, gPixels1, gPixels2], dtype=np.float64)
            #
            # if distinct_feature > 2.5:
            #     world_cord = self.object1
            #     if self.first_two_groups_counter == 0:
            #         world_cord = -self.object1
            #         image_cord = np.array([rPixel, gPixels2, gPixels1], dtype=np.float64)
            #
            #     self.first_two_groups_counter = self.first_two_groups_counter + 1
            # else:
            #     world_cord = self.object2
            #     if self.second_two_groups_counter == 0:
            #         world_cord = -self.object2
            #         image_cord = np.array([rPixel, gPixels2, gPixels1], dtype=np.float64)
            #
            #     self.second_two_groups_counter = self.second_two_groups_counter + 1
            #
            # ret, rvecs, tvecs = cv2.solvePnP(world_cord, image_cord,
            #                                  self.camera_matrix, self.dist, np.zeros((3, 1)), np.zeros((3, 1)),
            #                                  flags=cv2.SOLVEPNP_ITERATIVE,
            #                                  useExtrinsicGuess=True)
            # print(rvecs)


            self.nGroupsDetected = self.nGroupsDetected + 1
            self.finalPoint = (self.finalPoint + finalVec)

            try:
                # imgpts, jac = cv2.projectPoints(self.axis, rvecs, tvecs, self.camera_matrix, self.dist)
                # image = self.draw(image, np.array([rPixel, gPixels1, gPixels2], dtype=np.int), imgpts)
                gPixels1 = [int(x) for x in gPixels1]
                gPixels2 = [int(x) for x in gPixels2]
                gCenterPointPixels = [int(x) for x in gCenterPointPixels]
                rPixel = [int(x) for x in rPixel]
                finalVecPixels = [int(x) for x in finalVecPixels]
                cv2.line(image, tuple(gPixels1), tuple(gPixels2), (0, 255, 0), 2)
                cv2.arrowedLine(image, tuple(rPixel), tuple(finalVecPixels), (255, 255, 255), 2)
                cv2.circle(image, tuple(gCenterPointPixels), 5, (0, 0, 255), -1)


            except ValueError:
                pass

    def red_white_leds_detect(self, depth_frame, color_image, hBar):
        labeledImage = color_image.copy()
        # get hsv representation from the colored image
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        blue_leds_centers = self.ledsOnly([255, 0, 0], hsv_image, color_image, labeledImage, depth_frame, hBar)
        red_leds_centers = self.ledsOnly([0, 0, 255], hsv_image, color_image, labeledImage, depth_frame, hBar)
        grouped_leds = self.group_leds_by_dist(red_leds_centers, blue_leds_centers)
        for group in grouped_leds:
            self.draw_vectors(labeledImage, grouped_leds[group]['blue_leds'], grouped_leds[group]['red_led'])
        return labeledImage

    def blinking_leds_detect(self, depth_frame, color_image):

        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        ret, thresh1 = cv2.threshold(gray, 50, 255, 0)
        thresh1 = cv2.erode(thresh1, np.ones((3, 3), np.uint8), iterations=1)

        cnts = cv2.findContours(thresh1.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        cv2.drawContours(color_image, cnts[1], -1, (0, 230, 255), 2)
        cnts = imutils.grab_contours(cnts)

        pts = []
        pts_3D = []
        bounding_boxes = []
        if cnts:
            # loop over the contours
            for c in cnts:
                # compute the center of the contour
                x, y, w, h = cv2.boundingRect(c)
                bounding_boxes.append([x, y, x + w, y + h])
                # M = cv2.moments(c)
                cX = int(x + w / 2)
                cY = int(y + h / 2)

                # depth_point = rs.rs2_deproject_pixel_to_point(self.depth_intrin, [cX, cY], self.depth_scale)
                # color_point = rs.rs2_transform_point_to_point(self.depth_to_color_extrin, depth_point)
                # color_pixel = rs.rs2_project_point_to_pixel(self.color_intrin, color_point)
                # print(depth_point, color_point, color_pixel,sep = ',', end = '\n')
                depth = depth_frame.get_distance(cX, cY)
                # if depth == 0:
                #     continue
                pts.append((cX, cY))
                pt_3D = rs.rs2_deproject_pixel_to_point(
                    self.depth_intrin, [cX, cY], depth)
                pts_3D.append([round(pt_3D[0], 3), round(pt_3D[1], 3), round(pt_3D[2], 3)])

        if self.centroids:
            blinked_centroids = []  # check which centroids were blinking
            if pts_3D:
                centroids = list(self.centroids)
                for centroid in centroids:
                    if pts_3D:
                        # intersections = list(map(lambda x: self.bb_intersection_over_union(self.centroids[centroid][3], x), bounding_boxes))
                        # # print(intersections)
                        # max_intersection = np.max(intersections)
                        # max_intersection_arg = np.argmax(intersections)
                        # print(max_intersection)
                        # if max_intersection:
                        #     # update the centroid
                        #     self.centroids[centroid][0] = pts_3D[max_intersection_arg]
                        #     self.centroids[centroid][2] = pts[max_intersection_arg]
                        #     if len(self.centroids[centroid][1]) <= self.length_of_blink_queue:
                        #         self.centroids[centroid][1] += '1'
                        #     else:
                        #         self.centroids[centroid][1] = '1'
                        #     blinked_centroids.append(centroid)
                        # else:
                        #     # it's a new centroid
                        #     blinked_centroids.append(len(self.centroids))
                        #     self.centroids[len(self.centroids)] = [pts_3D[max_intersection_arg], '1', pts[max_intersection_arg], bounding_boxes[max_intersection_arg]]
                        #     centroids = list(self.centroids)
                        # pts_3D.remove(pts_3D[max_intersection_arg])
                        # pts.remove(pts[max_intersection_arg])
                        # bounding_boxes.remove(bounding_boxes[max_intersection_arg])

                        distances = list(
                            map(lambda x: np.linalg.norm(np.array(self.centroids[centroid][0]) - np.array(x)),
                                pts_3D))  # calculate distances between each centroid and current pt to try to find the correspondence
                        min_dist = np.min(distances)
                        min_dist_arg = np.argmin(distances)
                        if min_dist <= self.adequate_dist_to_correspont:
                            # update the centroid
                            self.centroids[centroid][0] = pts_3D[min_dist_arg]
                            self.centroids[centroid][2] = pts[min_dist_arg]
                            if len(self.centroids[centroid][1]) <= self.length_of_blink_queue:
                                self.centroids[centroid][1] += '1'
                            else:
                                self.centroids[centroid][1] = '1'
                            blinked_centroids.append(centroid)
                        elif min_dist >= 1.5 * self.adequate_dist_to_correspont:
                            # it's a new centroid
                            self.centroids[len(self.centroids)] = [pts_3D[min_dist_arg], '1', pts[min_dist_arg],
                                                                   bounding_boxes[min_dist_arg]]
                            blinked_centroids.append(len(self.centroids) - 1)
                        pts_3D.remove(pts_3D[min_dist_arg])
                        pts.remove(pts[min_dist_arg])
                    else:
                        break
            for centroid in self.centroids:
                if centroid not in blinked_centroids:
                    if len(self.centroids[centroid][1]) <= self.length_of_blink_queue:
                        self.centroids[centroid][1] += '0'
                    else:
                        self.centroids[centroid][1] = '0'
            if len(self.centroids) >= 12:
                self.centroids = {}

        else:
            for i in range(len(pts)):
                self.centroids[len(self.centroids)] = [pts_3D[i], '1', pts[i], bounding_boxes[i]]  # it didn't

        # look through the centroids' queues in order to find needed binary codes
        for centroid in self.centroids:
            cv2.putText(color_image, str(centroid), self.centroids[centroid][2], cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                        (255, 255, 255))
            print(self.centroids[centroid][1])
            # print(indeces)

    def bb_intersection_over_union(self, boxA, boxB):
        # determine the (x, y)-coordinates of the intersection rectangle
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
        # compute the area of intersection rectangle
        interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1)
        # compute the area of both the prediction and ground-truth
        # rectangles
        boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1)
        boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1)
        # compute the intersection over union by taking the intersection
        # area and dividing it by the sum of prediction + ground-truth
        # areas - the interesection area
        iou = float(interArea) / float(boxAArea + boxBArea - interArea)
        # return the intersection over union value
        return iou


class FPS():
    def __init__(self):
        self.fps = 0
        self.i = 1
        self.start_time = time.time()

    def update(self):
        stop_time = time.time()
        fps_frame = 1 / (stop_time - self.start_time)
        self.fps = (self.i - 1) * self.fps / self.i + fps_frame / self.i
        self.i += 1
        self.start_time = stop_time

    def get(self):
        return round(self.fps)