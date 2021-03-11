if max_intersection <= self.adequate_dist_to_correspont:
    # update the centroid
    self.centroids[centroid][0] = pts_3D[max_intersection_arg]
    self.centroids[centroid][2] = pts[max_intersection_arg]
    if len(self.centroids[centroid][1]) <= self.length_of_blink_queue:
        self.centroids[centroid][1] += '1'
    else:
        self.centroids[centroid][1] = '1'
    blinked_centroids.append(centroid) 
elif max_intersection >= 1.5*self.adequate_dist_to_correspont:
    # it's a new centroid
    self.centroids[len(self.centroids)] = [pts_3D[max_intersection_arg], '1', pts[max_intersection_arg], bounding_boxes[max_intersection_arg]]
    blinked_centroids.append(len(self.centroids)-1) 
pts_3D.remove(pts_3D[max_intersection_arg])
pts.remove(pts[max_intersection_arg]) 