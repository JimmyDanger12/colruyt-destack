import time
start_time = time.time()
import cv2
from PIL import Image, ImageDraw, ImageFont
import pyrealsense2 as rs
import numpy as np
import os
import glob
import math
from backend_logging import get_logger
import logging
import pandas as pd
from keras.backend import clear_session
from scipy.spatial.transform import Rotation as R
from vision.classification_model import ClassificationModel
from vision.segmentation_model import SegmentationModel

VISION_PATH = "vision/crops/monkey/predict"

class NoPickUpCrateException(Exception):
    pass

class NoDetectedCratesException(Exception):
    pass

class VisionClient():
    def __init__(self):
        self.pipeline = None
        self.crate_dims = None
        self.path = VISION_PATH
        self.sm = None
        self.cm = None
        self.k = None
        self.d = None
        self.rob = None
    
    def connect(self):
        self.sm = SegmentationModel()
        self.sm.load_model()
        get_logger(__name__).log(logging.DEBUG,
                        f"Segmentation model loaded")
        clear_session()
        self.cm = ClassificationModel()
        self.cm.load_model()
        get_logger(__name__).log(logging.DEBUG,
                            f"Classification model loaded")
        self.crate_dims = pd.read_csv("vision/data/crate_sizes.csv")
        
    def init_camera_streams(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.depth)
        config.enable_stream(rs.stream.color)

        # Start streaming
        self.pipeline.start(config)
        sensor = self.pipeline.get_active_profile().get_device().query_sensors()[1]

        # Set the exposure anytime during the operation
        sensor.set_option(rs.option.exposure, 500.000)
        #sensor.set_option(rs.option.laser_power, 180)
        align_to = rs.stream.color
        align = rs.align(align_to)
        filters = [rs.spatial_filter(),rs.temporal_filter()]
        get_logger(__name__).log(logging.DEBUG,
                                 f"Camera Initialization complete")
        return align, filters

    def get_frames(self, pipeline, align, filters):
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        for filter in filters:
            depth_frame = filter.process(depth_frame)
        depth_frame = depth_frame.as_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        self.color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
        self.k = np.array(((self.color_intrin.fx, 0, self.color_intrin.ppx),
                (0,self.color_intrin.fy, self.color_intrin.ppy),
                (0,0,1)))
        self.d = np.array(self.color_intrin.coeffs)
        return color_frame, depth_frame

    def get_valid_neighbors(self,coords,limits,size):
        neighbours = []
        for i in range(-size,size+1):
            for j in range(-size,size+1):
                xn, yn = coords[0]+i, coords[1]+j
                if (xn,yn) != coords:
                    if 0 <= xn < limits[0] and 0 <= yn < limits[1]:
                        neighbours.append((xn,yn))
        return neighbours

    def get_2d_points(self,mask_raw, limits, c1):
        polygon = mask_raw.xy[0]
        epsilon = 0.1 * cv2.arcLength(polygon, True)
        box = cv2.approxPolyDP(polygon, epsilon, True)
        if box.shape != (4,1,2):
            return
        box = box.reshape(4,2)
        #rect = cv2.minAreaRect(polygon)
        #box = cv2.boxPoints(rect)
        sorted_box = box[np.argsort(box[:, 0])]
        top_points = [tuple(sorted_box[0]),tuple(sorted_box[1])]
        c1.polygon(box,outline=(0,255,0),width=5)
        c1.line(top_points,fill=(255,255,0),width=5)

        top_x, top_y = np.mean(top_points, axis=0).astype(int)
        center_x, center_y = np.mean(box,axis=0).astype(int)
        
        rel_2d_points = list(sorted_box.copy())
        rel_2d_points.append([top_x,top_y])
        rel_2d_points.append([center_x,center_y])

        r = 5
        for i, (x,y) in enumerate(rel_2d_points):
            x = int(np.clip(x, 0, limits[0]-1))
            y = int(np.clip(y, 0, limits[1]-1))
            rel_2d_points[i] = [x,y]
            c1.ellipse([(x-r,y-r),(x+r,y+r)],fill=(0,0,255))
            
        return rel_2d_points

    def get_camera_3d_points(self,depth_frame, rel_2d_points, color_intrin, limits):
        rel_3d_points = []
        for x_2d,y_2d in rel_2d_points:
            depth = depth_frame.get_distance(x_2d,y_2d)
            if depth == 0:
                neighbours = self.get_valid_neighbors((x_2d,y_2d),(limits[0],limits[1]),1)
                distances = []
                for nx, ny in neighbours:
                    depth = depth_frame.get_distance(nx, ny)
                    if depth != 0:
                        distances.append(depth)
                
                if distances:
                    depth = np.median(distances)
                else:
                    depth = 0

            #right: x, down: y, forward: z
            result = rs.rs2_deproject_pixel_to_point(color_intrin, [x_2d, y_2d], depth)
            """
            Camera:     Robot:
            x: down     x: left
            y: left     y: back
            z: forward  z: up

            Robot = Camera:
            x -> y
            y -> -z
            z -> -x
            """
            x_cam,y_cam,z_cam = result
            x_rob = y_cam
            y_rob = -z_cam
            z_rob = -x_cam

            rel_3d_points.append([x_rob,y_rob,z_rob])
        return rel_3d_points

    def show_3d_points(self,points_3d, rot, trans, k, d, c1):
        for point_3d in points_3d:
            point_3d = np.array(point_3d,dtype=np.float64)
            point_2d, jacobian = cv2.projectPoints(point_3d, rot, trans, k, d)
            x,y = tuple(point_2d.flatten())
            r=5
            c1.ellipse([(x-r,y-r),(x+r,y+r)],fill=(0,255,255))
            #c1.text((x,y),f"{point_3d}")

    def calculate_rotational_angles(self,plane_coordinates):
        # Extracting the coordinates of the plane
        p1, p2, p3, p4 = plane_coordinates #(x,y,z)
        base_angles = (1.2,-1.2,1.2)

        d12 = np.array(p2) - np.array(p1)
        d13 = np.array(p3) - np.array(p1)
        dz = np.cross(d12, d13)

        X = d12 / np.linalg.norm(d12)
        Z = dz / np.linalg.norm(dz)
        Y = np.cross(X,Z)

        rot = np.column_stack((X,Y,Z))
        angles = R.from_matrix(rot).as_rotvec(degrees=False)
        corrected_angles = angles + base_angles

        return corrected_angles

    def get_robot_coords(self, camera_coords):
        R = np.array([[0.98965,0.14059,-0.028841],
                    [-0.14345,0.97505,-0.16939],
                    [0.0043076,0.17177,0.98513]])
        t = np.array([0.18212,0.11633,0.38649])
        new_coords = np.dot(R,camera_coords) + t
        return new_coords

    def get_pickup_locations(self):
        coords_3d = []
        align, filters = self.init_camera_streams()
        try:
            while True:
                color_frame, depth_frame = self.get_frames(self.pipeline, align, filters)
                if not depth_frame or not color_frame:
                    continue
                    
                color_image = np.asanyarray(color_frame.get_data())
                img_color = Image.fromarray(color_image)
                img_color.save("vision/input_color.jpg")
                get_logger(__name__).log(logging.DEBUG,
                                         f"Received images from camera")

                results = self.sm.predict(color_image,False,True,True)
                classes, conf_list = self.cm.predict(self.path+"/crops/Crate/")
                get_logger(__name__).log(logging.DEBUG,
                                         f"Received predictions from models")
                data_image = Image.open(self.path+"/image0.jpg") #seg prediction results
                color_draw = ImageDraw.Draw(data_image)
                depth_image = np.asanyarray(depth_frame.get_data())
                min_depth = 850
                max_depth = 1700
                depth_image_clipped = np.clip(depth_image, min_depth, max_depth)
                normalized_depth = (depth_image_clipped - min_depth) / (max_depth - min_depth)
                depth_colormap = cv2.applyColorMap((normalized_depth * 255).astype(np.uint8), cv2.COLORMAP_JET)
                cv2.imwrite("vision/depth.jpg", depth_colormap)

                masks = results[0].masks
                for (mask_raw,cls) in zip(masks,classes):  
                    rel_2d_points = self.get_2d_points(mask_raw, data_image.size, color_draw)
                    if not rel_2d_points:
                        get_logger(__name__).log(logging.DEBUG,
                                                 "Box has incorrect shape -> no crate")
                        continue

                    rel_3d_points = self.get_camera_3d_points(depth_frame, rel_2d_points, self.color_intrin,data_image.size)
                    success, rot, trans = cv2.solvePnP(np.array(rel_3d_points).astype("float32"),np.array(rel_2d_points).astype("float32"),self.k,self.d)
                    self.show_3d_points(rel_3d_points[:4],rot, trans, self.k, self.d, color_draw)

                    robot_coords = []
                    for coord in rel_3d_points[:4]:
                        rob_coord = self.get_robot_coords(coord)
                        robot_coords.append(rob_coord)
            
                    x,y,z = self.get_robot_coords(rel_3d_points[4])
                    rx,ry,rz = self.calculate_rotational_angles(robot_coords)
                    
                    def sort_robot_coords(coords):
                        top_points = np.sort(coords[:2],axis=0)[::-1]
                        bottom_points = np.sort(coords[2:],axis=0)[::-1]
                        return np.concatenate((top_points,bottom_points),axis=0)
                    robot_coords = sort_robot_coords(robot_coords)
                    
                    crate_height = self.get_crate_height(robot_coords)
                    coords_3d.append({"coords":(x,y,z,rx,ry,rz),"class":cls,"height":crate_height})
                    point = tuple(round(c,2) for c in (x,y,z,rx,ry,rz))
                    get_logger(__name__).log(logging.DEBUG,
                                             f"Calculated pickup point: {point}, crate_height: {point}")
                    color_draw.text(rel_2d_points[4], f"{cls,point},{crate_height}", fill=(255,255,255))
                    
                data_image.save("vision/distance_annot.jpg")
                files = glob.glob(os.path.join(self.path, '**/*.jpg'), recursive=True)
                for f in files:
                    os.remove(f)
                break
            return coords_3d
        except Exception as e:
            get_logger(__name__).log(logging.WARNING,
                                     f"Vision Exception {e}")
            pass
        finally:
            self.pipeline.stop()
    
    def get_valid_pickup_loc(self):
        """
        Goal:
        - Remove invalid results (limits & NoCrate)
        - Search for highest crate
        - If PickupCrate - return coords
        - If NoPickupCrate - return assistance message
        """
        results = self.get_pickup_locations()

        val_results = []
        for res in results:
            coords = res["coords"]
            cls = res["class"]
            if not -1.5 < coords[0] < 1.5 or not -1.6 < coords[1] < -0.6 or coords[2] > 1:
                get_logger(__name__).log(logging.DEBUG,"Coords not within limits!, Result ignored")
                continue
            elif cls == "NoCrate":
                get_logger(__name__).log(logging.DEBUG,
                                         "Ignored due to NoCrate class")
            else:
                val_results.append(res)
        
        if val_results:
            def get_coord_2(item):
                return item["coords"][2]
            highest_entry = max(val_results, key=get_coord_2)

            if highest_entry["class"] == "NoPickupCrate":
                raise NoPickUpCrateException("Highest Crate is NoPickupCrate")
            else:
                coords = list(highest_entry["coords"])
                coords[0] += 0.02
                coords[1] += 0.0475
                coords[2] += -0.135
                return coords,highest_entry["height"]
        else:
            raise NoDetectedCratesException()
    
    def get_crate_height(self,coords):
        def calc_3d_distance(a,b):
            x1,y1,z1 = a
            x2,y2,z2 = b
            distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
            return distance

        approx_height_1 = calc_3d_distance(coords[0][:3],coords[2][:3])
        approx_height_2 = calc_3d_distance(coords[1][:3],coords[3][:3])
        approx_height = np.mean([approx_height_1,approx_height_2],axis=0)

        actual_height = self.crate_dims.iloc[(self.crate_dims["height"]/1000 - approx_height).abs().argsort()[:1]]["height"].item()/1000
        print("Robot_Coords:",coords)
        print("Act.Hgt:",actual_height,"Apr.Hgt:",round(approx_height,3),"Apr.Diff:",round(approx_height_1-approx_height_2,3))
        return actual_height