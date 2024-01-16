import time
start_time = time.time()
import cv2
from PIL import Image, ImageDraw, ImageFont, ImageOps
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
import warnings
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
        self.font = ImageFont.truetype("vision/data/Arial.ttf",30)
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
        sorted_box = box[np.argsort(box[:, 0])]
        top_points = [tuple(sorted_box[0]),tuple(sorted_box[1])]
        c1.polygon(box,outline=(0,255,0),width=5)

        top_x, top_y = np.mean(top_points, axis=0).astype(int)
        center_x, center_y = np.mean(box,axis=0).astype(int)
        
        rel_2d_points = list(sorted_box.copy())
        rel_2d_points.append([top_x,top_y])
        rel_2d_points.append([center_x,center_y])

        r = 5
        for i, (x,y) in enumerate(rel_2d_points[:5]):
            x = int(np.clip(x, 0, limits[0]-1))
            y = int(np.clip(y, 0, limits[1]-1))
            rel_2d_points[i] = [x,y]
            c1.ellipse([(x-r,y-r),(x+r,y+r)],fill=(0,0,255))
            
        return rel_2d_points

    def get_camera_3d_points(self,depth_frame, rel_2d_points, color_intrin, limits):
        rel_3d_points = []
        temp_2d_points = []
        for x_2d,y_2d in rel_2d_points:
            depth = depth_frame.get_distance(x_2d,y_2d)
            if depth == 0 or depth > 1.4:
                neighbours = self.get_valid_neighbors((x_2d,y_2d),(limits[0],limits[1]),2)
                distances = []
                for nx, ny in neighbours:
                    depth = depth_frame.get_distance(nx, ny)
                    if depth != 0 and depth < 1.4:
                        distances.append(depth)
                
                if distances:
                    depth = np.median(distances)
                else:
                    depth = np.nan
                    get_logger(__name__).log(logging.WARNING,
                                             "No depth discovered")
            temp_2d_points.append([x_2d,y_2d,depth])
        
        temp_2d_points = np.array(temp_2d_points)
        for x_2d, y_2d, depth in temp_2d_points:
            if np.isnan(depth):
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore", category=RuntimeWarning)
                    depth = np.nanmean(temp_2d_points[:,2])
                    if depth == np.nan:
                        break
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
            c1.text((x,y),f"{point_3d}")

    def calculate_rotational_angles(self, coords):
        def get_normal_vector(coords):
            p1, p2, p3 = coords
            d12 = np.array(p2) - np.array(p1)
            d13 = np.array(p3) - np.array(p1)
            normal_vec = np.cross(d12, d13)
            return normal_vec
        
        def find_rotation_between_planes(normal_vector1, normal_vector2):
            # Normalize the normal vectors
            normal_vector1 = normal_vector1 / np.linalg.norm(normal_vector1)
            normal_vector2 = normal_vector2 / np.linalg.norm(normal_vector2)

            # Calculate the rotation axis
            rotation_axis = np.cross(normal_vector1, normal_vector2)

            # Calculate the rotation angle
            dot_product = np.dot(normal_vector1, normal_vector2)
            rotation_angle = np.arccos(dot_product)

            # Create a rotation matrix or quaternion
            rotation_matrix = rotation_matrix_from_axis_angle(rotation_axis, rotation_angle)
            
            return rotation_matrix

        def rotation_matrix_from_axis_angle(axis, angle):
            # Normalize the axis
            axis = axis / np.linalg.norm(axis)

            # Calculate the rotation matrix
            rotation_matrix = np.array([
                [np.cos(angle) + axis[0]**2 * (1 - np.cos(angle)), axis[0] * axis[1] * (1 - np.cos(angle)) - axis[2] * np.sin(angle), axis[0] * axis[2] * (1 - np.cos(angle)) + axis[1] * np.sin(angle)],
                [axis[1] * axis[0] * (1 - np.cos(angle)) + axis[2] * np.sin(angle), np.cos(angle) + axis[1]**2 * (1 - np.cos(angle)), axis[1] * axis[2] * (1 - np.cos(angle)) - axis[0] * np.sin(angle)],
                [axis[2] * axis[0] * (1 - np.cos(angle)) - axis[1] * np.sin(angle), axis[2] * axis[1] * (1 - np.cos(angle)) + axis[0] * np.sin(angle), np.cos(angle) + axis[2]**2 * (1 - np.cos(angle))]
            ])
            return rotation_matrix

        base_angles = np.array([1.209,-1.209,1.209])

        a_1 = np.mean([coords[0],coords[2]],axis=0)
        a_2 = np.mean([coords[1],coords[3]],axis=0)
        b_1 = np.mean([coords[0],coords[1]],axis=0)
        b_2 = np.mean([coords[2],coords[3]],axis=0)

        d_a = a_1 - a_2
        d_b = b_1 - b_2

        # Set up the system of equations
        A = np.array([[-d_a[0], d_b[0]],
                    [-d_a[1], d_b[1]],
                    [-d_a[2], d_b[2]]])
        b = a_2 - b_2

        # Solve the system of equations
        solution = np.linalg.lstsq(A, b, rcond=None)[0]

        # Extract the intersection point
        intersection_point = a_2 + d_a * solution[0]
        final_coords = [intersection_point,b_2,a_2]

        p1 = intersection_point
        p2 = [p1[0], p1[1], p1[2]+1]
        p3 = [p1[0]+1, p1[1], p1[2]]
        init_coords = np.array([p1, p2, p3])

        norm_init = get_normal_vector(init_coords)
        norm_final = get_normal_vector(final_coords)

        rotation_matrix = find_rotation_between_planes(norm_init, norm_final)
        Rx, Ry, Rz = R.from_matrix(rotation_matrix).as_rotvec(degrees=False)
        print("Rotation by (degrees)",np.round(np.rad2deg(Rx),2),np.round(np.rad2deg(Ry),2),np.round(np.rad2deg(Rz),2))
        Rx, Ry, Rz = base_angles + [Rx, Ry, Rz]
        return Rx, Ry, Rz

    def get_robot_coords(self, camera_coords):
        R = np.array([[0.98965,0.14059,-0.028841],
                    [-0.14345,0.97505,-0.16939],
                    [0.0043076,0.17177,0.98513]])
        t = np.array([0.18212,0.11633,0.38649])
        new_coords = np.dot(R,camera_coords) + t
        return new_coords
    
    def draw_class(self,coords, cls):
        txt=Image.new('L', (500,50))
        d = ImageDraw.Draw(txt)
        d.text(coords, cls,  font=self.font, fill=255)
        w=txt.rotate(90,  expand=1)

        self.data_image.paste( ImageOps.colorize(w, (0,0,0), (255,255,255)), (242,60),  w)
        #self.data_image.show("Class")
    
    def show_heighest_box(self, index_highest):
        coords_2d = np.array(self.coords_2d[index_highest])
        print("Coords for heighest box:",coords_2d[:4])
        self.color_draw.polygon(coords_2d[:4],(255,0,0),width=5)
        self.data_image.save("vision/distance_annot_2.jpg")

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
        get_logger(__name__).log(logging.DEBUG,("Robot_Coords:",coords))
        get_logger(__name__).log(logging.DEBUG,("Act.Hgt:",actual_height,"Apr.Hgt:",round(approx_height,3),"Apr.Diff:",round(approx_height_1-approx_height_2,3)))
        return actual_height
    
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
                self.data_image = Image.open("vision/input_color.jpg")#self.path+"/image0.jpg") #seg prediction results
                self.color_draw = ImageDraw.Draw(self.data_image)
                depth_image = np.asanyarray(depth_frame.get_data())
                min_depth = 850
                max_depth = 1700
                depth_image_clipped = np.clip(depth_image, min_depth, max_depth)
                normalized_depth = (depth_image_clipped - min_depth) / (max_depth - min_depth)
                depth_colormap = cv2.applyColorMap((normalized_depth * 255).astype(np.uint8), cv2.COLORMAP_JET)
                cv2.imwrite("vision/depth.jpg", depth_colormap)

                masks = results[0].masks
                self.coords_2d = []
                for (mask_raw,cls) in zip(masks,classes):  
                    rel_2d_points = self.get_2d_points(mask_raw, self.data_image.size, self.color_draw)
                    if not rel_2d_points:
                        get_logger(__name__).log(logging.DEBUG,
                                                 "Box has incorrect shape -> no crate")
                        continue
                    self.coords_2d.append(rel_2d_points) #used for heighest crate estimation
                    self.draw_class(rel_2d_points[5],cls)
                    rel_3d_points = self.get_camera_3d_points(depth_frame, rel_2d_points, self.color_intrin,self.data_image.size)
                    if not rel_3d_points:
                        continue
                    success, rot, trans = cv2.solvePnP(np.array(rel_3d_points).astype("float32"),np.array(rel_2d_points).astype("float32"),self.k,self.d)
                    self.show_3d_points([rel_3d_points[4]],rot, trans, self.k, self.d, self.color_draw)

                    def sort_robot_coords(coords):
                        top_points = np.sort(coords[:2],axis=0)[::-1]
                        bottom_points = np.sort(coords[2:],axis=0)[::-1]
                        return np.concatenate((top_points,bottom_points),axis=0)
                    
                    robot_coords = []
                    for coord in rel_3d_points[:4]:
                        rob_coord = self.get_robot_coords(coord)
                        robot_coords.append(rob_coord)
                    robot_coords = sort_robot_coords(robot_coords)
                    crate_height = self.get_crate_height(robot_coords)

                    x,y,z = self.get_robot_coords(rel_3d_points[4])
                    rx,ry,rz = [1.209, -1.209, 1.209] #self.calculate_rotational_angles(robot_coords) #TODO: not accurate enough
                    pickup_point = tuple(round(c,2) for c in (x,y,z,rx,ry,rz))
                    coords_3d.append({"coords":pickup_point,"class":cls,"height":crate_height})
                   
                    get_logger(__name__).log(logging.DEBUG,
                                             f"Calculated pickup point: {pickup_point}, crate_height: {crate_height}")
                    self.color_draw.text(rel_2d_points[4], f"{cls,pickup_point},{crate_height}", fill=(255,255,255))#, direction="ttb")
                    
                self.data_image.save("vision/distance_annot.jpg")
                files = glob.glob(os.path.join(self.path, '**/*.jpg'), recursive=True)
                for f in files:
                    os.remove(f)
                break
            return coords_3d
        except Exception as e:
            get_logger(__name__).log(logging.WARNING,
                                     f"Vision Exception {e}")
            return
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
        if not results:
            raise

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
            
            #highest_entry = max(val_results, key=get_coord_2)
            max_index, highest_entry = max(enumerate(val_results), key=lambda x: get_coord_2(x[1]))
            print(max_index)
            self.show_heighest_box(max_index)

            if highest_entry["class"] == "NoPickupCrate":
                raise NoPickUpCrateException("Highest Crate is NoPickupCrate")
            else:
                coords = list(highest_entry["coords"])
                def apply_vision_offset(coords):
                    def apply_z_offset(coords):
                        # Ensure that the input value is within the specified range
                        in_max = 0.3
                        in_min = -0.275
                        out_max = 0.005
                        out_min = -0.0075
                        value = max(min(coords[0], in_max), in_min)
                        # Perform the linear interpolation
                        z_offset = (value - in_min) / (in_max - in_min) * (out_max - out_min) + out_min
                        return z_offset
                        
                    coords[0] += 0.025
                    coords[1] += 0.0375
                    coords[2] += apply_z_offset(coords) - 0.12
                    return coords

                coords = apply_vision_offset(coords)
                return coords,highest_entry["height"]
        else:
            raise NoDetectedCratesException()