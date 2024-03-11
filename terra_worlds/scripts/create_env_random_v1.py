# Created by: Hunter Young
# Date: 11/19/17
#
# Script Description:
#     Script is designed to take various commandline arguments making a very simple
#     and user-friendly method to generate an array of (X,Y) coordinates to be used
#   as the centers of corn stalks for loading into ROS Gazebo. (Optional: additionally
#   include a randomized yaw angle or pitch angle to simulate lodging and model irregularity).
#
# Current Recommended Usage: (in terminal)
#     python generate_points.py -o name_of_output_data_file
#
# References:
#     XML file setup - https://stackoverflow.com/questions/3844360/best-way-to-generate-xml

import random
import numpy as np
import os
import shutil
import math
import argparse
import json
import matplotlib.pyplot as plt

from xml.dom import minidom


from trajectory_utils import *
from trajectory_utils2 import*


class EnvCreator:
    def __init__(self):
        self.stalk_models = []
        self.stalk_model_heights= []
        self.stalk_model_biodensities = []

        # Constant parameters
        self.num_variants = 0
        self.vert_num_rows = 7
        self.hori_num_rows = 3
        self.vert_row_len = 5.0  # (m)
        self.hori_row_len = 5.0
        self.radius = 0.05
        self.stalk_dist = 0.3048
        self.row_dist = 0.76
        self.min_row_len = 4.5  
        self.max_row_len = 5.0  
        self.max_row_len_diff = 0.3
        self.origin = [0, 0]
        self.gps_origin = [40.123456789,-81.123456789]
        self.stalk_min_emerg_prob = 0.75
        self.stalk_max_emerg_prob = 1.0
        self.stalk_min_height = 0.05
        self.stalk_max_height = 2.0
        self.stalk_height_prob = 1.0
        self.forward_plots = 1
        self.sideward_plots = 1
        self.min_slope = 2.5  # Slope range of the gap center
        self.max_slope = 6

        self.robot_init_margin = 0.25  # (m)
        self.robot_min_init_x = 3.8
        self.robot_max_init_x = 4.0
        self.robot_min_init_x_diff = 1.0
        self.robot_max_init_x_diff = 1.2
        self.robot_max_init_yaw = 0.174533 * 0.5  # 5 degrees in radians
        self.robot_min_init_yaw = -0.174533 * 0.5
        
        self.num_wp = 6  # Number of waypoints per route

        # Record stalk centers for visualization
        self.vert_centers = []
        self.hori_centers = []

        # Record the parametric equation for gap center for generating routes
        self.gap_center_eq = []

        # Unique parameters for each environment
        self.row_lens = np.zeros(7)
        self.gap_dist = 0.0
        self.variant_idx = 0

        self.routes_json = {}
        self.env_config = {}
    
    def create_stalk_variants(self):
        # for corn_var in range(2, 21):
        #     self.stalk_models.append(f"corn_variant_{corn_var}")
        for sorg_var in range(3, 9):
            self.stalk_models.append(f"sorghum_variant_{sorg_var}")
        for i in range(len(self.stalk_models)):
            self.stalk_model_heights.append(2.25)
            self.stalk_model_biodensities.append(1)
        self.num_variants = len(self.stalk_models)
    
    def define_gap_center(self):
        '''
        Define the paramatric equation for the tilted gap center
        '''
        gap_center = [self.vert_row_len + self.gap_dist / 2, 0]  # The end of the gap center intersects with world x-axis

        # Randomly select the slope of the gap center
        slope = random.uniform(self.min_slope, self.max_slope)  
        if random.uniform(0, 1) > 0.5:
            slope = -slope

        intersect = gap_center[1] - slope * gap_center[0]  # Determine the intersect of the line
        return (slope, intersect)

    @staticmethod
    def generate_random_coordinate(radius):
        # Return container
        output = []

        # Range of the choose-able angles
        range_angle = (0, 360)

        # Generate a random radius and angle sample
        r = random.random()*radius
        theta = random.randrange(*range_angle)

        # Calculate X-Y coordinates from the randomized radius and angle
        x = r * math.cos(math.radians(theta))
        y = r * math.sin(math.radians(theta))

        # Store Variable for output
        output.append(x)
        output.append(y)

        return output

    def generate_mean_centers(self, vert_rows, vert_cols, vert_dRow, vert_dCol, vert_origin,
                                    hori_rows, hori_cols, hori_dRow, hori_dCol, hori_origin,
                                    gap_center_eq):
        gap_slope = gap_center_eq[0]
        gap_intersect = gap_center_eq[1]

        # First generate stalks for vertical rows
        # Define Constants
        origin_x = vert_origin[0]
        origin_y = vert_origin[1]
        vert_centers = []  # Storage containers

        # For each row determine each stalks information
        for row in range(0, vert_rows):
            # For each stalk determine respective X-Y coordinates
            y_pos = origin_x + vert_dRow * row
            max_x_pos = (y_pos - gap_intersect) / gap_slope - self.gap_dist / 2 - random.uniform(0, self.max_row_len_diff)  # Create length diff for each row
            min_x_pos = 0
            x_pos = max_x_pos
            stalk = 0
            while x_pos >= min_x_pos:
                stalk_info = []
                stalk_info.append(x_pos)
                stalk_info.append(y_pos)
                stalk_info.append(row)
                stalk_info.append(stalk)
                vert_centers.append(stalk_info)

                stalk += 1
                x_pos -= self.stalk_dist 
        
        # 10% of the time the horizontal row does not exist
        if random.uniform(0.0, 1.0) < 0.9:
            # Generate stalks for horizontal rows
            origin_x = hori_origin[0]
            origin_y = hori_origin[1]
            hori_centers = []  # Storage containers

            # For each row determine each stalks information
            for row in range(0, hori_rows):
                # For each stalk determine respective X-Y coordinates
                y_pos = origin_x + hori_dRow * row
                min_x_pos = (y_pos - gap_intersect) / gap_slope + self.gap_dist / 2
                max_x_pos = min_x_pos + (self.hori_num_rows - 1) * self.row_dist
                x_pos = min_x_pos
                stalk = 0
                while x_pos <= max_x_pos:
                    stalk_info = []
                    stalk_info.append(x_pos)
                    stalk_info.append(y_pos)
                    stalk_info.append(row)
                    stalk_info.append(stalk)
                    hori_centers.append(stalk_info)

                    stalk += 1
                    x_pos += self.row_dist 
        else:
            hori_centers = None

        # Output centers
        return vert_centers, hori_centers

    def randomize_mean_centers(self, vert_centers, hori_centers, radius):
        '''
        Add noise to the designed position of the stalks
        '''
        for center in vert_centers:
            offset = self.generate_random_coordinate(radius)  # Generate a randomized offset
            center[0] = center[0] + offset[0]
            center[1] = center[1] + offset[1]
        if hori_centers is not None:
            for center in hori_centers:
                offset = self.generate_random_coordinate(radius)  # Generate a randomized offset
                center[0] = center[0] + offset[0]
                center[1] = center[1] + offset[1]

        return vert_centers, hori_centers

    @staticmethod
    def randomize_poses(vert_centers, hori_centers):
        '''
        Randomly generate the pose of each stalk
        '''
        vert_poses = []
        hori_poses = []
        range_angle = (0, 360)

        for i in range(len(vert_centers)):
            theta = math.radians(random.randrange(*range_angle))  # Generate a randomized angle
            vert_poses.append([0, 0, theta])
        if hori_centers is not None:
            for i in range(len(hori_centers)):
                theta = math.radians(random.randrange(*range_angle))  # Generate a randomized angle
                hori_poses.append([0, 0, theta])
        else:
            hori_poses = None

        return vert_poses, hori_poses

    def randomize_row_len(self):
        ''' 
        Randomize the length of each row. Currently, min is 4.7 and max is 5.0
        '''
        for i in range(self.vert_num_rows):
            self.row_lens[i] = random.uniform(self.min_row_len, self.max_row_len)

    # ========================================================
    #                 SDF Output Functions
    # ========================================================

    def load_stalk_info_sdf(self, type_t, vert_centers, hori_centers, vert_poses, hori_poses, 
                            plotID, stalkEmergenceP, meanStalkHeight, minModel, maxModel, headerXML, footerXML):
        # Create the XML document object to add elements to
        doc = minidom.Document()

        # Create a root to add child elements to
        root = doc.createElement('sdf')
        root.setAttribute('version','1.5')
        doc.appendChild(root)
        model = doc.createElement('model')
        model.setAttribute('name',str(type_t) + '_plot')
        root.appendChild(model)
        static = doc.createElement('static')
        static.appendChild(doc.createTextNode('true'))
        model.appendChild(static)
        model.appendChild(doc.createTextNode(' ')) 

        for i in range(len(vert_centers)):
            # Skip the stalk for certain gap probability
            if(random.random() > stalkEmergenceP):
                continue
            
            center = vert_centers[i]
            pose = vert_poses[i]
            x = str(center[0])
            y = str(center[1])
            r = str(pose[0])
            p = str(pose[1])
            yaw = str(pose[2])

            m_pose = x + ' ' + y + ' ' + str(0) + ' ' + r + ' ' + p + ' ' + yaw

            # Create XML element
            include = doc.createElement('include')
            uri = doc.createElement('uri')
            uri.appendChild(doc.createTextNode('model://' + str(self.stalk_models[self.variant_idx])))
            name = doc.createElement('name')
            name.appendChild(doc.createTextNode('plant_'+ str(plotID) + '_'+ str(i) + '_vert'))
            pose = doc.createElement('pose')
            pose.appendChild(doc.createTextNode(str(m_pose)))
            include.appendChild(uri)
            include.appendChild(name)
            include.appendChild(pose)

            # Add element to root
            model.appendChild(include)
            model.appendChild(doc.createTextNode(' '))
        
        if hori_centers is not None:
            for i in range(len(hori_centers)):
                # Skip the stalk for certain gap probability
                if(random.random() > stalkEmergenceP):
                    continue
                
                # Skip the stalk in vertical rows if its x position exceeds corresponding row length
                # In this design every row has 17 stalks originally
                # row_idx = int(stalk / 17)
                # if stalk < 119 and centers[0, stalk] > self.row_lens[row_idx]:
                #     continue

                center = hori_centers[i]
                pose = hori_poses[i]
                x = str(center[0])
                y = str(center[1])
                r = str(pose[0])
                p = str(pose[1])
                yaw = str(pose[2])

                m_pose = x + ' ' + y + ' ' + str(0) + ' ' + r + ' ' + p + ' ' + yaw

                # Create XML element
                include = doc.createElement('include')
                uri = doc.createElement('uri')
                uri.appendChild(doc.createTextNode('model://' + str(self.stalk_models[self.variant_idx])))
                name = doc.createElement('name')
                name.appendChild(doc.createTextNode('plant_'+ str(plotID) + '_' + str(i) + '_hori'))
                pose = doc.createElement('pose')
                pose.appendChild(doc.createTextNode(str(m_pose)))
                include.appendChild(uri)
                include.appendChild(name)
                include.appendChild(pose)

                # Add element to root
                model.appendChild(include)
                model.appendChild(doc.createTextNode(' '))

        # Convert XML document to string for viewing and file writing
        xml_str = doc.toprettyxml(indent="  ")

        if(headerXML == False):
            #remove first 2 lines of xml
            xml_str = xml_str[xml_str.find('\n')+1:]
            xml_str = xml_str[xml_str.find('\n')+1:]
            xml_str = xml_str[xml_str.find('\n')+1:]
            xml_str = xml_str[xml_str.find('\n')+1:]

        if(footerXML == False):
            #remove last line of xml
            xml_str = xml_str[0:xml_str.rfind('\n')]
            xml_str = xml_str[0:xml_str.rfind('\n')]
            xml_str = xml_str[0:xml_str.rfind('\n')]

        return xml_str


    # ========================================================
    #            Main function to create environment
    # ========================================================

    def create_env(self, env_idx):
        # Define output file names
        sdf_name = "model"  # The name of .sdf output file
        field_name = f"env_{env_idx}"  # The name of .field output file

        num_plots = self.forward_plots * self.sideward_plots  # Always only one plot
        plot_width = self.row_dist * (self.vert_num_rows - 1)
        self.gap_dist = 0.76  # Gap between horizontal and vertical plots (m)

        final_xml_str = ""
        shapefile_str = field_name + "\n\n" + "PLOTS\n" + str(1) + "\n\n"
        row_info_str = ""
        index = 0
        plot_center = np.array([plot_width / 2, self.vert_row_len / 2])
        plot_id = index
        stalk_emerg_prob = self.stalk_min_emerg_prob + random.random() * (self.stalk_max_emerg_prob - self.stalk_min_emerg_prob)
        mean_stalk_height = self.stalk_min_height + self.stalk_height_prob * random.random() * (self.stalk_max_height - self.stalk_min_height)

        gap_center_eq = self.define_gap_center()  # Determine the equation for the tilted gap center
        self.gap_center_eq = gap_center_eq
        vert_mean_centers, hori_mean_centers = self.generate_mean_centers(self.vert_num_rows, int(self.vert_row_len / self.stalk_dist + 1), self.row_dist, self.stalk_dist, [plot_center[0] - plot_width / 2, plot_center[1] - self.vert_row_len / 2],
                                                                          int(self.hori_row_len / self.stalk_dist + 1), self.hori_num_rows, self.stalk_dist, self.row_dist, [plot_center[0] - plot_width / 2, plot_center[1] - self.vert_row_len / 2 + self.vert_row_len + self.gap_dist],
                                                                          gap_center_eq)
        vert_centers, hori_centers = self.randomize_mean_centers(vert_mean_centers, hori_mean_centers, self.radius)
        vert_poses, hori_poses = self.randomize_poses(vert_centers, hori_centers)

        self.vert_centers = vert_centers
        self.hori_centers = hori_centers

        # Load the generated stalk info into xml format to be used by ROS
        mean_stalk_height = self.stalk_min_height + random.random()*(self.stalk_max_height - self.stalk_min_height)
        xml_str = self.load_stalk_info_sdf(field_name, vert_centers, hori_centers, vert_poses, hori_poses, plot_id, stalk_emerg_prob, mean_stalk_height, 0, 5, index == 0, index == (num_plots - 1))
        final_xml_str = final_xml_str + xml_str

        shapefile_str = shapefile_str + str(index) + "\n" + "PLOT" + str(index) + "_NAME" + "\nCORN" + "\nThis plot has candy corn" + "\nROWS\n" + str(self.vert_num_rows) + "\n"

        for r in range(0, self.vert_num_rows):
            x0 = plot_center[0] - plot_width / 2  + self.row_dist * (r - 1)
            x1 = x0
            y0 = plot_center[1] - self.vert_row_len / 2
            y1 = y0 + self.vert_row_len
            (lat0, lon0) = dxy2dlatlon(self.gps_origin[0], self.gps_origin[1], x0, y0)
            (lat1, lon1) = dxy2dlatlon(self.gps_origin[0], self.gps_origin[1], x1, y1)
            shapefile_str = shapefile_str + str(self.gps_origin[0] + lat0) + " " + str(self.gps_origin[1] + lon0) + " " + str(self.gps_origin[0] + lat1) + " " + str(self.gps_origin[1] + lon1) + "\n"
        shapefile_str = shapefile_str + "\n"

        # Save .sdf file
        if not os.path.exists(f"../models/env_{env_idx}"):
            os.mkdir(f"../models/env_{env_idx}")
        else:
            shutil.rmtree(f"../models/env_{env_idx}")
            os.mkdir(f"../models/env_{env_idx}")
        with open(f"../models/env_{env_idx}/" + sdf_name + ".sdf", "w") as f:
            f.write(final_xml_str)
        shutil.copyfile("../models/model.config", f"../models/env_{env_idx}/model.config")
        
        # Save .field file
        with open("../sdf/" + field_name + ".field", "w") as f:
            f.write(shapefile_str)
    

    # ========================================================
    #                  Generate Routes
    # ========================================================
    
    def plot_waypoints(self, init_x, init_y, target_x, target_y, ref_waypoints):
        ref_x = []
        ref_y = []

        ref_x.append(init_x)
        ref_y.append(init_y)
        for i in range(len(ref_waypoints)):
            ref_x.append(ref_waypoints[i][0])
            ref_y.append(ref_waypoints[i][1])
        ref_x.append(target_x)
        ref_y.append(target_y)

        vert_centers = np.array(self.vert_centers)
        hori_centers = np.array(self.hori_centers)
        x_vert = vert_centers[:, 0]
        y_vert = vert_centers[:, 1]
        x_hori = hori_centers[:, 0]
        y_hori = hori_centers[:, 1]
        
        plt.figure(0)
        plt.scatter(ref_x, ref_y, label='Reference path')
        plt.scatter(x_vert, y_vert, label='Vertical rows')
        plt.scatter(x_hori, y_hori, label='Horizontal rows')
        plt.legend()
        plt.savefig('plot.png')

    def create_routes(self, env_idx, num_routes):
        """Create different routes inside one environment
        """
        '''
        routes data structure: 
            [[(init_lane, target_lane), (init_x, init_y, init_yaw), (target_x, target_y), [(wp1), (wp2), ...]], [...], ...]
        
        robot_init_lane: The lane no. for robot initial position
        robot_target_lane: The lane no. for robot target position
        robot_init_x: Robot initial x position (m) in world frame
        robot_init_y: Robot initial y position (m) in world frame
        robot_init_yaw: Robot initial yaw angle (degrees) in world frame
        robot_target_x: Robot target x position (m) in world frame
        robot_target_y: Robot target y position (m) in world frame
        '''
        self.env_config = {}
        self.routes_json = {}
        for i in range(num_routes):
            route_dict = {}  # Dictionary to store route information

            # Detrmine initial lane and target lane
            init_lane = 0
            target_lane = 0
            while True:
                init_lane_ = random.randint(1, self.vert_num_rows - 1)
                target_lane_ = random.randint(1, self.vert_num_rows - 1)
                if abs(target_lane_ - init_lane_) <= 3 and abs(target_lane_ - init_lane_) > 1:
                    init_lane = init_lane_
                    target_lane = target_lane_
                    break
            route_dict["init_lane"] = init_lane
            route_dict["target_lane"] = target_lane
            
            slope = self.gap_center_eq[0]
            intersect = self.gap_center_eq[1]

            # Robot initial pose 
            init_y = self.row_dist * init_lane - random.uniform(self.robot_init_margin, self.row_dist - self.robot_init_margin)
            x_offset = random.uniform(self.robot_min_init_x_diff, self.robot_max_init_x_diff)  # Offset from the end of the row 
            init_x = (init_y - intersect) / slope - self.gap_dist / 2 - x_offset
            init_yaw = random.uniform(self.robot_min_init_yaw, self.robot_max_init_yaw)
            route_dict["init_x"] = init_x
            route_dict["init_y"] = init_y
            route_dict["init_yaw"] = init_yaw 

            # Reference initial and target pose
            ref_init_y = self.row_dist * init_lane - self.row_dist / 2
            ref_target_y = self.row_dist * target_lane - self.row_dist / 2
            ref_target_x = (ref_target_y - intersect) / slope - self.gap_dist / 2 - x_offset
            route_dict["ref_init_x"] = init_x
            route_dict["ref_init_y"] = ref_init_y
            route_dict["ref_target_x"] = ref_target_x
            route_dict["ref_target_y"] = ref_target_y
                                                                                                                                                                           
            # Fill in waypoints
            noi_waypoints = [[self.vert_row_len, self.row_dist * target_lane], [self.robot_min_init_x, self.row_dist * (target_lane - 1)]]
    
            # Generate reference path
            ref_waypoints = []
            ref_waypoints.append([(ref_init_y - intersect) / slope - self.gap_dist / 2 - 0.5, ref_init_y])
            ref_waypoints.append([(ref_init_y - intersect) / slope - self.gap_dist / 2 - 0.25, ref_init_y])
            ref_waypoints.append([(ref_init_y - intersect) / slope - self.gap_dist / 2, ref_init_y])
            interval = 7
            step = (ref_target_y - ref_init_y) / interval
            y_pos = ref_init_y
            step_count = 1
            while step_count <= interval - 1:
                y_pos += step
                x_pos = (y_pos - intersect) / slope 
                ref_waypoints.append([x_pos, y_pos])
                step_count += 1
            ref_waypoints.append([(ref_target_y - intersect) / slope - self.gap_dist / 2, ref_target_y])
            ref_waypoints.append([(ref_target_y - intersect) / slope - self.gap_dist / 2 - 0.25, ref_target_y])
            ref_waypoints.append([(ref_target_y - intersect) / slope - self.gap_dist / 2 - 0.5, ref_target_y])
            ref_waypoints.append([ref_target_x, ref_target_y])

            # self.plot_waypoints(init_x, ref_init_y, ref_target_x, ref_target_y, ref_waypoints)  # Debug purpose
            route_dict["noi_waypoints"] = noi_waypoints
            route_dict["ref_waypoints"] = ref_waypoints

            self.env_config[f"route_{i}"] = route_dict
            self.routes_json[f"route_{i}"] = route_dict
        
        
    # ========================================================
    #                     Save data
    # ========================================================

    def save_config(self, env_idx):
        if not os.path.exists("../configs"):
            os.mkdir("../configs")
        
        if not os.path.exists(f"../configs/env_{env_idx}"):
            os.mkdir(f"../configs/env_{env_idx}")
        else:
            shutil.rmtree(f"../configs/env_{env_idx}")
            os.mkdir(f"../configs/env_{env_idx}")

        json_object = json.dumps(self.routes_json)
        with open(f"../configs/env_{env_idx}/routes_config.json", "w") as outfile:
            outfile.write(json_object)


if __name__ == "__main__":
    # Parse arguments (specify number of environments to generate)
    parser = argparse.ArgumentParser()
    parser.add_argument("--num_env", type=int, default=1, help="Total number of environments to generate")
    parser.add_argument("--num_routes", type=int, default=1, help="Number of routes in one environment")
    args = vars(parser.parse_args())
    num_env = args["num_env"]
    num_routes = args["num_routes"]

    env_creator = EnvCreator()
    env_creator.create_stalk_variants()  # Specify stalk varaints to choose from

    for env_idx in range(num_env):
        env_creator.create_env(env_idx)
        env_creator.create_routes(env_idx, num_routes)
        env_creator.save_config(env_idx)
