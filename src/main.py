#!/usr/bin/env python3
import rospy
import rospkg
import numpy as np
import argparse
import time
from graic_msgs.msg import ObstacleList, ObstacleInfo
from graic_msgs.msg import LocationInfo, WaypointInfo
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleInfo
from graic_msgs.msg import LaneList
from graic_msgs.msg import LaneInfo

from GoHeelsRacing.msg import LineSegment, VoronoiPlannerInput, VoronoiPlannerOutput
from geometry_msgs.msg import Vector3
import carla
import math


def Loc_to_V3(l):
    return Vector3(l.x, l.y, l.z)


def V3_to_Loc(v):
    # Vector3 to carla.Location
    return carla.Location(v.x, v.y, v.z)


def V3_to_Rot(v):
    # Vector3 to carla.Rotation
    return carla.Rotation(v.x, v.y, v.z)


class VehicleController():

    def __init__(self, role_name='ego_vehicle'):
        # Publisher to publish the control input to the vehicle model
        self.role_name = role_name
        self.ackermannControlPub = rospy.Publisher(
            "/carla/%s/ackermann_cmd" % role_name, AckermannDrive, queue_size=1)
        self.carlaControlPub = rospy.Publisher(
            "/carla/%s/vehicle_control_cmd" % role_name, CarlaEgoVehicleControl, queue_size=1)
        self.subVehicleInfo = rospy.Subscriber(
            "/carla/%s/vehicle_info" % role_name, CarlaEgoVehicleInfo, self.vehicleInfoCallback)

        self.wheelbase = 2.0  # will be overridden by vehicleInfoCallback
        # will be overridden by vehicleInfoCallback
        self.max_steer_rad = math.radians(70)
        self.max_speed = 25
        self.min_speed = 10

        self.old_steering = 0.0 # new variable to tune the turning based on speed

        self.brake_coeff = 1.0  # To tune speed_diff-throttle curve



    def vehicleInfoCallback(self, data):
        p = [w.position for w in data.wheels]
        r2f_x = 0.5*(p[0].x + p[1].x - (p[2].x + p[3].x))
        r2f_y = 0.5*(p[0].y + p[1].y - (p[2].y + p[3].y))
        self.wheelbase = np.linalg.norm([r2f_x, r2f_y])
        print(f'Controller set wheelbase to {self.wheelbase} meters.')
        rospy.logwarn(f'Controller set wheelbase to {self.wheelbase} meters.')

        self.max_steer_rad = data.wheels[0].max_steer_angle
        print(f'Controller set max_steer_rad to {self.max_steer_rad} radians.')
        rospy.logwarn(f'Controller set max_steer_rad to {self.max_steer_rad} radians.')


    def initiate(self):
        self.carlaControlPub.publish(CarlaEgoVehicleControl())

    def stop(self):
        rospy.logwarn("stop")
        newAckermannCmd = AckermannDrive()
        newAckermannCmd.acceleration = -20
        newAckermannCmd.speed = 0
        newAckermannCmd.steering_angle = 0
        self.ackermannControlPub.publish(newAckermannCmd)

    def execute(self, currentState, targetState):
        """
            This function takes the current state of the vehicle and
            the target state to compute low-level control input to the vehicle
            Inputs:
                currentPose: ModelState, the current state of vehicle
                targetPose: The desired state of the vehicle
        """
        import math
        if not targetState[0] or not targetState[1]:
            self.carlaControlPub.publish(CarlaEgoVehicleControl())
            rospy.logwarn("no control")
            return

        currentEuler = currentState[1]
        curr_x = currentState[0][0]
        curr_y = currentState[0][1]

        currentSpeed = math.sqrt(currentState[2][0] ** 2 + currentState[2][1] ** 2)
        speed_error = targetState[2] - currentSpeed
        curr_speed = np.clip(currentSpeed, self.min_speed, self.max_speed)

        car_loc = carla.Location(curr_x, curr_y, 0)
        car_rot = carla.Rotation(0, np.degrees(currentEuler[2]), 0)
        car2map = carla.Transform(car_loc, car_rot)
        rear_axle = car2map.transform(
            carla.Location(-self.wheelbase/2.0, 0, 0))

        target_x = targetState[0]
        target_y = targetState[1]

        dx = target_x - curr_x
        dy = target_y - curr_y
        # Rotate (dx, dy) by -currentEuler[2] radians
        xError = dx*np.cos(currentEuler[2]) + dy*np.sin(currentEuler[2])
        yError = -dx*np.sin(currentEuler[2]) + dy*np.cos(currentEuler[2])

        # transform (xError, yError) to wheelbase coordinates
        xError += self.wheelbase/2.0

        # pure-pursuit steering rule
        import math
        d2 = xError**2 + yError**2
        steer_rad = math.atan(2 * self.wheelbase * yError / d2)
        #steer_error = steer_rad - self.old_steering

        #steer_rad = self.old_steering + steer_error* (1.0 / (1.0 + np.exp((curr_Speed - self.max_speed) / (curr_Speed - self.min_speed)))) # change the steering angle by a factor of (0.5: at max speed; 1.0: at minimum speed)

        # Map the steering angle to ratio of maximum possible steering angle
        steer_rad = np.clip(steer_rad, -self.max_steer_rad, self.max_steer_rad)
        steer_ratio = steer_rad/self.max_steer_rad

        self.old_steering = steer_rad
        accelerate_activation = 5.0
        brake_activation = -5.0
        coast_activation = 0.0
        if speed_error > coast_activation:
            newAckermannCmd = AckermannDrive()
            newAckermannCmd.acceleration = 0  # Change speed as quickly as possible
            newAckermannCmd.speed = targetState[2]
            newAckermannCmd.steering_angle = -steer_rad
            # Change angle as quickly as possible:
            newAckermannCmd.steering_angle_velocity = 5
            self.ackermannControlPub.publish(newAckermannCmd)
        elif speed_error >= brake_activation:
            print(f'Coast.')
            rospy.logwarn("coast")
            newControlCmd = CarlaEgoVehicleControl()
            newControlCmd.throttle = 0
            newControlCmd.steer = steer_ratio
            newControlCmd.brake = 0
            newControlCmd.hand_brake = False
            newControlCmd.reverse = False
            newControlCmd.manual_gear_shift = False
            self.carlaControlPub.publish(newControlCmd)
        else:  # brake_activation > speed_error
            brake = min(1,
                        math.atan(self.brake_coeff*(brake_activation-speed_error)))
            print(f'Brake: {brake}')
            rospy.logwarn("brake")
            newControlCmd = CarlaEgoVehicleControl()
            newControlCmd.throttle = 0
            newControlCmd.steer = steer_ratio
            newControlCmd.brake = brake
            newControlCmd.hand_brake = False
            newControlCmd.reverse = False
            newControlCmd.manual_gear_shift = False
            self.carlaControlPub.publish(newControlCmd)


class VehicleDecision():
    def __init__(self, role_name='ego_vehicle'):
        # rospy.Subscriber("/carla/%s/lane_markers" % role_name, LaneInfo, self.lanemarkerCallback)
        # rospy.Subscriber("/carla/%s/waypoints" % role_name, WaypointInfo, self.waypointCallback)
        rospy.Subscriber("/carla/%s/vehicle_info" % role_name, CarlaEgoVehicleInfo, self.vehicleInfoCallback)
        # rospy.Subscriber("/carla/%s/location" % role_name, LocationInfo, self.locationCallback)
        # rospy.Subscriber("/carla/%s/obstacles" % role_name, ObstacleList, self.obstacleCallback)

        self.voronoiPub = rospy.Publisher(
            "/planner_input", VoronoiPlannerInput, queue_size=1)
        self.subVoronoi = rospy.Subscriber(
            "/planner_output", VoronoiPlannerOutput, self.planCallback)


        self.position = None
        self.velocity = None
        self.rotation = None
        self.obstacleList = None

        self.lookahead = 5.0  # meters
        self.wheelbase = 2.0  # will be overridden by vehicleInfoCallback
        self.allowed_obs_dist = 1.7  # meters from Voronoi diagram to obstacles
        self.max_speed = 25
        self.min_speed = 10
        self.speed_coeff = 0.35  # to tune the speed controller
        self.friction_coeff = 0.5
        self.old_prediction = []
        self.history = []

        self.plan = []
        self.roadmap = []
        self.reachEnd = False

        self.milestone = None
        self.lane_info = None

        # For debuggin purposes. TODO delete later
        host = rospy.get_param('~host', 'localhost')
        port = rospy.get_param('~port', 2000)
        client = carla.Client(host, port)
        self.world = client.get_world()


    def vehicleInfoCallback(self, data):
        p = [w.position for w in data.wheels]
        r2f_x = 0.5*(p[0].x + p[1].x - (p[2].x + p[3].x))
        r2f_y = 0.5*(p[0].y + p[1].y - (p[2].y + p[3].y))
        self.wheelbase = np.linalg.norm([r2f_x, r2f_y])
        print(f'Planner set wheelbase to {self.wheelbase} meters.')

    def planCallback(self, data):
        #rospy.logwarn("planCallback")
        self.plan = [self.rearAxle_to_map(
            self.currState, V3_to_Loc(v)) for v in data.plan]

        self.roadmap = [(self.rearAxle_to_map(self.currState, V3_to_Loc(seg.start)),
                         self.rearAxle_to_map(self.currState, V3_to_Loc(seg.end)))
                        for seg in data.roadmap]

        #---- Visualization ----
        h0 = carla.Location(
            0, 0, self.lane_info.lane_markers_center.location[-1].z + 0.5)
        h1 = carla.Location(
            0, 0, self.lane_info.lane_markers_center.location[-1].z + 1.0)
        #Roadmap
        for loc0, loc1 in self.roadmap:
            self.world.debug.draw_line(
                loc0+h0, loc1+h0, thickness=0.1, color=carla.Color(255, 255, 255), life_time=0.1)
        # Plan
        for i in range(len(self.plan)-1):
            self.world.debug.draw_line(
                self.plan[i]+h1, self.plan[i+1]+h1, thickness=0.2, color=carla.Color(0, i*4, 0), life_time=0.1)

    def rearAxle_to_map(self, currentState, loc):
        currentEuler = currentState[1]
        curr_x = currentState[0][0]
        curr_y = currentState[0][1]

        car_loc = carla.Location(curr_x, curr_y, 0)
        car_rot = carla.Rotation(0, np.degrees(currentEuler[2]), 0)
        car2map = carla.Transform(car_loc, car_rot)
        loc_in_car = loc - carla.Location(self.wheelbase/2., .0, .0)
        return carla.Location(car2map.transform(loc_in_car))

    def map_to_rearAxle(self, currentState, loc):
        currentEuler = currentState[1]
        car_rot = carla.Rotation(0, np.degrees(currentEuler[2]), 0)

        forward = car_rot.get_forward_vector()
        right = car_rot.get_right_vector()

        ra = self.rearAxle(currentState)
        v = loc - ra
        vx = forward.x * v.x + forward.y * v.y
        vy = right.x * v.x + right.y * v.y
        return carla.Location(vx, vy, 0)

    def rearAxle(self, currentState):  # TODO use map_to_rearAxle instead
        currentEuler = currentState[1]
        curr_x = currentState[0][0]
        curr_y = currentState[0][1]
        car_loc = carla.Location(curr_x, curr_y, 0)
        car_rot = carla.Rotation(0, np.degrees(currentEuler[2]), 0)
        car2map = carla.Transform(car_loc, car_rot)
        rear_axle = car2map.transform(
            carla.Location(-self.wheelbase/2.0, 0, 0))
        return rear_axle

    def ccw(self, points):
        # Sort vertices of a convex polygon counter-clockwise
        ps = sorted(points, key=lambda p: p.x)
        pccw = []
        for p in ps[1:]:
            v = p-ps[0]
            l = math.sqrt(v.x**2+v.y**2)
            if (l < 0.001):  # Bicycles are thin
                continue
            sin = v.y/l
            pccw.append((sin, p))
        pccw.sort(key=lambda pair: pair[0])
        return [ps[0]] + [p[1] for p in pccw]

    def pubPlannerInput(self, currentState, obstacles):

        obstacles = self.obstacleList
        road_boundaries = []

        # If no milestones seen so far
        if not self.milestone:
            print('No milestones seen yet!')
            return
        if not self.lane_info:
            print('No lane info yet!')
            return

        right_border, left_border = [], []
        right_waypoints = self.lane_info.lane_markers_right
        center_waypoints = self.lane_info.lane_markers_center
        left_waypoints = self.lane_info.lane_markers_left
        for i, (loc, rot, center) in enumerate(zip(right_waypoints.location, right_waypoints.rotation, center_waypoints.location)):
            loc, right = V3_to_Loc(loc), V3_to_Rot(rot).get_right_vector()
            c = V3_to_Loc(center)
            lane_width = c.distance(loc)*2
            lane_count = abs(self.lane_info.RIGHT_LANE -
                             self.lane_info.lane_state)
            if c.distance(loc+right) < c.distance(loc):
                lane_count += 1
            border = loc + right*lane_width*lane_count + \
                right*0.8 + carla.Location(0, 0, 0.2)
            right_border.append(border)

        for loc, rot, center in zip(left_waypoints.location, left_waypoints.rotation, center_waypoints.location):
            loc, left = V3_to_Loc(loc), V3_to_Rot(rot).get_right_vector()*(-1)
            c = V3_to_Loc(center)
            lane_width = c.distance(loc)*2
            lane_count = abs(self.lane_info.LEFT_LANE -
                             self.lane_info.lane_state)
            if c.distance(loc+left) < c.distance(loc):
                lane_count += 1
            border = loc + left*lane_width*lane_count + \
                left*0.8 + carla.Location(0, 0, 0.2)
            left_border.append(border)

        road_center = left_border[-1]*0.5 + right_border[-1]*0.5

        # Linear extrapolation after the last boundary,
        # based on the direction from road center to milestone
        ext_size = 16  # meters
        center_to_milestone = self.milestone.distance(road_center)
        if center_to_milestone > 8.0:
            ext1 = (self.milestone - road_center) * ext_size \
                / center_to_milestone
        else:
            lane_center_rot = carla.Rotation(
                0, self.lane_info.lane_markers_center.rotation[-1].y, 0)
            ext1 = lane_center_rot.get_forward_vector()*ext_size

        road_boundaries += [(right_border[i], right_border[i+1])
                            for i in range(len(right_border)-1)]
        road_boundaries += [(right_border[-1], right_border[-1]+ext1)]

        road_boundaries += [(left_border[i], left_border[i+1])
                            for i in range(len(left_border)-1)]
        road_boundaries += [(left_border[-1], left_border[-1]+ext1)]

        milestone = road_center+ext1*1.5

        obstacle_boundaries = []
        for obs in obstacles:
            loc = self.map_to_rearAxle(currentState, V3_to_Loc(obs.location))
            if (loc.x < -2.0):
                continue  # ignore passed obstacles
            poly = []
            if len(obs.vertices_locations) == 0:
                continue
            for i in range(0, len(obs.vertices_locations), 2):
                vec = obs.vertices_locations[i].vertex_location
                poly.append(carla.Location(vec.x, vec.y, 0.0))
            poly = self.ccw(poly)
            obstacle_boundaries += [(poly[i-1], poly[i])
                                    for i in range(len(poly))]

        front = self.rearAxle_to_map(
            currentState, carla.Location(self.wheelbase*1.5, 0, 0))

        boundaries = road_boundaries + obstacle_boundaries

        # Publish
        data = VoronoiPlannerInput()
        data.allowed_obs_dist = self.allowed_obs_dist
        data.car_location = Loc_to_V3(
            self.map_to_rearAxle(currentState, front))
        data.milestone = Loc_to_V3(
            self.map_to_rearAxle(currentState, milestone))
        for segment in boundaries:
            s0 = self.map_to_rearAxle(currentState, segment[0])
            s1 = self.map_to_rearAxle(currentState, segment[1])
            start = Vector3(s0.x, s0.y, 0.0)
            end = Vector3(s1.x, s1.y, 0.0)
            data.obstacles.append(LineSegment(start, end))

        self.voronoiPub.publish(data)

        # ----- Visualization----
        # Road boundaries
        # for bound in road_boundaries:
        #     self.world.debug.draw_line(
        #         bound[0], bound[1], thickness=0.2, life_time=0.1)
        #
        # # Milestone
        # self.world.debug.draw_line(
        #     milestone, milestone+carla.Location(0, 0, 5), color=carla.Color(0, 255, 0), life_time=0.1)


    def map_to_line_segment(self, line_segment, loc, angle): # transform global location and angle to withrespect to the location frame of the line_segment
        origin = line_segment[0]
        line_angle = math.atan2(line_segment[1].y-line_segment[0].y, line_segment[1].x-line_segment[0].x)
        car_rot = carla.Rotation(0, math.degrees(line_angle), 0)
        forward = car_rot.get_forward_vector()
        right = car_rot.get_right_vector()
        v = loc - origin
        vx = forward.x * v.x + forward.y * v.y
        vy = right.x * v.x + right.y * v.y
        l = carla.Location(vx, vy, 0)
        a = angle - line_angle
        if a < -math.pi:
            a += 2*math.pi
        if a > math.pi:
            a -= 2*math.pi
        return (l, a)

    def line_segment_to_map(self, line_segment, loc, angle): # transform local location and angle with respect to a line_segment to global
        origin = line_segment[0]
        line_angle = math.atan2(line_segment[1].y - line_segment[0].y, line_segment[1].x - line_segment[0].x)
        car_loc = carla.Location(origin.x, origin.y, 0)
        car_rot = carla.Rotation(0, np.degrees(line_angle), 0)
        car2map = carla.Transform(car_loc, car_rot)
        l = carla.Location(car2map.transform(loc))
        a = angle + line_angle
        if a < -math.pi:
            a += 2 * math.pi
        if a > math.pi:
            a -= 2 * math.pi
        return (l, a)

    def define_circle(self, p1, p2, p3):
        """
        Returns the radius of the circle passing the given 3 points.
        """
        temp = p2[0] * p2[0] + p2[1] * p2[1]
        bc = (p1[0] * p1[0] + p1[1] * p1[1] - temp) / 2
        cd = (temp - p3[0] * p3[0] - p3[1] * p3[1]) / 2
        det = (p1[0] - p2[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p2[1])

        if abs(det) < 1.0e-6: # the points form a line
            return np.inf

        # Center of circle
        cx = (bc*(p2[1] - p3[1]) - cd*(p1[1] - p2[1])) / det
        cy = ((p1[0] - p2[0]) * cd - (p2[0] - p3[0]) * bc) / det

        radius = np.sqrt((cx - p1[0])**2 + (cy - p1[1])**2)
        return radius


    def get_min_turn_radius(self, pp): # use line segments to find the circle approximation of the path and determine max curv:
        predicted_pos = [pp[0]]
        for i in range(1, len(pp)):
            if pp[i].distance(predicted_pos[-1])<=0.1:
                continue
            predicted_pos.append(pp[i])
        min_turn_radius = math.inf
        min_turn_point = None
        for i in range(len(predicted_pos)-2):
            p1 = [predicted_pos[i].x,  predicted_pos[i].y]
            p2 = [predicted_pos[i+1].x,  predicted_pos[i+1].y]
            p3 = [predicted_pos[i+2].x,  predicted_pos[i+2].y]
            turn_radius = self.define_circle(p1, p2, p3)
            if turn_radius < min_turn_radius:
                min_turn_radius = turn_radius
                min_turn_point = predicted_pos[i]

        #visualization
        if min_turn_point != None:
            self.world.debug.draw_line(carla.Location(min_turn_point.x, min_turn_point.y, 0.0), carla.Location(min_turn_point.x, min_turn_point.y, 2.0), color=carla.Color(255, 0, 0), life_time=0.1)

        return min_turn_radius

    def skidding_control(self, currentState):
        # look for deviation from the predicted plan, update friction coeffecient accordingly
        currentSpeed = math.sqrt(currentState[1][0]**2 + currentState[1][1]**2)
        currentPose = carla.Location(currentState[0][0], currentState[0][1], 0)
        predicted_pose = self.old_prediction
        for i in range(len(predicted_pose)-1):

            break
        return

    def get_speed(self, plan_full, currentState, lookahead): # use model predictive control to calculate target speed
        plan = [plan_full[0]]
        for p in plan_full:
            if p.distance(plan[-1]) < 0.5: # track plan with line segments of 0.5m
                continue
            plan.append(p)
        nodes = []
        for i in range(1, len(plan)):
            if plan[i].distance(plan[0]) > 15:  # only check for 15m ahead
                break
            nodes += [(plan[i-1], plan[i])]

        global_loc = self.rearAxle(currentState)
        self.history.append(global_loc)
        predicted_pos = [global_loc]
        global_angle = currentState[1][2]
        current_node_ind = 0
        step_size = 0.1 # step size = 0.05s
        #current_speed = np.clip(math.sqrt(currentState[2][0]**2 + currentState[2][1]**2),self.min_speed, self.max_speed)
        #rospy.logwarn(current_speed)
        while current_node_ind < len(nodes):
            current_node = nodes[current_node_ind] #move on to the next line segment
            (relative_loc, relative_angle) = self.map_to_line_segment(current_node, global_loc, global_angle)
            line_segment_length = current_node[0].distance(current_node[1])
            #while math.sqrt(relative_loc.x**2 + relative_loc.y**2) < line_segment_length and relative_loc.x < line_segment_length:
            while  math.sqrt((line_segment_length-relative_loc.x)**2 + relative_loc.y**2) >= lookahead:
                angle_rate = 2*(-math.sqrt(lookahead**2-min(relative_loc.y**2, lookahead**2))*math.sin(relative_angle) - relative_loc.y*math.cos(relative_angle)) / lookahead**2
                relative_loc = carla.Location(relative_loc.x+step_size*math.cos(relative_angle), relative_loc.y+step_size*math.sin(relative_angle), 0.0)
                relative_angle = relative_angle + angle_rate*step_size
                if relative_angle > math.pi:
                    relative_angle -= 2*math.pi
                if relative_angle < -math.pi:
                    relative_angle += 2*math.pi
                (temp, _) = self.line_segment_to_map(current_node, carla.Location(relative_loc), relative_angle)
                predicted_pos.append(temp)
            (global_loc, global_angle) = self.line_segment_to_map(current_node, relative_loc, relative_angle)
            current_node_ind += 1

        # update friction coeffecient
        if len(self.history) >= 3: # enough data point to estimate actual vehicle path's curvature
            p1 = [self.history[-3].x, self.history[-3].y]
            p2 = [self.history[-2].x, self.history[-2].y]
            p3 = [self.history[-1].x, self.history[-1].y]
            path_radius = self.define_circle(p1, p2, p3)
            min_deviation_from_plan = math.inf
            for i in range(len(self.old_prediction)-1):
                p1 = np.array([self.old_prediction[i].x, self.old_prediction[i].y])
                p2 = np.array([self.old_prediction[i+1].x, self.old_prediction[i+1].y])
                p3 = np.array([global_loc.x, global_loc.y])
                min_deviation_from_plan = min(np.linalg.norm(np.cross(p2-p1, p1-p3))/np.linalg.norm(p2-p1), min_deviation_from_plan) # euler # distance between current position and old predicton
            current_speed_square = currentState[2][0] ** 2 + currentState[2][1] ** 2
            if min_deviation_from_plan < 0.2:
                self.friction_coeff = max(self.friction_coeff, current_speed_square / (path_radius*9.81))
            if min_deviation_from_plan > 0.5:
                self.friction_coeff = min(self.friction_coeff, current_speed_square / (path_radius*9.81))
        rospy.logwarn(self.friction_coeff)

        # visualization
        for i in range(len(predicted_pos) - 1):
            self.world.debug.draw_line(
                carla.Location(predicted_pos[i].x, predicted_pos[i].y, 1.5),
                carla.Location(predicted_pos[i + 1].x, predicted_pos[i + 1].y, 1.5),
                color=carla.Color(0, 0, 255), life_time=0.1)

        min_turn_radius = np.clip(self.get_min_turn_radius(predicted_pos), 1e-6, 1e6)
        speed = np.clip(math.sqrt(self.friction_coeff*min_turn_radius*9.81), self.min_speed, self.max_speed)
        self.old_prediction = predicted_pos

        return speed



    def updateState(self, position, rotation, velocity, obstacleList, lane_info, milestone):
        self.position = position
        self.rotation = rotation
        self.velocity = velocity
        self.obstacleList = obstacleList
        self.lane_info = lane_info
        self.milestone = milestone
        self.currState = [position, rotation, velocity]

    def get_ref_state(self): # use constant lookahead distance so it's consistent with the model predictive speed control
        """
            Get the reference state for the vehicle according to the current state and result from perception module
            Inputs:
                currentState: [Loaction, Rotation, Velocity] the current state of vehicle
                obstacleList: List of obstacles
            Outputs: reference state position and velocity of the vehicle
        """
        if self.reachEnd:
            return None

        currentState = [self.position, self.rotation, self.velocity]
        obstacleList = self.obstacleList

        # Publish obstacles for VoronoiPlanner
        self.pubPlannerInput(
            currentState, obstacleList if obstacleList != None else [])

        if len(self.plan) == 0:
            print('No plans received yet.')
            return [0, 0, 0]


        # setting lookahead distance dynamically by speed:
        # currentSpeed = np.clip(math.sqrt(currentState[1][0]**2 + currentState[1][1]**2), 0.0, self.max_speed)
        # self.lookahead = 2.0 + 8.0 * (currentSpeed / self.max_speed)
        # rospy.logwarn(self.lookahead)
        # Find the waypoint with lookahead distance from real axle
        ra = self.rearAxle_to_map(currentState, carla.Location())
        if ra.distance(self.plan[0]) >= self.lookahead:
            print('Plan does not start in the lookahead disk!')
            target = self.plan[0]
        elif ra.distance(self.plan[-1]) <= self.lookahead:
            print('Plan does not end outside of the lookahead disk!')
            target = self.plan[-1]
        else:
            for i, loc in enumerate(self.plan):
                if (loc.x - ra.x) ** 2 + (loc.y - ra.y) ** 2 > self.lookahead ** 2:
                    break
            p_in = self.map_to_rearAxle(currentState, self.plan[i - 1])
            p_out = self.map_to_rearAxle(currentState, self.plan[i])

            # Intercept the lookahead circle with the plan segment (p_in, p_out)
            x1 = p_in.x
            y1 = p_in.y
            x2 = p_out.x
            y2 = p_out.y
            dx = x2 - x1
            dy = y2 - y1
            A = dx * dx + dy * dy
            B = x1 * dx + y1 * dy
            C = x1 * x1 + y1 * y1 - self.lookahead ** 2
            t = (-B + math.sqrt(B * B - A * C)) / A

            target_in_rearAxle = carla.Location(x1 + t * dx, y1 + t * dy, 0)
            target = self.rearAxle_to_map(currentState, target_in_rearAxle)

        speed = self.get_speed(self.plan, currentState, self.lookahead)
        currentSpeed = math.sqrt(currentState[2][0] ** 2 + currentState[2][1] ** 2)
        print(f'speed: {currentSpeed:.2f}, target speed: {speed:.2f}')

        # The latest target computed by plannerCallback
        return [target.x, target.y, speed]


class VehiclePerception:
    def __init__(self, role_name='ego_vehicle'):
        rospy.Subscriber("/carla/%s/lane_markers" % role_name, LaneInfo, self.lanemarkerCallback)
        rospy.Subscriber("/carla/%s/waypoints" % role_name, WaypointInfo, self.waypointCallback)
        rospy.Subscriber("/carla/%s/location" % role_name, LocationInfo, self.locationCallback)
        rospy.Subscriber("/carla/%s/obstacles" % role_name, ObstacleList, self.obstacleCallback)

        self.position = None
        self.velocity = None
        self.rotation = None
        self.obstacleList = None
        self.milestone = None
        self.lane_info = None


    def locationCallback(self, data):
        self.position = (data.location.x, data.location.y)
        self.rotation = (np.radians(data.rotation.x), np.radians(
            data.rotation.y), np.radians(data.rotation.z))
        self.velocity = (data.velocity.x, data.velocity.y)

    def obstacleCallback(self, data):
        self.obstacleList = data.obstacles

    def lanemarkerCallback(self, data):
        self.lane_info = data

    def waypointCallback(self, data):
        self.reachEnd = data.reachedFinal
        self.milestone = carla.Location(
            data.location.x, data.location.y, data.location.z)

    def ready(self):
        return (self.position
                    is not None) and (self.rotation is not None) and (
                        self.velocity
                        is not None) and (self.obstacleList is not None) and (self.lane_info is not None) and (self.milestone is not None)

    def clear(self):
        self.position = None
        self.velocity = None
        self.rotation = None
        self.obstacleList = None
        self.milestone = None
        self.lane_info = None



def run_model(role_name):
  # 100 Hz

    perceptionModule = VehiclePerception(role_name=role_name)
    decisionModule = VehicleDecision(role_name)
    controlModule = VehicleController(role_name=role_name)

    def shut_down():
        controlModule.stop()
    rospy.on_shutdown(shut_down)

    time.sleep(1)
    controlModule.initiate()

    print("Starter code is running")
    #rospy.logwarn("Starter code is running")

    while not rospy.is_shutdown():
        if perceptionModule.ready():
            # Get the target state from perception module
            currState = [perceptionModule.position, perceptionModule.rotation, perceptionModule.velocity]
            decisionModule.updateState(perceptionModule.position, perceptionModule.rotation, perceptionModule.velocity, perceptionModule.obstacleList, perceptionModule.lane_info, perceptionModule.milestone)
            refState = decisionModule.get_ref_state()
            if not refState:
                controlModule.stop()
                exit(0)
            perceptionModule.clear()
            # Execute
            controlModule.execute(currState, refState)
        time.sleep(0.01)


if __name__ == "__main__":
    rospy.init_node("voronoi_main", anonymous=True)
    roskpack = rospkg.RosPack()
    config_path = roskpack.get_path('graic_config')
    race_config = open(config_path + '/' + 'race_config', 'rb')
    vehicle_typeid = race_config.readline().decode('ascii').strip()
    sensing_radius = race_config.readline().decode('ascii').strip()
    role_name = 'ego_vehicle'

    #rospy.init_node("voronoi_main")
    try:
        run_model(role_name)
    except rospy.exceptions.ROSInterruptException:
        print("stop")
