#!/usr/bin/env python
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
            "/carla/%s/ackermann_control" % role_name, AckermannDrive, queue_size=1)
        self.carlaControlPub = rospy.Publisher(
            "/carla/%s/vehicle_control" % role_name, CarlaEgoVehicleControl, queue_size=1)
        self.subVehicleInfo = rospy.Subscriber(
            "/carla/%s/vehicle_info" % role_name, CarlaEgoVehicleInfo, self.vehicleInfoCallback)

        self.wheelbase = 2.0  # will be overridden by vehicleInfoCallback
        self.brake_coeff = 1.0  # To tune speed_diff-throttle curve

    def vehicleInfoCallback(self, data):
        p = [w.position for w in data.wheels]
        r2f_x = 0.5*(p[0].x + p[1].x - (p[2].x + p[3].x))
        r2f_y = 0.5*(p[0].y + p[1].y - (p[2].y + p[3].y))
        self.wheelbase = np.linalg.norm([r2f_x, r2f_y])
        print(f'Controller set wheelbase to {self.wheelbase} meters.')

        self.max_steer_rad = data.wheels[0].max_steer_angle
        print(f'Controller set max_steer_rad to {self.max_steer_rad} radians.')

    def stop(self):
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
        if not targetState[0] or not targetState[1]:
            return

        currentEuler = currentState[1]
        curr_x = currentState[0][0]
        curr_y = currentState[0][1]

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

        # Map the steering angle to ratio of maximum possible steering angle
        steer_rad = np.clip(steer_rad, -self.max_steer_rad, self.max_steer_rad)
        steer_ratio = steer_rad/self.max_steer_rad

        currentSpeed = math.sqrt(currentState[2][0]**2+currentState[2][1]**2)
        speed_error = targetState[2] - currentSpeed

        brake_activation = -1.0
        coast_activation = 0.0
        if speed_error > coast_activation:
            newAckermannCmd = AckermannDrive()
            newAckermannCmd.acceleration = 0  # Change speed as quickly as possible
            newAckermannCmd.speed = targetState[2]
            newAckermannCmd.steering_angle = steer_rad
            # Change angle as quickly as possible:
            newAckermannCmd.steering_angle_velocity = 0
            self.ackermannControlPub.publish(newAckermannCmd)
        elif speed_error >= brake_activation:
            print(f'Coast.')
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
        self.subLaneMarker = rospy.Subscriber(
            "/carla/%s/lane_markers" % role_name, LaneInfo, self.lanemarkerCallback)

        self.subWaypoint = rospy.Subscriber(
            "/carla/%s/waypoints" % role_name, WaypointInfo, self.waypointCallback)

        self.subVehicleInfo = rospy.Subscriber(
            "/carla/%s/vehicle_info" % role_name, CarlaEgoVehicleInfo, self.vehicleInfoCallback)

        self.voronoiPub = rospy.Publisher(
            "/planner_input", VoronoiPlannerInput, queue_size=1)
        self.subVoronoi = rospy.Subscriber(
            "/planner_output", VoronoiPlannerOutput, self.planCallback)

        self.lookahead = 5.0  # meters
        self.wheelbase = 2.0  # will be overridden by vehicleInfoCallback
        self.allowed_obs_dist = 1.7  # meters from Voronoi diagram to obstacles
        self.max_speed = 25
        self.min_speed = 10
        self.speed_coeff = 0.3  # to tune the speed controller

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

    def lanemarkerCallback(self, data):
        self.lane_info = data

    def waypointCallback(self, data):
        self.reachEnd = data.reachedFinal
        self.milestone = carla.Location(
            data.location.x, data.location.y, data.location.z)

    def vehicleInfoCallback(self, data):
        p = [w.position for w in data.wheels]
        r2f_x = 0.5*(p[0].x + p[1].x - (p[2].x + p[3].x))
        r2f_y = 0.5*(p[0].y + p[1].y - (p[2].y + p[3].y))
        self.wheelbase = np.linalg.norm([r2f_x, r2f_y])
        print(f'Planner set wheelbase to {self.wheelbase} meters.')

    def planCallback(self, data):
        self.plan = [self.rearAxle_to_map(
            self.currentState, V3_to_Loc(v)) for v in data.plan]

        self.roadmap = [(self.rearAxle_to_map(self.currentState, V3_to_Loc(seg.start)),
                         self.rearAxle_to_map(self.currentState, V3_to_Loc(seg.end)))
                        for seg in data.roadmap]

        # ---- Visualization ----
        h0 = carla.Location(
            0, 0, self.lane_info.lane_markers_center.location[-1].z + 0.5)
        h1 = carla.Location(
            0, 0, self.lane_info.lane_markers_center.location[-1].z + 1.0)
        # Roadmap
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
        for bound in road_boundaries:
            self.world.debug.draw_line(
                bound[0], bound[1], thickness=0.2, life_time=0.1)

        # Milestone
        self.world.debug.draw_line(
            milestone, milestone+carla.Location(0, 0, 5), color=carla.Color(0, 255, 0), life_time=0.1)

    def get_speed(self, plan_full):
        plan = [plan_full[0]]
        for p in plan_full:
            if p.distance(plan[-1]) < 0.01:
                continue
            plan.append(p)

        horizon = 10  # meters
        plan_len = 0
        i = 0
        for i in range(len(plan)-1):
            v = plan[i+1] - plan[i]
            plan_len += math.sqrt(v.x**2 + v.y**2)
            if plan_len > horizon:
                break
        i_horizon = i

        max_curv = 0.0
        for i in range(i_horizon):
            v0 = plan[i+1] - plan[i]
            v1 = plan[i+2] - plan[i+1]
            v0_n = math.sqrt(v0.x**2 + v0.y**2)
            v1_n = math.sqrt(v1.x**2 + v1.y**2)
            inner = v0.x*v1.x + v0.y*v1.y
            cos = inner/(v0_n*v1_n)
            cos = np.clip(cos, -1, 1)
            max_curv = max(math.acos(cos), max_curv)

        m = self.min_speed
        M = self.max_speed
        k = self.speed_coeff
        speed = 1/(k*max_curv + 1/(M-m)) + m
        return speed

    def get_ref_state(self, currentState, obstacleList):
        """
            Get the reference state for the vehicle according to the current state and result from perception module
            Inputs:
                currentState: [Loaction, Rotation, Velocity] the current state of vehicle
                obstacleList: List of obstacles
            Outputs: reference state position and velocity of the vehicle
        """
        if self.reachEnd:
            return None

        self.currentState = currentState

        # Publish obstacles for VoronoiPlanner
        self.pubPlannerInput(
            currentState, obstacleList if obstacleList != None else [])

        if len(self.plan) == 0:
            print('No plans received yet.')
            return [0.0, 0.0, 0.0]

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
                if (loc.x-ra.x)**2 + (loc.y-ra.y)**2 > self.lookahead**2:
                    break
            p_in = self.map_to_rearAxle(currentState, self.plan[i-1])
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
            C = x1 * x1 + y1 * y1 - self.lookahead**2
            t = (-B + math.sqrt(B * B - A * C)) / A

            target_in_rearAxle = carla.Location(x1+t*dx, y1+t*dy, 0)
            target = self.rearAxle_to_map(currentState, target_in_rearAxle)

        speed = self.get_speed(self.plan)
        currentSpeed = math.sqrt(currentState[2][0]**2+currentState[2][1]**2)
        print(f'speed: {currentSpeed:.2f}, target speed: {speed:.2f}')

        # The latest target computed by plannerCallback
        return [target.x, target.y, speed]


class VehiclePerception:
    def __init__(self, role_name='ego_vehicle'):
        self.locationSub = rospy.Subscriber(
            "/carla/%s/location" % role_name, LocationInfo, self.locationCallback)
        self.obstacleSub = rospy.Subscriber(
            "/carla/%s/obstacles" % role_name, ObstacleList, self.obstacleCallback)
        self.position = None
        self.velocity = None
        self.rotation = None
        self.obstacleList = None

    def locationCallback(self, data):
        self.position = (data.location.x, data.location.y)
        self.rotation = (np.radians(data.rotation.x), np.radians(
            data.rotation.y), np.radians(data.rotation.z))
        self.velocity = (data.velocity.x, data.velocity.y)

    def obstacleCallback(self, data):
        self.obstacleList = data.obstacles


def run_model(role_name):

    rate = rospy.Rate(100)  # 100 Hz

    perceptionModule = VehiclePerception(role_name=role_name)
    decisionModule = VehicleDecision(role_name)
    controlModule = VehicleController(role_name=role_name)

    def shut_down():
        controlModule.stop()
    rospy.on_shutdown(shut_down)

    print("Starter code is running")

    while not rospy.is_shutdown():
        rate.sleep()  # Wait a while before trying to get a new state
        obstacleList = perceptionModule.obstacleList

        # Get the current position and orientation of the vehicle
        currState = (perceptionModule.position,
                     perceptionModule.rotation, perceptionModule.velocity)
        if not currState or not currState[0]:
            continue

        # Get the target state from decision module
        refState = decisionModule.get_ref_state(currState, obstacleList)
        if not refState:
            controlModule.stop()
            exit(0)

        # Execute
        controlModule.execute(currState, refState)


if __name__ == "__main__":
    roskpack = rospkg.RosPack()
    config_path = roskpack.get_path('config_node')
    race_config = open(config_path+'/'+'race_config', 'rb')
    vehicle_typeid = race_config.readline().decode('ascii').strip()
    sensing_radius = race_config.readline().decode('ascii').strip()
    role_name = 'ego_vehicle'

    rospy.init_node("voronoi_main")
    try:
        run_model(role_name)
    except rospy.exceptions.ROSInterruptException:
        print("stop")
