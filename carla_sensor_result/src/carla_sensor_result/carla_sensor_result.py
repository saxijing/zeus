#!/usr/bin/env python
# This ros package is designed to query info from carla service.
# Because of the simulator, following outputs are based on vehicle bounding box center, not mass center or rear axies center
#
# Ouput (Once): ego_vehicle dynamics parameters
# 1. four wheels' position and mass center pos to calculate a and b;
# 2. lateral stiffness of front and rear wheel Cf, Cr;
# 3. mass of vehicle, m;
# 4. moment of inertia, I;
#
# Ouput (Publish):
# 1. ego_vehicle status: position(x,y), velocity(x,y), acceleration(x,y), heading angle, yawrate, and calculate theta_v
# 2. roadedge and objects surrounding ego vehicle-- persudo sensor output

import math
import carla
import rospy
import time
import numpy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from carla_msgs.msg import CarlaEgoVehicleState
from cv_bridge import CvBridge
# import ros_compatibility as roscomp
# from ros_compatibility.node import CompatibleNode
# from ros_compatibility.qos import QoSProfile, DurabilityPolicy

class VehicleStateAndPerception():
    """
    publish 
    """
    def __init__(self, carla_world):
        self.ego_player=None
        self.cam_sensor=None
        self.world=carla_world
        self.im_width=800
        self.im_height=600
        self.ego_pos_and_state_publisher = rospy.Publisher(
                'zeus/ego_vehicle_state',
                CarlaEgoVehicleState,
                queue_size=10
        )
        self.ego_vehicle_cube_publisher = rospy.Publisher(
                '/rviz/ego_vehicle_state_cube',
                Marker,
                queue_size=10
        )
        self.ego_vehicle_arrow_publisher = rospy.Publisher(
                '/rviz/ego_vehicle_state_arrow',
                Marker,
                queue_size=10
        )
        self.cam_view_publisher = rospy.Publisher(
                '/rviz/camera_view',
                Image,
                queue_size=5
        )
        self.carla_ego_vehicle_state_msg=CarlaEgoVehicleState()
        self.ego_vehicle_cube_msg=Marker()
        self.ego_vehicle_arrow_msg=Marker()
        self.arrow_start=Point()
        self.arrow_end=Point()
        self.camera_msg=Image()
        self.camera_info_msg=CameraInfo()
        self.cv_bridge=CvBridge()
        
    def serarch_for_ego_vehicle(self):
        while self.ego_player is None:
            print("Searching for the ego vehicle...")
            time.sleep(1)
            possible_vehicles = self.world.get_actors().filter('vehicle.*')
            # print(possible_vehicles)
            actor_role_name="ego_vehicle"
            # print(actor_role_name)
            for vehicle in possible_vehicles:
                print(vehicle.attributes['role_name'])
                if vehicle.attributes['role_name'] == actor_role_name:
                    print("Ego vehicle got")
                    self.ego_player = vehicle
                    break
        return self.ego_player

    def degree_unify(self, degree):
        while(degree<0 or degree>=360):
            while(degree<0):
                degree=degree+360
            while(degree>=360):
                degree=degree-360
        return degree
    
    def radius_unify(self, rad):
        while(rad<0 or rad>=2*math.pi):
            while(rad<0):
                rad=rad+2*math.pi
            while(rad>=2*math.pi):
                rad=rad-2*math.pi
        return rad
    
    def callback_rgb_camera(self, data):
        ## refer to carla_ros_bridge/camera.py 'bgra8'
        carla_image_data_array = numpy.ndarray(
            shape=(data.height, data.width, 4),
            dtype=numpy.uint8, buffer=data.raw_data)
        encoding='bgra8'
        self.camera_msg=self.cv_bridge.cv2_to_imgmsg(carla_image_data_array, encoding=encoding)
        self.camera_msg.header.stamp=rospy.Time.now()
        self.camera_msg.header.frame_id="world"

    def catch_ego_vehicle_dynamics_para(self):
        ## set camera
        cam=self.world.get_blueprint_library().find('sensor.camera.rgb')
        cam.set_attribute('image_size_x',f'{self.im_width}')
        cam.set_attribute('image_size_y',f'{self.im_height}')
        cam.set_attribute('fov',f'110')
        cam.set_attribute('role_name','rgb_view')
        cam_transform=carla.Transform(carla.Location(x=-3.5,y=0,z=2.5), carla.Rotation(roll=0, pitch=-30,yaw=0))
        self.cam_sensor=self.world.spawn_actor(cam, cam_transform, attach_to=self.ego_player)
        self.cam_sensor.listen(lambda data: self.callback_rgb_camera(data))
        ## get dynamic para
        ego_pos=self.ego_player.get_transform()
        center_location=self.ego_player.bounding_box
        mass_center_location=self.ego_player.get_physics_control().center_of_mass
        mass=self.ego_player.get_physics_control().mass
        Im=self.ego_player.get_physics_control().moi
        FL_wheel_location=self.ego_player.get_physics_control().wheels[0].position
        FR_wheel_location=self.ego_player.get_physics_control().wheels[1].position
        RL_wheel_location=self.ego_player.get_physics_control().wheels[2].position
        RR_wheel_location=self.ego_player.get_physics_control().wheels[3].position
        FL_wheel_max_angle=self.ego_player.get_physics_control().wheels[0].max_steer_angle
        FR_wheel_max_angle=self.ego_player.get_physics_control().wheels[1].max_steer_angle
        RL_wheel_max_angle=self.ego_player.get_physics_control().wheels[2].max_steer_angle
        RR_wheel_max_angle=self.ego_player.get_physics_control().wheels[3].max_steer_angle
        Cf=self.ego_player.get_physics_control().wheels[0].lat_stiff_value
        Cr=self.ego_player.get_physics_control().wheels[1].lat_stiff_value
        lat_stiff_max_load=self.ego_player.get_physics_control().wheels[1].lat_stiff_max_load

        print("vehicle position: ", ego_pos)
        print("\n bounding box center location: ", center_location)
        print("\n ##########################")
        print("mass center location: ", mass_center_location)
        print("\n ##########################")
        print("vehicle mass=", mass,"\n")
        print("Inertial moment=", Im,"\n")
        print("Four wheels location :\n")
        print("front left: ",FL_wheel_location)
        print("\nfront right: ", FR_wheel_location)
        print("\nrear left: ", RL_wheel_location)
        print("\nrear right: ", RR_wheel_location)
        print("\n ##########################")
        print("Four wheels max steer angle: \n")
        print("front left: ",FL_wheel_max_angle)
        print("\nfront right: ",FR_wheel_max_angle)
        print("\n rear left: ",RL_wheel_max_angle)
        print("\n rear right: ",RR_wheel_max_angle)

        print("\nCf=",Cf)
        print("\nCr=",Cr)
        print("\nlat_stiff_max_load=",lat_stiff_max_load)

    def update_ego_vehicle_state(self):
        ## get ego vehicle pos, vel, and accel, AND left coord trans to right coord
        pos_x=self.ego_player.get_transform().location.x
        pos_y=-self.ego_player.get_transform().location.y
        pos_hdg=self.degree_unify(self.ego_player.get_transform().rotation.yaw)  # degrees
        pos_hdg=360-pos_hdg
        pos_hdg_rad=pos_hdg*math.pi/180
        vel_x=self.ego_player.get_velocity().x
        vel_y=-self.ego_player.get_velocity().y
        accel_x=self.ego_player.get_acceleration().x
        accel_y=-self.ego_player.get_acceleration().y
        yaw_rate=(self.ego_player.get_angular_velocity().z)*math.pi/180 #rad/s

        ## set the spectator
        spectator_dist=-10.0
        spectator_transform=carla.Transform(self.ego_player.get_transform().location+carla.Location(x=spectator_dist*math.cos(-pos_hdg_rad), y=spectator_dist*math.sin(-pos_hdg_rad),z=3),
                self.ego_player.get_transform().rotation)
        self.world.get_spectator().set_transform(spectator_transform)
        # print("Ego Vehicle Location: [ %.3f m, %.3f m, %.3f degree.\n ", pos_x, pos_y, pos_hdg)

        ## fulfill carla-ros msg
        self.carla_ego_vehicle_state_msg.header.stamp=rospy.Time.now()
        self.carla_ego_vehicle_state_msg.header.frame_id="/zeus"
        #   transport to the center of mass
        self.carla_ego_vehicle_state_msg.pose.x=pos_x
        self.carla_ego_vehicle_state_msg.pose.y=pos_y
        self.carla_ego_vehicle_state_msg.pose.theta=pos_hdg_rad #fai

        self.carla_ego_vehicle_state_msg.vel.x=vel_x
        self.carla_ego_vehicle_state_msg.vel.y=vel_y
        self.carla_ego_vehicle_state_msg.vel.theta=yaw_rate #yaw rate, fai rate

        self.carla_ego_vehicle_state_msg.accel.x=accel_x
        self.carla_ego_vehicle_state_msg.accel.y=accel_y
        self.carla_ego_vehicle_state_msg.accel.z=0
        
        #velocity_angle
        vel=math.sqrt(vel_x*vel_x+vel_y*vel_y)
        if vel<1e-6:
            self.carla_ego_vehicle_state_msg.theta=0
        elif abs(vel_x)<1e-6:
            if vel_y>1e-6:
                self.carla_ego_vehicle_state_msg.theta=0.5*math.pi
            if vel_y<-1e-6:
                self.carla_ego_vehicle_state_msg.theta=1.5*math.pi
        else:
            self.carla_ego_vehicle_state_msg.theta=math.atan2(vel_y,vel_x)
        self.carla_ego_vehicle_state_msg.theta=self.radius_unify(self.carla_ego_vehicle_state_msg.theta)
        
        ## fulfill rviz ros msg for ego vehicle cube
        self.ego_vehicle_cube_msg.header.stamp=rospy.Time.now()
        self.ego_vehicle_cube_msg.header.frame_id="world"
        self.ego_vehicle_cube_msg.id=1
        self.ego_vehicle_cube_msg.type= Marker.CUBE
        self.ego_vehicle_cube_msg.pose.position.x=pos_x
        self.ego_vehicle_cube_msg.pose.position.y=pos_y
        self.ego_vehicle_cube_msg.pose.position.z=0.0
        self.ego_vehicle_cube_msg.pose.orientation.w=math.cos(pos_hdg_rad/2)
        self.ego_vehicle_cube_msg.pose.orientation.x=0.0
        self.ego_vehicle_cube_msg.pose.orientation.y=0.0
        self.ego_vehicle_cube_msg.pose.orientation.z=math.sin(pos_hdg_rad/2)
        self.ego_vehicle_cube_msg.scale.x=4.75
        self.ego_vehicle_cube_msg.scale.y=2.13
        self.ego_vehicle_cube_msg.scale.z=1.625
        self.ego_vehicle_cube_msg.color.g=1.0
        self.ego_vehicle_cube_msg.color.a=0.5

        ## fulfill rviz ros msg for ego vehicle arrow
        self.ego_vehicle_arrow_msg.header.stamp=rospy.Time.now()
        self.ego_vehicle_arrow_msg.header.frame_id="world"
        self.ego_vehicle_arrow_msg.id=0
        self.ego_vehicle_arrow_msg.type= Marker.ARROW
        self.ego_vehicle_arrow_msg.scale.x=0.05
        self.ego_vehicle_arrow_msg.scale.y=0.25
        self.ego_vehicle_arrow_msg.scale.z=0.5
        self.ego_vehicle_arrow_msg.color.r=1.0
        self.ego_vehicle_arrow_msg.color.a=1.0
        self.ego_vehicle_arrow_msg.pose.orientation.w=1.0
        self.ego_vehicle_arrow_msg.pose.orientation.x=0.0
        self.ego_vehicle_arrow_msg.pose.orientation.y=0.0
        self.ego_vehicle_arrow_msg.pose.orientation.z=0.0
        self.arrow_start.x=pos_x#+(4.75/2)*math.cos(pos_hdg)
        self.arrow_start.y=pos_y#+(4.75/2)*math.sin(pos_hdg)
        self.arrow_start.z=0
        self.arrow_end.x=pos_x+(6.75/2)*math.cos(pos_hdg_rad)
        self.arrow_end.y=pos_y+(6.75/2)*math.sin(pos_hdg_rad)
        self.arrow_end.z=0
        self.ego_vehicle_arrow_msg.points.clear()
        self.ego_vehicle_arrow_msg.points.append(self.arrow_start)
        self.ego_vehicle_arrow_msg.points.append(self.arrow_end)

##########################
def main(args=None):
    client = carla.Client('localhost',2000)
    client.set_timeout(10.0)
    carla_world=client.get_world()
    print(carla_world.get_actors().filter('vehicle.*'))
    rospy.init_node("carla_vehicle_state_and_perception" )
    rate=rospy.Rate(100)
    vehicle_state_and_perception = VehicleStateAndPerception(carla_world)
    ego_player=vehicle_state_and_perception.serarch_for_ego_vehicle()
    vehicle_state_and_perception.catch_ego_vehicle_dynamics_para()
    while not rospy.is_shutdown():
        vehicle_state_and_perception.update_ego_vehicle_state()
        vehicle_state_and_perception.ego_pos_and_state_publisher.publish(vehicle_state_and_perception.carla_ego_vehicle_state_msg)
        vehicle_state_and_perception.ego_vehicle_cube_publisher.publish(vehicle_state_and_perception.ego_vehicle_cube_msg)
        vehicle_state_and_perception.ego_vehicle_arrow_publisher.publish(vehicle_state_and_perception.ego_vehicle_arrow_msg)
        vehicle_state_and_perception.cam_view_publisher.publish(vehicle_state_and_perception.camera_msg)
        rate.sleep()

if __name__ == '__main__':
    main()
