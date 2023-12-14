#!/usr/bin/env python

import glob
import os
import rospy
from zf_cvs_msgs.msg import AdoptOut, AdoptOutDyn, AdoptOutHitch
from  std_msgs.msg import Int32
import Adoptout
from sensor_msgs.msg import NavSatFix, Imu
from my_work.msg import ego_vehicle_parameters
import math


import sys


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import socket


global world, ego_vehicle, AdoptOut_Publisher
gnss_data = NavSatFix()
imu_data = Imu()
ego_vehicle_data = ego_vehicle_parameters()
ego_vehicle_id = None

def get_ego_vehicle_id(msg):
    global ego_vehicle, world
    ego_vehicle_id = msg.data
    ego_vehicle = world.get_actor(ego_vehicle_id)
    AdoptOut_data_dump()



def save_gnss_data(unsaved_gnss_data):
    global gnss_data
    gnss_data = unsaved_gnss_data



def save_imu_data(unsaved_imu_data):
    global imu_data
    imu_data = unsaved_imu_data



def save_ego_vehicle_data(unsaved_ego_vehicle_data):
    global ego_vehicle_data
    ego_vehicle_data = unsaved_ego_vehicle_data


def calculate_velocity_and_acceleration():
    linear_velocity = math.sqrt(ego_vehicle_data.linear_velocity.x**2 + ego_vehicle_data.linear_velocity.y**2 + ego_vehicle_data.linear_velocity.z**2)
    angular_velocity = math.sqrt(imu_data.angular_velocity.x**2 + imu_data.angular_velocity.y**2 + imu_data.angular_velocity.z**2)
    linear_acceleration = math.sqrt(imu_data.linear_acceleration.x**2 + imu_data.linear_acceleration.y**2 + imu_data.linear_acceleration.z**2)
    return linear_velocity, angular_velocity, linear_acceleration



def ego_vehicle_turning_radius():
    wheelbase = ego_vehicle.bounding_box.extent.x + ego_vehicle.bounding_box.extent.y
    vehicle_control = ego_vehicle.get_control()
    steering_angle_rad = vehicle_control.steer
    if steering_angle_rad != 0:
        turning_radius = wheelbase / math.tan(steering_angle_rad)
    else:
        turning_radius = 0
    return turning_radius



def get_yaw_rate():
    vehicle_transform = ego_vehicle.get_transform()
    current_yaw = vehicle_transform.rotation.yaw
    #euler_angles = rotation_quaternion.get_euler()
    #current_yaw = euler_angles[2]
    current_time = ego_vehicle.get_world().get_snapshot().timestamp.elapsed_seconds

    # Compute the yaw rate by taking the derivative of yaw with respect to time
    if hasattr(ego_vehicle, 'last_yaw') and hasattr(ego_vehicle, 'last_time'):
        yaw_rate = (current_yaw - ego_vehicle.last_yaw) / (current_time - ego_vehicle.last_time)
    else:
        yaw_rate = 0.0

    # Update the last_yaw and last_time for the next iteration
    ego_vehicle.last_yaw = current_yaw
    ego_vehicle.last_time = current_time

    return yaw_rate



def AdoptOut_data_dump():
    global gnss_data, imu_data, ego_vehicle_data, AdoptOut_Publisher

    adoptOut = Adoptout.TruckSim()

    adoptOut.header.stamp = gnss_data.header.stamp
    adoptOut.header.frame_id = str(gnss_data.header.frame_id)
    adoptOut.header.seq += 1

    ego_vehicle_linear_velocity, ego_vehicle_angular_velocity, ego_vehicle_linear_acceleration = calculate_velocity_and_acceleration()

    if ego_vehicle:
        adoptOut.adoptSt = AdoptOut.STATE_ACTIVE 
    else:
        adoptOut.adoptSt = AdoptOut.STATE_TEMP_NOT_AVAILABLE

    if ego_vehicle_linear_velocity != 0 and not ego_vehicle_data.reverse:
        adoptOut.actDtSt = AdoptOut.DT_STATE_DRIVE_FORWARD
    elif ego_vehicle_linear_velocity != 0 and ego_vehicle_data.reverse:
        adoptOut.actDtSt = AdoptOut.DT_STATE_DRIVE_REVERSE
    else:
        adoptOut.actDtSt = AdoptOut.DT_STATE_HOLD

    adoptOut.actCtrlMod = AdoptOut.CTRL_PAH_AND_VEL_FOL_MOD

    adoptOut.actCtrlSrc = 1

    adoptOut.estimdWghtMin = 8000
    adoptOut.estimdWghtMax = 10000

    adoptOut.actSlopAg = 0

    adopt_dyn = AdoptOutDyn()

    adopt_dyn.actLgtV = ego_vehicle_data.linear_velocity.x
    adopt_dyn.actLgtA = imu_data.linear_acceleration.x

    turning_radius = ego_vehicle_turning_radius() 
    if (ego_vehicle_data.reverse and turning_radius >0):
        adopt_dyn.actCrvt = turning_radius
    if (not ego_vehicle_data.reverse and turning_radius <0):
        adopt_dyn.actCrvt = -(turning_radius)  
    if (ego_vehicle_data.reverse and turning_radius <0):
        adopt_dyn.actCrvt = turning_radius
    if (not ego_vehicle_data.reverse and turning_radius >0):
        adopt_dyn.actCrvt = -(turning_radius)
    
    if (ego_vehicle_data.brake or ego_vehicle_data.hand_brake) and ego_vehicle_linear_velocity == 0:
        adopt_dyn.actMovgSt = AdoptOutDyn.STATE_SAFE_STANDSTILL
    elif (not ego_vehicle_data.brake or not ego_vehicle_data.hand_brake) and ego_vehicle_linear_velocity == 0:
        adopt_dyn.actMovgSt = AdoptOutDyn.STATE_MOVING_STATE_TRANSITION
    elif ego_vehicle_linear_velocity != 0:
        adopt_dyn.actMovgSt = AdoptOutDyn.STATE_MOVING
    else:
        adopt_dyn.actMovgSt = AdoptOutDyn.STATE_NA

    adopt_dyn.stabyCtrlSt = AdoptOutDyn.STABILITY_CONTROL_STANDBY

    adopt_dyn.actYawRate = get_yaw_rate()

    if turning_radius > 0:
        adopt_dyn.actLatA = -(imu_data.linear_acceleration.y)
    elif turning_radius < 0:
        adopt_dyn.actLatA = imu_data.linear_acceleration.y
    else:
        adopt_dyn.actLatA = 0

    adoptOut.AdoptOutDyn[:1] = [adopt_dyn]

    adopt_hitch = AdoptOutHitch()
    adopt_lim = Adoptout.AdoptOutLim()

    adoptOut.AdoptOutHitch[:1] = [adopt_hitch]
    adoptOut.lim = adopt_lim

    AdoptOut_Publisher.publish(adoptOut)


    




def get_ip(host):
    if host in ['localhost', '127.0.0.1']:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.connect(('10.255.255.255', 1))
            host = sock.getsockname()[0]
        except RuntimeError:
            pass
        finally:
            sock.close()
    return host



def main():
    global world, ego_vehicle, gnss_data, imu_data, ego_vehicle_data, AdoptOut_Publisher

    # First of all, we need to create the client that will send the requests, assume port is 2000
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)

    # Retrieve the world that is currently running
    world = client.get_world()

    # Initialize ROS node
    rospy.init_node('carla_ego_vehicle_id_subscriber', anonymous=True)
    print("Node initialized")
    AdoptOut_Publisher = rospy.Publisher('/carla/ego_vehicle/AdoptOut',AdoptOut, queue_size=10)

    try:
        

        rospy.Subscriber('/carla/ego_vehicle/gnss', NavSatFix, save_gnss_data)

        rospy.Subscriber('/carla/ego_vehicle/imu', Imu, save_imu_data)

        rospy.Subscriber('/carla/ego_vehicle/parameters',ego_vehicle_parameters, save_ego_vehicle_data)

        rospy.Subscriber('/carla/ego_vehicle/id', Int32, get_ego_vehicle_id)



        rospy.spin()



    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except RuntimeError as e:
        print(e)
    finally:
        print('Finished')



if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    except RuntimeError as e:
        print(e)