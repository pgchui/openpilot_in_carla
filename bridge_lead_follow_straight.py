#!/usr/bin/env python3

import argparse
import carla # pylint: disable=import-error
import math
import numpy as np
import time
import threading
from cereal import log
from multiprocessing import Process, Queue
from typing import Any
import random
import math

import cereal.messaging as messaging
from common.params import Params
from common.numpy_fast import clip
from common.realtime import Ratekeeper, DT_DMON
from lib.can import can_function
from selfdrive.car.honda.values import CruiseButtons
from selfdrive.controls.lib.cluster.fastcluster_py import cluster_points_centroid
from selfdrive.test.helpers import set_params_enabled

from sklearn.cluster import DBSCAN
import matplotlib.pyplot as plt

from sdq_tools import deltacovering_quant, operational_state_space
import os 
import pickle

parser = argparse.ArgumentParser(description='Bridge between CARLA and openpilot.')
parser.add_argument('--joystick', action='store_true')
parser.add_argument('--low_quality', action='store_true')

args = parser.parse_args()

W, H = 1164, 874
REPEAT_COUNTER = 5
PRINT_DECIMATION = 100
DATA_SAMPLE_DECIMATION = 10
STEER_RATIO = 15.

TIME_STEPS = 700 # 10 seconds - 100Hz * 10

pm = messaging.PubMaster(['roadCameraState', 'sensorEvents', 'can', "gpsLocationExternal"])
sm = messaging.SubMaster(['carControl', 'controlsState'])

MAX_BRAKING_ACCELERATION = -8.0  # m/s^2
TIME_STEP = 0.01 # second
TIME_DURATION = 6000 # steps
MPH_TO_MS_FACTOR = 2.2369362920544
INITIAL_SPEED = 25  # mph

def mph_to_ms(mph):
    return mph / MPH_TO_MS_FACTOR

class VehicleState:
  def __init__(self):
    self.speed = 0
    self.angle = 0
    self.bearing_deg = 0.0
    self.vel = carla.Vector3D()
    self.cruise_button= 0
    self.is_engaged=False

def steer_rate_limit(old, new):
  # Rate limiting to 0.5 degrees per step
  limit = 0.5
  if new > old + limit:
    return old + limit
  elif new < old - limit:
    return old - limit
  else:
    return new

frame_id = 0
def cam_callback(image):
  global frame_id
  img = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
  img = np.reshape(img, (H, W, 4))
  img = img[:, :, [0, 1, 2]].copy()

  dat = messaging.new_message('roadCameraState')
  dat.roadCameraState = {
    "frameId": image.frame,
    "image": img.tobytes(),
    "transform": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
  }
  pm.send('roadCameraState', dat)
  frame_id += 1

# resource: selfdrive/controls/radard.py search publish radarState
radar_points = np.empty((0, 4), float)
def radar_callback(radar_data):
  global radar_points
  for detect in radar_data:
    detect_array = np.array([[detect.altitude, detect.azimuth, detect.depth, detect.velocity]])
    radar_points = np.vstack([radar_points, detect_array])

def imu_callback(imu, vehicle_state):
  vehicle_state.bearing_deg = math.degrees(imu.compass)
  dat = messaging.new_message('sensorEvents', 2)
  dat.sensorEvents[0].sensor = 4
  dat.sensorEvents[0].type = 0x10
  dat.sensorEvents[0].init('acceleration')
  dat.sensorEvents[0].acceleration.v = [imu.accelerometer.x, imu.accelerometer.y, imu.accelerometer.z]
  # copied these numbers from locationd
  dat.sensorEvents[1].sensor = 5
  dat.sensorEvents[1].type = 0x10
  dat.sensorEvents[1].init('gyroUncalibrated')
  dat.sensorEvents[1].gyroUncalibrated.v = [imu.gyroscope.x, imu.gyroscope.y, imu.gyroscope.z]
  pm.send('sensorEvents', dat)

def panda_state_function(exit_event: threading.Event):
  pm = messaging.PubMaster(['pandaState'])
  while not exit_event.is_set():
    dat = messaging.new_message('pandaState')
    dat.valid = True
    dat.pandaState = {
      'ignitionLine': True,
      'pandaType': "blackPanda",
      'controlsAllowed': True,
      'safetyModel': 'hondaNidec'
    }
    pm.send('pandaState', dat)
    time.sleep(0.5)

def peripheral_state_function(exit_event: threading.Event):
  pm = messaging.PubMaster(['peripheralState'])
  while not exit_event.is_set():
    dat = messaging.new_message('peripheralState')
    dat.valid = True
    # fake peripheral state data
    dat.peripheralState = {
      'pandaType': log.PandaState.PandaType.blackPanda,
      'voltage': 12000,
      'current': 5678,
      'fanSpeedRpm': 1000
    }
    pm.send('peripheralState', dat)
    time.sleep(0.5)

def gps_callback(gps, vehicle_state):
  dat = messaging.new_message('gpsLocationExternal')

  # transform vel from carla to NED
  # north is -Y in CARLA
  velNED = [
    -vehicle_state.vel.y, # north/south component of NED is negative when moving south
    vehicle_state.vel.x, # positive when moving east, which is x in carla
    vehicle_state.vel.z,
  ]

  dat.gpsLocationExternal = {
    "timestamp": int(time.time() * 1000),
    "flags": 1, # valid fix
    "accuracy": 1.0,
    "verticalAccuracy": 1.0,
    "speedAccuracy": 0.1,
    "bearingAccuracyDeg": 0.1,
    "vNED": velNED,
    "bearingDeg": vehicle_state.bearing_deg,
    "latitude": gps.latitude,
    "longitude": gps.longitude,
    "altitude": gps.altitude,
    "speed": vehicle_state.speed,
    "source": log.GpsLocationData.SensorSource.ublox,
  }

  pm.send('gpsLocationExternal', dat)

def fake_driver_monitoring(exit_event: threading.Event):
  pm = messaging.PubMaster(['driverState','driverMonitoringState'])
  while not exit_event.is_set():
    # dmonitoringmodeld output
    dat = messaging.new_message('driverState')
    dat.driverState.faceProb = 1.0
    pm.send('driverState', dat)

    # dmonitoringd output
    dat = messaging.new_message('driverMonitoringState')
    dat.driverMonitoringState = {
      "faceDetected": True,
      "isDistracted": False,
      "awarenessStatus": 1.,
    }
    pm.send('driverMonitoringState', dat)

    time.sleep(DT_DMON)

def can_function_runner(vs: VehicleState, exit_event: threading.Event):
  global radar_points
  i = 1
  while not exit_event.is_set():
    if i % 5 != 0 or radar_points.shape[0] == 0:
      can_function(pm, vs.speed, vs.angle, i, vs.cruise_button, vs.is_engaged, None)
    else:
      # process radar points
      radar_points_clustering = DBSCAN(eps=0.5, min_samples=5).fit(radar_points / [math.radians(10), math.radians(17.5), 256.0, 35.0]) # normalize data points
      radar_points_clustering_centroids = np.zeros((16, 4), float)
      radar_points_clustering_label_counts = np.zeros((16, 1), int)
      # sum all tracks
      for idx, track_id in enumerate(radar_points_clustering.labels_):
        if track_id != -1 and track_id < 16:
          radar_points_clustering_centroids[track_id, :] += radar_points[idx, :]
          radar_points_clustering_label_counts[track_id] += 1
      # average all tracks to get centroids
      for idx, radar_point in enumerate(radar_points_clustering_centroids):
        if radar_points_clustering_label_counts[idx] != 0:
          radar_points_clustering_centroids[idx] = radar_point / radar_points_clustering_label_counts[idx]
      # calculate longitudinal_dist, lateral_dist, and relative_velocity
      radar_can_message = np.zeros((16, 3), float)
      for idx, radar_point_centroid in enumerate(radar_points_clustering_centroids):
        if radar_points_clustering_label_counts[idx] == 0:
          radar_can_message[idx, :] = np.array([[255.5, 0.0, 0.0]])
        else:
          radar_can_message[idx, 0] = math.cos(radar_point_centroid[0]) * math.cos(radar_point_centroid[1]) * radar_point_centroid[2] # radar_longitudinal_distance_offset # longitudinal distance 
          radar_can_message[idx, 1] = math.cos(radar_point_centroid[0]) * math.sin(radar_point_centroid[1]) * radar_point_centroid[2] # lateral distance
          radar_can_message[idx, 2] = radar_point_centroid[3] # relative velocity
      can_function(pm, vs.speed, vs.angle, i, vs.cruise_button, vs.is_engaged, radar_can_message)
      radar_points = np.empty((0, 4), float)
  
    time.sleep(0.01)
    i+=1

lead_vehicle_speed = 50 # mph
lead_vehicle_dist = 50 # m
ms_to_mph = 2.23694
mph_to_ms = 0.44704
radar_longitudinal_distance_offset = -2.0

collision_flag = False
def collision_callback(event):
  global collision_flag
  collision_flag = True


def bridge(q, world, initial_waypoint, lead_distance, sv_initial_v, lv_initial_v):
  if math.isclose(sv_initial_v, 0.0):
    return [lead_distance], [sv_initial_v], [lv_initial_v], False, True

  blueprint_library = world.get_blueprint_library()
  vehicle_bp = blueprint_library.filter('vehicle.tesla.*')[1]
  sv_initial_waypoint = initial_waypoint
  sv_spawn_point = sv_initial_waypoint.transform
  dummy_spawn_points = world.get_map().get_spawn_points()
  lead_vehicle = []
  lead_vehicle = world.spawn_actor(vehicle_bp, dummy_spawn_points[1])
  vehicle = []
  # spawn subject vechile and avoid collision with road
  attempt_count = 0
  while True:
    try:
      attempt_count += 1
      sv_spawn_point.location.z += 0.01
      vehicle = world.spawn_actor(vehicle_bp, sv_spawn_point)
      break
    except RuntimeError:
      pass

  lv_initial_waypoint = sv_initial_waypoint.next(lead_distance + vehicle.bounding_box.extent.x + lead_vehicle.bounding_box.extent.x + sv_initial_v*2)[0]
  lv_spawn_point = lv_initial_waypoint.transform
  lead_vehicle.set_transform(lv_spawn_point)
  
  max_steer_angle = vehicle.get_physics_control().wheels[0].max_steer_angle

  physics_control = vehicle.get_physics_control()
  physics_control.mass = 2326
  physics_control.torque_curve = [[20.0, 500.0], [5000.0, 500.0]]
  physics_control.gear_switch_time = 0.0
  vehicle.apply_physics_control(physics_control)

  blueprint = blueprint_library.find('sensor.camera.rgb')
  blueprint.set_attribute('image_size_x', str(W))
  blueprint.set_attribute('image_size_y', str(H))
  blueprint.set_attribute('fov', '70')
  blueprint.set_attribute('sensor_tick', '0.05')
  transform = carla.Transform(carla.Location(x=0.8, z=1.13))
  camera = world.spawn_actor(blueprint, transform, attach_to=vehicle)
  camera.listen(cam_callback)

  vehicle_state = VehicleState()

  # reenable IMU
  imu_bp = blueprint_library.find('sensor.other.imu')
  imu = world.spawn_actor(imu_bp, transform, attach_to=vehicle)
  imu.listen(lambda imu: imu_callback(imu, vehicle_state))

  gps_bp = blueprint_library.find('sensor.other.gnss')
  gps = world.spawn_actor(gps_bp, transform, attach_to=vehicle)
  gps.listen(lambda gps: gps_callback(gps, vehicle_state))

  # add radar (reference: https://carla.readthedocs.io/en/latest/tuto_G_retrieve_data/#radar-sensor)
  radar_bp = blueprint_library.find('sensor.other.radar')
  radar_bp.set_attribute('horizontal_fov', str(35))
  radar_bp.set_attribute('range', str(256))
  radar_location = carla.Location(x=vehicle.bounding_box.extent.x, z=1.0)
  radar_rotation = carla.Rotation()
  radar_transform = carla.Transform(radar_location, radar_rotation)
  radar = world.spawn_actor(radar_bp, radar_transform, attach_to=vehicle)
  radar.listen(lambda radar_data: radar_callback(radar_data))

  collision_sensor_bp = blueprint_library.find('sensor.other.collision')
  collision_sensor = world.spawn_actor(collision_sensor_bp, carla.Transform(), attach_to=vehicle)
  collision_sensor.listen(lambda event: collision_callback(event))

  # launch fake car threads
  threads = []
  exit_event = threading.Event()
  threads.append(threading.Thread(target=panda_state_function, args=(exit_event,)))
  threads.append(threading.Thread(target=peripheral_state_function, args=(exit_event,)))
  threads.append(threading.Thread(target=fake_driver_monitoring, args=(exit_event,)))
  threads.append(threading.Thread(target=can_function_runner, args=(vehicle_state, exit_event,)))
  for t in threads:
    t.start()

  # can loop
  rk = Ratekeeper(100, print_delay_threshold=0.05)

  # init
  throttle_ease_out_counter = REPEAT_COUNTER
  brake_ease_out_counter = REPEAT_COUNTER
  steer_ease_out_counter = REPEAT_COUNTER


  vc = carla.VehicleControl(throttle=0, steer=0, brake=0, reverse=False)

  is_openpilot_engaged = False
  throttle_out = steer_out = brake_out = 0
  throttle_op = steer_op = brake_op = 0
  throttle_manual = steer_manual = brake_manual = 0

  old_steer = old_brake = old_throttle = 0
  throttle_manual_multiplier = 0.7 #keyboard signal is always 1
  brake_manual_multiplier = 0.7 #keyboard signal is always 1
  steer_manual_multiplier = 45 * STEER_RATIO  #keyboard signal is always 1

  # add delay to wait for op to be fully started
  auto_start_counter = 0
  auto_start_threshold = 50
  auto_start_finished = False
  lead_vehicle_added = False
  lead_vehicle_stopped = False
  openpilot_enable_check = 0
  openpilot_enabled = True
  openpilot_retry_flag = False

  lv_v_prev = lv_initial_v
  lv_a_brake = -1.0
  lv_waypoint_cur = lv_initial_waypoint

  global collision_flag

  sv_v = [sv_initial_v] # subject vehicle velocity list; m/s
  lv_v = [lv_initial_v] # lead vehicle velocity list; m/s
  dhw = [lead_distance]  # distance headway; m

  for i in range(TIME_STEPS + auto_start_threshold):
    # 1. Read the throttle, steer and brake from op or manual controls
    # 2. Set instructions in Carla
    # 3. Send current carstate to op via can
      
    if collision_flag:
      break

    cruise_button = 0
    throttle_out = steer_out = brake_out = 0.0
    throttle_op = steer_op = brake_op = 0
    throttle_manual = steer_manual = brake_manual = 0.0

    # calucalate distance headway
    sv_front_bumper_location = vehicle.get_transform().transform(carla.Location(x=vehicle.bounding_box.extent.x, y=0.0, z=0.0))
    lv_rear_bumper_location = lead_vehicle.get_transform().transform(carla.Location(x=-lead_vehicle.bounding_box.extent.x, y=0.0, z=0.0))
    d = math.sqrt((sv_front_bumper_location.x - lv_rear_bumper_location.x)**2 + (sv_front_bumper_location.y - lv_rear_bumper_location.y)**2 + (sv_front_bumper_location.z - lv_rear_bumper_location.z)**2)

    # add delay to wait for op to be fully started
    if auto_start_counter < auto_start_threshold and not auto_start_finished:
      auto_start_counter += 1
      vehicle.enable_constant_velocity(carla.Vector3D(sv_initial_v, 0, 0))
    elif not auto_start_finished:
      auto_start_finished = True
    elif auto_start_finished and not lead_vehicle_added and d <= lead_distance:
      lead_vehicle_added = True
      cruise_button = CruiseButtons.DECEL_SET
      is_openpilot_engaged = True
      vehicle.disable_constant_velocity()
    elif lead_vehicle_added and not lead_vehicle_stopped and math.isclose(vc.throttle, 0.0) and math.isclose(vc.brake, 0.0):
      openpilot_enabled = False
      break
    elif lead_vehicle_added and not lead_vehicle_stopped:
      openpilot_enabled = True
      # constant speed moving
      if lv_v_prev != 0:
        lv_d_dist = lv_v_prev * TIME_STEP  
        lv_waypoint_cur = lv_waypoint_cur.next(lv_d_dist)[0]
        lead_vehicle.set_transform(lv_waypoint_cur.transform)

      if math.isclose(vehicle_state.speed, 0.0):
        break

      sv_v.append(vehicle_state.speed)
      lv_v.append(lv_v_prev)
      dhw.append(d)

    # --------------Step 1-------------------------------
    if not q.empty():
      message = q.get()
      m = message.split('_')
      if m[0] == "steer":
        steer_manual = float(m[1])
        is_openpilot_engaged = False
      elif m[0] == "throttle":
        throttle_manual = float(m[1])
        is_openpilot_engaged = False
      elif m[0] == "brake":
        brake_manual = float(m[1])
        is_openpilot_engaged = False
      elif m[0] == "reverse":
        #in_reverse = not in_reverse
        cruise_button = CruiseButtons.CANCEL
        is_openpilot_engaged = False
      elif m[0] == "cruise":
        if m[1] == "down":
          cruise_button = CruiseButtons.DECEL_SET
          is_openpilot_engaged = True
        elif m[1] == "up":
          cruise_button = CruiseButtons.RES_ACCEL
          is_openpilot_engaged = True
        elif m[1] == "cancel":
          cruise_button = CruiseButtons.CANCEL
          is_openpilot_engaged = False
      elif m[0] == "quit":
        break

      throttle_out = throttle_manual * throttle_manual_multiplier
      steer_out = steer_manual * steer_manual_multiplier
      brake_out = brake_manual * brake_manual_multiplier

      old_steer = steer_out
      old_throttle = throttle_out
      old_brake = brake_out

    if is_openpilot_engaged:
      sm.update(0)
      throttle_op = clip(sm['carControl'].actuators.accel/1.6, 0.0, 1.0)
      brake_op = clip(-sm['carControl'].actuators.accel/4.0, 0.0, 1.0)
      steer_op = sm['carControl'].actuators.steeringAngleDeg

      throttle_out = throttle_op
      steer_out = steer_op
      brake_out = brake_op

      steer_out = steer_rate_limit(old_steer, steer_out)
      old_steer = steer_out

    else:
      if throttle_out==0 and old_throttle>0:
        if throttle_ease_out_counter>0:
          throttle_out = old_throttle
          throttle_ease_out_counter += -1
        else:
          throttle_ease_out_counter = REPEAT_COUNTER
          old_throttle = 0

      if brake_out==0 and old_brake>0:
        if brake_ease_out_counter>0:
          brake_out = old_brake
          brake_ease_out_counter += -1
        else:
          brake_ease_out_counter = REPEAT_COUNTER
          old_brake = 0

      if steer_out==0 and old_steer!=0:
        if steer_ease_out_counter>0:
          steer_out = old_steer
          steer_ease_out_counter += -1
        else:
          steer_ease_out_counter = REPEAT_COUNTER
          old_steer = 0

    # --------------Step 2-------------------------------

    steer_carla = steer_out / (max_steer_angle * STEER_RATIO * -1)

    steer_carla = np.clip(steer_carla, -1,1)
    steer_out = steer_carla * (max_steer_angle * STEER_RATIO * -1)
    old_steer = steer_carla * (max_steer_angle * STEER_RATIO * -1)

    vc.throttle = throttle_out/0.6
    vc.steer = steer_carla
    vc.brake = brake_out
    vehicle.apply_control(vc)

    # --------------Step 3-------------------------------
    vel = vehicle.get_velocity()
    speed = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2) # in m/s
    vehicle_state.speed = speed
    vehicle_state.vel = vel
    vehicle_state.angle = steer_out
    vehicle_state.cruise_button = cruise_button
    vehicle_state.is_engaged = is_openpilot_engaged

    rk.keep_time()

  # Clean up resources in the opposite order they were created.
  exit_event.set()
  for t in reversed(threads):
    t.join()
  lead_vehicle.destroy()
  collision_sensor.destroy()
  radar.destroy()
  gps.destroy()
  imu.destroy()
  camera.destroy()
  vehicle.destroy()

  return sv_v, lv_v, dhw, collision_flag, openpilot_enabled

def filter_waypoint(world):
  waypoints = world.get_map().generate_waypoints(distance=5.0)
  road_ids = set([40, 48, 1091])
  lane_ids = {
      38: -1, 
      40: 1,
      46: -1,
      48: 1,
      49: 1,
      1072: -1,
      1091: -1,
      1400: 1
  }
  filtered_waypoints = list()
  for wp in waypoints:
    if wp.road_id in road_ids and lane_ids[wp.road_id] * wp.lane_id > 0:
        filtered_waypoints.append(wp)
  return filtered_waypoints

def bridge_keep_alive(q: Any):
  global collision_flag
  OSS = operational_state_space.OperationalStateSpace([0, 0, 0], [30, 15, 15], seed=2)
  SDQ = deltacovering_quant.DeltaCoveringSDQ(OSS, [3, 3, 3], epsilon=1e-1, beta=0.001)
  client = carla.Client('localhost', 2000)
  client.set_timeout(10.0)
  world = client.load_world('Town04')
  if args.low_quality:
    world.unload_map_layer(carla.MapLayer.Foliage)
    world.unload_map_layer(carla.MapLayer.Buildings)
    world.unload_map_layer(carla.MapLayer.ParkedVehicles)
    world.unload_map_layer(carla.MapLayer.Particles)
    world.unload_map_layer(carla.MapLayer.Props)
    world.unload_map_layer(carla.MapLayer.StreetLights)
  waypoints = filter_waypoint(world)
  done = False
  PATH = "data/lead_obstacle/delta_covering/"+"openpilot_dry/"
  if not os.path.exists(PATH):
    os.makedirs(PATH)
  i = 0
  while not done:
    s0 = SDQ.get_s0()
    try:
      collision_flag = False
      sv_v, lv_v, dhw, collision, openpilot_enabled = bridge(q, world, waypoints[45], s0[0], s0[1], s0[2])
      while not openpilot_enabled:
        collision_flag = False
        sv_v, lv_v, dhw, collision, openpilot_enabled = bridge(q, world, waypoints[45], s0[0], s0[1], s0[2])
      traj = np.array([dhw[::10], sv_v[::10], lv_v[::10]]).T
      done = SDQ.fit_traj(traj, failure=collision)
      delta_string = ""
      for d in SDQ.delta:
        delta_string += '{:2.4f}'.format(d)
        delta_string += " "
      print("Step %d, delta=%s, %d consecutive safe observations, %d desired. %d safe points, %d unsafe points, %d to check for fast removal" %
        (i, delta_string, SDQ.N, SDQ.N_max, len(SDQ.domain.D_s()), len(SDQ.domain.D_u()),len(SDQ.removal_candidates))) 
      if i%5==0:
        fig, ax = plt.subplots()
        Ds = np.array(list(SDQ.domain.D_s()))
        ax.plot(Ds.T[0], Ds.T[1],"g.")
        if len(list(SDQ.domain.D_u())) > 0:
          Du = np.array(list(SDQ.domain.D_u()))
          ax.plot(Du.T[0], Du.T[1],"r.")
        plt.savefig(PATH+str(i)+".png")
        plt.close("all")
        SDQ.save('data/lead_follow/delta_covering/')
      i += 1
    except RuntimeError:
      break
  if done:
    SDQ.save('data/lead_follow/delta_covering/')
    print("Done")
  else:
    print("Quitted due to error")

if __name__ == "__main__":
  # make sure params are in a good state
  set_params_enabled()

  msg = messaging.new_message('liveCalibration')
  msg.liveCalibration.validBlocks = 20
  msg.liveCalibration.rpyCalib = [0.0, 0.0, 0.0]
  Params().put("CalibrationParams", msg.to_bytes())

  q: Any = Queue()
  p = Process(target=bridge_keep_alive, args=(q,), daemon=True)
  p.start()

  if args.joystick:
    # start input poll for joystick
    from lib.manual_ctrl import wheel_poll_thread
    wheel_poll_thread(q)
    p.join()
  else:
    # start input poll for keyboard
    from lib.keyboard_ctrl import keyboard_poll_thread
    keyboard_poll_thread(q)
