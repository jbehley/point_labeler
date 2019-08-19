#!/usr/bin/python3

import sys
import os
import shutil
import numpy as np

from collections import namedtuple

# ===================================== PYKITTI =====================================
# stolen from https://github.com/utiasSTARS/pykitti/blob/master/pykitti/raw.py
# and utils.py


def transform_from_rot_trans(R, t):
  """Transforation matrix from rotation matrix and translation vector."""
  R = R.reshape(3, 3)
  t = t.reshape(3, 1)
  return np.vstack((np.hstack([R, t]), [0, 0, 0, 1]))


def rotx(t):
  """Rotation about the x-axis."""
  c = np.cos(t)
  s = np.sin(t)
  return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def roty(t):
  """Rotation about the y-axis."""
  c = np.cos(t)
  s = np.sin(t)
  return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rotz(t):
  """Rotation about the z-axis."""
  c = np.cos(t)
  s = np.sin(t)
  return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


OxtsPacket = namedtuple(
    'OxtsPacket',
    'lat, lon, alt, roll, pitch, yaw, vn, ve, vf, vl, vu, ax, ay, az, af, al, au, wx, wy, wz, wf, wl, wu, ' +
    'pos_accuracy, vel_accuracy, navstat, numsats, posmode, velmode, orimode')


def pose_from_oxts(packet, scale):
  """Helper method to compute SE(3) pose matrices from OXTS packets."""
  er = 6378137.  # earth radius (approx.) in meters

  # Use a Mercator projection to get the translation vector
  tx = scale * packet.lon * np.pi * er / 180.
  ty = scale * er * np.log(np.tan((90. + packet.lat) * np.pi / 360.))
  tz = packet.alt
  t = np.array([tx, ty, tz])

  # Use the Euler angles to get the rotation matrix
  Rx = rotx(packet.roll)
  Ry = roty(packet.pitch)
  Rz = rotz(packet.yaw)
  R = Rz.dot(Ry.dot(Rx))

  # Combine the translation and rotation into a homogeneous transform
  return transform_from_rot_trans(R, t)


# ===================================== PYKITTI =====================================

if __name__ == "__main__":

  if len(sys.argv) < 2:
    print("Convert KITTI's raw format into odometry format consumable by point_labeler.")
    print("./kitti_raw2odometry.py <input-folder> [<output-folder>]")
    exit(1)

  input_folder = sys.argv[1]

  if input_folder[-1] == "/": input_folder = input_folder[:-1]
  output_folder = input_folder.split("/")[-1]
  if len(sys.argv) == 3: output_folder = sys.argv[2]

  lidar_path = os.path.join(input_folder, "velodyne_points", "data")
  image_path = os.path.join(input_folder, "image_02", "data")
  pose_path = os.path.join(input_folder, "oxts", "data")

  # check if everything needed is there.
  if not os.path.exists(lidar_path): raise IOError("velodyne_points not found: {}".format(lidar_path))
  if not os.path.exists(image_path): raise IOError("images not found: {}".format(image_path))
  if not os.path.exists(pose_path): raise IOError("oxts information not found: {}".format(pose_path))

  out_lidar_path = os.path.join(output_folder, "velodyne")
  out_image_path = os.path.join(output_folder, "image_2")

  # last chance to not overwrite stuff
  if os.path.exists(out_lidar_path) or os.path.exists(out_image_path):
    print("Output folders already exists!", end="")
    answer = input("Do you still want to proceed? [y/N] ")
    if answer != "y":
      print("Aborted.")
      exit(1)

  # generate directories
  os.makedirs(out_lidar_path, exist_ok=True)
  os.makedirs(out_image_path, exist_ok=True)

  velodyne_files = sorted([os.path.join(lidar_path, file) for file in os.listdir(lidar_path) if file.endswith(".bin")])
  image_files = sorted([os.path.join(image_path, file) for file in os.listdir(image_path) if file.endswith(".png")])

  assert (len(velodyne_files) == len(image_files))

  pose_file = open(os.path.join(output_folder, "poses.txt"), "w")

  scale = None
  origin = None
  
  print("Processing: ", end = "", flush = True)
  progress = 10

  for index, file in enumerate(velodyne_files):
    if 100 * index / len(velodyne_files) > progress: 
      print("{} ".format(progress), end = "",flush = True)
      progress += 10

    # the filename of the velodyne point cloud should match the filename of the image at the same index!
    velo_basename = os.path.splitext(os.path.basename(file))[0]
    assert (velo_basename == os.path.splitext(os.path.basename(image_files[index]))[0])

    shutil.copy(file, os.path.join(out_lidar_path, "{:06d}.bin".format(index)))
    shutil.copy(image_files[index], os.path.join(out_image_path, "{:06d}.png".format(index)))

    with open(os.path.join(pose_path, velo_basename + ".txt")) as f:
      line = f.readline().split()
      # Last five entries are flags and counts
      line[:-5] = [float(x) for x in line[:-5]]
      line[-5:] = [int(float(x)) for x in line[-5:]]

      packet = OxtsPacket(*line)

      if index == 0:
        er = 6378137.  # earth radius (approx.) in meters
        scale = np.cos(packet.lat * np.pi / 180.)
        tx = scale * packet.lon * np.pi * er / 180.
        ty = scale * er * np.log(np.tan((90. + packet.lat) * np.pi / 360.))
        tz = packet.alt
       

      pose = pose_from_oxts(packet, scale)
      if origin is None: origin = np.linalg.inv(pose)
      pose = np.dot(origin, pose)
      pose_file.write(" ".join([str(v) for v in pose.reshape(-1)[:12]]) + "\n")

  pose_file.close()

  # write dummy calibration file.
  # Note: the actual calibration should be irrelevant for our purposes, since we have poses in the Velodyne coordinate system.
  # if you want to have the "real" thing, one has to transform the poses into the camera coordinate system via Tr.
  calib_file = open(os.path.join(output_folder, "calib.txt"), "w")
  calib_file.write("P0: 1 0 0 0 0 1 0 0 0 0 1 0\n")
  calib_file.write("P1: 1 0 0 0 0 1 0 0 0 0 1 0\n")
  calib_file.write("P2: 1 0 0 0 0 1 0 0 0 0 1 0\n")
  calib_file.write("P3: 1 0 0 0 0 1 0 0 0 0 1 0\n")
  calib_file.write("Tr: 1 0 0 0 0 1 0 0 0 0 1 0\n")
  calib_file.close()

  while progress < 110: 
      print("{} ".format(progress), end="", flush = True)
      progress += 10
  print("finished.")