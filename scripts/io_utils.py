import struct
import numpy as np
from laserscan import Laserscan


def read_labels(filename):
  """ read labels from given file. """
  contents = bytes()
  with open(filename, "rb") as f:  # rb = read binary
    f.seek(0, 2)  # move the cursor to the end of the file
    num_points = int(f.tell() / 4)
    f.seek(0, 0)
    contents = f.read()

  arr = [struct.unpack('<I', contents[4 * i:4 * i + 4])[0] for i in range(num_points)]

  return arr


def write_labels(filename, labels):
  """ write labels in given file. """
  arr = [struct.pack('<I', label) for label in labels]
  contents = bytes()
  for a in arr:
    contents += a

  with open(filename, "bw") as f:
    f.write(contents)


def read_points(filename):
  """ read pointcloud from given file in KITTI format."""
  contents = bytes()
  with open(filename, "rb") as f:  # rb = read binary
    f.seek(0, 2)  # move the cursor to the end of the file
    num_points = int(f.tell() / 4)
    f.seek(0, 0)
    contents = f.read()

  arr = [struct.unpack('<f', contents[4 * i:4 * i + 4])[0] for i in range(num_points)]
  points_arr = np.asarray(arr)  # convert list to array

  scan = Laserscan()
  scan.points = [np.array([x, y, z, 1]) for (x, y, z) in zip(points_arr[0::4], points_arr[1::4], points_arr[2::4])]
  scan.remissions = [r for r in points_arr[3::4]]

  # scan.x = points_arr[0::4]
  # scan.y = points_arr[1::4]
  # scan.z = points_arr[2::4]
  # scan.r = points_arr[3::4]

  return scan
