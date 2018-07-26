import os
import sys
import argparse
import struct


def read_labels(filename):
  """ read labels from given file. """
  contents = bytes()
  with open(filename, "rb") as f:
    f.seek(0, 2)  # move the cursor to the end of the file
    num_points = int(f.tell() / 4)
    f.seek(0, 0)
    contents = f.read()

  arr = [struct.unpack('<I', contents[4 * i:4 * i + 4])[0] for i in range(num_points)]

  return arr


def write_labels(filename, labels):
  """ write labels in given file. """
  arr = [struct.pack('<I', label) for label in labels]

  with open(filename, "bw") as f:
    for a in arr:
      f.write(a)


def getPointCount(filename):
  num_points = 0
  byte_count = 4
  if filename.endswith(".bin"): byte_count = 4 * 4  # 4 float
  if filename.endswith(".label"): byte_count = 4  # 1 float

  with open(filename, "rb") as f:
    f.seek(0, 2)  # move the cursor to the end of the file
    num_points = int(f.tell() / byte_count)

  return num_points


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='repair label files.')
  parser.add_argument('input', help='base directory containing labels folder to repair.')

  args = parser.parse_args()

  labels_path = os.path.join(args.input, "labels")
  velodyne_path = os.path.join(args.input, "velodyne")

  if not os.path.exists(labels_path):
    print("No 'labels' directory found inside '" + args.input + "'. Abort.")
    sys.exit(1)
  if not os.path.exists(velodyne_path):
    print("No 'velodyne' directory found inside '" + args.input + "'. Abort.")
    sys.exit(1)

  velodyne_files = {
      os.path.splitext(f)[0]: os.path.join(velodyne_path, f) for f in os.listdir(velodyne_path) if f.endswith(".bin")
  }
  label_files = {
      os.path.splitext(f)[0]: os.path.join(labels_path, f) for f in os.listdir(labels_path) if f.endswith(".label")
  }

  for file in sorted(velodyne_files.keys()):
    num_points = getPointCount(velodyne_files[file])

    if file not in label_files:
      label_filename = os.path.join(labels_path, file + ".label")
      print("-- Generate missing label file " + label_filename)
      write_labels(label_filename, [0] * num_points)

      continue
    num_labels = getPointCount(label_files[file])
    if num_points != num_labels:
      print("-- Inconsistent number of labels '{}': Expected {} labels, but got {} labels.".format(file, num_points, num_labels))
      os.rename(label_files[file], label_files[file] + ".bkp")
      write_labels(label_files[file], [0] * num_points)
