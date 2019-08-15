#!/usr/bin/python3

import struct
import sys
import os
import argparse


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
    for a in arr: f.write(a)


def get_labelfiles(directory):
  return {f: os.path.join(directory, f) for f in os.listdir(directory) if f.endswith(".label")}


if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Merge multiple label files.')
  parser.add_argument('input', nargs=2, help='directories containing labels to merge.')
  parser.add_argument('--out', nargs=1, help='output directory', default=None)
  parser.add_argument(
      "--keep",
      nargs=1,
      help="keep labels from given directory. Otherwise always newer labels are keept.",
      default=None)

  args = parser.parse_args()

  if args.out is None:
    os.makedirs("./labels_merged", exist_ok=True)
    out = os.path.abspath("./labels_merged")


  if not os.path.exists(out):
    os.makedirs(out)
    print("-- Creating directory {}".format(out))

  print("-- Writing merged labels to {}".format(out))

  if len([f.endswith(".label") for f in os.listdir(out)]) > 0:
    print("-- Found label files in output directory. Delete? ", end="")
    answer = input("[y/N/c(ontinue)] ")
    if answer.lower() == "y":
      print("-- Deleting files in output directory.")
      files = [os.path.join(out, f) for f in os.listdir(out) if f.endswith(".label")]
      for f in files:
        os.remove(f)
    elif answer.lower() == "c":
      print("-- Continue.")
    else:
      print("Aborted.")
      sys.exit(1)

  labels1 = get_labelfiles(args.input[0])
  labels2 = get_labelfiles(args.input[1])


  already_merged = [f for f in os.listdir(out) if f.endswith(".label")]
  print(already_merged[:10])

  if len(labels1) != len(labels2) or len(labels1) == 0:
    print("-- Error: Inconsistent number of labels. Aborting.")
    sys.exit(1)

  print("-- Merging: ", end="", flush=True)
  progress = 10
  count = 0
  for (f, f1) in labels1.items():

    count += 1
    if 100 * count / len(labels1) > progress:
      print(progress, end=" ", flush=True)
      progress += 10

    if f in already_merged: continue

    f2 = labels2[f]
    labels = [read_labels(f1), read_labels(f2)]

    if len(labels[0]) != len(labels[1]):
      print("Inconsistent number of labels for {}: Expected {}, but got {} labels.".format(f, len(labels[0]), len(labels[1])))
      sys.exit(1)

    keep_index = 0
    if args.keep is None:
      if os.stat(f2).st_mtime > os.stat(f1).st_mtime:
        keep_index = 1
    elif args.keep[0] == args.input[1]: keep_index = 1


    merged = labels[keep_index]
    for i in range(len(merged)):
      if merged[i] == 0:  merged[i] = labels[(keep_index + 1) % 2][i]

    write_labels(os.path.join(out, f), merged)

  while 100 * count / len(labels1) < 100:
    print(progress, end=" ")
    progress += 10
  print("finished.")

  # print("{} labels in file".format(len(labels1)))
  # #print(labels1[0:100])

  #
  # labels_test = read_labels("test.label")

  # labels_test == labels1
