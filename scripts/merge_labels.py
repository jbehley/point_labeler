import struct


def read_labels(filename):
  """ read labels from given file. """
  contents = bytes()
  with open(filename, "rb") as f:
    f.seek(0, 2) # move the cursor to the end of the file
    num_points = int(f.tell() / 4)
    f.seek(0, 0)
    contents = f.read()
  
  arr = [struct.unpack('<I', contents[4*i:4*i+4])[0] for i in range(num_points)]
  
  return arr
  
def write_labels(filename, labels):
  """ write labels in given file. """
  arr = [struct.pack('<I', label) for label in labels]
  contents = bytes()
  for a in arr: contents += a
    
  with open(filename, "bw") as f:
    f.write(contents)



if __name__ == "__main__":
	labels1 = read_labels("/home/snej/data/kitti/dataset/sequences/00/labels_old/labels/000000.label")
	# labels2 = read_labels("/home/snej/data/kitti/dataset/sequences/00/labels/000000.label")
	print("{} labels in file".format(len(labels1)))
	#print(labels1[0:100])

	write_labels("test.label", labels1)
	labels_test = read_labels("test.label")

	labels_test == labels1
