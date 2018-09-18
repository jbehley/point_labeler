import numpy as np


class Laserscan:
  """Class that contains Laserscan with x,y,z,r"""

  def __init__(self):
    self.points = []
    self.remissions = []

  def size(self):
    """ return the size of the point cloud. """
    return len(self.points)

  def __len__(self):
    return self.size()

  # self.x = []
  # self.y = []
  # self.z = []
  # self.r = []
