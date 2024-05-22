import numpy as np
import shapely
from shapely import box, LineString, normalize, Polygon
import math
from pqdict import pqdict
import heapq

class AStarNode(object):
  def __init__(self, pqkey, coord, hval):
    self.pqkey = pqkey
    self.coord = coord
    self.g = math.inf
    self.h = hval
    self.parent_node = None
    self.parent_action = None
    self.closed = False
  def __lt__(self, other):
    return self.g < other.g 

# for every node: [x,y,z,hval]
nodes = np.array([
    [1,1,1,10],[2,1,1,8],
    [3,2,1,20],[4,1,2,41],
    [5,2,1,32],[6,2,2,6],
    [7,1,2,15],[8,2,2,18],
    [9,1,1,22],[10,3,1,7],
    [11,1,3,30],[12,3,1,2],  
])

pq_nodes = pqdict({},key=lambda x:x.h)

for i in nodes:
    coords,hval = i[:3],i[-1]
    # name = "{},{},{}".format(i[0],i[1],i[2])
    name = tuple(coords)
    # creating astar node object
    a = AStarNode(name,coords,hval)
    # add it into pqdict
    pq_nodes[name] = a


# print(pq_nodes.keys)
if '12,3,1' in pq_nodes:
  print("**********")
else: 
  print("+++++++++++++")


while pq_nodes !=  None:
  # print(list(pq_nodes.keys()))   
  node = pq_nodes.popitem()
  print(node[1].h)
  if len(pq_nodes) == 0:
    print("empty")
    break


 







