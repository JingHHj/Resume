
# priority queue for OPEN list
from pqdict import pqdict
import numpy as np
import math

class AStarNode(object):
  def __init__(self, pqkey, coord, hval):
    self.pqkey = pqkey
    self.coord = coord
    self.g = math.inf
    self.h = hval
    self.f = self.g + self.h
    self.parent_node = None
    self.parent_action = None
    self.closed = False
  def __lt__(self, other):
    return self.g < other.g     


class AStar(object):
  @staticmethod
  def find_neighbours(self,node, res):
    
    [dX,dY,dZ] = np.meshgrid([-1,0,1],[-1,0,1],[-1,0,1])
    dR = np.vstack((dX.flatten(),dY.flatten(),dZ.flatten()))
    dR = np.delete(dR,13,axis=1)
    dR = dR.T
    neighbours = np.tile(node,(26,1))
    
    return neighbours + dR
  
  def relabel():
    return 
  
  @staticmethod
  def plan(start_coord, environment,res, epsilon = 1):
    # Initialize the graph and open list
    CLOSED = {}
    OPEN = pqdict({},key=lambda x:x.f)

    # start node
    start = AStarNode(tuple(start_coord), start_coord, environment.getHeuristic(start_coord))
    start.g = 0
    OPEN.additem(tuple(start_coord),start)
    
    while len(OPEN) != 0:
      # core loop
      # pop the item with the lowest f 
      curr = OPEN.popvalue()

      # find its neighbours
      neighbours = AStar.find_nieghbours(curr.coords,res)
      for i in neighbours:
        pqkey = tuple(i[:3])
        
        if pqkey in OPEN:
          next = OPEN[pqkey]
        else:
          next = AStarNode(pqkey,i[:3],environment.getHeuristic(i[:3]))
        
        cij = AStar.motion_model(environment,curr.coord,next.coord)
        # gi+ cij < gj
        if curr.g + cij < next.g:
          next.g = curr.g + cij # relabeling
          next.parent_node = curr # keep on track of the parent node
          next.parent_action = next.coord - curr.coord
          OPEN[pqkey] = next
        else:
          continue
          
      CLOSED[tuple(curr.coord)] = curr
      if tuple(environment.goal) in CLOSED:
        break
    
    # trace back to find the trajectory
    traj = np.copy(environment.goal)
    while tuple(traj[-1]) != tuple(start_coord):
      last = OPEN[tuple(traj[-1])].parent_node
      traj = np.vstack((traj,last))s
      
    
    return traj


  
  

class Environment(object):
  def __init__(self,blocks,boundary,start,goal):
    self.start = start
    self.goal = goal
    self.blocks = blocks # xmin ymin zmin xmax ymax zmax r g b (many)
    self.boundary = boundary # xmin ymin zmin xmax ymax zmax
    
  def getHeuristic(self,curr):
    """
    Get the heuristic of current node
    using manhattan distance (1-norm)

    Args:
        curr: currrent node (x,y,z)
    """
    return np.sum(np.abs(curr - self.goal))
  
  def motion_model(self,curr,next):
    """
    finding the Cij for i to j 
    curr: current node (x,y,z)
    next: the next node (x,y,z)
    env: environment
    
    return the cost
    """
    cost = math.sqrt(np.sum((curr - next)**2))
    if Environment.if_out(next):
      cost = math.inf
    return cost
  
  def if_out(self,node):
    """
    Tell if the given node is out of boundary or inside of obstacles

    node:
        out(bool): 
    """
    out = False
    
    if( node[0] < self.boundary[0,0] or node[0] > self.boundary[0,3] or \
        node[1] < self.boundary[0,1] or node[1] > self.boundary[0,4] or \
        node[2] < self.boundary[0,2] or node[2] > self.boundary[0,5] ):
        out = True
    else: 
      for k in range(self.blocks.shape[0]):
          if( node[0] >= self.blocks[k,0] and node[0] <= self.blocks[k,3] and\
              node[1] >= self.blocks[k,1] and node[1] <= self.blocks[k,4] and\
              node[2] >= self.blocks[k,2] and node[2] <= self.blocks[k,5] ):
            out = True
            break
    return out
    
    

