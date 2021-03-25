import numpy as np
import time
import matplotlib.pyplot as plt; plt.ion()
#from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
#import Planner_backup
import Planner
from numpy.linalg import norm
from Collision import collision_Check
from rrt_star import RRTStar
from plotting import Plot as rrt_plot
from search_space import SearchSpace

# deafult
# %matplotlib inline

# for 3D rotation
# %matplotlib qt

def tic():
  return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))
  

def load_map(fname):
  '''
  Loads the bounady and blocks from map file fname.
  
  boundary = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  
  blocks = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'],
            ...,
            ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  '''
  mapdata = np.loadtxt(fname,dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'),\
                                    'formats': ('S8','f', 'f', 'f', 'f', 'f', 'f', 'f','f','f')})
  blockIdx = mapdata['type'] == b'block'
  boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  return boundary, blocks


def draw_map(boundary, blocks, start, goal):
  '''
  Visualization of a planning problem with environment boundary, obstacle blocks, and start and goal points
  '''
  fig = plt.figure()
  ax = fig.add_subplot(111, projection='3d')
  hb = draw_block_list(ax,blocks)
  hs = ax.plot(start[0:1],start[1:2],start[2:],'ro',markersize=7,markeredgecolor='k')
  hg = ax.plot(goal[0:1],goal[1:2],goal[2:],'go',markersize=7,markeredgecolor='k')  
  ax.set_xlabel('X')
  ax.set_ylabel('Y')
  ax.set_zlabel('Z')
  ax.set_xlim(boundary[0,0],boundary[0,3])
  ax.set_ylim(boundary[0,1],boundary[0,4])
  ax.set_zlim(boundary[0,2],boundary[0,5])  
  return fig, ax, hb, hs, hg

def draw_block_list(ax,blocks):
  '''
  Subroutine used by draw_map() to display the environment blocks
  '''
  v = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],dtype='float')
  f = np.array([[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[0,1,2,3],[4,5,6,7]])
  clr = blocks[:,6:]/255
  n = blocks.shape[0]
  d = blocks[:,3:6] - blocks[:,:3] 
  vl = np.zeros((8*n,3)) 
  fl = np.zeros((6*n,4),dtype='int64')
  fcl = np.zeros((6*n,3))
  for k in range(n):
    vl[k*8:(k+1)*8,:] = v * d[k] + blocks[k,:3]
    fl[k*6:(k+1)*6,:] = f + k*8
    fcl[k*6:(k+1)*6,:] = clr[k,:]
  
  if type(ax) is Poly3DCollection:
    ax.set_verts(vl[fl])
  else:
    pc = Poly3DCollection(vl[fl], alpha=0.25, linewidths=1, edgecolors='k')
    pc.set_facecolor(fcl)
    h = ax.add_collection3d(pc)
    return h


def runtest(mapfile, start, goal, verbose = True):
  '''
  This function:
   * load the provided mapfile
   * creates a motion planner
   * plans a path from start to goal
   * checks whether the path is collision free and reaches the goal
   * computes the path length as a sum of the Euclidean norm of the path segments
  '''
  # Load a map and instantiate a motion planner
  boundary, blocks = load_map(mapfile)
  #MP = Planner_backup.MyPlanner(boundary, blocks) # TODO: replace this with your own planner implementation
  MP = Planner.MyPlanner(boundary, blocks)
  
  # Display the environment
  if verbose:
    fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)  
  
  # Call the motion planner
  t0 = tic()
  path = MP.plan(start, goal)
  toc(t0,"Planning")
  
  # Plot the path
  if verbose:
    ax.plot(path[:,0],path[:,1],path[:,2],'r-')
    
  collision = collision_Check(path, blocks)
  goal_reached = sum((path[-1]-goal)**2) <= 0.1
  success = (not collision) and goal_reached
  pathlength = np.sum(np.sqrt(np.sum(np.diff(path,axis=0)**2,axis=1)))
  return success, pathlength


def test_single_cube(verbose = False):
  print('Running A* single cube test...\n') 
  start = np.array([2.3, 2.3, 1.3])
  goal = np.array([7.0, 7.0, 5.5])
  success, pathlength = runtest('./maps/single_cube.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')
  
  
def test_maze(verbose = False):
  print('Running A* maze test...\n') 
  start = np.array([0.0, 0.0, 1.0])
  goal = np.array([12.0, 12.0, 5.0])
  success, pathlength = runtest('./maps/maze.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

    
def test_window(verbose = False):
  print('Running A* window test...\n') 
  start = np.array([0.2, -4.9, 0.2])
  goal = np.array([6.0, 18.0, 3.0])
  success, pathlength = runtest('./maps/window.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

  
def test_tower(verbose = False):
  print('Running A* tower test...\n') 
  start = np.array([2.5, 4.0, 0.5])
  goal = np.array([4.0, 2.5, 19.5])
  success, pathlength = runtest('./maps/tower.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

     
def test_flappy_bird(verbose = False):
  print('Running A* flappy bird test...\n') 
  start = np.array([0.5, 2.5, 5.5])
  goal = np.array([19.0, 2.5, 5.5])
  success, pathlength = runtest('./maps/flappy_bird.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength) 
  print('\n')

  
def test_room(verbose = False):
  print('Running A* room test...\n') 
  start = np.array([1.0, 5.0, 1.5])
  goal = np.array([9.0, 7.0, 1.5])
  success, pathlength = runtest('./maps/room.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')


def test_monza(verbose = False):
  print('Running A* monza test...\n')
  start = np.array([0.5, 1.0, 4.9])
  goal = np.array([3.8, 1.0, 0.1])
  success, pathlength = runtest('./maps/monza.txt', start, goal, verbose)
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')


def rrt_star_test(index = 0):
    t0 = tic()
    mapfile = ['./maps/single_cube.txt', './maps/maze.txt', './maps/window.txt',
               './maps/tower.txt', './maps/flappy_bird.txt', './maps/room.txt',
               './maps/monza.txt' ]
    name = ['single_cube', 'maze', 'window', 'tower', 'flappy_bird',
            'room', 'monza']
    start_list = [(2.3, 2.3, 1.3), (0.0, 0.0, 1.0), (0.2, -4.9, 0.2),
                  (2.5, 4.0, 0.5), (0.5, 2.5, 5.5), (1.0, 5.0, 1.5),
                  (0.5, 1.0, 4.9)]
    goal_list = [(7.0, 7.0, 5.5), (12.0, 12.0, 5.0), (6.0, 18.0, 3.0),
                 (4.0, 2.5, 19.5), (19.0, 2.5, 5.5), (9.0, 7.0, 1.5),
                 (3.8, 1.0, 0.1)]
    
    print('Running RRT* '+ name[index] + ' test...\n')
    
    start = start_list[index]
    goal = goal_list[index]
    boundary, blocks = load_map(mapfile[index])
    dimension = np.array([(boundary[0,0], boundary[0,3]),
                          (boundary[0,1], boundary[0,4]),
                          (boundary[0,2], boundary[0,5])])
    new_blocks = np.zeros((1,6))
    for block in blocks: new_blocks = np.concatenate((new_blocks, [block[:6]]))
    new_blocks = np.delete(new_blocks, 0, 0)
    Q = np.array([(0.5, 4)])
    r = 0.05
    max_samples = 102400
    prc = 0.1 
    rewire = 10 
    
    X = SearchSpace(dimension, new_blocks)
    
    rrt = RRTStar(X, Q, start, goal, max_samples, r, prc, rewire)
    path = rrt.rrt_star()
    plot_rrt(name[index], X, rrt, path, new_blocks, start, goal)
    toc(t0, name[index] + ' RRT*')
    
    pathlength = np.sum(np.sqrt(np.sum(np.diff(np.array(path),axis=0)**2,axis=1)))
    print('Path length is:', pathlength)
    
def plot_rrt(name, X, rrt, path, Obstacles, x_init, x_goal): # TODO
    plot = rrt_plot(name)
    plot.plot_tree(X, rrt.trees)
    if path is not None:
        plot.plot_path(X, path)
    plot.plot_obstacles(X, Obstacles)
    plot.plot_start(X, x_init)
    plot.plot_goal(X, x_goal)
    plot.draw(auto_open=False)

    


if __name__=="__main__":
    start = tic()
    
    # Search Based Algorithm A* 
    # test_single_cube(True)
    # test_maze(True)
    # test_flappy_bird(True)
    # test_monza(True)
    # test_window(True)
    # test_tower(True)
    # test_room(True)
    
    # Samplign Based Algorithm RRT*
    # rrt_star_test(0) # single_cube
    # rrt_star_test(1) # maze
    # rrt_star_test(2) # window
    # rrt_star_test(3) # tower
    # rrt_star_test(4) # flappy_bird
    # rrt_star_test(5) # room
    rrt_star_test(6) # monza
    
    toc(start, 'Total time')








