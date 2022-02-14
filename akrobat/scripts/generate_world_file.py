import math
import os
import random
import sys
import traceback



def createWorldFile(filename: str, models: list):
  with open(f'{filename}.world', 'w+') as world_file:
    world_file.write(f'''<sdf version='1.7'>
  <world name='default'>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
      <spot>
        <inner_angle>0</inner_angle>
        <outer_angle>0</outer_angle>
        <falloff>0</falloff>
      </spot>
    </light>
    {''.join(models)}
  </world>
</sdf>''')

class Block:

    def __init__(self, size: tuple, position: tuple) -> None:
        self.size = size
        self.position = position

    def __str__(self) -> str:
        return f'Block[size: {self.size}, position: {self.position}]'

    def __repr__(self) -> str:
        return self.__str__()

    @property
    def height(self):
      return self.size[2]

    @property
    def x(self):
      return self.position[0]
      
    @property
    def y(self):
      return self.position[1]

    @property
    def z(self):
      return self.position[2]
    

    def to_xml_string(self):
        return f'''<model name='{self.__hash__()}'>
      <static>1</static>
      <pose>{self.position[0]} {self.position[1]} {self.position[2]} 0 0 0</pose>
      <link name='link'>
        <inertial>
          <mass>1</mass>
          <inertia>
            <ixx>0</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0</iyy>
            <iyz>0</iyz>
            <izz>0</izz>
          </inertia>
          <pose>{self.position[0]} {self.position[1]} {self.position[2]} 0 0 0</pose>
        </inertial>
        <collision name='collision'>
          <geometry>
            <box>
              <size>{self.size[0]} {self.size[1]} {self.size[2]}</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <geometry>
            <box>
              <size>{self.size[0]} {self.size[1]} {self.size[2]}</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
              <uri>file://media/materials/scripts/gazebo.material</uri>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <enable_wind>0</enable_wind>
        <kinematic>0</kinematic>
      </link>
    </model>
    '''

    
class Terrain:

  def __init__(self, max_height_difference: float, block_size: float, block_amount: tuple, starting_platform_size=(0,0)) -> None:
      self._max_height_difference = max_height_difference
      self._block_size = block_size
      self._amnt_blocks_x, self._amnt_blocks_y = block_amount
      self._grid = [ [None] * self._amnt_blocks_x for _ in range(self._amnt_blocks_y)]
      self._starting_platform_size = starting_platform_size
      self._starting_platform = None

      self.__fill_grid()
      self.__position_grid()
      self.__make_starting_platform()

  @property
  def models(self):
    # flatten 3D list to 2D
    models = [item.to_xml_string() for row in self._grid for item in row if item != None]
    models.append('<include><uri>model://ground_plane</uri></include>')
    return models
     

  @property
  def _grid_center(self):
    return ((self._amnt_blocks_x-1)//2, (self._amnt_blocks_y-1)//2)


  def __fill_grid(self):
    for x in range(self._amnt_blocks_x):
      for y in range(self._amnt_blocks_y):
        min_height, max_height = self.__find_extremes(x, y)

        height = random.uniform(min_height, max_height).__round__(3)

        self._grid[y][x] = Block(
          (self._block_size, self._block_size, height),
          (x*self._block_size, y*self._block_size, height/2)
        )

  def __position_grid(self):
    offset_x = self._block_size * (1 - self._amnt_blocks_x) / 2
    offset_y = self._block_size * (1 - self._amnt_blocks_y) / 2

    for row in self._grid: 
      for block in row:
        x,y,z = block.position
        block.position = (x+offset_x, y+offset_y, z)

  def __find_extremes(self, x: int, y: int, treat_none_as_height_zero=False):
    lowest = 0
    highest = 0
    for x_offset in range(-1, 1):
      for y_offset in range(-1, 1):
        u = x + x_offset
        v = y + y_offset

        if not (u < 0 or u >= len(self._grid) or v < 0 or v >= len(self._grid[u])):
          if self._grid[u][v] is not None:
            value = self._grid[u][v].size[2]
            lowest = value if value < lowest else lowest
            highest = value if value > highest else highest
          elif treat_none_as_height_zero:
            lowest = 0


    max_height = lowest + self._max_height_difference
    min_height  = max(highest - self._max_height_difference, 0)

    return (min_height, max_height)



  def __make_starting_platform(self):
    if self._starting_platform_size is None: return
    
    x, y = self._starting_platform_size

    if (x == 0 or y == 0): return

    blocks_x = math.ceil(x/self._block_size)
    blocks_y = math.ceil(y/self._block_size)

    self.__remove_blocks_from_center(blocks_x, blocks_y)


  def __remove_blocks_from_center(self, x: int, y: int):
    mask = [(0,0),(0,0)]

    if (self._amnt_blocks_x % 2 == 0):
      if (x % 2 == 0):
        # print('both x-amount and x to remove are even')
        mask[0] = (-x/2+1, x/2)
      else:
        # print('x-amount is even but x to remove is odd')
        mask[0] = (-(x+1)/2+1, (x+1)/2)
    else:
      if (x % 2 == 0):
        # print('x-amount is odd but x to remove is even')
        mask[0] = (-((x+1)//2), (x+1)//2)
      else:
        # print('both x-amount and x to remove are odd')
        mask[0] = (-(x//2), x//2)


    if (self._amnt_blocks_y % 2 == 0):
      if (y % 2 == 0):
        # print('both y-amount and y to remove are even')
        mask[1] = (-y/2+1, y/2)
      else:
        # print('y-amount is even but y to remove is odd')
        mask[1] = (-(y+1)/2+1, (y+1)/2)
    else:
      if (y % 2 == 0):
        # print('y-amount is odd but y to remove is even ')
        mask[1] = (-((y+1)//2), (y+1)//2)
      else:
        # print('both y-amount and y to remove are odd')
        mask[1] = (-(y//2), y//2)

    mask_x, mask_y = mask


    start_x = int(self._grid_center[0] + mask_x[0])
    start_y = int(self._grid_center[1] + mask_y[0])
    end_x = int(self._grid_center[0] + mask_x[1] + 1)
    end_y = int(self._grid_center[1] + mask_y[1] + 1)

    print(start_x, end_x, start_y, end_y)

    for u in range(start_x, end_x):
      for v in range (start_y, end_y):
        self._grid[v][u] = None
        # print(f'removed at ({v}, {u})')

    for u in range(start_x-1, end_x+1):
      for v in range (start_y-1, end_y+1):
        self.__recalculate_height(u, v)

  def __recalculate_height(self, x: int, y: int):
    try:
      block = self._grid[y][x]
    except:
      print('grid is too small for starting platform, raise block_amount')
      sys.exit()
    if (block is None): return

    print(f'recalculate {x},{y} with height {block.height}')

    min_height, max_height = self.__find_extremes(x,y,True)

    if block.height < min_height or block.height > max_height:
      new_height = random.uniform(min_height, max_height).__round__(3)
      self._grid[y][x] = Block((self._block_size, self._block_size, new_height), (block.x, block.y, new_height/2))
      print(f'changed height to {new_height}')

      self.__recalculate_neighbors(x, y)

  def __recalculate_neighbors(self, x: int, y: int):
    for u in range(x-1, x+1):
      for v in range(y-1, y+1):
        if not (u is x and v is y):
          self.__recalculate_height(u, v)



import getopt

def main(argv):
    filename = 'default'
    platform = (0,0)
    outpath = ''
    try:
      opts, args = getopt.getopt(argv,'f:p:o:h',['filename=', 'platform=', 'outpath=', 'help'])
    except getopt.GetoptError:
      sys.exit(2)

    for opt, arg in opts:
        if opt in ('-f', '--filename'):
          filename = arg

        elif opt in ('-p', '--platform'):
          x, y = arg.split(',')
          platform = (float(x), float(y))

        elif opt in ('-o', '--outpath'):
          outpath = arg

        elif opt in ('-h', '--help'):
            print(f'''This script takes 3 arguments to create a .world-file for use in the gazebo simulator
Those arguments are in order:
- max_height_difference: float
    The maximum difference in height between adjacent blocks in meters
- block_size: float
    The size of the blocks in meters
- block_amount: int tuple
    The amount of blocks in the x and y-direction, write together, comma-separated with no space like this: \'x,y\'
    
Options (must come before the arguments):
- \'-p <platform size>\' or \'--platform <platform size>\'
    can be added to create a starting platform of at least the given sizes in meters
    write together, comma-separated with no space like this: \'x,y\'
- \'-f <filename>\' or \'--filename <filename>\'
    can be added to name the .world-file. The default name is \'default.world\'
- \'-o <output path>\' or \'--outpath <output path>\'
    can be added to save the file to a specific path. Can be absolute or relative to the directory the script is run in.

Example:
    This will create the file \'my.world\' in the subdirectory worlds.
    A starting platform of size 1x1 meters will be added.
    The height difference between blocks will be 0.2 meters with a block size of 0.5x0.5 meters
    The block grid will be 20x20 blocks.
    py {sys.argv[0]} -f my -o worlds -p 1.0,1.0 0.2 0.5 20,20
''')


    if len(args) < 3:
      print(f'Script needs 3 arguments to function. Run \'py {sys.argv[0]} -h\' for help')
    else:
      try:
        # max_height_difference: float, block_size: float, block_amount: tuple, starting_platform_size=(0,0)
        x, y = args[2].split(',')
        block_amount = (int(x), int(y))
        createWorldFile(os.path.join(outpath, filename), Terrain(float(args[0]), float(args[1]), block_amount, platform).models)
      except Exception:
        print(f'Something went wrong. Run \'py {sys.argv[0]} -h\' for help\n')
        traceback.print_exc()

if __name__ == "__main__":
   main(sys.argv[1:])
