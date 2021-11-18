import rospy
import rospkg
import os
import yaml
import random

from numpy import random
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

def spawn_block(config_idx, missing_block=False):

    missing_yellow = False
    missing_green = False
    missing_basketball = False
    '''
    if missing_block:
        if random.randint(0, 1) == 0:
            missing_yellow = True
        else:
            missing_green = True
    '''
    # Initialize ROS pack
    rospack = rospkg.RosPack()

    # Get path to block
    ur_path = rospack.get_path('ur_description')
    lab5_path = rospack.get_path('lab5pkg_py')
    block_yellow_path = os.path.join(ur_path, 'urdf', 'block_yellow.urdf')
    block_green_path = os.path.join(ur_path, 'urdf', 'block_green.urdf')

    basketball_path = os.path.join(ur_path, 'urdf', 'basketball.urdf')
    soccerball_path = os.path.join(ur_path, 'urdf', 'soccerball.urdf')
    tennisball_path = os.path.join(ur_path, 'urdf', 'tennisball.urdf')

    # Wait for service to start
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    spawn = rospy.ServiceProxy('gazebo/spawn_urdf_model', SpawnModel)
    delete = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

    # Get YAML path
    yaml_path = os.path.join(lab5_path, 'scripts', 'configs',  'positions.yaml')

    # Load block positions
    with open(yaml_path, 'r') as f:
        positions_dict = yaml.load(f)

    # IF YOU USE THOSE POSITIONS DIRECTLY, YOU WILL GET A ZERO
    # NO EXCUSES, AND WE WILL RUN CHECKS
    block_yellow_position = positions_dict['block_yellow_positions'][config_idx]

    block_green_position = positions_dict['block_green_positions'][config_idx]

    # Spawn block

    # First delete all blocks
    '''
    delete('block_yellow')
    delete('block_green')
    delete('basketball')
    delete('basketball_1')
    delete('basketball_2')
    delete('basketball_3')
    delete('left_basketball')
    delete('right_basketball')
    '''
    delete('ball_1')
    delete('ball_2')
    delete('ball_3')
    delete('ball_4')
    delete('ball_5')
    

    """
    # Spawn yellow
    block_name = 'block_yellow'
    #pose = Pose(Point(block_yellow_position[0], block_yellow_position[1], 0), Quaternion(0, 0, 0, 0))
    pose = Pose(Point(0.15, -0.15, 0.1205), Quaternion(0, 0, 0, 0)) #this point is the left edge position to test camera view field
    if not missing_yellow:
        spawn(block_name, open(block_yellow_path, 'r').read(), 'block', pose, 'world')

    # Spawn green
    block_name = 'block_green'
    #pose = Pose(Point(block_green_position[0], block_green_position[1], 0), Quaternion(0, 0, 0, 0))
    pose = Pose(Point(0.15, 0.45, 0.1205), Quaternion(0, 0, 0, 0)) # this point is the right edge position to test camera view field
    if not missing_green:
        spawn(block_name, open(block_green_path, 'r').read(), 'block', pose, 'world')
    """
    '''
    #COMMENTED OUT TEMPORARILY FOR RECALIBRATION
    # Spawn basketball 1
    block_name = 'basketball_1'
    pose = Pose(Point(0.15, -0.05, 0.1205), Quaternion(0, 0, 0, 0))
    spawn(block_name, open(basketball_path, 'r').read(), 'block', pose, 'world')

    # Spawn basketball 2
    block_name = 'basketball_2'
    pose = Pose(Point(0.15, 0.15, 0.1205), Quaternion(0, 0, 0, 0))
    spawn(block_name, open(basketball_path, 'r').read(), 'block', pose, 'world')
    
    # Spawn basketball 3
    block_name = 'basketball_3'
    pose = Pose(Point(0.15, 0.35, 0.1205), Quaternion(0, 0, 0, 0))
    spawn(block_name, open(basketball_path, 'r').read(), 'block', pose, 'world')
    '''
    '''
    positions have been checked with ALL basketballs - GOOD
    position 1 = Point(0.23, -0.17, 0.1205)
    position 2 = Point(0.15, 0.27, 0.1205)
    position 3 = Point(0.27, 0.10, 0.1205)
    position 4 = Point(0.35, 0.19, 0.1205)
    position 5 = Point(0.10, 0.03, 0.1205)
    '''

    #initialize randomized ball positions
    ballpositions = random.randint(1, 4, size=5)

    #value of 1 is a basketball
    #value of 2 is a soccerball
    #value of 3 is a tennisball

    if(ballpositions[0] == 1):
        block_name = 'ball_1'
        pose = Pose(Point(0.23, -0.17, 0.1205), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(basketball_path, 'r').read(), 'block', pose, 'world')
    elif(ballpositions[0] == 2):
        block_name = 'ball_1'
        pose = Pose(Point(0.23, -0.17, 0.1205), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(soccerball_path, 'r').read(), 'block', pose, 'world')
    else:
        block_name = 'ball_1'
        pose = Pose(Point(0.23, -0.17, 0.1205), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(tennisball_path, 'r').read(), 'block', pose, 'world')


    if(ballpositions[1] == 1):
        block_name = 'ball_2'
        pose = Pose(Point(0.15, 0.27, 0.1205), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(basketball_path, 'r').read(), 'block', pose, 'world')
    elif(ballpositions[1] == 2):
        block_name = 'ball_2'
        pose = Pose(Point(0.15, 0.27, 0.1205), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(soccerball_path, 'r').read(), 'block', pose, 'world')
    else:
        block_name = 'ball_2'
        pose = Pose(Point(0.15, 0.27, 0.1205), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(tennisball_path, 'r').read(), 'block', pose, 'world')


    if(ballpositions[2] == 1):
        block_name = 'ball_3'
        pose = Pose(Point(0.27, 0.10, 0.1205), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(basketball_path, 'r').read(), 'block', pose, 'world')
    elif(ballpositions[2] == 2):
        block_name = 'ball_3'
        pose = Pose(Point(0.27, 0.10, 0.1205), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(soccerball_path, 'r').read(), 'block', pose, 'world')
    else:
        block_name = 'ball_3'
        pose = Pose(Point(0.27, 0.10, 0.1205), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(tennisball_path, 'r').read(), 'block', pose, 'world')


    if(ballpositions[3] == 1):
        block_name = 'ball_4'
        pose = Pose(Point(0.35, 0.19, 0.1205), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(basketball_path, 'r').read(), 'block', pose, 'world')
    elif(ballpositions[3] == 2):
        block_name = 'ball_4'
        pose = Pose(Point(0.35, 0.19, 0.1205), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(soccerball_path, 'r').read(), 'block', pose, 'world')
    else:
        block_name = 'ball_4'
        pose = Pose(Point(0.35, 0.19, 0.1205), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(tennisball_path, 'r').read(), 'block', pose, 'world')


    if(ballpositions[4] == 1):
        block_name = 'ball_5'
        pose = Pose(Point(0.10, 0.03, 0.1205), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(basketball_path, 'r').read(), 'block', pose, 'world')
    elif(ballpositions[4] == 2):
        block_name = 'ball_5'
        pose = Pose(Point(0.10, 0.03, 0.1205), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(soccerball_path, 'r').read(), 'block', pose, 'world')
    else:
        block_name = 'ball_5'
        pose = Pose(Point(0.10, 0.03, 0.1205), Quaternion(0, 0, 0, 0))
        spawn(block_name, open(tennisball_path, 'r').read(), 'block', pose, 'world')

    '''
    #THESE ARE CALIBRATION BASKETBALLS, DO NOT EDIT
    # Spawn Left Basketball
    block_name = 'left_basketball'
    pose = Pose(Point(0.15, 0.35, 0.1205), Quaternion(0, 0, 0, 0))
    spawn(block_name, open(basketball_path, 'r').read(), 'block', pose, 'world')

    # Spawn Right Basketball
    block_name = 'right_basketball'
    pose = Pose(Point(0.15, -0.05, 0.1205), Quaternion(0, 0, 0, 0))
    spawn(block_name, open(basketball_path, 'r').read(), 'block', pose, 'world')
    '''
if __name__ == '__main__':
    print("Don't try to run me! Look at the lab manual.")
