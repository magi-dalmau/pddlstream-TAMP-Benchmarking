from __future__ import print_function

import numpy as np
import os

from examples.pybullet.utils.pybullet_tools.pr2_problems import create_pr2, create_table, Problem
from examples.pybullet.utils.pybullet_tools.pr2_utils import get_other_arm, get_carry_conf, set_arm_conf, open_arm, \
    arm_conf, REST_LEFT_ARM, close_arm, set_group_conf
from examples.pybullet.utils.pybullet_tools.utils import get_bodies, sample_placement, pairwise_collision, \
    add_data_path, load_pybullet, set_point, Point, create_box, stable_z, joint_from_name, get_point, wait_for_user,\
    RED, GREEN, BLUE, BLACK, WHITE, BROWN, TAN, GREY

def sample_placements(body_surfaces, obstacles=None, min_distances={}):
    if obstacles is None:
        obstacles = [body for body in get_bodies() if body not in body_surfaces]
    obstacles = list(obstacles)
    # TODO: max attempts here
    for body, surface in body_surfaces.items():
        min_distance = min_distances.get(body, 0.01)
        while True:
            pose = sample_placement(body, surface)
            if pose is None:
                return False
            if not any(pairwise_collision(body, obst, max_distance=min_distance)
                       for obst in obstacles if obst not in [body, surface]):
                obstacles.append(body)
                break
    return True
######################################################
#Funcions de test_tamp_xml

import colorsys
import glob
import os
import pybullet as p
import random
import sys

import numpy as np
from lxml import etree

from examples.pybullet.utils.pybullet_tools.pr2_utils import DRAKE_PR2_URDF, set_group_conf
from examples.pybullet.utils.pybullet_tools.utils import STATIC_MASS, CLIENT, connect, \
    disconnect, set_pose, wait_if_gui, load_model, HideOutput, base_values_from_pose, create_shape, \
    get_mesh_geometry, point_from_pose, set_camera_pose, draw_global_system
from examples.pybullet.utils.pybullet_tools.utils import quaternion_from_matrix

# https://docs.python.org/3.5/library/xml.etree.elementtree.html
# https://lxml.de/tutorial.html

def parse_array(element):
    return np.array(element.text.split(), dtype=float)

def parse_pose(element):
    homogenous = [0, 0, 0, 1]
    matrix = np.reshape(np.concatenate([parse_array(element), homogenous]), [4, 4])
    point = matrix[:3, 3] 
    quat = quaternion_from_matrix(matrix)
    return (point, quat)

def parse_boolean(element):
    text = element.text
    if text == 'true':
        return True
    if text == 'false':
        return False
    raise ValueError(text)

MAX_INT = sys.maxsize + 1

def parse_object(obj, mesh_directory,blocks,surfaces,objects_dims,problem_id,avoid_meshes=False):
    name = obj.find('name').text
    mesh_filename = obj.find('geom').text
    geom = get_mesh_geometry(os.path.join(mesh_directory, mesh_filename))
    pose = parse_pose(obj.find('pose'))
    movable = parse_boolean(obj.find('moveable'))
    goal=""
    plate=-1

    color = (.75, .75, .75, 1)
    if 'red' in name:
        color = (1, 0, 0, 1)
        
    elif 'green' in name:
        color = (0, 1, 0, 1)
    elif 'blue' in name:
        color = (0, 0, 1, 1)
    elif movable: # TODO: assign new color
        #size = 2 * MAX_INT
        #size = 255
        #n = id(obj) % size
        #n = hash(obj) % size
        #h = float(n) / size
        h = random.random()
        r, g, b = colorsys.hsv_to_rgb(h, .75, .75)
        color = (r, g, b, 1)
    print(name, mesh_filename, movable, color)

    
    if avoid_meshes:
        if 'stick' in name:            
            body_id=create_box(objects_dims["block_width"], objects_dims["block_width"],  objects_dims["block_height"], color=color)
            point=pose[0]
            point[2]=point[2]+0.5*objects_dims["block_height"]
            new_pose=(point,pose[1])
            set_pose(body_id, new_pose)
        elif 'table' in name:
            body_id=create_table(width=objects_dims["table_x"],length=objects_dims["table_y"],height=objects_dims["table_height"],thickness=objects_dims["table_thickness"])
            set_pose(body_id, pose)
            # if problem_id==4:
            #     if '3' in name:                
            #         plate = create_box(objects_dims["table_x"], objects_dims["table_y"], 0.001, color=BLUE)
            #         plate_z = stable_z(plate, body_id)                
            #         set_point(plate, Point(x=pose[0][0],y=pose[0][1],z=plate_z))
            #         surfaces.append(plate)
            #         print("Added plate "+str(plate)+" in table "+name+" for colour BLUE")
            #     elif '4' in name:
            #         plate = create_box(objects_dims["table_x"], objects_dims["table_y"], 0.001, color=GREEN)
            #         plate_z = stable_z(plate, body_id)
            #         set_point(plate, Point(x=pose[0][0],y=pose[0][1],z=plate_z))
            #         surfaces.append(plate)
            #         print("Added plate "+str(plate)+" in table "+name+" for colour GREEN")

            # elif problem_id==3:
            #     if '2' in name:
            #         print("pass")                
            #         # plate = create_box(objects_dims["table_x"], objects_dims["table_y"], 0.001, color=BLUE)
            #         # plate_z = stable_z(plate, body_id)                
            #         # set_point(plate, Point(x=pose[0][0],y=pose[0][1],z=plate_z))
            #         # surfaces.append(plate)
            #         # print("Added plate "+str(plate)+" in table "+name+" for colour BLUE")

            #     elif '3' in name:
            #         plate = create_box(objects_dims["table_x"], objects_dims["table_y"], 0.001, color=RED)
            #         plate_z = stable_z(plate, body_id)
            #         set_point(plate, Point(x=pose[0][0],y=pose[0][1],z=plate_z))
            #         surfaces.append(plate)
            #         print("Added plate "+str(plate)+" in table "+name+" for colour RED")


        else:
            collision_id, visual_id = create_shape(geom, color=color)
            body_id = p.createMultiBody(baseMass=STATIC_MASS, baseCollisionShapeIndex=collision_id,
                                baseVisualShapeIndex=visual_id, physicsClientId=CLIENT)
            set_pose(body_id, pose)
    else:    
        collision_id, visual_id = create_shape(geom, color=color)
        body_id = p.createMultiBody(baseMass=STATIC_MASS, baseCollisionShapeIndex=collision_id,
                                baseVisualShapeIndex=visual_id, physicsClientId=CLIENT)
        set_pose(body_id, pose)
    
    
    
    if movable:
        blocks.append(body_id)
        if problem_id==4:
            if 'blue' in name:
                goal='table3'
            elif 'green' in name:
                goal='table4'
            else: 
                goal=''
        elif problem_id==3:
            if 'blue' in name:
                goal='table2'
            elif 'red' in name:
                goal='table3'
            else: 
                goal=''
        else: 
            goal=''

    elif 'table' in name:
        surfaces.append(body_id)        
        goal=name

    return body_id,movable,goal,plate

def parse_robot(robot):
    name = robot.find('name').text
    urdf = robot.find('urdf').text
    fixed_base = not parse_boolean(robot.find('movebase'))
    print(name, urdf, fixed_base)
    pose = parse_pose(robot.find('basepose'))
    torso = parse_array(robot.find('torso'))
    left_arm = parse_array(robot.find('left_arm'))
    right_arm = parse_array(robot.find('right_arm'))
    assert (name == 'pr2')

    with HideOutput():
        robot_id = load_model(DRAKE_PR2_URDF, fixed_base=True)
    set_pose(robot_id, pose)
    #set_group_conf(robot_id, 'base', base_values_from_pose(pose))
    set_group_conf(robot_id, 'torso', torso)
    set_group_conf(robot_id, 'left_arm', left_arm)
    set_group_conf(robot_id, 'right_arm', right_arm)
    #set_point(robot_id, Point(z=point_from_pose(pose)[2]))
    # TODO: base pose isn't right
    # print(robot.tag)
    # print(robot.attrib)
    # print(list(robot.iter('basepose')))
    return robot_id

DISC = 'disc'
BLOCK = 'cube'
PEG = 'peg'

#######################################################

def packed(arm='left', grasp_type='side', num=5):
    # TODO: packing problem where you have to place in one direction
    base_extent = 5.0

    base_limits = (-base_extent/2.*np.ones(2), base_extent/2.*np.ones(2))
    block_width = 0.07
    block_height = 0.1
    #block_height = 2*block_width
    block_area = block_width*block_width

    #plate_width = 2*math.sqrt(num*block_area)
    #plate_width = 0.27 #el que fan servir
    plate_width = 0.5
    #plate_width = 0.3
    print('Width:', plate_width)
    plate_width = min(plate_width, 0.6)
    plate_height = 0.001

    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    add_data_path()
    floor = load_pybullet("plane.urdf")
    pr2 = create_pr2()
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)
    set_group_conf(pr2, 'base', [-1.0, 0, 0]) # Be careful to not set the pr2's pose

    table = create_table()
    plate = create_box(plate_width, plate_width, plate_height, color=GREEN)
    plate_z = stable_z(plate, table)
    set_point(plate, Point(z=plate_z))
    surfaces = [table, plate]

    blocks = [create_box(block_width, block_width, block_height, color=BLUE) for _ in range(num)]
    initial_surfaces = {block: table for block in blocks}

    min_distances = {block: 0.05 for block in blocks}
    sample_placements(initial_surfaces, min_distances=min_distances)

    return Problem(robot=pr2, movable=blocks, arms=[arm], grasp_types=[grasp_type], surfaces=surfaces,
                   #goal_holding=[(arm, block) for block in blocks])
                   goal_on=[(block, plate) for block in blocks], base_limits=base_limits)


######################################################
def benchmark(arm='left', grasp_type='side', num=5):
    benchmark = 'tmp-benchmark-data'
    #problem = 'problem1' # Hanoi
    #problem = 'problem2' # Blocksworld
    # problem = 'problem3' # Clutter
    problem = 'problem3-simple'
    #problem = 'problem4' # Nonmono
    # TODO: Fer una versió sense fer servir meshes per esquivar l'error de pybullet que no posa l'arxiu de la collision mesh.
    problem_id=1 #4 full benchkmark; 3: 3 tables version; 1 one table version with specific placements
    avoid_mesh=True #Change mesh for harcode shapes
    objects_dims={}
    objects_dims["block_height"] = 0.2 #TODO check dimensions
    objects_dims["block_width"] = 0.04 #TODO check dimensions
    objects_dims["table_height"]=0.767
    objects_dims["table_x"]=1.5
    objects_dims["table_y"]=0.7
    objects_dims["table_thickness"]=0.02 #TODO check dimensions
    # x_extent = 5.5 #TODO check base limits 
    x_extent = 10 #TODO check base limits 

    base_limits = (-x_extent/2.*np.ones(2), x_extent/2.*np.ones(2))

    root_directory = os.path.dirname(os.path.abspath(__file__))
    #directory = os.path.join(root_directory, '..', 'problems', benchmark, problem)
    directory= os.path.join("/home/magi.dalmau/git/pddlstream/examples/pybullet/utils/problems", benchmark, problem) #TODO put the path nicely
    [mesh_directory] = list(filter(os.path.isdir, (os.path.join(directory, o)
                                                 for o in os.listdir(directory) if o.endswith('meshes'))))
    # [xml_path] = [os.path.join(directory, o) for o in os.listdir(directory) if o.endswith('xml')]
    # if os.path.isdir(xml_path):
    #     xml_path = glob.glob(os.path.join(xml_path, '*.xml'))[0]
    xml_path=os.path.join(directory,"clutter_"+str(problem_id)+"_tables.xml")

    print(mesh_directory)
    print(xml_path)
    xml_data = etree.parse(xml_path)
 


    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)
    add_data_path()
    floor = load_pybullet("plane.urdf")
    #PR2 stuff
    pr2 = create_pr2()
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)
    set_group_conf(pr2, 'base', [-1.0, 0, 0]) # Be careful to not set the pr2's pose
    
    #OBJECTS and Goals
    blocks=[]
    surfaces=[]
    goal_on=[]
    goal_pose=[]
    goal_pose_map={}
    object_name_map={}
    goal_prov={}
    table_map={}
    for obj in xml_data.findall('/objects/obj'):
        name = obj.find('name').text
        if 'target' in name:
            targ_object=name.split('_')[1]
            pose= parse_pose(obj.find('pose'))
            goal_pose_map[targ_object]=pose
        else:    
            id,movable,goal,plate= parse_object(obj, mesh_directory,blocks,surfaces,objects_dims,problem_id,avoid_meshes=avoid_mesh)  
            if movable:
                if problem_id==1:
                    object_name_map[name.split('_')[1]]=id
                else:    
                    if 'table' in goal:
                        goal_prov[id]=goal
            elif 'table' in goal:
                # if plate!=-1:
                    # table_map[goal]=plate
                table_map[goal]=id
    #Set goals
    # print("appending goals")
    # print( goal_prov)
    
    for key, value in goal_prov.items():
        goal_on.append((key,table_map[value]))
    for key, value in goal_pose_map.items():
        goal_pose.append(((object_name_map[key],table_map['table1']),value))
    

    print("goals are:")
    print(goal_on)

    return Problem(robot=pr2, movable=blocks, arms=[arm], grasp_types=[grasp_type], surfaces=surfaces,
                   #goal_holding=[(arm, block) for block in blocks])
                   goal_on=goal_on,goal_pose=goal_pose, base_limits=base_limits)




#######################################################

def blocked(arm='left', grasp_type='side', num=1):
    x_extent = 10.0

    base_limits = (-x_extent/2.*np.ones(2), x_extent/2.*np.ones(2))
    block_width = 0.07
    #block_height = 0.1
    block_height = 2*block_width
    #block_height = 0.2
    plate_height = 0.001
    table_x = (x_extent - 1) / 2.

    other_arm = get_other_arm(arm)
    initial_conf = get_carry_conf(arm, grasp_type)

    add_data_path()
    floor = load_pybullet("plane.urdf")
    pr2 = create_pr2()
    set_arm_conf(pr2, arm, initial_conf)
    open_arm(pr2, arm)
    set_arm_conf(pr2, other_arm, arm_conf(other_arm, REST_LEFT_ARM))
    close_arm(pr2, other_arm)
    set_group_conf(pr2, 'base', [x_extent/4, 0, 0]) # Be careful to not set the pr2's pose

    table1 = create_table()
    set_point(table1, Point(x=+table_x, y=0))
    table2 = create_table()
    set_point(table2, Point(x=-table_x, y=0))
    #table3 = create_table()
    #set_point(table3, Point(x=0, y=0))

    plate = create_box(0.6, 0.6, plate_height, color=GREEN)
    x, y, _ = get_point(table1)
    plate_z = stable_z(plate, table1)
    set_point(plate, Point(x=x, y=y-0.3, z=plate_z))
    #surfaces = [table1, table2, table3, plate]
    surfaces = [table1, table2, plate]

    green1 = create_box(block_width, block_width, block_height, color=BLUE)
    green1_z = stable_z(green1, table1)
    set_point(green1, Point(x=x, y=y+0.3, z=green1_z))
    # TODO: can consider a fixed wall here instead

    spacing = 0.15

    #red_directions = [(-1, 0), (+1, 0), (0, -1), (0, +1)]
    red_directions = [(-1, 0)]
    #red_directions = []
    red_bodies = []
    for red_direction in red_directions:
        red = create_box(block_width, block_width, block_height, color=RED)
        red_bodies.append(red)
        x, y = get_point(green1)[:2] + spacing*np.array(red_direction)
        z = stable_z(red, table1)
        set_point(red, Point(x=x, y=y, z=z))

    wall1 = create_box(0.01, 2*spacing, block_height, color=GREY)
    wall2 = create_box(spacing, 0.01, block_height, color=GREY)
    wall3 = create_box(spacing, 0.01, block_height, color=GREY)
    z = stable_z(wall1, table1)
    x, y = get_point(green1)[:2]
    set_point(wall1, Point(x=x+spacing, y=y, z=z))
    set_point(wall2, Point(x=x+spacing/2, y=y+spacing, z=z))
    set_point(wall3, Point(x=x+spacing/2, y=y-spacing, z=z))

    green_bodies = [create_box(block_width, block_width, block_height, color=BLUE) for _ in range(num)]
    body_types = [(b, 'green') for b in [green1] + green_bodies] #  + [(table1, 'sink')]

    movable = [green1] + green_bodies + red_bodies
    initial_surfaces = {block: table2 for block in green_bodies}
    sample_placements(initial_surfaces)

    return Problem(robot=pr2, movable=movable, arms=[arm], grasp_types=[grasp_type], surfaces=surfaces,
                   #sinks=[table1],
                   #goal_holding=[(arm, '?green')],
                   #goal_cleaned=['?green'],
                   goal_on=[('?green', plate)],
                   body_types=body_types, base_limits=base_limits, costs=True)

#######################################################


PROBLEMS = [
    packed,
    blocked,
    benchmark,
]