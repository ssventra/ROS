#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Abhyudaya Srinet", "Pulkit Verma", "Rushang Karia"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import rospy
import problem
import heapq
import argparse
import os
import json
import random
from std_msgs.msg import String

from parser import parse_args
from server import initialize_planning_server
from server import generate_maze
from utils import initialize_ros
from utils import cleanup_ros

from hw2.srv import MoveActionMsg
from hw2.srv import PlaceActionMsg
from hw2.srv import PickActionMsg

import planner
import utils
import search
import traceback
import sys

"""
Do not change anything above this line, except if you want to import some package.
"""

class Refine:
	"""
	This class has code to refine the high level actions used by PDDL to the the low level actions used by the TurtleBot.

	"""

	def __init__(self, action_list):
		"""
		:param plan_file: Path to the file where plan is stored. This is generally stored at the path from where the planner was called.
		:type plan_file: str
		:param planner: Planner that was used to generate this plan. This is needed to parse the plan and convert to/store in a data structure in Python. Only possible values are FF, FD, and PP.
		:type planner: str 

		Attributes:
			**status_subscriber**: To subscribe to the "/status" topic, which gives the State of the robot.

			**actions_queue** (tuple(action_name, action_params)): Used to store the refined action tuples. You can decide what parameters you want to store for each action. Store the parameters that you need to pass to execute_<action_name>_action methods of RobotActionServer class.

			**action_index** (int): Stores the index of action that needs to be sent to RobotActionServer for execution.
		"""

		self.helper = problem.Helper()
		with open('%s/objects.json' % (utils.ROOT_PATH)) as f:
			self.env_data = json.load(f)

		self.action_index = 0
		self.actions_queue = self.refine_plan(action_list)

	def get_load_locations(self, location):
		obj = location[:location.index("_iloc")]

		if obj in self.env_data["object"].keys():
			return self.env_data["object"][obj]["load_loc"]
		elif obj in self.env_data["goal"].keys():
			return self.env_data["goal"][obj]["load_loc"]
		else:
			exit(-1)

	def refine_plan(self, action_list):
		"""
		Perform downward refinement to convert the high level plan into a low level executable plan.
		
		:param plan_file: Path to the file where plan is stored. This is generally stored at the path from where the planner was called.
		:type plan_file: str
		:param planner: Planner that was used to generate this plan. This is needed to parse the plan and convert to/store in a data structure in Python. Only possible values are FF, FD, and PP.
		:type planner: str 

		:returns: List of refined action tuples that will be added to the execution queue.
		:rtype: list(tuples)

		.. note::

			.. hlist::
				:columns: 1
				
				* Parse the plan from plan_file. This has to be according to the planner you are using.
				* use get_path(current_state, load_locations) to refine Move action.
				* Type of the object and goal does not match.
				* Robot is not in the viscinity of the goal.
		"""

		current_state = self.helper.get_initial_state()
		actions = []
		print("BOO")
		for high_level_action in action_list:
			
			
			#This step performs downward refinement.
			if(high_level_action[0] == "move"):
				print("HI")
				loc_load=self.get_load_locations(high_level_action[3])
				action_lists,final_state,reach= self.get_path(current_state,loc_load)
				print action_lists
				actions.append(action_lists)
			elif(high_level_action[0] == "pick"):
				obj = high_level_action[1]
				actions.append(("pick", obj))
				self.helper.remove_edge(obj)
			elif(high_level_action[0] == "place"):
				obj = high_level_action[1]
				target = high_level_action[3]
				actions.append(("place", obj, target))

		return actions

	# --------------- HELPER FUNCTIONS --------------- #

	def is_goal_state(self, current_state, goal_state):
		"""
		Checks if the current_state is goal_state or not. 
		If you are wondering why we are checking orientation, remember this is a different Homework. :)

		"""
		if(current_state.x == goal_state.x and current_state.y == goal_state.y and current_state.orientation == goal_state.orientation):
			return True
		return False

	def get_manhattan_distance(self, from_state, to_state):
		"""
		Returns the manhattan distance between 2 states
		
		"""
		return abs(from_state.x - to_state.x) + abs(from_state.y - to_state.y)

	def build_goal_states(self, locations):
		"""
		Creates a State representations for given list of locations
		
		"""
		states = []
		for location in locations:
			states.append(problem.State(location[0], location[1], "EAST"))
			states.append(problem.State(location[0], location[1], "WEST"))
			states.append(problem.State(location[0], location[1], "NORTH"))
			states.append(problem.State(location[0], location[1], "SOUTH"))
		return states


	def get_path(self, init_state, goal_locations):
		"""
		This method searches for a path from init_state to one of the possible goal_locations

		:param init_state: Current state of robot
		:type init_state: State
		:param goal_locations: list of target locations to search the path e.g. [(x1, y1), (x2, y2), ..]. This is important if there are multiple objects of a type.
		:type goal_locations: list(State)

		:returns: 
			.. hlist::
				:columns: 1

				* **action_list**: list of actions to execute to go from source to target
				* **final_state**: target state that is reached (will be one of goal_locations)
				* **goal_reached**: True/False indicating if one of the goal_locations was reached
		
		"""
		final_state = None
		goal_states = self.build_goal_states(goal_locations)
		goal_reached = False

		for goal_state in goal_states:
		
			action_list, nodes_expanded = search.search(init_state, goal_state, self.helper, "gbfs")
			if action_list is not None:

				return action_list, goal_state, True

		return [], init_state, False

def generate_plan(env):
	'''
	Run a planner to evaluate problem.pddl and domain.pddl to generate a plan.
	Writes the plan to an output file
	'''
	domain_file_path = utils.DOMAIN_FILEPATH[:-5] + "_" + env + ".pddl"
	action_list = planner.run_planner("fd", domain_file_path,
		utils.PROBLEM_FILEPATH)

	return action_list
  
if __name__ == "__main__":

	random.seed(0xDEADC0DE)

	args = parse_args()
	roscore_process = initialize_ros()
	server_process = initialize_planning_server()
	
	# Generate the world.
	generate_maze(args.objtypes, args.objcount, args.seed, args.env)

	if args.generate_only:
	
		sys.exit(0)
	
	FILE_PATH = utils.ROOT_PATH + "/" + args.file_name
	if os.path.exists(FILE_PATH) and args.clean:

		assert not os.path.isdir(FILE_PATH)                
		os.remove(FILE_PATH)

	if os.path.exists(FILE_PATH):
	
		assert not os.path.isdir(FILE_PATH)        
		file_handle = open(FILE_PATH, "a")
	else:
	
		file_handle = open(FILE_PATH, "w")
		file_handle.write("Object_types; object_count; seed; exception; refined_plan\n")
	
	try:

		# Call the external planner for finding a high level plan.
		action_list = generate_plan(args.env)
		#print(args)
		
		# Perform downward refinement.
		refinement = Refine(action_list)
		#print(refinement.actions_queue)

		file_handle.write("%u; %u; %u; %s; %s; %s\n" % (args.objtypes,
			args.objcount,
			args.seed,
			None,
			str(refinement.actions_queue),args.env))
	except Exception as e:
	
		print("Exception caught!")
		file_handle.write("%u; %u; %u; %s; %s; %s\n" % (args.objtypes,
			args.objcount,
			args.seed,
			type(e),
			[],args.env))

		traceback.print_exc()

	# Cleanup ROS core.
	cleanup_ros(roscore_process.pid, server_process.pid)
