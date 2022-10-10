#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2022, AAIR Lab, ASU"
__authors__ = ["Naman Shah", "Rushang Karia", "Rashmeet Kaur Nayyar", "Pulkit Verma"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.3"
__maintainers__ = ["Pulkit Verma", "Karthik Nelapati"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

from cmath import sqrt
import os
import time
import rospy
import argparse
import traceback
import subprocess
from std_msgs.msg import String

import problem 
from node import Node
from parser import parse_args
from utils import cleanup_ros
from server import generate_maze
from utils import initialize_ros
from priority_queue import PriorityQueue
from server import initialize_search_server

SUBMIT_FILENAME = "hw1_results.csv"
SUBMIT_SEARCH_TIME_LIMIT = 300

class SearchTimeOutError(Exception):
    pass

def is_invalid(state):
    """
        Parameters
        ===========
            state: State
                The state to be checked.
                
        Returns
        ========
            bool
                True if the state is invalid, False otherwise.
    """
    
    return state.x == -1 or state.y == -1


def compute_g(algorithm, node, goal_state):
    """
        Evaluates the g() value.

        Parameters
        ===========
            algorithm: str
                The algorithm type based on which the g-value will be computed.
            node: Node
                The node whose g-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The g-value for the node.
    """

    if algorithm == "bfs":
        '''
        YOUR CODE HERE
        '''
        return node.get_depth()

    if algorithm == "astar":
        '''
        YOUR CODE HERE
        '''
        return node.get_depth()
        #return node.get_total_action_cost()

    elif algorithm == "gbfs":
        '''
        YOUR CODE HERE
        '''
        return 0

    elif algorithm == "ucs":
        '''
        YOUR CODE HERE
        '''
        #return node.get_depth()
        return node.get_total_action_cost()

    elif algorithm == "custom-astar":
        '''
        YOUR CODE HERE
        '''
        return node.get_depth()

    # Should never reach here.
    assert False
    return float("inf")


def f_bfs(node, goal_state):


    """
        Evaluates the f() value for Best First Search.

        Parameters
        ===========
            node: Node
                The node whose f-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The f-value for the node.
    """

    return node.get_depth()
def manhattan_dist(node, goal_state):
    return abs(node.get_state().get_x()-goal_state.get_x())-abs(node.get_state().get_y()-goal_state.get_y())

def equclidean_dist(node,goal_state):
    return sqrt((node.get_state().get_x()-goal_state.get_x())**2+(node.get_state().get_y()-goal_state.get_y())**2)

def f_ucs(node, goal_state):
    """
        Evaluates the f() value for UCS.

        Parameters
        ===========
            node: Node
                The node whose f-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The f-value for the node.
    """

    '''
    YOUR CODE HERE
    '''
    return compute_g('ucs',node,goal_state)


def f_astar(node, goal_state):
    """
        Evaluates the f() value for A*.

        Parameters
        ===========
            node: Node
                The node whose f-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The f-value for the node.
    """

    '''
    YOUR CODE HERE
    '''

    return compute_g('astar', node, goal_state)+manhattan_dist(node, goal_state)



def f_gbfs(node, goal_state):
    """
        Evaluates the f() value for GBFS.

        Parameters
        ===========
            node: Node
                The node whose f-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The f-value for the node.
    """

    '''
    YOUR CODE HERE
    '''

    return compute_g('gbfs', node, goal_state)+manhattan_dist(node, goal_state)



def f_custom_astar(node, goal_state):
    """
        Evaluates the f() value for A* using a custom heuristic.

        Parameters
        ===========
            node: Node
                The node whose f-value is to be computed.
            goal_state: State
                The goal state for the problem.

        Returns
        ========
            int
                The f-value for the node.
    """

    '''
    YOUR CODE HERE
    '''

    return compute_g('custom-astar', node, goal_state)+manhattan_dist(node, goal_state)



def graph_search(algorithm, time_limit):
    """
        Performs a graph search using the specified algorithm.
        

        Maintains two importnt variables:
        1. action_list: It is a list and it contain the sequence of actions to execute to reach from init_state to goal_state.
        2. total_nodes_expanded: It is an integer and it maintains the total number of nodes expanded during the search.
        
        Parameters
        ===========
            algorithm: str
                The algorithm to be used for searching.
            time_limit: int
                The time limit in seconds to run this method for.
                
        Returns
        ========
            tuple(list, int)
                A tuple of the action_list and the total number of nodes
                expanded.
    """
    
    # The helper allows us to access the problem functions.
    helper = problem.Helper()

    # Dictionary to call correct functions to calculate f value of a node
    f_value = {'bfs': f_bfs, 'gbfs': f_gbfs, 'astar': f_astar, 'ucs': f_ucs, 'custom-astar': f_custom_astar}
    
    # Get the initial and the goal states.
    init_state = helper.get_initial_state()
    goal_state = helper.get_goal_state()[0]
    
    # Initialize the init node of the search tree and compute its f_score.
    init_node = Node(init_state, None, 0, None, 0)
    f_score = f_value[algorithm](init_node, goal_state)
    
    # Initialize the fringe as a priority queue.
    priority_queue = PriorityQueue()
    priority_queue.push(f_score, init_node)
    
    # action_list should contain the sequence of actions to execute to reach from init_state to goal_state
    action_list = []
    visited_list=[]

    # total_nodes_expanded maintains the total number of nodes expanded during the search
    total_nodes_expanded = 0
    time_limit = time.time() + time_limit
    
    # loop through the priority queue and pop the first node
    while not priority_queue.is_empty():
        #print(time.time() - time_limit)
        present_node = priority_queue.pop()
        if time.time() >= time_limit:
            raise SearchTimeOutError("Search timed out after %u secs." % (time_limit))
        if is_invalid(present_node.get_state()):
            continue
        if present_node.get_state() not in visited_list:
            total_nodes_expanded+=1
            visited_list.append(present_node.get_state())
            
            if present_node.get_state() == goal_state:
                print("reached the goal state")
                while present_node.get_parent() is not None:
                    action_list.append(present_node.get_action())
                    present_node=present_node.get_parent()
                return action_list, total_nodes_expanded
            else:
                succesor_nodes=helper.get_successor(present_node.get_state())
                if succesor_nodes is not None:
                    for i in succesor_nodes:
                        nxt_state, nxt_cost = succesor_nodes[i]
                        nxt_node = Node(nxt_state, present_node, present_node.get_depth()+1, i, nxt_cost)
                        priority_queue.push(f_value[algorithm](nxt_node,goal_state), nxt_node)
                    
                
            
    '''
    YOUR CODE HERE

    Your code for graph search should populate action_list and set total_nodes_expanded
    The automated script will verify their values

    In addition to this you must also write code for:
    1. f_ucs
    2. f_gbfs
    3. f_astar
    4. compute_g
    5. f_custom_astar (If attempting bonus question)
    '''


def submit(file_handle, env):
    """
        Runs the tests that need to be submitted as a part of this Homework.
        
        Parameters
        ===========
            file_handle: int
                The file descriptor where the results will be output.
    """
    
    SEEDS = [0xDEADC0DE, 0xBADC0D3, 0x500D]

    dim_pairs = {
        'canWorld':[

            # (Grid dimension, Num obstacles)
            # Each grid dimension contains runs with 0%, 10%, 20%, 30%, 40% of max
            # obstacles possible.
            ("4x4", 0), ("4x4", 4),
            ("8x8", 0), ("8x8", 14),
            ("12x12", 0), ("12x12", 31),
            ("16x16", 0), ("16x16", 54)
        ],
        'cafeWorld':[

            # (Grid dimension, Num obstacles)
            ("3x6",0),("3x6",1),("3x6",2),("3x6",4)
        ],

    }

    for env in ['canWorld','cafeWorld']:
        DIMENSION_PAIRS = dim_pairs[env]
        total = len(SEEDS) * len(DIMENSION_PAIRS)
        current = 0
        print("env=%s"% (env) )
        for dimension, obstacles in DIMENSION_PAIRS:
            for seed in SEEDS:
    
                current += 1 
                print("(%3u/%3u) Running dimension=%s, obstacles=%s, seed=%s" % (
                    current,
                    total,
                    dimension,
                    obstacles,
                    seed))
                
                run_search(file_handle, dimension, obstacles, seed, env, algorithms,
                    time_limit=SUBMIT_SEARCH_TIME_LIMIT, debug=False)
   
def run_search(file_handle, dimension, obstacles, seed, env, algorithms, 
    time_limit=float("inf"), debug=True):
    """
        Runs the search for the specified algorithms.
        
        Parameters
        ===========
            file_handle: int
                A descriptor for the output file where the results will be
                written.
            dimension: int
                The dimensions of the grid.
            obstacles: int
                The number of obstacles in the grid.
            seed: int
                The random seed to use in generating the grid.
            algorithms: list(str)
                The algorithms to run.
            time_limit: int
                The time limit in seconds.
            debug: bool
                True to enable debug output, False otherwise.
    """
    
    # Generate the world.
    dimension_x, dimension_y = dimension.split("x")
    dimension_x, dimension_y = int(dimension_x), int(dimension_y)
    generate_maze(dimension_x,dimension_y, obstacles, seed, env)
    
    # Run search for each algorithm.
    for algorithm in algorithms:
    
        error = "None"
        actions = []
        total_nodes_expanded = 0
        start_time = time.time()
        
        # Catch any errors and set the error field accordingly.
        try:
            actions, total_nodes_expanded = graph_search(algorithm, time_limit)
        except NotImplementedError:
        
            error = "NotImplementedError"
        except MemoryError:
        
            error = "MemoryError"
        except Exception as e:
        
            error = str(type(e))
            traceback.print_exc()
            
        time_taken = time.time() - start_time
        time_taken = "%.2f" % (time_taken)
        
        if debug:
        
            print("==========================")
            print("Dimension..........: " + str(dimension))
            print("Obstacles..........: " + str(obstacles))
            print("Seed...............: " + str(seed))
            print("Environment........: " + env)
            print("Algorithm..........: " + algorithm)
            print("Error..............: " + error)
            print("Time Taken.........: " + str(time_taken))
            print("Nodes expanded.....: " + (str(total_nodes_expanded)))
            print("Plan Length........: " + str(len(actions)))
            print("Plan...............: " + str(actions))
        
        if file_handle is not None:
        
            plan_str = '_'.join(action for action in actions)
            file_handle.write("%s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n" % (
                dimension, obstacles, seed, algorithm,
                time_taken, total_nodes_expanded, len(actions), error, 
                plan_str,env))


if __name__ == "__main__":

    # Parse the arguments.
    args = parse_args()

    # Check which algorithms we are running.
    if args.algorithm is None or "all" == args.algorithm or args.submit:
    
        algorithms = ["bfs", "ucs", "gbfs", "astar", "custom-astar"]
    else:
    
        algorithms = [args.algorithm]
    
    # Setup the output file.
    if args.output_file is not None:
        
        file_handle = open(args.output_file, "w")
    elif args.submit:
    
        file_name = os.path.join(os.path.dirname(__file__), SUBMIT_FILENAME)
        file_handle = open(file_name, "w")
    else:
    
        file_handle = None
    
    # Write the header if we are writing output to a file as well.
    if file_handle is not None:
    
        file_handle.write("Dimension, Obstacles, Seed, Algorithm, Time, "
            "Nodes Expanded, Plan Length, Error, Plan, Env\n")
    
    # Initialize ROS core.
    roscore_process = initialize_ros()

    # Initialize this node as a ROS node.
    rospy.init_node("search")
    
    # Initialize the search server.
    server_process = initialize_search_server()

    # If using submit mode, run the submission files.
    if args.submit:
    
        submit(file_handle, args.env)
    else:

        # Else, run an individual search.
        run_search(file_handle, args.dimension, args.obstacles, args.seed,
            args.env, algorithms)

    if file_handle is not None:
        file_handle.close()

    # Cleanup ROS core.
    cleanup_ros(roscore_process.pid, server_process.pid)
