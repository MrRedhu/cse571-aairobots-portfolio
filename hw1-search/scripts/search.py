#!/usr/bin/env python3
# encoding: utf-8

__copyright__ = "Copyright 2021, AAIR Lab, ASU"
__authors__ = ["Naman Shah"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import traceback

import problem 
import rospy
from std_msgs.msg import String
import argparse
from evaluate import graph_search
from node import Node
import os
from priority_queue import PriorityQueue
from parser import parse_args
from server import initialize_search_server
from server import generate_maze
import subprocess
import time
from utils import initialize_ros
from utils import cleanup_ros

SUBMIT_FILENAME = "submission/hw1_results.csv"
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

def build_solution(best_path, current_node):
    """
        Builds the solution from the best path recorded.
        
        Parameters
        ===========
            
            best_path: dict
                A map of state -> node representing the best path.
            current_node: Node
                The current node from which the path is to be recorded.
                
        Returns
        ========
            list
                A list of the actions that will lead from the initial state to
                the state represented by the current_node.
    """
    
    action_list = []
    
    # Iterate all the way until the parent.
    # The initial node's parent pointer will be None so we only check till we
    # reach the initial state.
    while current_node.get_parent() is not None:
    
        current_state = current_node.get_state()
        action_list.append(current_node.get_action())
        
        # Consult the map to get the next for the next state (in reverse order).
        current_node = best_path[current_state]
        
    # Since we went bottom-up, to ensure actions are executable, we need to 
    # reverse the action_list.
    #
    # Python's list.reverse() reverses in-place.
    action_list.reverse()
    
    return action_list


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
        
    print("Done")


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
    
        file_name = os.path.join(os.path.dirname(os.path.dirname(__file__)), SUBMIT_FILENAME)
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
