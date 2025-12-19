#!/usr/bin/env python3
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Naman Shah", "Rushang Karia"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Naman Shah"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import sys
import problem
import json
import os
import random
import utils
from tqdm.auto import trange
import time

from parser import parse_args
from server import initialize_planning_server
from server import generate_maze
from utils import initialize_ros
from utils import cleanup_ros


class QLearning:

    def __init__(self, objtypes, objcount, seed, file_path, alpha, gamma,
        episodes, max_steps, epsilon_task, env, clean):
        
        self.objtypes = objtypes
        self.objcount = objcount
        self.seed = seed
        self.epsilon_task = epsilon_task
        self.env = env
        self.obj_json_file = utils.ROOT_DIR + "/objects.json"
        self.obj = json.load(open(self.obj_json_file))
        self.helper = problem.Helper()
        self.helper.reset_world()
        
        assert not os.path.exists(file_path) or not os.path.isdir(file_path)
        
        self.file_path = file_path
        if clean:
            self.file_handle = open(file_path, "w")
            self.write_file_header(file_path)
        else:
            self.file_handle = open(file_path, "a")

        self.alpha = alpha
        self.gamma = gamma
        self.max_steps = max_steps

        q_values = self.learn(episodes)

        with open(utils.ROOT_DIR + "/q_values.json", "w") as fout:
            json.dump(q_values, fout)
            
    def write_file_header(self, file_path):
       
        with open(file_path, "w") as f:
            f.write("Env;Object Types;Num of Objects;Seed;Gamma;Episode #;Alpha;Epsilon;Cumulative Reward;Total Steps;Goal Reached\n")

    def write_to_file(self, file_path, episode_num, alpha, epsilon,
        cumulative_reward, total_steps, is_goal_satisfied):

        with open(file_path, "a") as f:
            f.write("%s;%u;%u;%u;%.6f;%u;%.6f;%.6f;%.2f;%u;%s\n" % (
                self.env,
                self.objtypes,
                self.objcount,
                self.seed,
                self.gamma,
                episode_num,
                alpha,
                epsilon,
                cumulative_reward,
                total_steps,
                is_goal_satisfied))

    def get_q_value(self,alpha, gamma, reward, q_s_a, q_s_dash_a_dash):
        '''
        Use the Q-Learning update rule to calculate and return the q-value.

        return type: float
        '''

        td_target = reward + gamma * q_s_dash_a_dash
        td_error = td_target - q_s_a
        q_new = q_s_a + alpha * td_error
        return q_new
    
    def compute_cumulative_reward(self, current_cumulative_reward, gamma, step, reward):
        '''
        Calculate the running cumulative reward at every step using 
        current value of the cumulative reward,
        discount factor (gamma), 
        current step number (step), 
        the rewards for the current state (reward)

        return type: float
        '''
        return current_cumulative_reward + (gamma ** step) * reward


        

    def get_epsilon(self, current_epsilon, episode):
        '''
        Calculate the value for decaying epsilon/
        
        Input: 
        current_epsilon: current value for the epsilon.
        episode: episode number

        Output:

        new value for the epsilon

        return type: float 
        '''

        new_epsilon = current_epsilon * 0.99
        if new_epsilon < 0.01:
            new_epsilon = 0.01
        return new_epsilon


    def alpha(self, current_alpha, episode, step):
        return current_alpha

    def learn(self, episodes):
        q_values = {} # Use this dictionary to keep track of q values

        root_path = os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir))
        actions_config_file = open(root_path + "/action_config.json",'r')
        actions_config = json.load(actions_config_file)

        objects_file = open(root_path + "/objects.json",'r')
        objects = json.load(objects_file)

        pick_loc=[]
        place_loc=[]
        for object in objects['object'].keys():
            for e in objects['object'][object]['load_loc']:
                pick_loc.append(e)
        
        for goal in objects['goal'].keys():
            for e in objects['goal'][goal]['load_loc']:
                place_loc.append(e)
        
        epsilon = 1.0
        for i in trange(0, episodes, desc="Episode", unit="episode"):

            epsilon = self.get_epsilon(epsilon, i) # Complete get_epsilon()
            curr_state = self.helper.get_current_state()
            cumulative_reward = 0
            
            for step in range(self.max_steps):

                if self.helper.is_terminal_state(curr_state):
                    break
                    
                actions_list = self.helper.get_all_actions()
                curr_loc = [curr_state['robot']['x'],curr_state['robot']['y']]    
                possible_actions_list = actions_list
                                    

                # ---------- Build a hashable state key ----------
                state_key = json.dumps(curr_state, sort_keys=True)

                # ---------- Initialize Q(s,·) for this state if unseen ----------
                if state_key not in q_values:
                    q_values[state_key] = {}
                    for a in possible_actions_list:
                        q_values[state_key][a] = 0.0

                # ---------- Epsilon-greedy action selection over FULL labels ----------
                # Example labels:
                #   "normal_moveF"
                #   "careful_TurnCW"
                #   "normal_pick book_1"
                #   "normal_place book_1 goal_1"
                if random.random() < epsilon:
                    # Explore
                    action_label = random.choice(possible_actions_list)
                else:
                    # Exploit: choose action with highest Q-value
                    state_qs = q_values[state_key]
                    max_q = max(state_qs.values())
                    best_actions = [a for a, q in state_qs.items() if q == max_q]
                    action_label = random.choice(best_actions)

                # ---------- Parse action_label into base_action + params ----------
                parts = action_label.split()
                base_action = parts[0]    # e.g. "normal_pick", "normal_place", "normal_moveF"
                action_params = {}

                # Pick up: "normal_pick object_name" or "careful_pick object_name"
                if base_action in ["normal_pick", "careful_pick"]:
                    # Format: "<base_action> <object_name>"
                    if len(parts) >= 2:
                        action_params["object_name"] = parts[1]

                # Place: "normal_place object_name goal_name" or "careful_place object_name goal_name"
                elif base_action in ["normal_place", "careful_place"]:
                    # Format: "<base_action> <object_name> <goal_name>"
                    if len(parts) >= 3:
                        action_params["object_name"] = parts[1]
                        action_params["goal_name"] = parts[2]

                # Move/turn actions have no params ("normal_moveF", "careful_moveF", "normal_TurnCW", etc.)
                # so action_params stays empty for those.

                # ---------- Execute action in the environment ----------
                # NOTE: we pass base_action (matches keys in action_config.json)
                success, next_state = self.helper.execute_action(base_action, action_params)

                # ---------- Get reward ----------
                # get_reward also expects the base_action key (not the full label)
                reward = self.helper.get_reward(curr_state, base_action, next_state)

                # ---------- Prepare Q(s',·) for next_state ----------
                next_state_key = json.dumps(next_state, sort_keys=True)
                next_actions_list = self.helper.get_all_actions()
                if next_state_key not in q_values:
                    q_values[next_state_key] = {}
                    for a in next_actions_list:
                        q_values[next_state_key][a] = 0.0

                # ---------- Compute max_a' Q(s', a') over FULL labels ----------
                max_q_next = max(q_values[next_state_key].values()) if q_values[next_state_key] else 0.0

                # ---------- Current Q(s,a) over FULL label ----------
                q_s_a = q_values[state_key][action_label]

                # ---------- Q-learning update ----------
                q_new = self.get_q_value(self.alpha, self.gamma, reward, q_s_a, max_q_next)
                q_values[state_key][action_label] = q_new

                # ---------- Update cumulative reward ----------
                cumulative_reward = self.compute_cumulative_reward(
                    cumulative_reward, self.gamma, step, reward
                )

                # ---------- Move to next state ----------
                curr_state = next_state


            self.write_to_file(self.file_path, i, self.alpha, epsilon,
                cumulative_reward, step, 
                self.helper.is_terminal_state(curr_state))
            self.helper.reset_world()

        return q_values

def run_qlearning(objtypes, objcount, seed, file_name, alpha, 
                  gamma, episodes, max_steps, epsilon_task, env, clean):
    
    file_path = utils.ROOT_DIR + "/" + file_name
    
    rosprocess = initialize_ros()
    planserver_process = initialize_planning_server()
    
    # Generate the world.
    generate_maze(objtypes, objcount, seed, 1, env)
   

    QLearning(objtypes, objcount, seed, file_path, alpha, 
        gamma, episodes, max_steps, epsilon_task, env, clean)
    
    cleanup_ros(planserver_process.pid, rosprocess.pid)
    time.sleep(2)


def submit(args):
    task_name = "task2"
    fname = "qlearning.csv"
    for i, env in enumerate(['cafeWorld','bookWorld']):
        print("Submission: running {} for {}".format(task_name, env))
        run_qlearning(objtypes=1, objcount=1, seed=100,
                      file_name=fname, alpha=0.3, gamma=0.9,
                      episodes=500, max_steps=500, epsilon_task=2,
                      env=env, clean=not(i))


if __name__ == "__main__":

    random.seed(111)

    args = parse_args()

    if args.submit:
        submit(args)
    else:
        task_name = "task2"
        fname = "qlearning.csv"
        run_qlearning(objtypes=args.objtypes, objcount=args.objcount, seed=args.seed,
                      file_name=args.file_name, alpha=args.alpha, gamma=args.gamma,
                      episodes=args.episodes, max_steps=args.max_steps, epsilon_task=2,
                      env=args.env, clean=args.clean)