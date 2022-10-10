#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Abhyudaya Srinet"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import sys
import problem
import json
import os
import random
import utils
#import self
from tqdm.auto import trange

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

    def get_q_value(self, alpha, gamma, reward, q_s_a, q_s_dash_a_dash):
        '''
        Use the Q-Learning update rule to calculate and return the q-value.

        return type: float
        '''

        '''
        YOUR CODE HERE
        '''
        # print(q_s_dash_a_dash)
        Q_value = (1- alpha)*q_s_a + alpha*(reward+ gamma*max(q_s_dash_a_dash))
        return Q_value
    
    def compute_cumulative_reward(self, current_cumulative_reward, gamma, step, reward):
        '''
        Calculate the running cumulative reward at every step using 
        current value of the cumulative reward,
        discount factor (gamma), 
        current step number (step), 
        the rewards for the current state (reward)

        return type: float
        '''


        '''
        YOUR CODE HERE
        '''

        R_cummulative = current_cumulative_reward+(gamma**step)*reward
        return R_cummulative

    def get_epsilon(self, current_epsilon, episode, epsilon_task):

        if epsilon_task == 1:
            return max(0.05, 1.0 - 0.05 * episode)
        elif epsilon_task == 2:
            '''
            YOUR CODE HERE
            '''

            return max(0.01, current_epsilon*0.99)
        else:
        
            # Should not be here.
            assert False
        
    def alpha(self, current_alpha, episode, step):
        return current_alpha

    def learn(self, episodes):
        """
        q_val = {
            "Curr_state": {
                "Action": "Qvalue",
            }
        }
        """
        q_values = {} # Use this dictionary to keep track of q values
        # q_values_reverse = {}

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
        print("^^^^^^^^^^^^^^^^^^^^^")
        print(pick_loc)
        print(place_loc)
        print("^^^^^^^^^^^^^^^^^^^^^")
        for i in trange(0, episodes, desc="Episode", unit="episode"):

            epsilon = self.get_epsilon(epsilon, i, self.epsilon_task) # Complete get_epsilon()
            curr_state = self.helper.get_current_state()
            cumulative_reward = 0

            print("epsilon:" + str(epsilon))

            is_picked_up = False

            reward_sum = 0
            
            for step in trange(0, self.max_steps, leave=False, desc="Step", unit="step"):

                curr_loc = [curr_state['robot']['x'],curr_state['robot']['y']]   

                curr_loc = tuple(curr_loc)
                # print self.helper.is_terminal_state(curr_state), curr_loc
                if self.helper.is_terminal_state(curr_state):
                    break
                    
                actions_list = self.helper.get_all_actions() 
                possible_actions_list = actions_list

                if list(curr_loc) in pick_loc and not is_picked_up:
                    possible_actions_list = [action for action in actions_list if actions_config[action.split()[0]]['function'] != 'execute_place']
                elif list(curr_loc) in place_loc and is_picked_up:
                    possible_actions_list = [action for action in actions_list if actions_config[action.split()[0]]['function'] != 'execute_pick']
                else:
                    possible_actions_list = [action for action in actions_list if actions_config[action.split()[0]]['function'] != 'execute_place' and actions_config[action.split()[0]]['function'] != 'execute_pick']
                       
                # explore or exploit
                action = random.choice(possible_actions_list)

                if random.random() > epsilon:
                    # get the best possible actions
                    if curr_loc not in q_values:
                        break
                    if len(q_values[curr_loc].values()) == 0:
                        break
                    q_max = max(q_values[curr_loc].values())
                    action = None
                    for a in q_values[curr_loc]:
                        if(q_values[curr_loc][a] == q_max):
                            action = a
                            break
                    # action = q_values_reverse[curr_loc][]
                

                '''
                YOUR CODE HERE
                '''
                # rev_map = {}
                # for action in possible_actions_list:
                splitted_action = action.split()
                action_params = actions_config[splitted_action[0]]

                if action_params["function"] == 'execute_pick':
                    is_picked_up = True
                if action_params["function"] == 'execute_place':
                    is_picked_up = False
                # action_params['params'] = map(lambda x: x.encode("ascii"), splitted_action[1:])
                params = {}
                for idx, param in enumerate(action_params['params']):
                    try:
                        params[param] = splitted_action[idx+1]
                    except:
                        print action, action in possible_actions_list, splitted_action, action_params

                # try:
                success, next_state = self.helper.execute_action(splitted_action[0], params)
                # except:
                    # print(splitted_action, action_params)
                    # return
                next_loc = (next_state['robot']['x'],next_state['robot']['y'])

                reward = self.helper.get_reward(curr_state, splitted_action[0], next_state)

                q_s_a = 0
                if curr_loc in q_values:
                    if action in q_values[curr_loc]:
                        q_s_a = q_values[curr_loc][action]
                else:
                    q_values[curr_loc] = {}

                q_sd_ad = [0]
                if next_loc in q_values:
                    if len(q_values[next_loc].values()) != 0:
                        q_sd_ad = list(q_values[next_loc].values())
                else:
                    q_values[next_loc] = {}
                q_val = self.get_q_value(self.alpha, self.gamma, reward, q_s_a, q_sd_ad)
                # rev_map[q_val] = (splitted_action[0], reward)
                q_values[curr_loc][action] = q_val
                # # print q_values, 123
                # # return
                # max_q_val = max(rev_map.keys())
                cumulative_reward = self.compute_cumulative_reward(cumulative_reward, self.gamma, step, reward)
                curr_state = next_state

                step = step + 1
                # if actions_config[action.split()[0]]['function'] == 'execute_pick':
                #     print("I Picked")
                # if actions_config[action.split()[0]]['function'] == 'execute_place':
                #     print("I Placed")

                reward_sum = reward + reward_sum
            print reward_sum
            # print self.helper.is_terminal_state(curr_state)
            self.write_to_file(self.file_path, i, self.alpha, epsilon, cumulative_reward, step,  self.helper.is_terminal_state(curr_state))
            self.helper.reset_world()

        q_values_str = {}
        for state in q_values:
            action_val = q_values[state]
            state_str = str(state)
            q_values_str[state_str] = action_val 
        return q_values_str

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


def submit(args):
    task_name = "task2"
    fname = task_name+".csv"
    for i, env in enumerate(['cafeWorld','bookWorld']):
        print("Submission: running {} for {}".format(task_name, env))
        run_qlearning(objtypes=1, objcount=1, seed=32,
                      file_name=fname, alpha=0.3, gamma=0.9,
                      episodes=500, max_steps=500, epsilon_task=args.epsilon_task,
                      env=env, clean=not(i))


if __name__ == "__main__":

    random.seed(111)

    args = parse_args()

    if args.submit:
        submit(args)
    else:
        run_qlearning(args.objtypes, args.objcount, args.seed,
                      args.file_name, args.alpha, args.gamma,
                      args.episodes, args.max_steps, args.epsilon_task, 
                      args.env, args.clean)