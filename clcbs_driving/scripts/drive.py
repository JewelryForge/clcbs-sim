#!/usr/bin/env python3
import argparse
import yaml
import math
import numpy as np


class StateManager:
    def __init__(self, name, states) -> None:
        self.name, self.states = name, states
        print(self.name, self.states)
        print()

    def get_state(self, t):
        idx = int(t)

        if idx == 0:
            start_state = self.states[0]
            return np.array([start_state['x'], start_state['y'], start_state['yaw']])
            
        elif idx < len(self.states):
            last_state, next_state = self.states[idx - 1], self.states[idx]
            last_yaw, next_yaw = last_state['yaw'], next_state['yaw']

            if (last_yaw - next_yaw) > math.pi:
                last_yaw = last_yaw - 2 * math.pi
            elif (next_yaw - last_yaw) > math.pi:
                last_yaw = last_yaw + 2 * math.pi

            last_pos = np.array([last_state['x'], last_state['y'], last_yaw])
            next_pos = np.array([next_state['x'], next_state['y'], next_yaw])

            dt = next_state['t'] - last_state['t']
            ratio = (t - last_state['t']) / dt
            interp = (next_pos - last_pos) * ratio + last_pos
            return interp
        else:
            final_state = self.states[-1]
            return np.array([final_state['x'], final_state['y'], final_state['yaw']])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--map", default=None,
                        help="input file containing map")
    parser.add_argument("-s", "--schedule", default=None,
                        help="schedule for agents")
    args = parser.parse_args()

    with open(args.map) as map_file:
        map = yaml.load(map_file, Loader=yaml.FullLoader)

    with open(args.schedule) as states_file:
        schedule: dict = yaml.load(
            states_file, Loader=yaml.FullLoader)['schedule']

    managers = []
    for name, sch in schedule.items():
        managers.append(StateManager(name, sch))
    
    m: StateManager = managers[0]
    print(m.get_state(0))
    print(m.get_state(1.5))
    print(m.get_state(10.2))
    print(m.get_state(15))
    # print(map)
    # print(schedule)
    # try:
    #     with open(os.path.abspath(os.path.join(
    #             os.getcwd(), "..")) + "/src/config.yaml") as config_file:
    #         carConfig = yaml.load(config_file, Loader=yaml.FullLoader)
    #         # global carWidth, LF, LB, obsRadius, framesPerMove
    #         carWidth = carConfig["carWidth"]
    #         LF = carConfig["LF"]
    #         LB = carConfig["LB"]
    #         obsRadius = carConfig["obsRadius"] - 0.1
    # except IOError:
    #     # do things with your exception
    #     print("ERROR loading config file", os.path.abspath(os.path.join(
    #         os.getcwd(), "..")) + "/src/config.yaml", " using default param to plot")
