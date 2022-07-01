import heapq
import sys
import os
import timeit

from game_env import GameEnv

"""
solution.py

Template file for you to implement your solution to Assignment 1.

This file should include a 'main' method, allowing this file to be executed as a program from the
command line.

The program accepts 3 command line arguments:
    1) input filename
    2) output filename
    3) mode (either 'ucs' or 'a_star')
"""


#
#
# Code for any classes or functions you need can go here.
#
#

class StateNode:
    """Represents any node in the game state."""

    def __init__(self, env, state, actions, path_cost):
        self.env = env
        self.state = state
        self.actions = actions
        self.path_cost = path_cost

    def get_successors(self):
        successors = []
        for a in GameEnv.ACTIONS:
            success, new_state = self.env.perform_action(self.state, a)
            if success:
                successors.append(StateNode(self.env,
                                            new_state,
                                            self.actions + [a],
                                            self.path_cost + GameEnv.ACTION_COST[a]))
        return successors

    def __lt__(self, other):
        # we won't use this as a priority directly, so result doesn't matter
        return True


""""""""""" HEURISTICS """""""""""


def simple_weighted_manhattan_dist_heuristic(env, state):
    """
        Manhattan Distance between current player position and goal state, weighted by a lower
        bounds for all possible action costs.
    """
    min_action_cost = 1 / 6
    return min_action_cost * (abs(env.exit_row - state.row) + abs(env.exit_col - state.col))


def complex_weighted_manhattan_dist_heuristic(env, state):
    """
    Manhattan Distance between current player position and goal state, weighted by lower bounds
    of the costs of each action.
    """
    min_action_cost = 1 / 6  # minimum possible cost of an action (d3 / 3)
    min_up_cost = 2  # Jump is only way to go up and has cost 2
    min_left_right_cost = 0.3  # Lower bound on cost of all left/right moves (inc. gliding and
    # walking)

    if env.exit_row < state.row:  # If exit is above player
        h = min_up_cost * abs(env.exit_row - state.row) +\
            min_left_right_cost * abs(env.exit_col - state.col)
    else:
        h = min_action_cost * abs(env.exit_row - state.row) +\
            min_left_right_cost * abs(env.exit_col - state.col)
    return h


def perform_search(mode, heuristic, game_env, initial_state, verbose=True):
    """
    Performs either UCS or a_star, based on mode.

    HEAVILY BASED OFF OF TUTORIAL 3 SOLUTION CODE.

    :param mode: type of search
    :param heuristic: if mode == 'a_star', the heuristic to use
    :param game_env: the game environment
    :param initial_state: the initial state of the game environment
    :param verbose: for output logging
    """

    # Used as indicator function to switch between UCS and A*
    is_a_star = mode == 'a_star'

    container = [(0 + heuristic(game_env, initial_state) * is_a_star,
                  StateNode(game_env, initial_state, [], 0))]
    heapq.heapify(container)

    # dict: state --> path_cost
    visited = {initial_state: 0}
    n_expanded = 0
    while len(container) > 0:
        n_expanded += 1
        _, node = heapq.heappop(container)

        # check if this state is the goal
        if game_env.is_solved(node.state):
            if verbose:
                print(f'Visited Nodes: {len(visited.keys())},\t\tExpanded Nodes: {n_expanded},\t\t'
                      f'Nodes in Container: {len(container)}')
                print(f'Cost of Path (with Costly Moves): {node.path_cost}')
            return node.actions

        # add unvisited (or visited at higher path cost) successors to container
        successors = node.get_successors()
        for s in successors:
            if s.state not in visited.keys() or s.path_cost < visited[s.state]:
                visited[s.state] = s.path_cost
                heapq.heappush(container, (s.path_cost + heuristic(game_env, s.state) *
                                           is_a_star, s))

    return None


def write_output_file(filename, actions):
    """
    Write a list of actions to an output file.
    :param filename: name of output file
    :param actions: list of actions where is action an element of GameEnv.ACTIONS
    """
    f = open(filename, 'w')
    for i in range(len(actions)):
        f.write(str(actions[i]))
        if i < len(actions) - 1:
            f.write(',')
    f.write('\n')
    f.close()


def main(arglist):
    if len(arglist) != 3:
        print("Running this file launches your solver.")
        print("Usage: play_game.py [input_filename] [output_filename] [mode = 'ucs' or 'a*']")

    input_file = arglist[0]
    output_file = arglist[1]
    mode = arglist[2]

    assert os.path.isfile(input_file), '/!\\ input file does not exist /!\\'
    assert mode == 'ucs' or mode == 'a_star', '/!\\ invalid mode argument /!\\'

    # Read the input testcase file
    game_env = GameEnv(input_file)
    initial_state = game_env.get_init_state()

    actions = []

    t0 = timeit.default_timer()
    actions.extend(perform_search(mode, complex_weighted_manhattan_dist_heuristic, game_env,
                                  initial_state))
    print("Elapsed Time:", timeit.default_timer() - t0)

    # Write the solution to the output file
    write_output_file(output_file, actions)


if __name__ == '__main__':
    main(sys.argv[1:])
