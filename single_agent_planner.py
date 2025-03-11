"""
This file contains single agent planner functions  that can be used by other planners.
Consider functions in this file as supporting functions.
"""

import heapq
import networkx as nx
import math

def calc_heuristics(graph, nodes_dict):
    """
    Calculates the exact heuristic dict (shortest distance between two nodes) to be used in A* search.
    INPUT:
        - graph = networkX graph object
        - nodes_dict = dictionary with nodes and node properties
    RETURNS:
        - heuristics = dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
    """
    
    heuristics = {}
    for i in nodes_dict:
        heuristics[nodes_dict[i]["id"]] = {}
        for j in nodes_dict:
            path, path_length = heuristicFinder(graph, nodes_dict[i]["id"], nodes_dict[j]["id"])
            if path == False:
                pass
            else:
                heuristics[nodes_dict[i]["id"]][nodes_dict[j]["id"]] = path_length
    return heuristics

def heuristicFinder(graph, start_node, goal_node):
    """
    Finds exact distance between start_node and goal_node using the NetworkX graph.
    INPUT:
        - graph = networkX graph object
        - start_node, goal_node [int] = node_ids of start and goal node
    RETURNS:
        - path = list with node_ids that are part of the shortest path
        - path_length = length of the shortest path
    """
    try:
        path = nx.dijkstra_path(graph, start_node, goal_node, weight="weight")
        path_length = nx.dijkstra_path_length(graph, start_node, goal_node)
    except:
        path = False
        path_length = False
        raise Exception('Heuristic cannot be calculated: No connection between', start_node, "and", goal_node)
    return path, path_length


def simple_single_agent_astar(nodes_dict, from_node, goal_node, heuristics, time_start):
    # def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """
    Single agent A* search. Time start can only be the time that an agent is at a node.
    INPUT:
        - nodes_dict = [dict] dictionary with nodes and node properties
        - from_node = [int] node_id of node from which planning is done
        - goal_node = [int] node_id of node to which planning is done
        - heuristics = [dict] dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
        - time_start = [float] planning start time. 
        - Hint: do you need more inputs?
    RETURNS:
        - success = True/False. True if path is found and False is no path is found
        - path = list of tuples with (loc, timestep) pairs -> example [(37, 1), (101, 2)]. Empty list if success == False.
    """
    
    from_node_id = from_node
    goal_node_id = goal_node
    time_start = time_start
    
    open_list = []
    closed_list = dict()
    earliest_goal_timestep = time_start
    h_value = heuristics[from_node_id][goal_node_id]
    root = {'loc': from_node_id, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': time_start}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root
    while len(open_list) > 0:
        curr = pop_node(open_list)
        if curr['loc'] == goal_node_id and curr['timestep'] >= earliest_goal_timestep:
            return True, get_path(curr)
        
        for neighbor in nodes_dict[curr['loc']]["neighbors"]:
            child = {'loc': neighbor,
                    'g_val': curr['g_val'] + 0.5,
                    'h_val': heuristics[neighbor][goal_node_id],
                    'parent': curr,
                    'timestep': curr['timestep'] + 0.5}
            if (child['loc'], child['timestep']) in closed_list:
                existing_node = closed_list[(child['loc'], child['timestep'])]
                if compare_nodes(child, existing_node):
                    closed_list[(child['loc'], child['timestep'])] = child
                    push_node(open_list, child)
            else:
                closed_list[(child['loc'], child['timestep'])] = child
                push_node(open_list, child)
    print("No path found, "+str(len(closed_list))+" nodes visited")
    return False, [] # Failed to find solutions

def push_node(open_list, node):
    heapq.heappush(open_list, (node['g_val'] + node['h_val'], node['h_val'], node['loc'], node))

def pop_node(open_list):
    _, _, _, curr = heapq.heappop(open_list)
    return curr

def compare_nodes(n1, n2):
    """Return true if n1 is better than n2."""
    return n1['g_val'] + n1['h_val'] < n2['g_val'] + n2['h_val']

def get_path(goal_node):
    """Construct path if goal node is reached"""
    path = []
    curr = goal_node
    while curr is not None:
        path.append((curr['loc'], curr['timestep']))
        curr = curr['parent']
    path.reverse()
    #print(path)
    return path

def build_constraint_table(constraints, agent, delta_t):
    ##############################
    # Task 2.2/2.3: Return a table that constains the list of constraints of
    #               the given agent for each time step. The table can be used
    #               for a more efficient constraint violation check in the 
    #               is_constrained function.

    # max_timestep = -1  # the maximum timestep in these constraints
    # #  collect constraints that are related to this agent
    # for constraint in constraints:
    #     if constraint['agent'] == agent:
    #         max_timestep = max(max_timestep, constraint['timestep'])

    # constraint_table = [[] for _ in range(max_timestep + 1)]

    # for constraint in constraints:
    #     if constraint['agent'] == agent:
    #         constraint_table[constraint['timestep']].append({'loc': constraint['loc']})

    # return constraint_table

    positive = []  # constraints that apply to everyone (or for agent, if positive)
    negative = []  # constraints that only apply to the given agent

    # Separate constraints based on agent and positive flag.
    for constraint in constraints:
        if constraint['positive']:
            if constraint['agent'] == agent:
                positive.append(constraint)
            else:
                positive.append(constraint)
        elif constraint['agent'] == agent:
            negative.append(constraint)

    constraint_table = {}

    # Process positive constraints.
    for constraint in positive:
        t = constraint['timestep']
        if len(constraint['loc']) == 1:  # vertex constraint
            constraint_table.setdefault(t, []).append({'loc': constraint['loc'], 'positive': True})
        else:  # edge constraint: split into two parts using delta_t instead of 1.
            t_from = t - delta_t
            t_to = t
            constraint_table.setdefault(t_from, []).append({'loc': [constraint['loc'][0]], 'positive': True})
            constraint_table.setdefault(t_to, []).append({'loc': [constraint['loc'][1]], 'positive': True})

    # Process negative constraints.
    for constraint in negative:
        t = constraint['timestep']
        if len(constraint['loc']) == 1:  # vertex constraint
            constraint_table.setdefault(t, []).append({'loc': constraint['loc'], 'positive': False})
        elif constraint.get('positive', False):  # if for some reason a positive edge constraint is here for other agents
            t_from = t - delta_t
            t_to = t
            constraint_table.setdefault(t_from, []).append({'loc': [constraint['loc'][0]], 'positive': False})
            constraint_table.setdefault(t_to, []).append({'loc': [constraint['loc'][1]], 'positive': False})
            constraint_table.setdefault(t_to, []).append({'loc': [constraint['loc'][1], constraint['loc'][0]], 'positive': False})
        else:  # negative edge constraint
            constraint_table.setdefault(t, []).append({'loc': constraint['loc'], 'positive': False})

    return constraint_table

def is_constrained(curr_loc, next_loc, next_time, constraint_table):
    ##############################
    # Task 2.2/2.3: Check if a move from curr_loc to next_loc at time step next_time violates
    #               any given constraint. For efficiency the constraints are indexed in a constraint_table
    #               by time step, see build_constraint_table.

    # if len(constraint_table) <= next_time:
    #     return False

    # for constraint in constraint_table[next_time]:
    #     if len(constraint['loc']) == 1:  # vertex constraint
    #         if constraint['loc'][0] == next_loc:
    #             return True
    #     else:  # edge constraint
    #         if constraint['loc'] == [curr_loc, next_loc]:
    #             return True

    # return False
    constraints_at_time = constraint_table.get(next_time, [])
    for constraint in constraints_at_time:
        if constraint['positive']:
            # For a positive constraint, the move is only allowed if next_loc matches.
            if constraint['loc'][0][0] != next_loc:
                return True
        else:
            # For a negative constraint, if it's a vertex constraint check for equality.
            if len(constraint['loc']) == 1:
                if constraint['loc'][0][0] == next_loc:
                    return True
            else:
                # For an edge constraint, check if the move matches the edge.
                if [constraint['loc'][0][0], constraint['loc'][1][0]] == [curr_loc, next_loc]:
                    return True
    return False

def simple_single_agent_astar_prioritized(nodes_dict, from_node, goal_node, heuristics, time_start, delta_t, agent, constraints):
    # def a_star(my_map, start_loc, goal_loc, h_values, agent, constraints):
    """
    Single agent A* search. Time start can only be the time that an agent is at a node.
    INPUT:
        - nodes_dict = [dict] dictionary with nodes and node properties
        - from_node = [int] node_id of node from which planning is done
        - goal_node = [int] node_id of node to which planning is done
        - heuristics = [dict] dict with shortest path distance between nodes. Dictionary in a dictionary. Key of first dict is fromnode and key in second dict is tonode.
        - time_start = [float] planning start time. 
        - Hint: do you need more inputs?
    RETURNS:
        - success = True/False. True if path is found and False is no path is found
        - path = list of tuples with (loc, timestep) pairs -> example [(37, 1), (101, 2)]. Empty list if success == False.
    """
    
    time_start = time_start
    agent = agent.id
    constraint_table = build_constraint_table(constraints, agent, delta_t)

    open_list = []
    closed_list = {}
    earliest_goal_timestep = time_start
    h_value = heuristics[from_node][goal_node]
    root = {'loc': from_node, 'g_val': 0, 'h_val': h_value, 'parent': None, 'timestep': time_start}
    push_node(open_list, root)
    closed_list[(root['loc'], root['timestep'])] = root

    while len(open_list) > 0:
        curr = pop_node(open_list)
        
        # Check if goal is reached and no future constraint blocks the goal.
        if curr['loc'] == goal_node and curr['timestep'] >= earliest_goal_timestep:
            if constraint_table is not None:
                found = True
                for t in sorted(constraint_table.keys()):
                    if t >= curr['timestep'] + delta_t:
                        if is_constrained(goal_node, goal_node, t, constraint_table):
                            found = False
                            earliest_goal_timestep = t + delta_t
                            break
                if found:
                    return True, get_path(curr)
            else:
                return True, get_path(curr)
        
        neighbors = nodes_dict[curr['loc']]["neighbors"].copy()
        neighbors.add(curr['loc']) # Adding the "wait" action as a valid neighbor.

        # Expand each neighbor from the current node.
        for neighbor in neighbors:
            new_time = curr['timestep'] + delta_t
            # If using constraints, skip this neighbor if a constraint is active.
            if constraint_table is not None and is_constrained(curr['loc'], neighbor, new_time, constraint_table):
                continue
            child = {'loc': neighbor,
                    'g_val': curr['g_val'] + delta_t,  # update cost as needed
                    'h_val': heuristics[neighbor][goal_node],
                    'parent': curr,
                    'timestep': new_time}
            key = (child['loc'], child['timestep'])
            if key in closed_list:
                existing_node = closed_list[key]
                if compare_nodes(child, existing_node):
                    closed_list[key] = child
                    push_node(open_list, child)
            else:
                closed_list[key] = child
                push_node(open_list, child)

    print("No path found, visited", len(closed_list), "nodes")
    return False, []