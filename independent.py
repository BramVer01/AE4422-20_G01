"""
This is an example planner, that calls all agents to plan their route independently.
"""

def run_independent_planner(tug, nodes_dict, edges_dict, heuristics, t):
    tug.plan_independent(nodes_dict, edges_dict, heuristics, t)