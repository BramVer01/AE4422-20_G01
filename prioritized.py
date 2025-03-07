"""
Implement prioritized planner here
"""

def run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t):
    for ac in aircraft_lst:
        if ac.spawntime == t:
            ac.status = "taxiing" 
            ac.position = nodes_dict[ac.start]["xy_pos"]
            ac.plan_prioritized(nodes_dict, edges_dict, heuristics, t)

    #raise Exception("Prioritized planner not defined yet.")
    #return