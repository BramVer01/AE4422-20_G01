"""
Implement prioritized planner here
"""

def run_prioritized_planner(aircraft_lst, nodes_dict, edges_dict, heuristics, t):
    constraints = []
    for ac in aircraft_lst:
        if ac.spawntime == t:
            ac.status = "taxiing" 
            ac.position = nodes_dict[ac.start]["xy_pos"]
            ac.plan_prioritized(nodes_dict, edges_dict, heuristics, t, ac, constraints)
            for j in aircraft_lst:
                if j.id > ac.id:
                    previous_node = None
                    for node in ac.path_to_goal:
                        con = {'positive': False, 'agent': j.id, 'loc': [node], 'timestep': int(node[1]*2)}
                        constraints.append(con)
                        if previous_node is not None:
                            con = {'positive': False, 'agent': j.id, 'loc': [node, previous_node], 'timestep': int(node[1]*2)}
                        constraints.append(con)
                        previous_node = node
                        if node == ac.goal:
                            for k in range(1, 50):
                                con = {'positive': False, 'agent': j.id, 'loc': [node], 'timestep': int(node[1]*2+k)}
                                constraints.append(con)
                    print(constraints)

    #raise Exception("Prioritized planner not defined yet.")
    #return