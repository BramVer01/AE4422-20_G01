"""
Implement prioritized planner here
"""

def run_prioritized_planner(tug_lst, tug, nodes_dict, edges_dict, heuristics, t, delta_t, constraints):
    tug.plan_prioritized(nodes_dict, edges_dict, heuristics, t, delta_t, constraints)
    for j in tug_lst:
        if j.id != tug.id:
            previous_node = None
            for node in tug.path_to_goal:
                con = {'positive': False, 'agent': j.id, 'loc': [node[0]], 'timestep': node[1], 'constrainting_tug': tug.id}
                constraints.append(con)
                if previous_node is not None:
                    con = {'positive': False, 'agent': j.id, 'loc': [node[0], previous_node[0]], 'timestep': node[1], 'constrainting_tug': tug.id}
                constraints.append(con)
                previous_node = node
                # if node == ac.goal:   # We will need this if we have aircraft staying at a gate later on, will need to modify before use!
                #     for k in range(1, 50):
                #         con = {'positive': False, 'agent': j.id, 'loc': [node], 'timestep': node[1]+k/2}
                #         constraints.append(con)
    return constraints
    #raise Exception("Prioritized planner not defined yet.")
    #return