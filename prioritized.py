"""
Implement prioritized planner here
"""

def remove_constraints(constraints, param_key, param_value):
    """
    Remove all entries from the constraints list where the specified parameter matches the given value.
    INPUT:
        - constraints = [list] list of constraint dictionaries
        - param_key = [str] the key of the parameter to check
        - param_value = the value of the parameter to match
    RETURNS:
        - filtered_constraints = [list] list of constraint dictionaries with matching entries removed
    """
    filtered_constraints = [con for con in constraints if con[param_key] != param_value]
    return filtered_constraints

def run_prioritized_planner(tug_lst, tug, nodes_dict, edges_dict, heuristics, t, delta_t, constraints):
    constraing_tug = None
    tug.plan_prioritized(nodes_dict, edges_dict, heuristics, t, delta_t, constraints)
    if tug.wait:
        constraing_tug = tug.constraining_tug
        constraints = remove_constraints(constraints, 'constraining_tug', constraing_tug)
        tug.plan_prioritized(nodes_dict, edges_dict, heuristics, t, delta_t, constraints)
    for j in tug_lst:
        if j.id != tug.id:
            previous_node = None
            for node in tug.path_to_goal:
                con = {'positive': False, 'agent': j.id, 'loc': [node[0]], 'timestep': node[1], 'constraining_tug': tug}
                constraints.append(con)
                if previous_node is not None:
                    con = {'positive': False, 'agent': j.id, 'loc': [node[0], previous_node[0]], 'timestep': node[1], 'constraining_tug': tug}
                constraints.append(con)
                previous_node = node
                # if node == ac.goal:   # We will need this if we have aircraft staying at a gate later on, will need to modify before use!
                #     for k in range(1, 50):
                #         con = {'positive': False, 'agent': j.id, 'loc': [node], 'timestep': node[1]+k/2}
                #         constraints.append(con)
    if constraing_tug is not None:
        constraing_tug.plan_prioritized(nodes_dict, edges_dict, heuristics, t, delta_t, constraints)
        for j in tug_lst:
            if j.id != constraing_tug.id:
                previous_node = None
                for node in constraing_tug.path_to_goal:
                    con = {'positive': False, 'agent': j.id, 'loc': [node[0]], 'timestep': node[1], 'constraining_tug': constraing_tug}
                    constraints.append(con)
                    if previous_node is not None:
                        con = {'positive': False, 'agent': j.id, 'loc': [node[0], previous_node[0]], 'timestep': node[1], 'constraining_tug': constraing_tug}
                    constraints.append(con)
                    previous_node = node
    return constraints
    #raise Exception("Prioritized planner not defined yet.")
    #return