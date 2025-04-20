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
    if tug.start == 0 or tug.goal == 0:
        tug.wait = True
        return constraints
    constraining_tug = None
    tug.plan_prioritized(nodes_dict, edges_dict, heuristics, t, delta_t, constraints)
    if tug.wait:
        constraining_tug = tug.constraining_tug
        constraints = remove_constraints(constraints, 'constraining_tug', constraining_tug)
        tug.plan_prioritized(nodes_dict, edges_dict, heuristics, t, delta_t, constraints)
    for j in tug_lst:
        if j.id != tug.id:
            if tug.wait:
                con = {'positive': False, 'agent': j.id, 'loc': [tug.current_node], 'timestep': round(t+delta_t, 1), 'constraining_tug': tug}
                constraints.append(con)
            else:
                previous_node = None
                for node in tug.path_to_goal:
                    con = {'positive': False, 'agent': j.id, 'loc': [node[0]], 'timestep': node[1], 'constraining_tug': tug}
                    constraints.append(con)
                    if previous_node is not None:
                        con = {'positive': False, 'agent': j.id, 'loc': [node[0], previous_node[0]], 'timestep': node[1], 'constraining_tug': tug}
                        constraints.append(con)
                        con = {'positive': False, 'agent': j.id, 'loc': [previous_node[0], node[0]], 'timestep': node[1], 'constraining_tug': tug}
                        constraints.append(con)
                        if node[0] == tug.goal:
                            next_time = round(node[1]+delta_t, 1)
                            con = {'positive': False, 'agent': j.id, 'loc': [node[0]], 'timestep': next_time, 'constraining_tug': tug}
                            constraints.append(con)
                            if previous_node is not None:
                                con = {'positive': False, 'agent': j.id, 'loc': [node[0], previous_node[0]], 'timestep': next_time, 'constraining_tug': tug}
                                constraints.append(con)
                                con = {'positive': False, 'agent': j.id, 'loc': [previous_node[0], node[0]], 'timestep': next_time, 'constraining_tug': tug}
                                constraints.append(con)
                    previous_node = node
    if constraining_tug is not None:
        constraining_tug.plan_prioritized(nodes_dict, edges_dict, heuristics, t, delta_t, constraints)
        for j in tug_lst:
            if j.id != constraining_tug.id:
                if constraining_tug.wait:
                    con = {'positive': False, 'agent': j.id, 'loc': [constraining_tug.current_node], 'timestep': round(t+delta_t, 1), 'constraining_tug': constraining_tug}
                    constraints.append(con)
                else:
                    previous_node = None
                    for node in constraining_tug.path_to_goal:
                        con = {'positive': False, 'agent': j.id, 'loc': [node[0]], 'timestep': node[1], 'constraining_tug': constraining_tug}
                        constraints.append(con)
                        if previous_node is not None:
                            con = {'positive': False, 'agent': j.id, 'loc': [node[0], previous_node[0]], 'timestep': node[1], 'constraining_tug': constraining_tug}
                            constraints.append(con)
                            con = {'positive': False, 'agent': j.id, 'loc': [previous_node[0], node[0]], 'timestep': node[1], 'constraining_tug': constraining_tug}
                            constraints.append(con)
                        if node[0] == constraining_tug.goal:
                            next_time = round(node[1]+delta_t, 1)
                            con = {'positive': False, 'agent': j.id, 'loc': [node[0]], 'timestep': next_time, 'constraining_tug': constraining_tug}
                            constraints.append(con)
                            if previous_node is not None:
                                con = {'positive': False, 'agent': j.id, 'loc': [node[0], previous_node[0]], 'timestep': next_time, 'constraining_tug': constraining_tug}
                                constraints.append(con)
                                con = {'positive': False, 'agent': j.id, 'loc': [previous_node[0], node[0]], 'timestep': next_time, 'constraining_tug': constraining_tug}
                                constraints.append(con)
                        previous_node = node
    return constraints
    #raise Exception("Prioritized planner not defined yet.")
    #return