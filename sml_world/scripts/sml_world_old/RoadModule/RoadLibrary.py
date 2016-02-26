import math

def create_lanelet_adjacency_matrix(osm_lanelet_dict, osm_node_dict):
    '''
    Creates a lanelet adjacency matrix, which contains information about
    how lanelets are connected.

    If two lanelets not adjacent, it's respective matrix entry is 0
    If two lanelets are adjacent, it's respective matrix entry is the
    average length of the first lanelet.

    NOTE: The definition of "adjacent" in this context is not the usual.
    Two lanelets, lanelet_start and lanelet_end, are adjacent if 
    it is possible to go from lanelet_start to lanelet_end.
    Being possible to go from lanelet_start to lanelet_end does not imply
    that is is possible to go from lanelet_end to lanelet_start (imagine
    unidirectional lanelets, that is lanelets with just one possible direction
    of movement)

    Inputs:

    osm_lanelet_dict:
        The dictionary of OSMLanelets
    osm_node_dict:
        The dictionary of OSMNodes

    Returns:

    adjacency_matrix:
        A square matrix (list of lists) with the n*n entries, where n is the 
        number of lanelets in osm_lanelet_dict.
        Each entry n,m of the matrix, indicates if lanelet_n is adjacent to lanelet_m,
        that is, if it is possible to go from lanelet_n to lanelet_m.

        NOTE: The indexes of the matrix go from 0 to n-1, do not confuse with the
        lanelet ids which are a different number
        If you know the id of a lanelet you can find its corresponding index in
        the matrix by doing: lanelet_index = osm_lanelet_dict.keys().index(lanelet_id)
    '''

    adjacency_matrix = [[10e10 for x in range( len(osm_lanelet_dict) )] for x in range ( len(osm_lanelet_dict) )]

    for lanelet_id_a in osm_lanelet_dict:

        for lanelet_id_b in osm_lanelet_dict:

            if lanelet_id_a == lanelet_id_b:

                # Lanelets are the same lanelet, ajacency is zero.
                continue

            "Check if Lanelet_A connects to Lanelet_B (one directional only)"

            is_adjacent = check_lanelet_adjacency(osm_lanelet_dict[lanelet_id_a], osm_lanelet_dict[lanelet_id_b])

            if is_adjacent:

                lanelet_length = get_lanelet_length( osm_lanelet_dict[lanelet_id_a], osm_node_dict)

                lanelet_index_a = osm_lanelet_dict.keys().index(lanelet_id_a)
                lanelet_index_b = osm_lanelet_dict.keys().index(lanelet_id_b)

                adjacency_matrix[lanelet_index_a][lanelet_index_b] = lanelet_length

    return adjacency_matrix


def check_lanelet_adjacency(lanelet_start, lanelet_end):
    '''
    Checks if two given lanelets are adjacent. Two lanelets are adjacent if 
    it is possible to go from lanelet_start to lanelet_b. 
    This function is not commutative:
    check_lanelet_adjacency(A, B) != check_lanelet_adjacency(B, A)

    Inputs:
        lanelet_start:
            the starting lanelet
        lanelet_end:
            the ending lanelet

    Returns:
        A boolean indicating if the lanelets are adjacent
    '''

    if ( lanelet_start.left_osm_way.node_ids[-1] == lanelet_end.left_osm_way.node_ids[0] ) and ( lanelet_start.right_osm_way.node_ids[-1] == lanelet_end.right_osm_way.node_ids[0] ) :

        return True
            
    return False

def get_lanelet_length(osm_lanelet, osm_node_dict):
    '''
    Computes an approximation of the lanelet length.
    This approximation correnponds to the length of
    a path centered in the lanelet.
    This path is computed by computing intermediate points
    between the nodes that compose the left and right ways
    of the lanelet.

    Inputs:
        osm_lanelet:
            the OSMLanelet we whish to know the length of
        lanelet_end:
            the dictionary of OSMLanelets

    Returns:
        A boolean indicating if the lanelets are adjacent
    '''


    left_lane_length = 0
    right_lane_length = 0

    left_osm_way = osm_lanelet.left_osm_way
    right_osm_way = osm_lanelet.right_osm_way


    for way_node_id in range( len(left_osm_way.node_ids) - 1 ):

        prev_node_left = osm_node_dict[left_osm_way.node_ids[way_node_id]]
        prev_node_right = osm_node_dict[right_osm_way.node_ids[way_node_id]]

        next_node_left = osm_node_dict[left_osm_way.node_ids[way_node_id+1]]
        next_node_right = osm_node_dict[right_osm_way.node_ids[way_node_id+1]]

        current_left_lane_length = prev_node_left.distance_to(next_node_left)
        current_right_lane_length = prev_node_right.distance_to(next_node_right)

        left_lane_length = left_lane_length + current_left_lane_length
        right_lane_length = right_lane_length + current_right_lane_length

    average_length = ( left_lane_length + right_lane_length )/2

    return average_length

def get_lanelets_containing_node_id(osm_lanelet_dict, osm_node_id, right_way_only = True):
    '''
    Given an OSMNode id, will return the ids of the lanelets that contain this node in it's
    right and/or left way.

    Inputs:

    osm_lanelet_dict:
        The dictionary of OSMLanelets
    osm_node_id:
        The id of the OSMNode
    right_way_only:
        If we only consider nodes in the right way of if we also consider nodes that belong
        to the left way. Optional argument, DEFAULT = True, only right ways are considered

    Returns:

    containing_lanelet_ids:
        A list with the ids of the lanelets that contain the given OSMNode id. Empty list
        if no lanelets were found.
    '''

    containing_lanelet_ids = []

    for lanelet_id in osm_lanelet_dict:

        current_lanelet = osm_lanelet_dict[lanelet_id]

        if osm_node_id in current_lanelet.right_osm_way.node_ids:

            containing_lanelet_ids.append(lanelet_id)

            continue

        if not right_way_only:

            if osm_node_id in current_lanelet.left_osm_way.node_ids:

                containing_lanelet_ids.append(lanelet_id)

                continue

    return containing_lanelet_ids




def convert_lanelet_to_path(osm_lanelet, osm_node_dict, points_per_meter):
    '''
    Given a lanelet, it will compute the trajectory corresponding to it.

    Inputs:
        osm_lanelet:
        The lanelet we wish to know the trajectory of.
        osm_node_dict:
        The dictionary of OSM Nodes.
        points_per_meter:
        The resolution of the trajectory to return. The points of the trajectory
        will be spaced 1/points_per_meter between them.

    Returns:
        traj_x:
        A list with the x coordinates of the trajectory corresponding
        to the given lanelet
        traj_y:
        A list with the y coordinates of the trajectory corresponding
        to the given lanelet

    '''

    left_osm_way = osm_lanelet.left_osm_way 
    right_osm_way = osm_lanelet.right_osm_way

    center_points_x = []
    center_points_y = []

    left_nodes = []
    
    for left_id in left_osm_way.node_ids:

        left_nodes.append( osm_node_dict[left_id] )

    right_nodes = []
    
    for right_id in right_osm_way.node_ids:

        right_nodes.append( osm_node_dict[right_id] )

    for idx in range( len( left_nodes ) ):

        [x, y] = get_center_between_nodes(left_nodes[idx], right_nodes[idx])
        center_points_x.append(x)
        center_points_y.append(y)
    
    cumulative_length_center = [0]
    current_cumulative_length_center = 0

    cumulative_length_left = [0]
    current_cumulative_length_left = 0

    cumulative_length_right = [0]
    current_cumulative_length_right = 0

    for idx in range(1, len( left_nodes ) ):

        prev_x = center_points_x[idx-1]
        new_x = center_points_x[idx]

        prev_y = center_points_y[idx-1]
        new_y = center_points_y[idx]

        current_distance = ( ( new_x - prev_x )**2. + ( new_y - prev_y )**2 )**0.5

        current_cumulative_length_center = current_cumulative_length_center + current_distance
        cumulative_length_center.append(current_cumulative_length_center)

        prev_node = left_nodes[idx-1]
        new_node = left_nodes[idx]

        current_cumulative_length_left = current_cumulative_length_left + prev_node.distance_to(new_node)
        cumulative_length_left.append(current_cumulative_length_left)

        prev_node = right_nodes[idx-1]
        new_node = right_nodes[idx]

        current_cumulative_length_right = current_cumulative_length_right + prev_node.distance_to(new_node)
        cumulative_length_right.append(current_cumulative_length_right)

    cumulative_length_desired = []

    center_length = cumulative_length_center[-1]

    number_interpolation_points = int( math.ceil( center_length*float(points_per_meter) ) )

    for idx in range(number_interpolation_points):

        cumulative_length_desired.append( (float(idx)/number_interpolation_points) * center_length )

    traj_x = []
    traj_x.append( center_points_x[0] )
    traj_y = []
    traj_y.append( center_points_y[0] )

    for idx in range(1, number_interpolation_points):

        for search_idx in range( len(cumulative_length_center)-1 ):

            if cumulative_length_desired[idx] > cumulative_length_center[search_idx] and cumulative_length_desired[idx] < cumulative_length_center[search_idx+1]:

                distance_a = cumulative_length_desired[idx] - cumulative_length_center[search_idx]
                distance_b = cumulative_length_center[search_idx+1] - cumulative_length_desired[idx]

                interpolated_x = ( distance_b*center_points_x[search_idx] + distance_a*center_points_x[search_idx+1] )/( distance_a + distance_b )
                interpolated_y = ( distance_b*center_points_y[search_idx] + distance_a*center_points_y[search_idx+1] )/( distance_a + distance_b )

                traj_x.append(interpolated_x)
                traj_y.append(interpolated_y)


    new_interpolation_distance = 0
    new_interpolation_cumulative_distance = [new_interpolation_distance]

    for i in range( len( center_points_x ) - 1 ):

        current_distance =  ( ( center_points_x[i+1] - center_points_x[i] )**2 + ( center_points_y[i+1] - center_points_y[i] )**2 )**0.5
        new_interpolation_distance = new_interpolation_distance + current_distance
        new_interpolation_cumulative_distance.append(new_interpolation_distance)

    return [traj_x, traj_y]

def get_center_between_nodes(osm_node_start, osm_node_end):
    '''
    Given two OSMNodes it will simply compute the coordinates
    of the point centered between them (center of mass)
    '''

    x = ( osm_node_start.x + osm_node_end.x )/2.
    y = ( osm_node_start.y + osm_node_end.y )/2.

    return [x, y]

def crop_path_to_node_ids(traj_x, traj_y, osm_node_dict, start_id, end_id):
    '''
    Given a trajectory and two node ids, it will crop the trajectory such that
    it starts as close as possible to the start node, and finishes as close
    as possible to the end node.

    Inputs:
        traj_x:
        A list with the x coordinates of the trajectory
        traj_y:
        A list with the y coordinates of the trajectory
        osm_node_dict:
        The dictionary of OSM Nodes
        start_id:
        The id of the starting node, where the trajectory should begin
        end_id:
        The id of the ending node, where the trajectory should finish

    Returns:
        traj_x:
        A list with the x coordinates of the trajectory after being cropped to 
        start and end at the given nodes
        traj_y:
        A list with the y coordinates of the trajectory after being cropped to 
        start and end at the given nodes

    '''
    
    start_node = osm_node_dict[start_id] 
    end_node = osm_node_dict[end_id]

    best_start_distance = 10e10
    best_end_distance = 10e10

    best_start_id = -1
    best_end_id = -1

    for idx in range( len( traj_x ) ):

        current_distance = ( ( start_node.x - traj_x[idx] )**2. + ( start_node.y - traj_y[idx] )**2 )**0.5

        if current_distance < best_start_distance:

            best_start_distance = current_distance
            best_start_id = idx

        current_distance = ( ( end_node.x - traj_x[idx] )**2. + ( end_node.y - traj_y[idx] )**2 )**0.5

        if current_distance < best_end_distance:

            best_end_distance = current_distance
            best_end_id = idx

    if best_start_id == -1 or best_end_id == -1:

        return [traj_x, traj_y]

    else:

        return [traj_x[best_start_id:best_end_id], traj_y[best_start_id:best_end_id]]

