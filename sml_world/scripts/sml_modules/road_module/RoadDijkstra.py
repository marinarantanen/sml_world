def lanelet_dijkstra_algorithm(road_module, source_lanelet_id, destination_lanelet_id):
    '''
    Interface for dijkstra algorithm.
    Given a source_lanelet_id, and a destinanion_lanelet_id, it will find the shortest
    lanelet path between them.

    Inputs:
        road_module:
        the instance of the RoadModule class.
        source_lanelet_id:
        The id of the lanelet we wish to start the path at
        destination_lanelet_id:
        The id of the lanelet we wish to end the path at

    Returns:
        lanelet_ids_path:
        The ids of the lanelets that form the shortest path from the
        start lanelet to the end lanelet.
        NOTE: Does not include the start and end ids lanelets in it.

        To add it easily do:

        complete_lanelet_ids_path = [source_lanelet_id]
        complete_lanelet_ids_path.extend(lanelet_ids_path)
        complete_lanelet_ids_path.append(destination_lanelet_id)

    '''

    source_lanelet_index = road_module.osm_lanelet_dict.keys().index(source_lanelet_id)
    destination_lanelet_index = road_module.osm_lanelet_dict.keys().index(destination_lanelet_id)

    lanelet_indexes_path = dijkstra_algorithm(road_module.lanelet_adjacency_matrix, source_lanelet_index, destination_lanelet_index)

    lanelet_ids_path = []

    for lanelet_index in lanelet_indexes_path:

        lanelet_ids_path.append( road_module.osm_lanelet_dict.keys()[lanelet_index] )

    return lanelet_ids_path


def dijkstra_algorithm(adjacency_matrix, source_index, destination_index = None):
    '''
    Applies the Dijkstra Algorithm, given an adjacency matrix and a source index.
    Returns the shortest distances from the source index to every node in the adjacency matrix,
    and the for each index, the best previous index that forms the path to the source index.

    If it receives a destination index, it will simply return a the shortest path to this
    destination index.

    Input:
    adjacency_matrix:
        The adjacency matrix (a list of lists)
    source_index:
        The source index to be used in the Dijkstra algorithm
    destination_index: (Optional)
        The index of the node we wish to find the shortest path to

    Returns: (When not given a destination_index)

    distances:
        A list containing the shortest distance from the source_index
        to all the indexes
    previous_node:
        A list containing the previous index for each index. The previous
        index is defined as one that will result in the shortest path to the
        source index. If a given index cannot connect to the source index
        then it's previous index will have the value -1

    Returns: (When given a destination_index)
    path:
        The indexes of the shortest path between the source_index to
        the destination_index. This path is a list composed of the indexes
        in the shortest path. NOTE: Does not include the source_index,
        neither the destination_index in the list.
        To add them easily simply do:

        complete_path = [source_index]
        complete_path.extend(path)
        complete_path.append(destination_index)

    '''

    num_nodes = len(adjacency_matrix)
    distances = [ 10e10 for i in range(num_nodes) ]
    distances[source_index] = 0
    previous_node = [ -1 for i in range(num_nodes) ]

    nodes_to_visit = [ source_index ]

    while nodes_to_visit:

        current_node_to_visit = nodes_to_visit.pop(0)

        current_adjacencies = adjacency_matrix[current_node_to_visit]

        for i in range( len(current_adjacencies) ):

            if i == current_node_to_visit:

                "Same node"
                continue

            if current_adjacencies[i] == 0:

                "No connection"
                continue

            if distances[i] >  distances[current_node_to_visit] + current_adjacencies[i]:

                distances[i] = distances[current_node_to_visit] + current_adjacencies[i]
                previous_node[i] = current_node_to_visit
                nodes_to_visit.append(i)

    if destination_index != None:

        path = []
        new_node = previous_node[destination_index]

        while True:

            if new_node == -1 or new_node == source_index:

                break

            path.append(new_node)

            new_node = previous_node[new_node]

        path.reverse()

        return path

    else:

        return distances, previous_node



def lanelet_circular_dijkstra_algorithm(road_module, source_lanelet_id):
    '''
    Interface for circular dijkstra algorithm.
    Given a source_lanelet_id, it will find the shortest closed circular
    lanelet path that conects the lanelet to itself.

    Inputs:
        source_lanelet_id:
        The id of the lanelet we wish to know the shortest path

    Returns:
        lanelet_ids_path:
        The ids of the lanelets that form the shortest path from the
        desired lanelet to itself.
        NOTE: Does not include the id of the provided lanelet.

    '''

    osm_lanelet_dict_keys = road_module.osm_lanelet_dict.keys()

    source_lanelet_index = osm_lanelet_dict_keys.index(source_lanelet_id)

    lanelet_indexes_path = circular_dijkstra_algorithm(road_module.lanelet_adjacency_matrix, source_lanelet_index)

    lanelet_ids_path = []

    for lanelet_index in lanelet_indexes_path:

        lanelet_ids_path.append( osm_lanelet_dict_keys[lanelet_index] )

    return lanelet_ids_path


def circular_dijkstra_algorithm(adjacency_matrix, source_index):
    '''
    A particular case of the dijkstra algorithm in which we desire
    to find the shortest path from a node to itself.
    Given a source_index of the adjacency_matrix, it will compute the
    shortest path from this node to itself.

    Input:

    adjacency_matrix:
        The adjacency matrix (a list of lists)
    source_index:
        The index of the node we wish to find the shortest path to

    Returns:

    path:
        The indexes of the shortest path between the node to itself.
        This path is a list composed of the indexes of the nodes in
        the shortest path (does not include the source_index in either
        the start or the end of the list)
    '''

    '''
    The circular Dijkstra algorithm works as follows:
    Image we want to find the shortest circular path
    from node 3 to node 3.
    1) Create node 3a which connects to all the nodes
    that 3 connects to.
    2) Create node 3b which is connected from all the nodes
    that 3 is connected from.
    3) Run a Dijkstra from node 3a to 3b
    This requires the creation of a new graph, or in
    our case of a new adjacency matrix, which we
    call extended_adjacency_matrix
    '''
    extended_adjacency_matrix = []

    remapping_indexes = []
    cnt = 0

    for i, line in enumerate(adjacency_matrix):

        remapping_indexes.append(cnt)

        new_line = []

        for j, value in enumerate(line):

            if j == source_index:

                new_line.append ( 10e10 )

            new_line.append ( value )


        extended_adjacency_matrix.append(new_line)

        if i == source_index:

            remapping_indexes.append(cnt)

            extra_line = []

            for dummy_var in line:

                extra_line.append(10e10)

            extra_line.append(10e10)

            extended_adjacency_matrix.append(extra_line)

        cnt += 1

    [distances, previous_node] = dijkstra_algorithm(extended_adjacency_matrix, source_index)

    path = []
    new_node = source_index + 1

    while True:

        if new_node == -1:

            break

        path.append( remapping_indexes[new_node] )

        new_node = previous_node[new_node]

    path.pop(0)
    path.pop()
    path.reverse()

    return path

