
import RoadModule
import LaneletLibrary
import pygame
import time


road_module = RoadModule.RoadModule("myHighwayCircleDetailShorterSMLDebug.xml")

# print road_module.osm_lanelet_dict

# for lanelet_id in road_module.osm_lanelet_dict:

#     # print "lanelet_id = " + str(lanelet_id) 
#     print road_module.lanelet_circular_dijkstra_algorithm(lanelet_id)


# print LaneletLibrary.circular_dijkstra_algorithm(adj_matrix, source_index)

# print "road_module.osm_lanelet_dict = " + str(road_module.osm_lanelet_dict)

# print road_module.lanelet_dijkstra_algorithm(-608, -588)
# print road_module.lanelet_dijkstra_algorithm(-608, -590)
# print road_module.lanelet_dijkstra_algorithm(-608, -592)
# print road_module.lanelet_dijkstra_algorithm(-608, -594)
# print road_module.lanelet_dijkstra_algorithm(-608, -596)
# print road_module.lanelet_dijkstra_algorithm(-608, -598)
# print road_module.lanelet_dijkstra_algorithm(-608, -600)
# print road_module.lanelet_dijkstra_algorithm(-608, -602)
# print road_module.lanelet_dijkstra_algorithm(-608, -604)
# print road_module.lanelet_dijkstra_algorithm(-608, -606)
# print road_module.lanelet_dijkstra_algorithm(-608, -608)

# osm_node = road_module.osm_node_dict[ road_module.osm_node_dict.keys()[50] ]

# [x, y] = road_module.get_closed_trajectory_from_node_tag("right_lane_node", points_per_meter = 5)
# [x, y] = road_module.get_closed_trajectory_from_node_id(osm_node.id, points_per_meter = 5)

[x, y] = road_module.get_trajectory_between_node_tags( "right_lane_node", "right_lane_node_center", points_per_meter = 5)




surface = road_module.get_world_surface()
