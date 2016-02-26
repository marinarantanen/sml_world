'''
This file defines the container classes to store the OSM Map file
structures.

This file is used by RoadModule.py

'''
import utm # Library to convert from GPS coordinates to cartesian coordinates

class OSMNode:
    "This class is the OSM Node container"
    def __init__(self, id, lat, lon, tag = None):

        if not isinstance(id, int):
            raise NameError('In class OSMNode: constructor: id must be an integer')

        self.id = id

        if not isinstance(lat, float):
            raise NameError('In class OSMNode: constructor: lat must be a float')

        if not isinstance(lon, float):
            raise NameError('In class OSMNode: constructor: lon must be a float')

        # WARNING: There is a small loss of precision when the string is converted to a float
        self.lat = lat
        self.lon = lon

        # self.x and self.y are variables of type float
        (self.x, self.y, unused, unused) = utm.from_latlon(self.lat, self.lon)

        self.tag = tag

        self.pixel_x = -1
        self.pixel_y = -1

    def distance_to(self, other_osm_node):
        '''
        Get the euclidean distance between this node and the node provided in the argument.

        Inputs:
        other_osm_node:
            The OSMNode we wish to know the distance to

        Returns:
        distance:
            The euclidean distance between the nodes

        '''

        distance = ( ( self.x - other_osm_node.x )**2 + ( self.y - other_osm_node.y )**2 )**0.5

        return distance


class OSMWay:
    "This class is the OSM Way container"

    def __init__(self, id):

        if not isinstance(id, int):
            raise NameError('In class OSMWay: constructor: id must be an integer')

        self.id = id
        self.node_ids = []
        self.line_type = None
        
    def add_node_id(self, id):

        if not isinstance(id, int):
            raise NameError('In class OSMWay: add_node_id: id must be an integer')

        self.node_ids.append( id )

    def set_line_type(self, line_type):

        if not isinstance(line_type, str):
            raise NameError('In class OSMWay: set_line_type: line_type must be a string')

        self.line_type = line_type

class OSMLanelet:
    "This class is the OSM Lanelet container"

    def __init__(self, id, left_osm_way, right_osm_way):

        if not isinstance(id, int):
            raise NameError('In class OSMLanelet: constructor: id must be an integer')

        if not isinstance(left_osm_way, OSMWay):
            raise NameError('In class OSMLanelet: constructor: left_osm_way must be of type OSMWay')

        if not isinstance(right_osm_way, OSMWay):
            raise NameError('In class OSMLanelet: constructor: right_osm_way must be of type OSMWay')

        self.id = id

        self.left_osm_way = left_osm_way
        self.right_osm_way = right_osm_way

class RoadTrajectory:
    "A simple class containing a trajectory. Consists of lists of x, y and t coordinates"

    def __init__(self, x_road, y_road, t_road):

        for num in x_road:
            if not isinstance(num, float):
                raise NameError('In class RoadTrajectory: constructor: x_road is not a list of floats')

        for num in y_road:
            if not isinstance(num, float):
                raise NameError('In class RoadTrajectory: constructor: y_road is not a list of floats')

        for num in t_road:
            if not isinstance(num, float):
                raise NameError('In class RoadTrajectory: constructor: t_road is not a list of floats')

        if ( len(x_road) != len(y_road) ) or ( len(y_road) != len(t_road) ):
            raise NameError('In class RoadTrajectory: constructor: x_road, y_road and t_road do not have consistent lengths')

        self.x = x_road
        self.y = y_road
        self.t = t_road

