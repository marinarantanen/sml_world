'''
This file implements functions to read the OSM Map file
that describes the SML World environment.

This file is used by RoadModule.py

'''


import RoadAuxClasses
import copy
from xml.etree import ElementTree as ET


def create_osm_node_dict(xml_file_location):
    '''
    Receives:

    xml_file_location:
        a string with the location of the OSM file to be processed

    Returns:

    osm_node_dict:
        a dictionary with all the nodes in the map
    osm_tag_dict:
        a dictionary with all the found tags, and it's corresponding node ids.
        Each key of the fictionary corresponds to a tag, and the value of each 
        key is a list with the id/ids of node/s with this tag.
    origin_node:
        an OMSNode corresponding to the origin node
        If the OSM Map does not define an origin node, it will return None
    '''

    XML_file = xml_file_location

    tree = ET.parse(XML_file)
    xml_root = tree.getroot()

    osm_node_dict = dict()
    osm_tag_dict = dict()
    origin_node = None

    for node in xml_root.findall('node'):

        add_flag = True

        for tag in node.findall('tag'):

            node_id = int( node.get('id') )
            origin_lat = float( node.get('lat') )
            origin_lon = float( node.get('lon') )

            if tag.get('k') == "origin":

                if tag.get('v') == "true":
                    
                    origin_node = RoadAuxClasses.OSMNode(node_id, origin_lat, origin_lon)
                    # This node should not be added to the node list

                    osm_tag_dict['origin'] = []
                    osm_tag_dict['origin'].append(node_id)
                    
                    continue

            else:

                tag_string = tag.get('v')

                special_node = RoadAuxClasses.OSMNode(node_id, origin_lat, origin_lon, tag = tag_string)
                osm_node_dict[node_id] = special_node

                if osm_tag_dict.has_key(tag_string):

                    osm_tag_dict[tag_string].append(node_id)

                else:

                    osm_tag_dict[tag_string] = [node_id]
                
                continue    

        node_id = int( node.get('id') )

        lat = float( node.get('lat') )
        lon = float( node.get('lon') )

        osm_node_dict[node_id] = RoadAuxClasses.OSMNode(node_id, lat, lon) 


    return [osm_node_dict, osm_tag_dict, origin_node]


def create_osm_way_dict(xml_file_location):
    '''
    Receives:

    xml_file_location:
        a string with the location of the OSM file to be processed

    Returns:

    osm_way_dict:
        a dictionary with all the ways in the map
    '''

    XML_file = xml_file_location

    tree = ET.parse(XML_file)
    xml_root = tree.getroot()

    osm_way_dict = dict()

    for way in xml_root.findall('way'):

        way_id = int( way.get('id') )

        current_way = RoadAuxClasses.OSMWay( way_id )

        free_space_way = False

        for tag in way.findall('tag'):

            if tag.get('k') == "line_type":

                current_way.set_line_type( tag.get('v') )

                if tag.get('v') == "free_space":

                    free_space_way = True

            elif tag.get('k') == "region":

                current_way.set_line_type( tag.get('v') )
                
            else:

                print "Unrecognized tag in way xml"

        for node in way.findall('nd'):
            ref_id = int( node.get('ref') )
            current_way.add_node_id( ref_id )


        osm_way_dict[way_id] = current_way

    return osm_way_dict


def create_osm_lanelet_dict(osm_node_dict, osm_way_dict, xml_file_location):
    '''
    Receives:

    xml_file_location:
        a string with the location of the OSM file to be processed

    Returns:

    osm_lanelet_dict:
        a dictionary with all the lanelets in the map
    '''

    XML_file = xml_file_location

    tree = ET.parse(XML_file)
    xml_root = tree.getroot()

    osm_lanelet_dict = dict()

    for relation in xml_root.findall('relation'):
        
        lanelet = False

        for tag in relation.findall('tag'):

            if tag.get('k') == 'type' and tag.get('v') == 'lanelet':

                lanelet = True

        if lanelet:

            temp_lanelet = lanelet
            lanelet_id = int( relation.get('id') )

            left_osm_way = []
            right_osm_way = []

            for member in relation.findall('member'):

                if member.get('type') == 'way':

                    way_id = int( member.get('ref') )

                    tempOSMWay = osm_way_dict[way_id]

                    if member.get('role') == 'left_lane_marking':

                        left_osm_way = copy.deepcopy( tempOSMWay )

                    else:

                        right_osm_way = copy.deepcopy( tempOSMWay )

            if not left_osm_way:
                raise NameError('left_lane_marking is empty, lanelet creation aborted. Tip: Maybe you have empty/incomplete lanelets in the JOSM XML file.')
            if not right_osm_way:
                raise NameError('right_lane_marking is empty, lanelet creation aborted. Tip: Maybe you have empty/incomplete lanelets in the JOSM XML file.')

            starting_right_node = osm_node_dict[right_osm_way.node_ids[0]]

            starting_left_node = osm_node_dict[left_osm_way.node_ids[0]] 
            ending_left_node = osm_node_dict[left_osm_way.node_ids[-1]]


            if ( starting_right_node.distance_to(starting_left_node) > starting_right_node.distance_to(ending_left_node) ):

                # Need to flip the left way, because it is in the wrong direction!
                left_osm_way.node_ids.reverse()

            osm_lanelet_dict[lanelet_id] = RoadAuxClasses.OSMLanelet(lanelet_id, left_osm_way, right_osm_way) 

            inverted_left_osm_way = copy.deepcopy( left_osm_way )
            inverted_right_osm_way = copy.deepcopy( right_osm_way )

            if len(left_osm_way.node_ids) != len(right_osm_way.node_ids):
                raise NameError('Lanelet ways have a different number of nodes')

            inverted_left_osm_way.node_ids.reverse()
            inverted_right_osm_way.node_ids.reverse()

    return osm_lanelet_dict