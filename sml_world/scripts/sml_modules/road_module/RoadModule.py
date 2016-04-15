import RoadModuleFileReader # To read the OSM file and parse it's information
import RoadDrawer # Defines auxiliary functions related to image generation of the roads

import RoadLibrary # Defines several functions needed for lanelets
import RoadDijkstra # Defines some Dijkstra functions for lanelets

import os, pygame

class RoadModule(object):
    '''
    This class implements the Road Module to be used in the SML World
    '''

    def __init__(self, base_path, file_path, image_width = 1920., image_height = 1080., image_pixel_per_meter = 5.):

        print "RoadModule instanciated"
        self.base_path = base_path
        map_file_location = self.base_path + file_path
        # Dictionary containing the OSM nodes
        # The keys are the node ids, as assigned by the OSM Tool.
        # The values are the corresponding OSMNode instance, as defined in
        # RoadAuxClasses.py
        self.osm_node_dict = None

        # Dictionary containing the OSM ways
        # The keys are the way ids, as assigned by the OSM Tool.
        # The values are the corresponding OSMWay instance, as defined in
        # RoadAuxClasses.py
        self.osm_way_dict = None

        # Dictionary containing the OSM lanelets
        # The keys are the lanelet ids, as assigned by the OSM Tool.
        # The values are the corresponding OSMLanelet instance, as defined in
        # RoadAuxClasses.py
        self.osm_lanelet_dict = None

        # A matrix (list of lists) containing the
        # adjacency values between lanelets. If two
        # lanelets are not adjacent they will have a value
        # of 10e10, otherwise they will have a value corresponding
        # to the distance of the first lanelet
        self.lanelet_adjacency_matrix = None

        # A dictionary to store the OSM Node tags. Each key
        # will be a tag found while parsing the .xml file nodes
        # and the value of said key is a list with the id/s of the
        # node/s with said key
        self.osm_node_tag_dict = None

        # The width and height of the environment image
        self.image_width = None
        self.image_height = None

        # Stores the world_surface if it was generated
        self.world_surface = None

        self.desired_image_width = image_width
        self.desired_image_height = image_height
        self.desired_image_pixel_per_meter = image_pixel_per_meter

        # Will fill the previous attributes with
        # the correct information
        self.load_xml_world_info(map_file_location)


        self.check_world_image_up_to_date(map_file_location)



    def load_xml_world_info(self, xml_file_location):
        '''
        This method loads the information contained in the .xml created by JOSM
        This method has to be run before doing anything with the class

        Input:
            xml_file_location:
            A string with the location of the OSM Xml file.

        '''

        # osm_node_dict:
        #     a dictionary with all the nodes in the map
        # origin_osm_node:
        #     an OMSNode corresponding to the origin node
        #     If the OSM Map does not define an origin node, it will return an empty list
        xml_file_location += ".xml"
        [self.osm_node_dict, self.osm_node_tag_dict, origin_osm_node] = RoadModuleFileReader.create_osm_node_dict(xml_file_location)

        self.normalize_node_coordinates(origin_osm_node)

        self.osm_way_dict = RoadModuleFileReader.create_osm_way_dict(xml_file_location)

        self.osm_lanelet_dict = RoadModuleFileReader.create_osm_lanelet_dict(self.osm_node_dict, self.osm_way_dict, xml_file_location)  # List of nodes makes sense

        self.lanelet_adjacency_matrix = RoadLibrary.create_lanelet_adjacency_matrix(self.osm_lanelet_dict, self.osm_node_dict)

        RoadDrawer.set_pixel_values_for_nodes(self, self.desired_image_width, self.desired_image_height, self.desired_image_pixel_per_meter)

        return

    def normalize_node_coordinates(self, origin_osm_node = None):
        '''
        Centers all the OSMNode around a central point.

        If origin_osm_node is provided it will set all the node coordinates
        to be in a cartesian referential centered in origin_osm_node
        If origin_osm_node is not provided (= None) then an average center
        of mass of all the nodes is computed and used as a the origin point
        on which referential is located.

        NOTE: This function alters the contens of self.osm_node_dict

        Input:

        origin_osm_node:
            An OSMNode defining where where the origin referential should be placed.
            If not provided, or equals to None, a center of mass is used for the origin
            referential

        '''

        x_average = 0
        y_average = 0

        if origin_osm_node:

            x_average = origin_osm_node.x
            y_average = origin_osm_node.y

        else:

            number_nodes = len( self.osm_node_dict )

            for node_id in self.osm_node_dict:

                node = self.osm_node_dict[node_id]

                x_average = x_average + node.x/number_nodes
                y_average = y_average + node.y/number_nodes


        for node_id in self.osm_node_dict:

            node = self.osm_node_dict[node_id]

            node.x = node.x - x_average
            node.y = node.y - y_average

        return

    def get_lanelets_containing_node_id(self, osm_node_id, right_way_only = True):
        '''
        Interface for RoadLibrary.get_lanelets_containing_node_id
        '''

        return RoadLibrary.get_lanelets_containing_node_id(self.osm_lanelet_dict, osm_node_id, right_way_only)



    def get_closed_path_from_node_tag(self, osm_node_tag, points_per_meter = 5):
        '''
        Given a node tag, it will compute a closed trajectory that passes through this node.
        If more than one node has the given tag, it will choose the first node with this tag
        and produce a warning to the terminal.

        Inputs:
        osm_node_tag:
            The tag of the OSMNode where we wish the trajectory to pass.
        points_per_meter: (optional)
            Defines the resolution of the points of the trajectory. The
            trajectory will be composed of pointer_per_meter points for
            each meter of length.

        Returns:
        traj_x:
            A list with the x coordinates of the trajectory
        traj_y:
            A list with the y coordinates of the trajectory

        '''
        node_ids = self.osm_node_tag_dict[osm_node_tag]

        if len(node_ids) != 1:

            print "WARNING: Found multiple nodes for given tag in "
            "get_closed_path_from_node_tag() functions in RoadModule.py"

        return self.get_closed_path_from_node_id(node_ids[0], points_per_meter)



    def get_open_path(self, start_osm_node_id, end_osm_node_id, points_per_meter = 5):
        '''
        Computes the trajectory from one node to another
        '''
        start_lanelet_ids = self.get_lanelets_containing_node_id(start_osm_node_id, True)
        end_lanelet_ids = self.get_lanelets_containing_node_id(end_osm_node_id, True)
        containing_lanelet_ids = self.get_lanelets_containing_node_id(osm_node_id, True)

        best_distance = 10e10
        best_lanelets_path = []

        for lanelet_id in containing_lanelet_ids:

            lanelet_ids_path = RoadDijkstra.lanelet_dijkstra_algorithm(self, start_lanelet_id, end_lanelet_id)

            current_distance = 0

            for temp_lanelet_id_path in lanelet_ids_path:

                current_distance += RoadLibrary.get_lanelet_length(self.osm_lanelet_dict[temp_lanelet_id_path], self.osm_node_dict)

            if current_distance < best_distance:

                best_distance = current_distance

                # Need to add current lanelet_id to shortest path, since the
                # shortest path does not include it
                lanelet_ids_path.append(lanelet_id)

                best_lanelets_path = lanelet_ids_path


        traj_x = []
        traj_y = []

        for lanetet_id in best_lanelets_path:

            [x, y] = RoadLibrary.convert_lanelet_to_path(self.osm_lanelet_dict[lanetet_id], self.osm_node_dict, points_per_meter)

            traj_x.extend(x)
            traj_y.extend(y)

        return traj_x, traj_y



    def get_closed_path_from_node_id(self, osm_node_id, points_per_meter = 5):
        '''
        Given a node id, will compute a closed trajectory that passes through this node.

        Inputs:
        osm_node_id:
            The id of the OSMNode where we wish the trajectory to pass.
        points_per_meter: (optional)
            Defines the resolution of the points of the trajectory. The
            trajectory will be composed of pointer_per_meter points for
            each meter of length.

        Returns:
        traj_x:
            A list with the x coordinates of the trajectory
        traj_y:
            A list with the y coordinates of the trajectory

        '''

        containing_lanelet_ids = self.get_lanelets_containing_node_id(osm_node_id, True)

        best_distance = 10e10
        best_lanelets_path = []

        for lanelet_id in containing_lanelet_ids:

            lanelet_ids_path = RoadDijkstra.lanelet_circular_dijkstra_algorithm(self, lanelet_id)

            current_distance = 0

            for temp_lanelet_id_path in lanelet_ids_path:

                current_distance += RoadLibrary.get_lanelet_length(self.osm_lanelet_dict[temp_lanelet_id_path], self.osm_node_dict)

            if current_distance < best_distance:

                best_distance = current_distance

                # Need to add current lanelet_id to shortest path, since the
                # shortest path does not include it
                lanelet_ids_path.append(lanelet_id)

                best_lanelets_path = lanelet_ids_path


        traj_x = []
        traj_y = []

        for lanetet_id in best_lanelets_path:

            [x, y] = RoadLibrary.convert_lanelet_to_path(self.osm_lanelet_dict[lanetet_id], self.osm_node_dict, points_per_meter)

            traj_x.extend(x)
            traj_y.extend(y)

        return traj_x, traj_y


    def get_path_between_node_ids(self, start_osm_node_id, end_osm_node_id, points_per_meter = 5):
        '''
        Computes the trajectory between two OSMNodes given their ids. The trajectory is formed of points
        separated by points_per_meter.
        Raises an error if no trajectory is found between the nodes given.

        Inputs:
        start_osm_node_id:
            The id of the OSMNode where we wish the trajectory to start.
        end_osm_node_id:
            The id of the OSMNode where we wish the trajectory to finish.
        points_per_meter: (optional)
            Defines the resolution of the points of the trajectory. The
            trajectory will be composed of pointer_per_meter points for
            each meter of length.

        Returns:
        traj_x:
            A list with the x coordinates of the trajectory
        traj_y:
            A list with the y coordinates of the trajectory

        '''

        start_lanelet_ids = self.get_lanelets_containing_node_id(start_osm_node_id, True)
        end_lanelet_ids = self.get_lanelets_containing_node_id(end_osm_node_id, True)

        best_distance = 10e10
        best_lanelets_path = []

        for start_lanelet_id in start_lanelet_ids:

            for end_lanelet_id in end_lanelet_ids:

                lanelet_ids_path = RoadDijkstra.lanelet_dijkstra_algorithm(self, start_lanelet_id, end_lanelet_id)

                if not lanelet_ids_path:

                    # lanelet_ids_path is empty, ignore
                    continue

                current_distance = 0

                for temp_lanelet_id_path in lanelet_ids_path:

                    current_distance += RoadLibrary.get_lanelet_length(self.osm_lanelet_dict[temp_lanelet_id_path], self.osm_node_dict)

                if current_distance < best_distance and current_distance > 0.1:

                    # Need to add current start_lanelet_id and end_lanelet_id to
                    # shortest path, since the shortest path does not include it
                    best_lanelets_path = [start_lanelet_id]
                    best_lanelets_path.extend(lanelet_ids_path)
                    best_lanelets_path.append(end_lanelet_id)

                    best_distance = current_distance

        total_x = []
        total_y = []

        if not best_lanelets_path:

            raise NameError("In RoadModule.py get_path_between_node_ids(): Could not find a trajectory"
                " between the given nodes.")

        for lanetet_id in best_lanelets_path:

            [x, y] = RoadLibrary.convert_lanelet_to_path(self.osm_lanelet_dict[lanetet_id], self.osm_node_dict, points_per_meter)

            total_x.extend(x)
            total_y.extend(y)

        [traj_x, traj_y] = RoadLibrary.crop_path_to_node_ids(total_x, total_y, self.osm_node_dict, start_osm_node_id, end_osm_node_id)

        return traj_x, traj_y



    def get_path_between_node_tags(self, start_osm_node_tag, end_osm_node_tag, points_per_meter = 5):
        '''
        Computes the trajectory between two OSMNodes given their tags.
        See get_path_between_node_ids() for a detailed explanation.
        '''

        if len( self.osm_node_tag_dict[start_osm_node_tag] ) != 1 or len( self.osm_node_tag_dict[end_osm_node_tag] ) != 1:

            print "WARNING: Found multiple nodes for given tag in "
            "get_path_between_node_tags() functions in RoadModule.py"
            " using the first node with that tag"

        start_osm_node_id = self.osm_node_tag_dict[start_osm_node_tag][0]
        end_osm_node_id = self.osm_node_tag_dict[end_osm_node_tag][0]

        return self.get_path_between_node_ids(start_osm_node_id, end_osm_node_id, points_per_meter)




    def get_shortest_path_distance(self, adjacency_matrix, previous_node, destination_id):
        "Simply receives Dijkstra outputs and sums the distances composing the shortest path"

        distance = 0

        while destination_id != -1:

            distance = distance + adjacency_matrix[ previous_node[destination_id] ][ destination_id ]
            destination_id = previous_node[destination_id]


        return distance


    def get_environment_image(self, image_width = 1920, image_height = 1080, pixel_per_meter=1020/6):
        "This method returns a pygame.Surface that corresponds to the image of the world"

        print 'get_environment_image'

        self.set_image_properties(image_width, image_height, pixel_per_meter)

        if self.world_surface == None:

            self.world_surface = RoadDrawer.create_world_surface(self, self.image_width, self.image_height)

        # For debugging purposes
        #pygame.image.save(world_surface, 'world_surface.bmp')

        return self.world_surface

    def set_image_properties(self, image_width=1920, image_height=1080, pixel_per_meter=1020/6):

        self.image_width = image_width
        self.image_height = image_height

        [self.center_pixel_x, self.center_pixel_y, self.pixel_per_meter] = RoadDrawer.set_pixel_values_for_nodes(
            self, image_width, image_height, pixel_per_meter)

    def check_world_image_up_to_date(self, file_location):

        print 'check_world_image_up_to_date'
        print "file_location = " + str(file_location)

        image_file_location = file_location + ".bmp"

        if not os.path.isfile(image_file_location):
            # Image does not exist need to generate it
            print "Image does not exist need to generate it."

            self.generate_world_image(file_location)

            return

        xml_file_location = file_location + ".xml"

        if os.path.getmtime(xml_file_location) > os.path.getmtime(image_file_location):
            # XML file was modified later than the last time the image file was modified
            print "XML file was modified later than the last time the image file was modified"

            self.generate_world_image(file_location)

        [current_image_width, current_image_height, current_pixel_per_meter] = self.get_image_meta_data(file_location)

        is_image_properties_right = self.check_image_meta_data(file_location)

        if not is_image_properties_right:

            print "Desired image properties differ from the current existing image."

            self.generate_world_image(file_location)

        # TO BE REMOVED
        map_surface = pygame.image.load(file_location+'.bmp')
        #pygame.image.save(map_surface, 'resources/world_surface.bmp')

        return

    def generate_world_image(self, file_location):

        image_file_location = file_location + ".bmp"

        print "Will generate new image: " + image_file_location

        world_surface = self.get_environment_image(self.desired_image_width, self.desired_image_height, self.desired_image_pixel_per_meter)

        pygame.image.save(world_surface, image_file_location)

        self.save_image_meta_data(file_location, self.desired_image_width, self.desired_image_height, self.desired_image_pixel_per_meter)

        return

    def save_image_meta_data(self, file_location, image_width, image_height, pixel_per_meter):

        image_meta_data_file_location = file_location + ".meta"

        meta_data_string = ""

        meta_data_string += "image_width=" + str(image_width) + ";"
        meta_data_string += "image_height=" + str(image_height) + ";"
        meta_data_string += "pixel_per_meter=" + str(pixel_per_meter)

        f = open(image_meta_data_file_location, 'w')
        f.write(meta_data_string)
        f.close()


    def check_image_meta_data(self, file_location):
        '''
        Returns True if current stored image has the same
        properties as the ones we desire.
        Returns false otherwise
        '''

        [current_image_width, current_image_height, current_pixel_per_meter] = self.get_image_meta_data(file_location)

        if current_image_width != float(self.desired_image_width):

            return False

        if current_image_height != float(self.desired_image_height):

            return False

        if current_pixel_per_meter != float(self.desired_image_pixel_per_meter):

            return False

        return True


    def get_image_meta_data(self, file_location):

        meta_data_file_location = file_location + '.meta'

        f = open(meta_data_file_location, 'r')

        meta_data_string = f.read()

        f.close()

        tokens = meta_data_string.split(';')

        for token in tokens:

            property_tokens = token.split('=')

            if len(property_tokens) != 2:

                print "Error parsing meta data file!"
                continue

            attribute_token = property_tokens[0]
            value_token = property_tokens[1]

            if attribute_token == 'image_width':

                current_image_width = float(value_token)

            elif attribute_token == 'image_height':

                current_image_height = float(value_token)

            elif attribute_token == 'pixel_per_meter':

                current_pixel_per_meter = float(value_token)

            else:

                print "Error parsing meta data file!"
                continue

        return [current_image_width, current_image_height, current_pixel_per_meter]
