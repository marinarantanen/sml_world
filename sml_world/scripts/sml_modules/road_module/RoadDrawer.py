'''
This file implements functions related to image generation and visualistation
of the environment and roads.
Used by RoadModule.py
'''


import pygame
import os

def create_world_surface(road_module, image_width, image_height):
    '''
    Generates and returns the environemnt and roads image in the form of a pygame surface.

    Inputs:
        road_module:
        The road module instance of RoadModule class.
        image_width:
        The desired image width.
        image_height:
        The desired image height.

    Returns:
        world_surface:
        The environment and roads pygame surface (image).
    '''

    image_path = road_module.base_path + "/resources"

    world_surface = pygame.Surface((image_width, image_height), 0, 32)

    BLACK = 0, 0, 0
    WHITE = 255, 255, 255

    ''' Drawing forest background'''
    # Drawing the background of the environment, 
    # composed of a tree pattern

    # Load the image
    forest_image = pygame.image.load(os.path.join(image_path,"forest.jpg"))
    original_width, original_height = forest_image.get_size()

    # The image might need resizing for the pattern to
    # have the correct scale
    scale_ratio = 1.0/1.5
    new_size = (int(scale_ratio*original_width), int(scale_ratio*original_height) )
    forest_image_rescaled = pygame.transform.scale(forest_image,  new_size )

    # Create the mask, which defines where the forest pattern will be applied
    # In this case we want the forest to be everywhere in the image so
    # the mask is fully white.
    forest_mask = pygame.Surface((image_width, image_height), 0, 32)
    forest_mask.fill(WHITE)

    # Apply the forest_rescaled pattern to the world_surface, in
    # the region defined by mask
    repeat_pattern(world_surface, forest_image_rescaled, forest_mask, brightness = 0.7)

    ''' Drawing roads'''
    # Load the image
    asphalt_image = pygame.image.load( os.path.join(image_path,"asphalt.jpg") )
    original_width, original_height = asphalt_image.get_size()

    # The image might need resizing for the pattern to
    # have the correct scale
    scale_ratio = 1.0/2.0
    new_size = (int(scale_ratio*original_width), int(scale_ratio*original_height) )
    asphalt_image_rescaled = pygame.transform.scale(asphalt_image, new_size )

    # Create the lanelet mask, which defines where the road pattern will be applied
    # Where the pattern is applied, it overrides the previous pixels, thus
    # the forest will not be visible where there is road on top
    mask = get_lanelet_mask(road_module, image_width, image_height,
        negative_color = WHITE, positive_color = BLACK)
    repeat_pattern(world_surface, asphalt_image_rescaled, mask)

    # Draws the lines correspoding to the OSM ways.
    draw_all_ways(road_module, world_surface, brightness = 0.9)

    return world_surface


def get_lanelet_mask(road_module, image_width, image_height, negative_color, positive_color):
    '''
    Generates an image mask for the lanelets, where the background color 
    is negative_color, and the interest region color is positive_color.
    This mask corresponds to an image with positive_color in the lanelets
    and negative_color everywhere else.

    Inputs:
        road_module:
        The road module instance of RoadModule class.
        image_width:
        The desired image width.
        image_height:
        The desired image height.
        negative_color:
        The background (default) color of the mask.
        positive_color:
        The color of places where the mask is active.

    Returns:
        mask_image:
        The mask image, where the region of the lanelets is colored
        positive_color and everywhere else is negative_color
    '''

    # Generates the base mask_image, and fill it with background_color
    mask_image = pygame.Surface((image_width, image_height), 0, 32)
    mask_image.fill(negative_color)

    # Iterate over all of the lanelets
    for lanelet_id in road_module.osm_lanelet_dict:

        lanelet = road_module.osm_lanelet_dict[lanelet_id]

        node_tuple_list = []

        for node_id in lanelet.left_osm_way.node_ids:

            current_node = road_module.osm_node_dict[node_id]

            current_tuple = ( int( current_node.pixel_x ) , int( current_node.pixel_y ) )
            node_tuple_list.append( current_tuple )

        for idx in range( len ( lanelet.right_osm_way.node_ids ) ):

            current_node = road_module.osm_node_dict[lanelet.right_osm_way.node_ids[-1 - idx]]

            current_tuple = ( int( current_node.pixel_x ) , int( current_node.pixel_y ) )
            node_tuple_list.append( current_tuple )

        # For each lanelet draw its corresponding polygon on the mask, with
        # a color of positive_color
        pygame.draw.polygon(mask_image, positive_color, tuple( node_tuple_list ) )

    return mask_image


def repeat_pattern(destination_image, pattern_image, mask_image, brightness = 1.0):
    '''
    Applies a pattern image to a destination image, given a mask.

    Inputs:
        destination_image:
        The image in which the pattern will be applied.
        pattern_image:
        The pattern image to be applied.
        mask_image:
        The mask image to defining where the pattern will be applied.
        The pattern is applied everywhere where the mask_image has a 
        color different than black (0, 0, 0)
        brightness: (Optional)
        Defines how bright/dark the way will be drawn.
        1.0 = full brightness, corresponds to the original color
        0.0 = zero brightness, the color will be completely black
    '''

    width, height =  destination_image.get_size()
    pattern_width, pattern_height = pattern_image.get_size()

    if brightness != 1.0:

        for i in range(width):

            for j in range(height):

                color = mask_image.get_at( (i,j) )

                if color[0] != 0:

                    color = pattern_image.get_at( ( i%pattern_width , j%pattern_height ) )

                    # print "color = " + str(color)
                    new_color = ( brightness*color[0], brightness*color[1], brightness*color[2] )
                    # print "new_color = " + str(new_color)

                    destination_image.set_at( (i,j) , new_color )

    else:

        for i in range(width):

            for j in range(height):

                color = mask_image.get_at( (i,j) )

                if color[0] == 0:

                    destination_image.set_at( (i,j) , pattern_image.get_at( ( i%pattern_width , j%pattern_height ) ) )

    return


def get_way_line_type_mask(road_module, image_width, image_height, line_type, negative_color, positive_color):
    '''
    NOT BEING USED IN THIS VERSION

    Given a line_type, it will look for OSM Ways with this 
    line_type and will return a mask image corresponding 
    to the regions of these ways.

    Inputs:
        road_module:
        The road module instance of RoadModule class.
        image_width:
        The desired image width.
        image_height:
        The desired image height.
        line_type:
        The desired line_type of the ways we wish to know the
        region of.
        negative_color:
        The background (default) color of the mask.
        positive_color:
        The color of places where the mask is active.

    Returns:
        mask_image:
        The mask image, where the region of the ways with the given 
        line_type are colored positive_color and everywhere else is
        colored negative_color

    '''

    mask_image = pygame.Surface((image_width, image_height), 0, 32)
    mask_image.fill(negative_color)

    for way_id in road_module.osm_way_dict:

        way = road_module.osm_way_dict[way_id]

        if way.line_type != line_type:

            continue
        
        node_tuple_list = []

        for node_id in way.node_ids:

            current_node = laneletlibrary.get_osm_node_by_id(road_module.osm_node_dict, node_id)

            current_tuple = ( int( current_node.pixel_x ) , int( current_node.pixel_y ) )
            node_tuple_list.append( current_tuple )

        pygame.draw.polygon(mask_image, positive_color, tuple( node_tuple_list ) )

    return mask_image


def draw_all_ways(road_module, world_image, brightness = 1.0):
    '''
    It will draw all of the ways in road_module into the 
    world image.

    Inputs:
        road_module:
        The road module instance of RoadModule class.
        world_image:
        The image of the world. (A pygame.Surface)
        brightness: (Optional)
        Defines how bright/dark the way will be drawn.
        1.0 = full brightness, corresponds to the original color
        0.0 = zero brightness, the color will be completely black
    '''

    for way_id in road_module.osm_way_dict:

        way = road_module.osm_way_dict[way_id]

        draw_way(way, road_module.osm_node_dict, world_image, brightness)


def draw_way(osm_way, osm_node_dict, world_image, brightness = 1.0):
    '''
    It will draw a way onto the world image.

    Inputs:
        osm_way:
        The OsmWay to be drawn.
        osm_node_dict:
        The dictionary of osm_nodes.
        world_image:
        The image of the world. (A pygame.Surface)
        brightness: (Optional)
        Defines how bright/dark the way will be drawn.
        1.0 = full brightness, corresponds to the original color
        0.0 = zero brightness, the color will be completely black
    '''


    if osm_way.line_type == None:
        return

    node_tuple_list = []

    for node_id in osm_way.node_ids:

        current_node = osm_node_dict[node_id]

        current_tuple = ( int( current_node.pixel_x ) , int( current_node.pixel_y ) )
        node_tuple_list.append( current_tuple )

    COLOR = ()
    line_thickness = 0

    if osm_way.line_type == 'exterior':
        COLOR = (191, 179, 8) # Dirty yellow
        line_thickness = 2

    elif osm_way.line_type == 'interior':
        COLOR = (245, 243, 223) # Dirty white 
        line_thickness = 2

    else:

        return

    if brightness != 1.0:

        NEW_COLOR = (brightness*COLOR[0], brightness*COLOR[1], brightness*COLOR[2])

    else:

        NEW_COLOR = COLOR

    line_closed = False

    pygame.draw.lines(world_image, NEW_COLOR, line_closed, tuple( node_tuple_list ), line_thickness)


def set_pixel_values_for_nodes(road_module, canvas_width, canvas_height, pixel_per_meter):
    '''
    This function will set the pixel position values for the OSMNodes in osm_node_dict
    The pixel positions are used for drawing and visualisation purposes

    '''



    print 'set_pixel_values_for_nodes'


    print "road_module.osm_node_tag_dict = " + str(road_module.osm_node_tag_dict)

    if 'origin' in road_module.osm_node_tag_dict:

        origin_node_id = road_module.osm_node_tag_dict['origin'][0]

        print "origin_node_id = " + str(origin_node_id) 

        origin_x = road_module.osm_node_dict[origin_node_id].x
        origin_y = road_module.osm_node_dict[origin_node_id].y

    else:

        min_x = 10e+10
        max_x = - 10e+10
        min_y = 10e+10
        max_y = - 10e+10

        avg_x = 0
        avg_y = 0

        number_nodes = len( road_module.osm_node_dict )

        for osm_node_id in road_module.osm_node_dict:

            node = road_module.osm_node_dict[osm_node_id]

            avg_x = avg_x + node.x
            avg_y = avg_y + node.y

            if ( min_x > node.x ):
                min_x = node.x
            if ( max_x < node.x ):
                max_x = node.x
            if ( min_y > node.y ):
                min_y = node.y
            if ( max_y < node.y ):
                max_y = node.y

        avg_x = avg_x/number_nodes
        avg_y = avg_y/number_nodes

        origin_x = avg_x
        origin_y = avg_y

    print "pixel_per_meter = " + str(pixel_per_meter) 

    for way_id in road_module.osm_way_dict:

        way = road_module.osm_way_dict[way_id]

        for idx in xrange(0, len(way.node_ids)):

            OSMNode = road_module.osm_node_dict[way.node_ids[idx]]

            # print "avg_x = " + str(avg_x) 
            # print "avg_y = " + str(avg_y) 

            # OSMNode.pixel_x = canvas_width/2. + (OSMNode.x - avg_x)*pixel_per_meter
            # OSMNode.pixel_y = canvas_height/2. - (OSMNode.y - avg_y)*pixel_per_meter

            # print "[OSMNode.x, OSMNode.y] = " + str([OSMNode.x, OSMNode.y])

            OSMNode.pixel_x = canvas_width/2. + (OSMNode.x)*pixel_per_meter
            OSMNode.pixel_y = canvas_height/2. - (OSMNode.y)*pixel_per_meter

    # center_pixel_x = canvas_width/2. + -avg_x*pixel_per_meter
    # center_pixel_y = canvas_height/2. - -avg_y*pixel_per_meter
    center_pixel_x = canvas_width/2.
    center_pixel_y = canvas_height/2.

    return [center_pixel_x, center_pixel_y, pixel_per_meter]
