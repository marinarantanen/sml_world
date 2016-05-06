"""
Visualization module for the SML World.

Created on Feb 25, 2016

@author: U{Rui Oliviera<rfoli.se>}, U{David Spulak<spulak@kth.se>}
@organization: KTH
"""

import pygame
import os
import math

from sml_modules import bodyclasses
from sml_modules.vehicle_models import BaseVehicle, DummyVehicle, Bus

from sml_world.srv import GetTrajectory
import rospy

class Visualization:
    """
    Class to diplay the current state of the SML World.

    Visualization is a class to visually display the current state of the
    SML World.  It is necessary that the thread running this class is the main
    program thread, otherwise Pygame will not work correctly.
    Due to that reason this class can be lauched from two ways:

    1) Using Process from multiprocessing library;
    2) By an individual secondary script that is simply giving this class the
       main thread of processing.
    """

    def __init__(self, base_path, file_path, window_width, window_height,
                 pixel_per_meter=-1, ground_projection=False):
        """
        Inizialize the class Visualization.

        @param base_path:
        @param file_path:
        """
        # The desired dimensions and pixel resolution
        # for the window to be shown
        self.desired_window_width = float(window_width)
        self.desired_window_height = float(window_height)
        self.desired_pixel_per_meter = float(pixel_per_meter)

        # The filename which defines the map/image to
        # serve as background
        self.base_path = base_path
        map_filename = base_path + file_path
        # Determines if the visualization is meant to
        # be used for the ground projector at the
        # SML
        self.ground_projection = ground_projection

        # As measured by Rui [LEFT, RIGHT, DOWN, UP]
        self.projector_area = [3.360, 4.490, 2.920, 2.970]
#        self.projector_area = [300, 400, 20, 20]
        # The refresh rate of the visualization screen
        self.refresh_rate = float(20)

        self.bg_surface = None

        self.vehicles_dict = dict()

        self.areas_to_blit = []

        self.load_image_meta_data(map_filename)
        self.load_image(map_filename)

        self.load_red_car_image()
        self.load_yellow_car_image()
        self.load_green_car_image()
        self.load_blue_car_image()
        self.load_white_car_image()

        self.load_smart_car_image()
        self.load_truck_image()
        self.load_red_bus_image()
        self.load_green_bus_image()
        self.load_yellow_bus_image()
        self.load_bus_image()
        self.load_big_box_image()
        self.load_small_box_image()
        self.load_goal_image()
        self.setup_id_font()
        self.load_block_image()

        self.show_ids = True

    def loop_iteration(self, world_state):
        """
        Draw the received world state.

        The main loop iteration of the class.  It will try to run a fixed rate,
        as defined by self.refresh_rate.  If it does not manage to keep this
        time it will issue terminal warnings to the user.
        The loop consists in:
            1) Receive the latest vehicle states information
               through UDP
            2) Draw said states

        Returns:
        A boolean indicating if the user closed the
        visualization window (True) or not (False)
        """
        # Receive the latest vehicle states information
        self.vehicles_dict = world_state
        # Draw the the latest vehicle states
        self.display_image()

        for event in pygame.event.get():

            # Checks if the user tried to close the Window
            if event.type == pygame.QUIT:

                pygame.quit()
                return True

    def load_image_meta_data(self, map_filename):
        """
        Load the meta data corresponding to map_filename.

        Given the map_filename, it will find the corresponding image metadata
        of said map file.  The file is parsed, and the image metadata
        (image_width, image_height, pixel_per_meter) is gathered and stored
        in the class.
        """
        # Opening the file, reading it into a string
        # and closing it.
        meta_data_filename = map_filename + '.meta'

        f = open(meta_data_filename, 'r')

        meta_data_string = f.read()

        f.close()

        # Process the string containing the metadata
        tokens = meta_data_string.split(';')

        for token in tokens:

            property_tokens = token.split('=')

            if len(property_tokens) != 2:
                print "Error parsing meta data file!"
                continue

            attribute_token = property_tokens[0]
            value_token = property_tokens[1]

            if attribute_token == 'image_width':
                self.loaded_image_width = float(value_token)

            elif attribute_token == 'image_height':
                self.loaded_image_height = float(value_token)

            elif attribute_token == 'pixel_per_meter':
                self.loaded_image_pixel_per_meter = float(value_token)

            else:
                print "Error parsing meta data file!"
                continue

        self.original_image_width = self.loaded_image_width
        self.original_image_height = self.loaded_image_height
        self.original_image_pixel_per_meter = self.loaded_image_pixel_per_meter

        self.image_width = self.loaded_image_width
        self.image_height = self.loaded_image_height

        # By default, the center x of an image,
        # this is, the point where x and y are 0 in
        # real world meters corresponds to the center
        # of the image
        self.image_center_x = self.loaded_image_width / 2.
        self.image_center_y = self.loaded_image_height / 2.

        self.image_pixel_per_meter = self.loaded_image_pixel_per_meter

        return

    def load_image(self, map_filename):
        """
        Load the image corresponding to map_filename.

        Given the map_filename, it will find the corresponding image of said
        map file.  Depending on the desired window width/height and
        pixel_per_meter it will resize and crop the image. If in
        ground_projection mode, it will also apply the necessary
        transformations so that the image can be correctly projected on the
        ground.
        """
        pygame.init()

        image_filename = map_filename + '.bmp'
        self.bg_surface = pygame.image.load(image_filename)

        # Just a sanity check
        (loaded_image_width, loaded_image_height) = self.bg_surface.get_size()
        if (loaded_image_width != self.loaded_image_width or
            loaded_image_height != self.loaded_image_height):

            raise NameError("Meta data does not comply with the loaded image!")

        if self.ground_projection:
            # Need to crop the original image to the Projector dimensions

            if self.desired_pixel_per_meter != -1:
                print ("WARNING: Ground Projection will ignore the provided" +
                       "pixel_per_meter parameter")

            # NOTE: This bugged my mind a lot, the 2.92 and 2.97
            # should be switched
            top_left_x = ((self.loaded_image_width / 2.) -
                          self.projector_area[0] * 32. *
                          self.loaded_image_pixel_per_meter)
            top_left_y = ((self.loaded_image_height / 2.) -
                          self.projector_area[2] * 32. *
                          self.loaded_image_pixel_per_meter)
            bot_right_x = ((self.loaded_image_width / 2.) +
                           self.projector_area[1] * 32. *
                           self.loaded_image_pixel_per_meter)
            bot_right_y = ((self.loaded_image_height / 2.) +
                           self.projector_area[3] * 32. *
                           self.loaded_image_pixel_per_meter)

            top_left_x = int(round(top_left_x))
            top_left_y = int(round(top_left_y))
            bot_right_x = int(round(bot_right_x))
            bot_right_y = int(round(bot_right_y))

            background_array = pygame.PixelArray(self.bg_surface)
            cropped_image_array = background_array[top_left_x:bot_right_x,
                                                   top_left_y:bot_right_y]

            self.bg_surface = cropped_image_array.make_surface()
            self.bg_surface = pygame.transform.smoothscale(
                                self.bg_surface,
                                (int(self.desired_window_width),
                                 int(self.desired_window_height)))

            self.image_pixel_per_meter = (self.desired_window_width /
                                          ((self.projector_area[0] +
                                            self.projector_area[1]) * 32.))
            self.image_pixel_per_meter = (self.desired_window_height /
                                          ((self.projector_area[3] +
                                            self.projector_area[2]) * 32.))

            self.image_center_x = (self.desired_window_width *
                                   self.projector_area[0] /
                                   (self.projector_area[0] +
                                    self.projector_area[1]))
            self.image_center_y = (self.desired_window_height *
                                   self.projector_area[2] /
                                   (self.projector_area[2] +
                                    self.projector_area[3]))

        elif self.desired_pixel_per_meter == -1:
            # CURENTLY BUGGED
            # Only need to rescale the image in order to comply with the
            # desired dimensions
            self.bg_surface = pygame.transform.smoothscale(
                                self.bg_surface,
                                (int(self.desired_window_width),
                                 int(self.desired_window_height)))
            self.bg_surface_array = pygame.surfarray.array3d(self.bg_surface)

            x_scale_ratio = (float(self.desired_window_width) /
                             float(self.loaded_image_width))
            y_scale_ratio = (float(self.desired_window_height) /
                             float(self.loaded_image_height))

            self.image_pixel_per_meter = (self.loaded_image_pixel_per_meter *
                                          x_scale_ratio)
            # self.image_pixel_per_meter = (self.loaded_image_pixel_per_meter *
            #                               y_scale_ratio)

            self.image_center_x = self.loaded_image_width / 2. * x_scale_ratio
            self.image_center_y = self.loaded_image_height / 2. * y_scale_ratio

        else:
            # Need to rescale the image in order to comply with the desired
            # dimensionsand pixel_per_meter

            # CURENTLY BUGGED

            half_width_meters = (self.desired_window_width / 2. /
                                 self.desired_pixel_per_meter)
            half_height_meters = (self.desired_window_height / 2. /
                                  self.desired_pixel_per_meter)

            top_left_x = ((self.loaded_image_width / 2.) -
                          half_width_meters *
                          self.loaded_image_pixel_per_meter)
            top_left_y = ((self.loaded_image_height / 2.) -
                          half_height_meters *
                          self.loaded_image_pixel_per_meter)
            bot_right_x = ((self.loaded_image_width / 2.) +
                           half_width_meters *
                           self.loaded_image_pixel_per_meter)
            bot_right_y = ((self.loaded_image_height / 2.) +
                           half_height_meters *
                           self.loaded_image_pixel_per_meter)

            top_left_x = int(round(top_left_x))
            top_left_y = int(round(top_left_y))
            bot_right_x = int(round(bot_right_x))
            bot_right_y = int(round(bot_right_y))

            background_array = pygame.PixelArray(self.bg_surface)
            cropped_image_array = background_array[top_left_x:bot_right_x,
                                                   top_left_y:bot_right_y]

            self.bg_surface = cropped_image_array.make_surface()

            self.bg_surface = pygame.transform.smoothscale(
                                self.bg_surface,
                                (int(self.desired_window_width),
                                 int(self.desired_window_height)))
            self.bg_surface_array = pygame.surfarray.array3d(self.bg_surface)

            self.image_pixel_per_meter = self.desired_pixel_per_meter
            self.image_pixel_per_meter = self.desired_pixel_per_meter

            self.image_center_x = (self.desired_window_width / 2.)
            self.image_center_y = (self.desired_window_height / 2.)

        # self.bg_surface_pixel_array = pygame.PixelArray(self.bg_surface)

        self.areas_to_blit.append([0, 0, int(self.desired_window_width),
                                   int(self.desired_window_height)])

        image_size = (int(self.desired_window_width),
                      int(self.desired_window_height))

        if self.ground_projection:

            # This sets up the window position on the top left corner of the
            # screen
            x = 0
            y = 0
            os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (x, y)

            # pygame.NOFRAME makes the visualization window not have a Frame
            self.window = pygame.display.set_mode(image_size, pygame.NOFRAME)

        else:

            self.window = pygame.display.set_mode(image_size)

        return

    def load_block_image(self):
        """Load the block image."""
        block_width_meters = 2.096
        block_length_meters = (4.779 - 0.910) * 2.

        self.block_image = self.get_car_image(
                        self.base_path + '/resources/block.png',
                        block_width_meters, block_length_meters)

        return

    def load_smart_car_image(self):
        """Load the car image, used for displaying the current vehicles."""
        car_width_meters = 2.096
        car_length_meters = (4.779 - 0.910) * 2.

        self.smart_car_image = self.get_car_image(
                        self.base_path + '/resources/carSmartOffset.png',
                        car_width_meters, car_length_meters)

        return

    def load_red_car_image(self):
        """Load the car image, used for displaying the current vehicles."""
        car_width_meters = 2.096
        car_length_meters = (4.779 - 0.910) * 2.

        self.red_car_image = self.get_car_image(
                    self.base_path + '/resources/carRedOffset.png',
                    car_width_meters, car_length_meters)

        return

    def load_green_car_image(self):
        """Load the car image, used for displaying the current vehicles."""
        car_width_meters = 2.096
        car_length_meters = (4.779 - 0.910) * 2.

        self.green_car_image = self.get_car_image(
                            self.base_path + '/resources/carGreenOffset.png',
                            car_width_meters, car_length_meters)

        return

    def load_blue_car_image(self):
        """Load the car image, used for displaying the current vehicles."""
        car_width_meters = 2.096
        car_length_meters = (4.779 - 0.910) * 2.

        self.blue_car_image = self.get_car_image(
                        self.base_path + '/resources/carBlueOffset.png',
                        car_width_meters, car_length_meters)

        return

    def load_white_car_image(self):
        """Load the car image, used for displaying the current vehicles."""
        car_width_meters = 2.096
        car_length_meters = (4.779 - 0.910) * 2.

        self.white_car_image = self.get_car_image(
                        self.base_path + '/resources/carWhiteOffset.png',
                        car_width_meters, car_length_meters)

        return

    def load_yellow_car_image(self):
        """Load the car image, used for displaying the current vehicles."""
        car_width_meters = 2.096
        car_length_meters = (4.779 - 0.910) * 2.

        self.yellow_car_image = self.get_car_image(
                        self.base_path + '/resources/carYellowOffset.png',
                        car_width_meters, car_length_meters)

        return

    def get_car_image(self, car_image_filename, car_width_meters,
                      car_length_meters):
        """Load the car image stored in the file car_image_filename."""
        car_image = pygame.image.load(car_image_filename)

        (car_image_width, car_image_height) = car_image.get_size()

        # pixel_per_meter_image = car_image_height/car_width_meters
        [x_pixel_1, _] = self.convert_position_to_image_pixel(0, 0)
        [x_pixel_2, _] = self.convert_position_to_image_pixel(car_width_meters,
                                                              0)

        desired_car_width_pixels = float(x_pixel_2 - x_pixel_1)

        scale_down_ratio = desired_car_width_pixels / car_image_height

        new_size = (int(round(scale_down_ratio * car_image_width)),
                    int(round(scale_down_ratio * car_image_height)))

        car_image = pygame.transform.smoothscale(car_image, new_size)

        return car_image

    def load_truck_image(self):
        """Load the truck image, used for displaying the current vehicles."""
        minitruck_width_meters = 0.08
        # minitruck_length_meters = 0.19
        minitruck_length_meters = 0.145 * 2
        # This is the SML world meters
        # of the minitruck image width

        truck_width_meters = 32. * minitruck_width_meters
        truck_length_meters = 32. * minitruck_length_meters

        self.truck_image = self.get_car_image(
                    self.base_path + '/resources/truckTopOffset.png',
                    truck_width_meters, truck_length_meters)

        return

    def load_bus_image(self):
        """Load the bus image, used for displaying the current vehicles."""
        minibus_width_meters = 0.08
        # minibus_length_meters = 0.19
        minibus_length_meters = 0.145 * 2
        # This is the SML world meters
        # of the minibus image width
        bus_width_meters = 32. * minibus_width_meters
        bus_length_meters = 32. * minibus_length_meters
        self.bus_image = self.get_bus_image(
                        self.base_path + '/resources/busTopOffset.png',
                        bus_width_meters, bus_length_meters)
        return

    def load_red_bus_image(self):
        """Load the bus image, used for displaying the current vehicles."""
        minibus_width_meters = 0.08
        # minibus_length_meters = 0.19
        minibus_length_meters = 0.145 * 2
        # This is the SML world meters
        # of the minibus image width
        bus_width_meters = 32. * minibus_width_meters
        bus_length_meters = 32. * minibus_length_meters
        self.red_bus_image = self.get_bus_image(
                        self.base_path + '/resources/redBusOffset.png',
                        bus_width_meters, bus_length_meters)
        return

    def load_green_bus_image(self):
        """Load the bus image, used for displaying the current vehicles."""
        minibus_width_meters = 0.08
        # minibus_length_meters = 0.19
        minibus_length_meters = 0.145 * 2
        # This is the SML world meters
        # of the minibus image width
        bus_width_meters = 32. * minibus_width_meters
        bus_length_meters = 32. * minibus_length_meters
        self.green_bus_image = self.get_bus_image(
                        self.base_path + '/resources/greenBusOffset.png',
                        bus_width_meters, bus_length_meters)
        return

    def load_yellow_bus_image(self):
        """Load the bus image, used for displaying the current vehicles."""
        minibus_width_meters = 0.08
        # minibus_length_meters = 0.19
        minibus_length_meters = 0.145 * 2
        # This is the SML world meters
        # of the minibus image width
        bus_width_meters = 32. * minibus_width_meters
        bus_length_meters = 32. * minibus_length_meters
        self.yellow_bus_image = self.get_bus_image(
                        self.base_path + '/resources/yellowBusOffset.png',
                        bus_width_meters, bus_length_meters)
        return

    def get_bus_image(self, bus_image_filename, bus_width_meters,
                      bus_length_meters):
        """Load the bus image, used for displaying the current vehicles."""
        bus_image = pygame.image.load(bus_image_filename)
        (bus_image_width, bus_image_height) = bus_image.get_size()

        # pixel_per_meter_image = car_image_height/car_width_meters
        [x_pixel_1, _] = self.convert_position_to_image_pixel(0, 0)
        [x_pixel_2, _] = self.convert_position_to_image_pixel(bus_width_meters,
                                                              0)

        desired_bus_width_pixels = float(x_pixel_2 - x_pixel_1)

        scale_down_ratio = desired_bus_width_pixels / bus_image_height

        new_size = (int(round(scale_down_ratio * bus_image_width)),
                    int(round(scale_down_ratio * bus_image_height)))

        bus_image = pygame.transform.smoothscale(bus_image, new_size)

        return bus_image

    def load_big_box_image(self):
        """Load the big_box image, used for displaying the current vehicles."""
        box_length_meters = 32. * 0.6

        box_image = pygame.image.load(
                        self.base_path + '/resources/waterOffset.png')

        (box_image_width, box_image_height) = box_image.get_size()

        [x_pixel_1, _] = self.convert_position_to_image_pixel(0, 0)
        [x_pixel_2, _] = self.convert_position_to_image_pixel(
                                box_length_meters,
                                0)

        desired_box_width_pixels = float(x_pixel_2 - x_pixel_1)

        scale_down_ratio = desired_box_width_pixels / (box_image_width / 2.)

        new_size = (int(round(scale_down_ratio * box_image_width)),
                    int(round(scale_down_ratio * box_image_height)))

        box = pygame.transform.smoothscale(box_image, new_size)

        self.box_image = box

        return

    def load_small_box_image(self):
        """Load small_box image, used for displaying the current vehicles."""
        box_length_meters = 32. * 0.4

        box_image = pygame.image.load(
                        self.base_path + '/resources/waterOffset.png')

        (box_image_width, box_image_height) = box_image.get_size()

        [x_pixel_1, _] = self.convert_position_to_image_pixel(0, 0)
        [x_pixel_2, _] = self.convert_position_to_image_pixel(
                                box_length_meters,
                                0)

        desired_box_width_pixels = float(x_pixel_2 - x_pixel_1)

        scale_down_ratio = desired_box_width_pixels / (box_image_width / 2.)

        new_size = (int(round(scale_down_ratio * box_image_width)),
                    int(round(scale_down_ratio * box_image_height)))

        box = pygame.transform.smoothscale(box_image, new_size)

        self.small_box_image = box

        return

    def load_goal_image(self):
        """Load the goal image, used for displaying the current vehicles."""
        box_length_meters = 32. * 0.2

        flag_image = pygame.image.load(
                        self.base_path + '/resources/finishFlag.png')

        (flag_image_width, flag_image_height) = flag_image.get_size()

        [x_pixel_1, _] = self.convert_position_to_image_pixel(0, 0)
        [x_pixel_2, _] = self.convert_position_to_image_pixel(
                                box_length_meters,
                                0)

        desired_box_width_pixels = float(x_pixel_2 - x_pixel_1)

        scale_down_ratio = desired_box_width_pixels / (flag_image_width / 2.)

        new_size = (int(round(scale_down_ratio * flag_image_width)),
                    int(round(scale_down_ratio * flag_image_height)))

        flag = pygame.transform.smoothscale(flag_image, new_size)

        self.goal_image = flag

        return

    def setup_id_font(self):
        """Define the font properties used for writing the vehicle ids."""
        font_size = 10

        self.ids_font = pygame.font.SysFont('monospace', font_size)

        self.ids_font.set_bold(True)

        self.font_color = (255, 255, 0)

        return

    def dumb_background_blit(self):
        """
        Blit the whole background image into the visualization window.

        Blits the background picture into the visualization window.  It is
        named dumb since it blits ALL of the pixels of the background into the
        window, even if this results in pixels not changing their value.
        """
        # self.window.blit(self.bg_surface, (0,0))
        # Average time: 0.033915

        pygame.surfarray.blit_array(self.window, self.bg_surface_array)
        # Average time: 0.022714

    def smart_background_blit(self):
        """
        Blit the background image only into areas of interest.

        Blits the background picture into the visualization window.  It is
        named smart since it blits only interest regions as defined by
        self.areas_to_blit.  These areas are areas that we consider that need
        to be blitted, because they were previously drawn with things that are
        not the background (e.g.: a vehicle)
        """
        for area_to_blit in self.areas_to_blit:
            self.window.blit(self.bg_surface, area_to_blit[:2], area_to_blit)
        # Empty the areas to blit
        self.areas_to_blit = []

        return

    def draw_events(self):
#        event = self.events_dict[event_id]
        event = 1
        if event == 1:
            self.draw_block(0, 0)
        elif event == 2:
            self.draw_block(10, 10)
        return

    def trajectory(self, prevpos):
        poslist = prevpos
        if len(self.vehicles_dict) > 0:
            value = dict((key, value) for key, value
                in self.vehicles_dict.iteritems() if key == 1)
            vehicle_info = value.values()
            vehicle_stats = vehicle_info[0].values()
            x = vehicle_stats[1]
            y = vehicle_stats[4]
            [pixel_x, pixel_y] = self.convert_position_to_image_pixel(x, y)
            pos = [pixel_x, pixel_y]
            poslist.append(pos)
        return poslist

    def draw_vehicles(self):
        """Iterate over the vehicles in self.vehicles_dict and draw them."""

        red = (255, 0, 0)

        for vehicle_id in self.vehicles_dict:

            vehicle = self.vehicles_dict[vehicle_id]

            vehicle_class_name = vehicle['class_name']

#            def prevpos(self):
#                prevx = vehicle['x']
#                prevy = vehicle['y']
#                [pix_prevx, pix_prevy] = self.convert_position_to_image_pixel(prevx, prevy)
#                prev = [pix_prevx, pix_prevy]
#                prevpos.append(prev)
#            return prevpos

            prevx = vehicle['x']
            prevy = vehicle['y']
            [pix_prevx, pix_prevy] = self.convert_position_to_image_pixel(prevx-5, prevy-10)
            prev = [(pix_prevx, pix_prevy)]
            poslist = self.trajectory(prev)

            if vehicle_class_name == bodyclasses.QualisysGoal.__name__:
                self.draw_goal(vehicle['x'], vehicle['y'])

            elif vehicle_class_name == bodyclasses.QualisysBigBox.__name__:
                self.draw_box(vehicle['x'], vehicle['y'], vehicle['yaw'])

            elif vehicle_class_name == bodyclasses.QualisysSmallBox.__name__:
                self.draw_small_box(vehicle['x'], vehicle['y'], vehicle['yaw'])

            elif (vehicle_class_name == DummyVehicle.__name__ or
                  vehicle_class_name == BaseVehicle.__name__):
                if vehicle_id > -100:
                    color = vehicle_id % 5

                    if color == 0:
                        self.draw_white_car(vehicle['x'], vehicle['y'],
                                            vehicle['yaw'])

                    elif color == 1:
                        self.draw_green_car(vehicle['x'], vehicle['y'],
                                            vehicle['yaw'])

                    elif color == 2:
                        self.draw_blue_car(vehicle['x'], vehicle['y'],
                                           vehicle['yaw'])

                    elif color == 3:
                        self.draw_yellow_car(vehicle['x'], vehicle['y'],
                                             vehicle['yaw'])

                    elif color == 4:
                        self.draw_red_car(vehicle['x'], vehicle['y'],
                                          vehicle['yaw'])
            elif (vehicle_class_name == Bus.__name__):
                if vehicle_id > -100:
                    color = vehicle_id % 4

                    if color == 0:
                        self.draw_bus(vehicle['x'], vehicle['y'],
                                      vehicle['yaw'])

                    elif color == 1:
                        self.draw_red_bus(vehicle['x'], vehicle['y'],
                                          vehicle['yaw'])
                        pygame.draw.lines(self.window, red, True, poslist, 3)

                    elif color == 2:
                        self.draw_green_bus(vehicle['x'], vehicle['y'],
                                            vehicle['yaw'])

                    elif color == 3:
                        self.draw_yellow_bus(vehicle['x'], vehicle['y'],
                                             vehicle['yaw'])

            # elif vehicle_class_name == smartvehicle.SmartVehicle.__name__:

            # self.draw_smart_car(vehicle['x'], vehicle['y'], vehicle['yaw'])

            else:

                print "vehicle_class_name = " + str(vehicle_class_name)
                raise NameError("Unexpected")
                # self.draw_truck(vehicle['x'], vehicle['y'], vehicle['yaw'])

            if self.show_ids:
                self.draw_id(vehicle_id, vehicle['x'], vehicle['y'])

        for vehicle_id in self.vehicles_dict:

            vehicle = self.vehicles_dict[vehicle_id]

            # Debug purposes
            # circle_radius = int(round(0.5*self.image_pixel_per_meter))
            # pygame.draw.circle(self.window, (0,255,0), (pixel_x,pixel_y),
            #                    circle_radius, 0)
            # self.areas_to_blit.append([pixel_x - circle_radius * 10,
            #                            pixel_y - circle_radius * 2,
            #                            pixel_x + circle_radius * 10,
            #                            pixel_y + circle_radius * 10])

        return

    def draw_car(self, car_x, car_y, car_yaw):
        """
        Draw a car, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the car image as an
        argument.
        """
        self.draw_vehicle_image(car_x, car_y, car_yaw, self.car_image)
        return

    def draw_red_car(self, car_x, car_y, car_yaw):
        """
        Draw a car, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the car image as an
        argument.
        """
        self.draw_vehicle_image(car_x, car_y, car_yaw, self.red_car_image)
        return

    def draw_yellow_car(self, car_x, car_y, car_yaw):
        """
        Draw a car, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the car image as an
        argument.
        """
        self.draw_vehicle_image(car_x, car_y, car_yaw, self.yellow_car_image)
        return

    def draw_blue_car(self, car_x, car_y, car_yaw):
        """
        Draw a car, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the car image as an
        argument.
        """
        self.draw_vehicle_image(car_x, car_y, car_yaw, self.blue_car_image)
        return

    def draw_green_car(self, car_x, car_y, car_yaw):
        """
        Draw a car, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the car image as an
        argument.
        """
        self.draw_vehicle_image(car_x, car_y, car_yaw, self.green_car_image)
        return

    def draw_white_car(self, car_x, car_y, car_yaw):
        """
        Draw a car, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the car image as an
        argument.
        """
        self.draw_vehicle_image(car_x, car_y, car_yaw, self.white_car_image)
        return

    def draw_smart_car(self, car_x, car_y, car_yaw):
        """
        Draw a smart car, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the car image as an
        argument.
        """
        self.draw_vehicle_image(car_x, car_y, car_yaw, self.smart_car_image)
        return

    def draw_truck(self, truck_x, truck_y, truck_yaw):
        """
        Draw a truck, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the truck image as an
        argument.
        """
        self.draw_vehicle_image(truck_x, truck_y, truck_yaw, self.truck_image)
        return

    def draw_green_bus(self, truck_x, truck_y, truck_yaw):
        """
        Draw a truck, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the truck image as an
        argument.
        """
        self.draw_vehicle_image(truck_x, truck_y, truck_yaw, self.green_bus_image)
        return

    def draw_red_bus(self, truck_x, truck_y, truck_yaw):
        """
        Draw a truck, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the truck image as an
        argument.
        """
        self.draw_vehicle_image(truck_x, truck_y, truck_yaw, self.red_bus_image)
        return

    def draw_yellow_bus(self, truck_x, truck_y, truck_yaw):
        """
        Draw a truck, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the truck image as an
        argument.
        """
        self.draw_vehicle_image(truck_x, truck_y, truck_yaw, self.yellow_bus_image)
        return

    def draw_bus(self, truck_x, truck_y, truck_yaw):
        """
        Draw a bus, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the bus image as an
        argument.
        """
        self.draw_vehicle_image(truck_x, truck_y, truck_yaw, self.bus_image)
        return

    def draw_box(self, truck_x, truck_y, truck_yaw):
        """
        Draw a box, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the box image as an
        argument.
        """
        self.draw_vehicle_image(truck_x, truck_y, truck_yaw, self.box_image)
        return

    def draw_small_box(self, truck_x, truck_y, truck_yaw):
        """
        Draw a small box, given its state: x, y and yaw.

        It does so by calling draw_vehicle_image with the small box image as an
        argument.
        """
        self.draw_vehicle_image(truck_x, truck_y, truck_yaw,
                                self.small_box_image)
        return

    def draw_vehicle_image(self, vehicle_x, vehicle_y, vehicle_yaw,
                           vehicle_image):
        """Draw a given vehicle image, given its position and yaw."""
        [pixel_x, pixel_y] = self.convert_position_to_image_pixel(vehicle_x,
                                                                  vehicle_y)
        vehicle_yaw = math.degrees(vehicle_yaw)
        vehicle_rotated = pygame.transform.rotate(vehicle_image, vehicle_yaw)
        vehicle_size_rotated = vehicle_rotated.get_size()

        new_x = pixel_x - vehicle_size_rotated[0] / 2
        new_y = pixel_y - vehicle_size_rotated[1] / 2
        pos = (int(round(new_x)), int(round(new_y)))

        if self.should_be_blit(vehicle_rotated, pos):

            self.window.blit(vehicle_rotated, pos)

            self.add_surface_to_areas_to_blit(vehicle_rotated, pos)

        return

    def draw_block(self, block_x, block_y):
        """
        Draw a road block, given its position, x, y.

        It does so by calling draw_vehicle_image with the block image as an
        argument
        """
        block_yaw = 0
        self.draw_vehicle_image(block_x, block_y, block_yaw, self.block_image)
        return

    def draw_goal(self, goal_x, goal_y):
        """
        Draw a goal, given its position, x, y.

        It does so by calling draw_vehicle_image with the goal image as an
        argument
        """
        goal_yaw = 0
        self.draw_vehicle_image(goal_x, goal_y, goal_yaw, self.goal_image)
        return

    def draw_goal_circle(self, vehicle_x, vehicle_y):
        """Draw a goal circle, given its state, x, y."""
        goal_radius_meters = 0.25 * 32.
        # pixel_per_meter_image = car_image_height/car_width_meters
        [x_pixel_1, _] = self.convert_position_to_image_pixel(0, 0)
        [x_pixel_2, _] = self.convert_position_to_image_pixel(
                                        goal_radius_meters,
                                        0)

        [pixel_x, pixel_y] = self.convert_position_to_image_pixel(vehicle_x,
                                                                  vehicle_y)

        color = (255, 0, 255)
        pos = (pixel_x, pixel_y)
        radius = int(round(x_pixel_2 - x_pixel_1))
        width = 0  # Makes the circle filled

        pygame.draw.circle(self.window, color, pos, radius, width)

        blit_radius = int(round(1.2 * radius))

        self.areas_to_blit.append([pos[0] - blit_radius, pos[1] - blit_radius,
                                   pos[0] + blit_radius, pos[1] + blit_radius])
        return

    def should_be_blit(self, surface, surface_pos):
        """
        Decide if surface is worth blitting or not.

        TO BE IMPLEMENTED
        This function should receive a surface and the position
        of said surface, and decide if this image is worth blitting
        or not.
        An image is not worth blitting if it falls completely outside
        of the image area. Otherwise it should be blit
        """
        should_be_blit = True

        # (image_width, image_height) = self.bg_surface_pixel_array.get_size()

        # if surface_pos[0] < 0 and surface_pos[1] < 0

        return should_be_blit

    def draw_id(self, vehicle_id, vehicle_x, vehicle_y):
        """Draw a vehicle id, given its id, x, y and yaw."""
        [pixel_x, pixel_y] = self.convert_position_to_image_pixel(vehicle_x,
                                                                  vehicle_y)

        text_surface = self.ids_font.render(str(vehicle_id), True,
                                            self.font_color)

        text_pos = (pixel_x, pixel_y)

        self.window.blit(text_surface, text_pos)

        self.add_surface_to_areas_to_blit(text_surface, text_pos)

        return

    def add_surface_to_areas_to_blit(self, surface, surface_pos):
        """
        Add surface region to the self.areas_to_blit list.

        Add the area of the region defined by surface and surface_pos to
        self.areas_to_blit.  This will tell the smart_background_blit method
        that this area needs to be blitted with the background in the next
        screen refresh
        """
        (surface_width, surface_height) = surface.get_size()

        self.areas_to_blit.append([surface_pos[0], surface_pos[1],
                                   surface_width, surface_height])
        return

    def display_image(self):
        """Call the methods needed to refresh and display the current image."""
        # First, redraw the image to be the original
        # brackground, with no vehicles in it.

        # self.dumb_background_blit()
        self.smart_background_blit()

        # Once the background is drawn,
        # draw the vehicles
        self.draw_vehicles()
        self.draw_events()

        # Pygame functions to update the visualization
        # window
        pygame.display.flip()
        pygame.event.pump()

        return

    def convert_position_to_image_pixel(self, x_pos, y_pos):
        """
        Convert world coordinates to pixel coordinates.

        Given a position in real world meters, it will return the equivalent
        pixel in the visualization window image.
        """
        x_pixel = self.image_center_x + x_pos * self.image_pixel_per_meter
        y_pixel = self.image_center_y - y_pos * self.image_pixel_per_meter

        x_pixel = int(round(x_pixel))
        y_pixel = int(round(y_pixel))

        return [x_pixel, y_pixel]
