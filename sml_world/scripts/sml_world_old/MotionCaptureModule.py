import time, math, threading, socket
import QualisysComms
import bodyclasses

class MotionCaptureModule:
    '''
    Class that interfaces with the Qualisys
    Motion Capture System. It does so by using the
    QualisysComms class.
    Once new readings are obtained, the SML World
    bodies dictionary is updates accordingly.
    '''

    def __init__(self, sml_world, qualisys_host = 'sml-qualisys.ddns.net', qualisys_port = 22224, desired_rate = 24.):

        qualisys_host = '192.168.1.146'
        # Get a reference to the sml_world,
        # so that we are able to access the
        # bodies_dict
        self.sml_world = sml_world

        # Creates the QualisysComms class which 
        # will manage the communications with the 
        # Qualisys Motion Capture System
        self.qualisys_comms = QualisysComms.QualisysComms(qualisys_host, qualisys_port)

        # The loop rate of this class' thread loop
        self.loop_rate = desired_rate

        # If something is wrong with the Qualisys Capture
        # we will issue warnings at this rate
        self.verbose_warning_rate = 0.2

        # Launches the tread that will run the main
        # loop of the MotionCaptureModule
        t = threading.Thread( target = self.thread_loop, args=([]) )
        t.daemon = True
        t.start()

    def thread_loop(self):
        '''
        This function is executed in a thread, thus running 
        in parallel with the rest of the SML World.
        It consists of a loop, where we check for the most 
        recent readings from the Capture System, and afterwards 
        we change the respective vehicle states in the SML World
        '''

        # Time keepers
        start_loop_time = time.time()
        end_loop_time = time.time()
        last_warning_time = time.time()

        while not self.sml_world.close:

            start_loop_time = time.time()

            mocap_bodies_dict = self.qualisys_comms.get_updated_bodies()

            if mocap_bodies_dict == None:

                if start_loop_time - last_warning_time > 1./self.verbose_warning_rate:

                    print "WARNING: Qualisys is OFF"

                    last_warning_time = start_loop_time

            else:

                self.update_sml_world_mocap_bodies(mocap_bodies_dict)


            end_loop_time = time.time()

            time_elapsed = end_loop_time - start_loop_time

            time_to_sleep = 1./self.loop_rate - time_elapsed

            if time_to_sleep > 0:

                time.sleep(time_to_sleep)

            else:

                print "MotionCaptureModule failed desired rate."

        self.close_sockets()

        return


    def update_sml_world_mocap_bodies(self, mocap_bodies_dict):
        '''
        This function, will take the new Motion Capture readings
        mocap_bodies_dict and will update the respective Vehicle 
        States in the SML World.
        '''

        for mocap_body_id in mocap_bodies_dict:

            mocap_body = mocap_bodies_dict[mocap_body_id]

            if mocap_body_id in self.sml_world.bodies_dict:
                # If the body is already in the SML World
                # vehicles simply update the respective vehicle

                current_qualisys_body = self.sml_world.bodies_dict[mocap_body_id]

                current_qualisys_body.x = mocap_body['x']*32.
                current_qualisys_body.y = mocap_body['y']*32.
                current_qualisys_body.z = mocap_body['z']*32.
                current_qualisys_body.yaw = mocap_body['yaw']
                current_qualisys_body.pitch = mocap_body['pitch']
                current_qualisys_body.roll = mocap_body['roll']
                current_qualisys_body.timestamp = mocap_body['ts']/1e6
                # Dividing by 1e6, converts to seconds

            else:
                # If the body does not exist in the SML World
                # create it

                new_qualisys_body = bodyclasses.QualisysBody()

                new_qualisys_body.id = mocap_body['id']
                new_qualisys_body.x = mocap_body['x']*32.
                new_qualisys_body.y = mocap_body['y']*32.
                new_qualisys_body.z = mocap_body['z']*32.
                new_qualisys_body.yaw = mocap_body['yaw']
                new_qualisys_body.pitch = mocap_body['pitch']
                new_qualisys_body.roll = mocap_body['roll']
                new_qualisys_body.timestamp = mocap_body['ts']/1e6
                # Dividing by 1e6, converts to seconds

                new_qualisys_body.x_speed = 0.
                new_qualisys_body.y_speed = 0.

                self.sml_world.bodies_dict[mocap_body_id] = new_qualisys_body

        return
