import os
import sys

from PIL import Image

from PyQt4 import QtCore, QtGui

class GuiClass(object):

	def __init__(self, sml_world_instance):

		self.sml_world = sml_world_instance

		self.list_of_modules = ['main','mocap','highlight','projector_manager','command_central','v2i','v2v','simulated_vehicles','vehicle_simulation','simulator_receiver','simulator','vehicle_manager','vehicle_controller','vehicle_command_manager','vehicle_output_manager']

		for module in self.list_of_modules:
			setattr(self.sml_world, module + '_connected', False)
			setattr(self.sml_world, module + '_desired_rate', 1)
			setattr(self.sml_world, module + '_info', dict())

		self.previous_module_states = []

		print "GuiClass started"

	def update_gui(self):

		self.check_gui_change()

		self.get_settings_from_gui()

		self.update_vehicle_list()


		if self.sml_world.vehicle_simulator_ready:
			self.sml_world.gui.stop.setVisible(True)
			self.sml_world.gui.waiting_label.setVisible(False)
			self.sml_world.vehicle_simulator_ready = False
			self.sml_world.gui.simulatedprogressbar.setVisible(False)
		else:
			# self.sml_world.gui.simulatedprogressbar.setStyleSheet("QProgressBar {background-color: red};");
			self.sml_world.gui.simulatedprogressbar.setRange(0,self.sml_world.number_vehicles)
			self.sml_world.gui.simulatedprogressbar.setValue(abs(self.sml_world.current_simulated_id))
			self.sml_world.gui.simulatedprogressbar.setFormat(str(abs(self.sml_world.current_simulated_id))+'/'+str(self.sml_world.number_vehicles) + ' vehicles')

		if bool(self.sml_world.main_info):
			self.sml_world.gui.main_text.setText(self.sml_world.main_info['rate'] + ' Hz')

		if bool(self.sml_world.simulated_vehicles_info):
			self.sml_world.gui.simulated_vehicles_text.setText(str(self.sml_world.number_vehicles) + ' vehicles' + '\n')
			self.sml_world.gui.simulated_vehicles_text.append(self.sml_world.simulated_vehicles_info['rate'] + ' Hz')

		if bool(self.sml_world.mocap_info):
			self.sml_world.gui.mocap_text.setText(self.sml_world.mocap_info['ip'])
			self.sml_world.gui.mocap_text.append(self.sml_world.mocap_info['port'])
			self.sml_world.gui.mocap_text.append(self.sml_world.mocap_info['rate'] + ' Hz')

		if bool(self.sml_world.v2v_info):
			if self.sml_world.v2v_method != '':

				self.sml_world.gui.v2v_text.setText(self.sml_world.v2v_method)

				if self.sml_world.v2v_method != 'off':
					self.sml_world.gui.v2v_text.append('Range: ' + str(self.sml_world.v2v_range))
					if self.sml_world.v2v_method == 'sensors':
						self.sml_world.gui.v2v_text.append('Angle: ' + str(self.sml_world.v2v_angle))

				self.sml_world.gui.v2v_text.append(self.sml_world.v2v_info['rate'] + ' Hz')
			else:
				self.sml_world.gui.v2v_text.setText(self.sml_world.v2v_info['rate'] + ' Hz')

		if bool(self.sml_world.v2i_info):
			self.sml_world.gui.v2i_text.setText(self.sml_world.v2i_info['rate'] + ' Hz')

		if bool(self.sml_world.vehicle_simulation_info):
			self.sml_world.gui.vehicle_simulation_text.setText(self.sml_world.vehicle_simulation_info['rate'] + ' Hz')

		if bool(self.sml_world.projector_manager_info):
			self.sml_world.gui.projector_manager_text.setText(self.sml_world.projector_manager_info['ip'])
			self.sml_world.gui.projector_manager_text.append(self.sml_world.projector_manager_info['port'])
			self.sml_world.gui.projector_manager_text.append(self.sml_world.projector_manager_info['rate'] + ' Hz')

		if bool(self.sml_world.highlight_info):
			self.sml_world.gui.highlight_text.setText(self.sml_world.highlight_info['ip'])
			self.sml_world.gui.highlight_text.append(self.sml_world.highlight_info['port'])
			self.sml_world.gui.highlight_text.append(self.sml_world.highlight_info['rate'] + ' Hz')

		if bool(self.sml_world.command_central_info):
			self.sml_world.gui.command_central_text.setText(self.sml_world.command_central_info['ip'])
			self.sml_world.gui.command_central_text.append(self.sml_world.command_central_info['port'])
			self.sml_world.gui.command_central_text.append(self.sml_world.command_central_info['rate'] + ' Hz')

		if bool(self.sml_world.vehicle_manager_info):
			self.sml_world.gui.vehicle_manager_text.setText(self.sml_world.vehicle_manager_info['ip'])
			self.sml_world.gui.vehicle_manager_text.append(self.sml_world.vehicle_manager_info['port'])
			self.sml_world.gui.vehicle_manager_text.append('Controllable Vehicles')

			self.sml_world.gui.vehicle_manager_text.append(str(self.sml_world.controllables_list) )
			self.sml_world.gui.vehicle_manager_text.append(self.sml_world.vehicle_manager_info['rate'] + ' Hz')

		if bool(self.sml_world.vehicle_command_manager_info):

			if self.sml_world.vehicle_command_manager_info.has_key('rate'):
				self.sml_world.gui.vehicle_command_manager_text.setText(self.sml_world.vehicle_command_manager_info['rate'] + ' Hz')

		if bool(self.sml_world.vehicle_output_manager_info):
			self.sml_world.gui.vehicle_output_manager_text.setText(self.sml_world.vehicle_output_manager_info['ip'])
			self.sml_world.gui.vehicle_output_manager_text.append(self.sml_world.vehicle_output_manager_info['port'])

		if bool(self.sml_world.simulator_info):
			self.sml_world.gui.simulator_text.setText(self.sml_world.simulator_info['ip'])
			self.sml_world.gui.simulator_text.append(self.sml_world.simulator_info['port'])
			self.sml_world.gui.simulator_text.append(self.sml_world.simulator_info['rate'] + ' Hz')

		if bool(self.sml_world.simulator_receiver_info):
			self.sml_world.gui.simulator_receiver_text.setText(self.sml_world.simulator_receiver_info['ip'])
			self.sml_world.gui.simulator_receiver_text.append(self.sml_world.simulator_receiver_info['port'])
			self.sml_world.gui.simulator_receiver_text.append(self.sml_world.simulator_receiver_info['rate'] + ' Hz')


		for module in self.list_of_modules:
			# By comparing with the previous module_states I can know when there was a change, and so, I will
			# only change its color (style) when this change happens
			# (before every cycle changed the style, even if it was to the same style as before)
			if getattr(self.sml_world, module + '_connected') != self.previous_module_states[module + '_connected']:

				if getattr(self.sml_world, module + '_connected'):
					getattr(self.sml_world.gui, module + '_frame').setStyleSheet("background-color: rgb(54,99,9); color: rgb(255, 255, 255);")
					getattr(self.sml_world.gui, module + '_desired_rate').setStyleSheet("background-color: rgb(54,99,9); color: rgb(255, 255, 255); border: rgb(212,255,255)")
				else:
					getattr(self.sml_world.gui, module + '_frame').setStyleSheet("background-color: rgb(135,26,26); color: rgb(255, 255, 255);")
					getattr(self.sml_world.gui, module + '_desired_rate').setStyleSheet("background-color: rgb(135,26,26); color: rgb(255, 255, 255); border: rgb(212,255,255)")
					getattr(self.sml_world.gui, module + '_text').setText(' ')


	def save_previous_module_states(self):

		# print "save_previous_module_states"

		self.previous_module_states = dict()

		for module in self.list_of_modules:

			self.previous_module_states[module + '_connected'] = getattr(self.sml_world, module + '_connected')

		# print "previous_module_states = " + str(previous_module_states) 

	def save_settings_in_file(self):

		attribute_list_1 = ['qualisys_info', 'projector_info', 'xml_file_location', 'activate_draw_origin', 'draw_sensors', 'draw_signal', 'show_ids', 'activate_fps', 'activate_states_from_mocap', 'activate_states_from_world', 'only_simulated_vehicles', 'only_real_vehicles']
		attribute_list_2 = [ele + '_desired_rate' for ele in self.list_of_modules]
		attribute_list_3 = ['projector_area', 'load_saved_background', 'projector_wait_for_command_central', 'log_xml', 'number_vehicles', 'server_port', 'buffersize', 'v2v_method', 'v2v_range', 'v2v_angle']

		attribute_list = attribute_list_1 + attribute_list_2 + attribute_list_3

		with open('file_sources/default_settings.txt', 'w') as f:

			for attribute in attribute_list:

				f.write(attribute + ':' + str(getattr(self.sml_world, attribute)) + '\n')

		self.sml_world.timed_print('Save default setting to file')


	def read_default_settings_from_file(self):

		with open('file_sources/default_settings.txt', 'r') as f:

				settings = f.read()

				if not settings:
					return

				settings = settings.split('\n')

				for setting in settings[:-1]:

					setting_attribute, setting_value = setting.split(':')

					if setting_attribute in [ele + '_desired_rate' for ele in self.list_of_modules]:
						setattr(self.sml_world, setting_attribute, int(setting_value))
						getattr(self.sml_world.gui, setting_attribute).setValue(int(setting_value))

					elif setting_attribute in ['number_vehicles', 'server_port', 'buffersize']:
						setattr(self.sml_world, setting_attribute, int(setting_value.strip()))
						getattr(self.sml_world.gui, setting_attribute).setText(setting_value.strip())

					elif setting_attribute in ['activate_draw_origin', 'draw_sensors', 'draw_signal', 'show_ids', 'activate_fps', 'activate_states_from_mocap', 'activate_states_from_world', 'only_simulated_vehicles', 'only_real_vehicles', 'load_saved_background', 'projector_wait_for_command_central', 'log_xml']:
						state = setting_value == 'True'
						setattr(self.sml_world, setting_attribute, state)
						if state:
							getattr(self.sml_world.gui, setting_attribute).setChecked(state)

					elif setting_attribute in ['v2v_range', 'v2v_angle']:
						setattr(self.sml_world, setting_attribute, float(setting_value))
						getattr(self.sml_world.gui, setting_attribute).setText(setting_value)

					elif setting_attribute in ['v2v_method']:
						setattr(self.sml_world, setting_attribute, setting_value)

						if self.sml_world.v2v_method == 'communications':
							self.sml_world.gui.v2v_comm.setChecked(True)
						elif self.sml_world.v2v_method == 'sensors':
							self.sml_world.gui.v2v_sensors.setChecked(True)
						elif self.sml_world.v2v_method == 'off':
							self.sml_world.gui.v2v_off.setChecked(True)

					elif setting_attribute in ['xml_file_location']:
						setattr(self.sml_world, setting_attribute, setting_value)
						setting_value_1 = setting_value.split('/')
						setting_value_2 = setting_value.split('\\')
						if len(setting_value_1) > 1:
							setting_value = setting_value_1[-1]
						elif setting_value_2:
							setting_value = setting_value_2[-1]
						print setting_value
						getattr(self.sml_world.gui, setting_attribute).setText(setting_value)

					elif setting_attribute in ['projector_area']:
						area = [float(ele) for ele in setting_value[1:-1].split(',')]
						setattr(self.sml_world, setting_attribute, area)
						self.sml_world.gui.xmin.setText(str(area[0]))
						self.sml_world.gui.xmax.setText(str(area[1]))
						self.sml_world.gui.ymin.setText(str(area[2]))
						self.sml_world.gui.ymax.setText(str(area[2]))

					elif setting_attribute in ['qualisys_info', 'projector_info']:
						string_part, number_part = setting_value[1:-1].split(',')
						setattr(self.sml_world, setting_attribute, (string_part, int(number_part.strip())))

						name = setting_attribute.split('_')[0]

						getattr(self.sml_world.gui, name + '_host').setText(string_part[1:-1])
						getattr(self.sml_world.gui, name + '_port').setText(number_part.strip())

		self.sml_world.timed_print('Default settings loaded')



	def get_settings_from_gui(self):

		# Qualisys
		qualisys_host = str(self.sml_world.gui.qualisys_host.text())
		qualisys_port = int(self.sml_world.gui.qualisys_port.text() if str(self.sml_world.gui.qualisys_port.text()).isdigit() else 0)
		self.sml_world.qualisys_info = (qualisys_host, qualisys_port)

		# Projector
		projector_host = str(self.sml_world.gui.projector_host.text())
		projector_port = int(self.sml_world.gui.projector_port.text() if str(self.sml_world.gui.projector_port.text()).isdigit() else 0)

		projector_width = int(self.sml_world.gui.projector_width.text() if str(self.sml_world.gui.projector_width.text()).isdigit() else 0)
		projector_height = int(self.sml_world.gui.projector_height.text() if str(self.sml_world.gui.projector_height.text()).isdigit() else 0)

		self.sml_world.projector_resolution = (projector_width,projector_height)

		self.sml_world.projector_info = (projector_host, projector_port)

		self.sml_world.xml_file_location = os.path.join('RoadModule', str(self.sml_world.gui.xml_file_location.text()))

		self.sml_world.activate_draw_origin = bool(self.sml_world.gui.activate_draw_origin.checkState())
		self.sml_world.draw_sensors = bool(self.sml_world.gui.draw_sensors.checkState())
		self.sml_world.draw_signal = bool(self.sml_world.gui.draw_signal.checkState())
		self.sml_world.show_ids = bool(self.sml_world.gui.show_ids.checkState())
		self.sml_world.activate_fps = bool(self.sml_world.gui.activate_fps.checkState())
		self.sml_world.activate_states_from_mocap = bool(self.sml_world.gui.activate_states_from_mocap.checkState())
		self.sml_world.activate_states_from_world = bool(self.sml_world.gui.activate_states_from_world.checkState())
		self.sml_world.only_simulated_vehicles = bool(self.sml_world.gui.only_simulated_vehicles.checkState())
		self.sml_world.only_real_vehicles = bool(self.sml_world.gui.only_real_vehicles.checkState())


		for module in self.list_of_modules:
			try:
				setattr(self.sml_world, module + '_desired_rate', getattr(self.sml_world.gui, module + '_desired_rate').value())
			except:
				raise
				pass


		xmin = float(self.sml_world.gui.xmin.text() if str(self.sml_world.gui.xmin.text()).replace('.','',1).isdigit() else 0)
		xmax = float(self.sml_world.gui.xmax.text() if str(self.sml_world.gui.xmax.text()).replace('.','',1).isdigit() else 0)
		ymin = float(self.sml_world.gui.ymin.text() if str(self.sml_world.gui.ymin.text()).replace('.','',1).isdigit() else 0)
		ymax = float(self.sml_world.gui.ymax.text() if str(self.sml_world.gui.ymax.text()).replace('.','',1).isdigit() else 0)

		self.sml_world.projector_area = [xmin, xmax, ymin, ymax]

		# Settings

		self.sml_world.load_saved_background = self.sml_world.gui.load_saved_background.checkState() == 2
		self.sml_world.projector_wait_for_command_central = self.sml_world.gui.projector_wait_for_command_central.checkState() == 2
		self.sml_world.log_xml = self.sml_world.gui.log_xml.checkState() == 2

		self.sml_world.number_vehicles = int(self.sml_world.gui.number_vehicles.text() if str(self.sml_world.gui.number_vehicles.text()).isdigit() else 0)
		self.sml_world.server_port = int(self.sml_world.gui.server_port.text() if str(self.sml_world.gui.server_port.text()).isdigit() else 0)
		self.sml_world.buffersize = int(self.sml_world.gui.buffersize.text() if str(self.sml_world.gui.buffersize.text()).isdigit() else 0)

		if self.sml_world.gui.v2v_comm.isChecked():
			self.sml_world.v2v_method = 'communications'
			self.sml_world.v2v_range = float(self.sml_world.gui.v2v_range.text() if str(self.sml_world.gui.v2v_range.text()).replace('.','',1).isdigit() else 0)
			self.sml_world.gui.v2v_range.setDisabled(False)
			self.sml_world.gui.range_label.setDisabled(False)
			self.sml_world.gui.v2v_angle.setDisabled(True)
			self.sml_world.gui.angle_label.setDisabled(True)
		elif self.sml_world.gui.v2v_sensors.isChecked():
			self.sml_world.v2v_method = 'sensors'
			self.sml_world.v2v_range = float(self.sml_world.gui.v2v_range.text() if str(self.sml_world.gui.v2v_range.text()).replace('.','',1).isdigit() else 0)
			self.sml_world.v2v_angle = float(self.sml_world.gui.v2v_angle.text() if str(self.sml_world.gui.v2v_angle.text()).replace('.','',1).isdigit() else 0)
			self.sml_world.gui.v2v_range.setDisabled(False)
			self.sml_world.gui.range_label.setDisabled(False)
			self.sml_world.gui.v2v_angle.setDisabled(False)
			self.sml_world.gui.angle_label.setDisabled(False)
		elif self.sml_world.gui.v2v_off.isChecked():
			self.sml_world.v2v_method = 'off'
			self.sml_world.gui.v2v_angle.setDisabled(True)
			self.sml_world.gui.angle_label.setDisabled(True)
			self.sml_world.gui.v2v_range.setDisabled(True)
			self.sml_world.gui.range_label.setDisabled(True)


	def initialize_gui_module_colors(self):

		for module in self.list_of_modules:

			if getattr(self.sml_world, module + '_connected'):
				getattr(self.sml_world.gui, module + '_frame').setStyleSheet("background-color: rgb(54,99,9); color: rgb(255, 255, 255);")
				getattr(self.sml_world.gui, module + '_desired_rate').setStyleSheet("background-color: rgb(54,99,9); color: rgb(255, 255, 255); border: rgb(212,255,255)")
			else:
				getattr(self.sml_world.gui, module + '_frame').setStyleSheet("background-color: rgb(135,26,26); color: rgb(255, 255, 255);")
				getattr(self.sml_world.gui, module + '_desired_rate').setStyleSheet("background-color: rgb(135,26,26); color: rgb(255, 255, 255); border: rgb(212,255,255)")
				getattr(self.sml_world.gui, module + '_text').setText(' ')


	def check_gui_change(self):

		test_list = [self.sml_world.activate_draw_origin == bool(self.sml_world.gui.activate_draw_origin.checkState()),
					self.sml_world.draw_sensors == bool(self.sml_world.gui.draw_sensors.checkState()),
					self.sml_world.draw_signal == bool(self.sml_world.gui.draw_signal.checkState()),
					self.sml_world.show_ids == bool(self.sml_world.gui.show_ids.checkState()),
					self.sml_world.activate_fps == bool(self.sml_world.gui.activate_fps.checkState()),
					self.sml_world.activate_states_from_mocap == bool(self.sml_world.gui.activate_states_from_mocap.checkState()),
					self.sml_world.activate_states_from_world == bool(self.sml_world.gui.activate_states_from_world.checkState()),
					self.sml_world.only_simulated_vehicles == bool(self.sml_world.gui.only_simulated_vehicles.checkState()),
					self.sml_world.only_real_vehicles == bool(self.sml_world.gui.only_real_vehicles.checkState())]

		new_v2v = dict()

		if self.sml_world.v2v_method != '':
			if self.sml_world.gui.v2v_comm.isChecked():
				new_v2v['method'] = 'communications'
				new_v2v['range'] = float(self.sml_world.gui.v2v_range.text())
				test_list.append(new_v2v['method'] == self.sml_world.v2v_method)
				test_list.append(new_v2v['range'] == self.sml_world.v2v_range)
			elif self.sml_world.gui.v2v_sensors.isChecked():
				new_v2v['method'] = 'sensors'
				new_v2v['range'] = float(self.sml_world.gui.v2v_range.text())
				new_v2v['angle'] = float(self.sml_world.gui.v2v_angle.text())
				test_list.append(new_v2v['method'] == self.sml_world.v2v_method)
				test_list.append(new_v2v['range'] == self.sml_world.v2v_range)
				test_list.append(new_v2v['angle'] == self.sml_world.v2v_angle)
			elif self.sml_world.gui.v2v_off.isChecked():
				new_v2v['method'] = 'off'
				test_list.append(new_v2v['method'] == self.sml_world.v2v_method)

		xmin = float(self.sml_world.gui.xmin.text() if str(self.sml_world.gui.xmin.text()).replace('.','',1).isdigit() else 0.0)
		xmax = float(self.sml_world.gui.xmax.text() if str(self.sml_world.gui.xmax.text()).replace('.','',1).isdigit() else 0.0)
		ymin = float(self.sml_world.gui.ymin.text() if str(self.sml_world.gui.ymin.text()).replace('.','',1).isdigit() else 0.0)
		ymax = float(self.sml_world.gui.ymax.text() if str(self.sml_world.gui.ymax.text()).replace('.','',1).isdigit() else 0.0)

		test_list.append(self.sml_world.projector_area == [xmin, xmax, ymin, ymax])

		projector_width = int(self.sml_world.gui.projector_width.text() if str(self.sml_world.gui.projector_width.text()).isdigit() else 0)
		projector_height = int(self.sml_world.gui.projector_height.text() if str(self.sml_world.gui.projector_height.text()).isdigit() else 0)

		test_list.append(self.sml_world.projector_resolution == (projector_width,projector_height))

		if not self.sml_world.gui_changed:
			self.sml_world.gui_changed = not all(test_list)

	
	def update_vehicle_list(self):

		current_list = [int(self.sml_world.gui.body_id.itemText(i)) for i in xrange(self.sml_world.gui.body_id.count())]
		current_trailer_list = [int(self.sml_world.gui.trailer_id.itemText(i)) for i in xrange(self.sml_world.gui.trailer_id.count())]

		body_list = [body['id'] for body in self.sml_world.bodies_list if body['id'] > 0]

		trailer_list = [body['id'] for body in self.sml_world.bodies_list if body['id'] > 0 and 'body_type' in body and body['body_type'] == 'trailer']
		# trailer_list = []

		for body_id in list(set(body_list) - set(current_list)):
			self.sml_world.gui.body_id.addItem(str(body_id))

		for trailer_id in list(set(trailer_list) - set(current_trailer_list)):
			self.sml_world.gui.trailer_id.addItem(str(trailer_id))

		for body_id in list(set(current_list) - set(body_list)):
			itemindex = self.sml_world.gui.body_id.findText(str(body_id))
			self.sml_world.gui.body_id.removeItem(itemindex)

		for trailer_id in list(set(current_trailer_list) - set(trailer_list)):
			itemindex = self.sml_world.gui.trailer_id.findText(str(trailer_id))
			self.sml_world.gui.trailer_id.removeItem(itemindex)



	@staticmethod
	def read_obstacles_from_file():

		obstacle_file = "file_sources/obstacles_list.txt"

		obstacles = []

		if os.path.exists(obstacle_file):

			f = open(obstacle_file, 'r')

			data = f.read()

			f.close()

			if len(data) == 0:
				return obstacles

			if data[-1] == '\n':
				data = data[:-1]

			data = data.split('\n')

			for obstacle in data:

				obstacle_elements = obstacle.split(' ')

				if obstacle_elements[1] in ['rectangle', 'rrt']:
					obstacles.append([int(obstacle_elements[0]), obstacle_elements[1], float(obstacle_elements[2]), float(obstacle_elements[3])])
				elif obstacle_elements[1] in ['circle', 'gun']:
					obstacles.append([int(obstacle_elements[0]), obstacle_elements[1], float(obstacle_elements[2])])
				else:
					obstacles.append([int(obstacle_elements[0]), obstacle_elements[1]])
		return obstacles


	def save_vehicle_info_in_file(self):

		f = open('file_sources/vehicle_info.txt', 'w')

		for info in self.vehicle_info:
			info_str = str(info[0]) + ' trailer:' + str(info[0] in self.sml_world.trailer_list) + ' type:' + str(info[1]) + ' trailer_id:' + str(info[2])
			f.write(info_str)
			f.write('\n')

		vehicle_ids = [ele[0] for ele in self.sml_world.vehicle_info]

		for vehicle in [ele for ele in self.sml_world.trailer_list if ele not in vehicle_ids]:
			info_str = str(vehicle) + ' trailer:' + str(vehicle in self.sml_world.trailer_list) + ' type:vehicle' + ' trailer_id:' + str(0)
			f.write(info_str)
			f.write('\n')

		f.close()


	@staticmethod
	def read_vehicle_info_from_file():

		vehicle_file = "file_sources/vehicle_info.txt"

		trailer_list = []

		vehicle_info = []

		if os.path.exists(vehicle_file):

			f = open(vehicle_file, 'r')

			data = f.read()

			if data == '':
				return [[], []]

			if data[-1] == '\n':
				data = data[:-1]

			data = data.split('\n')

			for vehicle in data:

				vehicle_elements = vehicle.split(' ')

				body_id = vehicle_elements[0]

				trailer_id = 0
				body_type = 'vehicle'

				for element in vehicle_elements:

					tags = element.split(':')

					if tags[0] == 'trailer_id':
						trailer_id = tags[1]
					if tags[0] == 'trailer' and tags[1] == 'True':
						trailer_list.append(int(body_id))
					if tags[0] == 'type':
						body_type = tags[1]

				vehicle_info.append([int(body_id), body_type, trailer_id])

		return [trailer_list, vehicle_info]





	def add_obstacle(self):

		new_obstacle_id = self.sml_world.gui.selected_obstacle_id.text()

		if new_obstacle_id == '':
			return

		new_obstacle_id = int(new_obstacle_id)

		if not self.sml_world.obstacles_list:
			self.sml_world.gui.obstacle_id.setDisabled(False)
			self.sml_world.gui.obstacle_type.setDisabled(False)

		obstacles_id_list = [obstacle[0] for obstacle in self.sml_world.obstacles_list]

		if new_obstacle_id not in obstacles_id_list:

			self.sml_world.obstacles_list.append([new_obstacle_id, 'circle', 0.05])

			self.sml_world.gui.obstacle_id.addItem(str(new_obstacle_id))

			new_item_index = self.sml_world.gui.obstacle_id.findText(str(new_obstacle_id))

			self.sml_world.gui.obstacle_id.setCurrentIndex(new_item_index)

			self.sml_world.gui.selected_obstacle_id.setText('')

			self.change_obstacle_id()

			self.change_obstacle_type()

			f = open('file_sources/obstacles_list.txt', 'w')

			for obstacle in self.sml_world.obstacles_list:
				for element in obstacle:
					f.write(str(element) + ' ')
				f.write('\n')

	def remove_obstacle(self):

		to_delete_obstacle_id = self.sml_world.gui.selected_obstacle_id.text()

		if to_delete_obstacle_id == '':
			return

		to_delete_obstacle_id = int(to_delete_obstacle_id)

		obstacles_id_list = [obstacle[0] for obstacle in self.sml_world.obstacles_list]

		try:
			self.sml_world.gui.selected_obstacle_id.setText('')
			id_index = obstacles_id_list.index(to_delete_obstacle_id)

			itemindex = self.sml_world.gui.obstacle_id.findText(str(self.sml_world.obstacles_list[id_index][0]))

			self.sml_world.gui.obstacle_id.removeItem(itemindex)

			self.sml_world.obstacles_list.pop(id_index)

			f = open('file_sources/obstacles_list.txt', 'w')

			for obstacle in self.sml_world.obstacles_list:
				for element in obstacle:
					f.write(str(element) + ' ')
				f.write('\n')

		except:
			pass

	def change_obstacle_radius(self):

		new_radius = self.sml_world.gui.obstacle_radius.value()

		current_obstacle_id = int(self.sml_world.gui.obstacle_id.currentText())

		obstacles_id_list = [obstacle[0] for obstacle in self.sml_world.obstacles_list]

		id_index = obstacles_id_list.index(current_obstacle_id)

		self.sml_world.obstacles_list[id_index][2] = new_radius

		f = open('file_sources/obstacles_list.txt', 'w')

		for obstacle in self.sml_world.obstacles_list:
			for element in obstacle:
				f.write(str(element) + ' ')
			f.write('\n')

	def change_obstacle_width(self):

		new_width = self.sml_world.gui.obstacle_width.value()

		current_obstacle_id = int(self.sml_world.gui.obstacle_id.currentText())

		obstacles_id_list = [obstacle[0] for obstacle in self.sml_world.obstacles_list]

		id_index = obstacles_id_list.index(current_obstacle_id)

		self.sml_world.obstacles_list[id_index][2] = new_width

		f = open('file_sources/obstacles_list.txt', 'w')

		for obstacle in self.sml_world.obstacles_list:
			for element in obstacle:
				f.write(str(element) + ' ')
			f.write('\n')

	def change_obstacle_height(self):

		new_height = self.sml_world.gui.obstacle_height.value()

		current_obstacle_id = int(self.sml_world.gui.obstacle_id.currentText())

		obstacles_id_list = [obstacle[0] for obstacle in self.sml_world.obstacles_list]

		id_index = obstacles_id_list.index(current_obstacle_id)

		self.sml_world.obstacles_list[id_index][3] = new_height

		f = open('file_sources/obstacles_list.txt', 'w')

		for obstacle in self.sml_world.obstacles_list:
			for element in obstacle:
				f.write(str(element) + ' ')
			f.write('\n')

	def new_xml_file_location(self):

		file_path = str(QtGui.QFileDialog.getOpenFileName(directory="RoadModule"))

		file_name = os.path.split(file_path)[1]

		self.sml_world.gui.xml_file_location.setText(file_name)



	def vehicle_id_change(self):

		current_body_id = self.sml_world.gui.body_id.currentText()

		if current_body_id == '':
			self.sml_world.gui.body_type.setDisabled(True)
			self.sml_world.gui.body_type_label.setDisabled(True)
			self.sml_world.gui.trailer_status.setDisabled(True)
			return

		self.sml_world.gui.body_type.setDisabled(False)
		self.sml_world.gui.body_type_label.setDisabled(False)
		self.sml_world.gui.trailer_status.setDisabled(False)

		current_body_id = int(current_body_id)

		self.sml_world.gui.trailer_status.setChecked(current_body_id in self.sml_world.trailer_list)

		self.sml_world.gui.trailer_id.setDisabled(not self.sml_world.gui.trailer_status.isChecked())
		self.sml_world.gui.trailer_id_label.setDisabled(not self.sml_world.gui.trailer_status.isChecked())

		vehicle_info_ids = [ele[0] for ele in self.sml_world.vehicle_info]

		current_body_type = 'vehicle'

		if current_body_id in vehicle_info_ids:
			for index, vehicle in enumerate(self.sml_world.vehicle_info):
				if vehicle[0] == current_body_id:
					current_body_type = vehicle[1]
					break

		itemindex = self.sml_world.gui.body_type.findText(current_body_type)
		self.sml_world.gui.body_type.setCurrentIndex(itemindex)

	def vehicle_type_change(self):

		current_body_id = self.sml_world.gui.body_id.currentText()
		if current_body_id == '':
			return
		current_body_id = int(current_body_id)

		current_body_type = self.sml_world.gui.body_type.currentText()

		if self.sml_world.gui.trailer_status.isChecked():
			current_trailer_id = self.sml_world.gui.trailer_id.currentText()
		else:
			current_trailer_id = 0

		vehicle_info_ids = [ele[0] for ele in self.sml_world.vehicle_info]

		if current_body_id in vehicle_info_ids:
			for index, vehicle in enumerate(self.sml_world.vehicle_info):
				if vehicle[0] == current_body_id:
					self.sml_world.vehicle_info.pop(index)

		self.sml_world.vehicle_info.append([current_body_id, current_body_type, current_trailer_id])

		self.save_vehicle_info_in_file()


	
	def trailer_id_change(self):

		current_body_id = self.sml_world.gui.body_id.currentText()

		if current_body_id == '':
			return

		for vehicle in self.sml_world.vehicle_info:
			if vehicle[0] == int(current_body_id) and self.sml_world.gui.trailer_status.isChecked():
				vehicle[2] = int(self.sml_world.gui.trailer_id.currentText())
				break

		self.save_vehicle_info_in_file()

	def trailer_status_change(self):

		current_body_id = self.sml_world.gui.body_id.currentText()

		if current_body_id == '':
			return

		self.sml_world.gui.trailer_id.setDisabled(not self.sml_world.gui.trailer_status.isChecked())
		self.sml_world.gui.trailer_id_label.setDisabled(not self.sml_world.gui.trailer_status.isChecked())

		current_body_id = int(current_body_id)

		if current_body_id not in self.sml_world.trailer_list and self.sml_world.gui.trailer_status.isChecked():

			self.sml_world.trailer_list.append(current_body_id)

			if int(current_body_id) not in [ele[0] for ele in self.sml_world.vehicle_info]:
				self.sml_world.vehicle_info.append([current_body_id, self.sml_world.gui.body_type.currentText(), int(self.sml_world.gui.trailer_id.currentText())])
			else:
				for vehicle in self.sml_world.vehicle_info:
					if vehicle[0] == int(current_body_id):
						vehicle[2] = self.sml_world.gui.trailer_id.currentText()
						break

		elif current_body_id in self.sml_world.trailer_list and not self.sml_world.gui.trailer_status.isChecked():

			self.sml_world.trailer_list.pop(self.sml_world.trailer_list.index(current_body_id))

			for vehicle in self.sml_world.vehicle_info:
				if vehicle[0] == int(current_body_id):
					vehicle[2] = 0
					break

		self.save_vehicle_info_in_file()

	@staticmethod
	def open_background_image():

		bg_image = os.path.join('resources', 'world_surface.bmp')

		im = Image.open(bg_image)
		im.show()


	def change_obstacle_type(self):

		obstacles_id_list = [obstacle[0] for obstacle in self.sml_world.obstacles_list]

		current_obstacle_id = int(self.sml_world.gui.obstacle_id.currentText())
		current_obstacle_type = str(self.sml_world.gui.obstacle_type.currentText())

		try:
			id_index = obstacles_id_list.index(current_obstacle_id)
			changed_obstacle = [current_obstacle_id, current_obstacle_type]
			if current_obstacle_type in ['circle', 'gun']:
				changed_obstacle.append(0.05)
			elif current_obstacle_type in ['rectangle', 'rrt']:
				if len(self.sml_world.obstacles_list[id_index]) == 4:
					changed_obstacle.append(self.sml_world.obstacles_list[id_index][2])
					changed_obstacle.append(self.sml_world.obstacles_list[id_index][3])
				else:
					changed_obstacle.append(1.0)
					changed_obstacle.append(1.0)

			self.sml_world.obstacles_list[id_index] = changed_obstacle

			self.change_obstacle_id()
		except:
			raise

		f = open('file_sources/obstacles_list.txt', 'w')

		for obstacle in self.sml_world.obstacles_list:
			for element in obstacle:
				f.write(str(element) + ' ')
			f.write('\n')

	def change_obstacle_id(self):

		current_obstacle_id = self.sml_world.gui.obstacle_id.currentText()

		if current_obstacle_id == '':
			self.sml_world.gui.obstacle_type.setDisabled(True)
			self.sml_world.gui.obstacle_id.setDisabled(True)
			self.sml_world.gui.obstacle_radius.setVisible(False)
			self.sml_world.gui.obstacle_radius_label.setVisible(False)
			self.sml_world.gui.obstacle_height.setVisible(False)
			self.sml_world.gui.obstacle_height_label.setVisible(False)
			self.sml_world.gui.obstacle_width.setVisible(False)
			self.sml_world.gui.obstacle_width_label.setVisible(False)
			return

		current_obstacle_id = int(current_obstacle_id)

		obstacles_id_list = [obstacle[0] for obstacle in self.sml_world.obstacles_list]

		try:
			id_index = obstacles_id_list.index(current_obstacle_id)
			obstacle_type = self.sml_world.obstacles_list[id_index][1]

			itemindex = self.sml_world.gui.obstacle_type.findText(obstacle_type)
			self.sml_world.gui.obstacle_type.setCurrentIndex(itemindex)

			if obstacle_type in ['circle', 'gun']:

				self.sml_world.gui.obstacle_radius.setValue(self.sml_world.obstacles_list[id_index][2])

				self.sml_world.gui.obstacle_radius.setVisible(True)
				self.sml_world.gui.obstacle_radius_label.setVisible(True)
				self.sml_world.gui.obstacle_height.setVisible(False)
				self.sml_world.gui.obstacle_height_label.setVisible(False)
				self.sml_world.gui.obstacle_width.setVisible(False)
				self.sml_world.gui.obstacle_width_label.setVisible(False)

			elif obstacle_type in ['rectangle', 'rrt']:

				self.sml_world.gui.obstacle_width.setValue(self.sml_world.obstacles_list[id_index][2])
				self.sml_world.gui.obstacle_height.setValue(self.sml_world.obstacles_list[id_index][3])

				self.sml_world.gui.obstacle_radius.setVisible(False)
				self.sml_world.gui.obstacle_radius_label.setVisible(False)
				self.sml_world.gui.obstacle_height.setVisible(True)
				self.sml_world.gui.obstacle_height_label.setVisible(True)
				self.sml_world.gui.obstacle_width.setVisible(True)
				self.sml_world.gui.obstacle_width_label.setVisible(True)
			else:
				self.sml_world.gui.obstacle_radius.setVisible(False)
				self.sml_world.gui.obstacle_radius_label.setVisible(False)
				self.sml_world.gui.obstacle_height.setVisible(False)
				self.sml_world.gui.obstacle_height_label.setVisible(False)
				self.sml_world.gui.obstacle_width.setVisible(False)
				self.sml_world.gui.obstacle_width_label.setVisible(False)

		except:
			return
