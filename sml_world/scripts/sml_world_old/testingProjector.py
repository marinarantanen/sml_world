import VisualisationModule
import yappi

# file_location = "RoadModule/HighwaySML"
file_location = "RoadModule/Intersection3Way"

# yappi.start()

visualisation_module = VisualisationModule.VisualisationModule(file_location, 1400, 1050, pixel_per_meter = 5, ground_projection = True)
# visualisation_module = VisualisationModule.VisualisationModule(file_location, 800, 600, 4)
# visualisation_module = VisualisationModule.VisualisationModule(file_location, 1600, 900, pixel_per_meter = 7)
# visualisation_module = VisualisationModule.VisualisationModule(file_location, 1920, 1200)

# yappi.get_func_stats().print_all()