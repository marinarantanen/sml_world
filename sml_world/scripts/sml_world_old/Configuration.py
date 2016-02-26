import ConfigParser
import shutil

def init_config_file(filename):
	shutil.copyfile(filename,"current_configuration.ini")

def read_configuration_file():
	config = ConfigParser.ConfigParser()
	config.read("current_configuration.ini")

	Scenarios = {"Scenario 1":1,"Scenario 2":2,"Scenario 3":3}

	L = dict(config.items("General"))
	L["scenario"] = Scenarios[L["scenario"]]

	for k,v in L.iteritems():
		try:
			L[k] = int(v)
		except ValueError:
			try:
				L[k] = float(v)
			except ValueError:
				pass
	return L
