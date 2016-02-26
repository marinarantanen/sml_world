from threading import _Event


class PingReturn(_Event):
	""" This event is fired when the ping is returned. Besides being an event you can get the pinged info directly from
		PingReturn.getPingDict
	"""
	def __init__(self):
		super(PingReturn, self).__init__()

		self.pingId = None
		self.pingDict = None

		
	def setPingDict(self, pingDict):
		self.pingDict = pingDict

	def getPingDict(self):
		return self.pingDict
