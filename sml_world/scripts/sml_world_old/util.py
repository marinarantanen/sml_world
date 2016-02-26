from threading import Thread, Lock
import copy

mutex = Lock()

output = []

def gui_print(s):
	mutex.acquire()
	output.append(str(s))
	mutex.release()

def get_print():
	mutex.acquire()
	d = copy.deepcopy(output)
	mutex.release()
	return d