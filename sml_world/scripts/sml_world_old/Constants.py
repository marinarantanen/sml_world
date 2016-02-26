
class StateScen1(object):
	INIT 			= "INIT"
	WAIT_RSU		= "WAIT_RSU"

	WAIT_FWD_PAIR 	= "WAIT_FWD_PAIR"
	B_PAIRUP		= "B_PAIRUP"
	GAP_MAKING		= "GAP_MAKING"
	A_IS_MERGING	= "A_IS_MERGING"

	WAIT_PAIR 		= "WAIT_PAIR"
	WAIT_MERGE		= "WAIT_MERGE"
	SEND_PAIR		= "SEND_PAIR"
	WAIT_STOM		= "WAIT_STOM"
	MERGING 		= "MERGING"

class Lane(object):
	RIGHT  	= "right_lane"
	CENTER 	= "center_lane"
	LEFT 	= "left_lane"
	A = "right_lane"
	B = "center_lane"
