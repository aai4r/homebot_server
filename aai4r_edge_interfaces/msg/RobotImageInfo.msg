# Two-integer timestamp that is expressed as seconds and nanoseconds.
builtin_interfaces/Time stamp
string agent_id	    # ex) hancom_qi_1
string format		# jpeg
string hash		    # image hash
uint64 seq_id		# tick count
uint16 height		# 640
uint16 width		# 480
float32 distance	# Meter
uint8 zone		    # 1.Entrance 2.Lobby 3.table
uint8[] data		# Compressed image buffer
string params       # Additional Parameters in JSON