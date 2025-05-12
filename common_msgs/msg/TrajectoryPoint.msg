# Custom point, ment to be used as output of global_planner and optimized_local_planner

geometry_msgs/Point pose # standard ros message
float32 track_yaw # the tanget of the track in "pose"
float32 curvature # the curvature of the track in "pose"
float32 s # the cumulative arc length up to "pose" [0.0 , 1.0]
float32 speed # target car speed in "pose"