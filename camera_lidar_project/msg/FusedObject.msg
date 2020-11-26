std_msgs/Header header

# Time stamp of when the object was created.
# Subtract header.stamp - spawn_time to get
# object age as a ros::Duration
time spawn_time

# ID of a unique object being tracked through time
uint32 id

# Classified Object
string Class

# Position and orientation of the object in the
# frame specified in 'header'
geometry_msgs/Pose pose

# Velocity vector of the object in the
# frame specified in 'header'
geometry_msgs/Twist velocity

# Length of each axis of the bounding box
geometry_msgs/Vector3 bounding_box_scale

# Offset vector of the bounding box n the
# frame specified in 'header'
geometry_msgs/Vector3 bounding_box_offset
