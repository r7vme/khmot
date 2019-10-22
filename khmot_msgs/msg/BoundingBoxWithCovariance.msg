# Copy of jsk_recognition_msgs/BoundingBox with Covariance.
# NOTE: Tracker outputs tracks as jsk_recognition_msgs/BoundingBox
Header header
# NOTE: Only x,y,yaw are used by this tracker
#       and only diagonal covariance for these values.
geometry_msgs/PoseWithCovariance pose # Only x,y,yaw are used by this tracker!
geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)
# You can use this field to hold value such as likelihood
float32 value
uint32 label
