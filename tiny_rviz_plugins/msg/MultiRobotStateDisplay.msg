# The number of joint_states and transforms_to_root should be same.
# Each component of `transforms_to_root` is a transform from the world coordinate to a root coordinate of a robot.

std_msgs/Header header
sensor_msgs/JointState[] joint_states
geometry_msgs/TransformStamped[] transforms_to_root
