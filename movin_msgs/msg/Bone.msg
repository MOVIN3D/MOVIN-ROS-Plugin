# One bone of a MOVIN Studio mocap frame, exactly as streamed by the SDK.
# Convention: Unity left-handed, Y-up, meters. Nothing is mirrored or rotated
# here; the TF tree published by mocap_node carries the ROS (REP-103) pose.

int32 bone_index
int32 parent_index                     # -1 for the stream root (e.g. RootBone)
string name
geometry_msgs/Vector3 position         # SDK "p": offset in the parent's frame
geometry_msgs/Quaternion rest_rotation # SDK "rq": rest-pose quaternion, informational only (never remove it from rotation)
geometry_msgs/Quaternion rotation      # SDK "q": full local rotation, rest included
geometry_msgs/Vector3 scale            # SDK "s"
