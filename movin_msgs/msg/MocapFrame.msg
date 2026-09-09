# One assembled MOVIN Studio frame (all bones of one actor), lossless.
# Quaternions are reordered from the SDK's (w, x, y, z) to geometry_msgs (x, y, z, w);
# values are otherwise untouched.

uint8 CONVENTION_UNITY_LH_YUP=0

std_msgs/Header header        # stamp: node clock when the frame was taken from the receiver; frame_id: world frame
string actor                  # actor name as streamed by MOVIN Studio
string actor_frame            # TF frame of this actor's anchor, e.g. "movin/MOVINMan"
int32 frame_idx               # MOVIN Studio frame index (OSC int32)
string source_timestamp       # MOVIN Studio timestamp string, e.g. "2026-09-09 20:22:18.345"
string preset                 # detected skeleton preset: "movinman", "movinman_v3" or ""
uint8 convention              # always CONVENTION_UNITY_LH_YUP
Bone[] bones
