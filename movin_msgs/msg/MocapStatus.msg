# Periodic (1 Hz) status of mocap_node.

std_msgs/Header header
string mode                   # "live" or "replay"
bool receiving                # a frame arrived within stale_timeout_sec
string bind_address           # "host:port" actually bound (live), "" when not bound
string error                  # last start() error, "" when healthy
float32 receive_rate_hz       # OSC packet rate reported by the SDK receiver
string frame_delivery         # "drain" (all buffered frames) or "latest_only" (SDK fallback)
uint64 frames_published
uint64 frames_dropped         # sum of frame_idx gaps per actor
string[] actors
string preset
bool recording
string recording_path
uint32 recording_segment
uint64 recording_messages
float64 recording_duration_sec
bool replay_finished
