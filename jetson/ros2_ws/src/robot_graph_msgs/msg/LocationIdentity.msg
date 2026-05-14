# Identification of the robot's current location from visual scene recognition.

string location_id          # UUID; empty = unknown / not yet learned
string name                 # e.g. "kitchen", "office"; "" if unnamed
string parent_name          # e.g. "home"; "" if no parent

float32 confidence          # 0..1
bool is_unknown             # true if best match below threshold

builtin_interfaces/Time stamp
