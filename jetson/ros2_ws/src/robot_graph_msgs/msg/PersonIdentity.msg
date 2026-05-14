# Identification of a person across one or more modalities.

string person_id            # UUID; empty = unknown
string primary_name         # display name; "" if unnamed
string[] aliases

float32 voice_confidence    # 0..1
float32 face_confidence     # 0..1
float32 fused_confidence    # 0..1

bool is_new                 # true on first registration
builtin_interfaces/Time stamp
