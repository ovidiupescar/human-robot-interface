# Interrupt signal — published by reflex_node when user starts speaking
# during robot's TTS playback. Consumers: tts_service (cancel synthesis),
# audio_player (flush buffer), Hermes adapter (record interruption position).

string reason          # "user_voice" | "yield" | "error" | "manual"
builtin_interfaces/Time stamp

# Optional context — what was the robot saying when interrupted?
string tts_active_text     # current sentence being synthesized; "" if none
uint32 tts_byte_offset     # ~position in current PCM stream; 0 if none
