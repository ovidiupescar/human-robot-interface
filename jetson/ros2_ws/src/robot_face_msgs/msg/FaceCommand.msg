# Robot face state command
# Sent to /face/command to change face expression

uint8 STATE_STANDBY=0
uint8 STATE_PROCESSING=1
uint8 STATE_SPEAKING=2
uint8 STATE_AGGRESSIVE=3

# Target state (use STATE_* constants)
uint8 state

# Intensity 0.0..1.0 (only used by SPEAKING for now)
float32 amplitude
