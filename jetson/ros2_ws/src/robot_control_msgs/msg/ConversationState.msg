# Single-source-of-truth conversation state — published by reflex_node.

uint8 IDLE=0          # nobody speaking, robot silent
uint8 LISTENING=1     # user is speaking, robot capturing
uint8 THINKING=2      # user done speaking, LLM processing (filler may play)
uint8 SPEAKING=3      # robot is talking; barge-in armed
uint8 INTERRUPTED=4   # robot was speaking, user interrupted

uint8 state
builtin_interfaces/Time stamp
