# Generic raw journal entry — flat key/value plus a free-form JSON payload.
# Written by every node that wants to be on the record.

builtin_interfaces/Time stamp
string kind                 # e.g. "perception.voice", "agent.speech", "actuator.face"
string source               # producing node name
string payload_json         # JSON-encoded arbitrary payload
