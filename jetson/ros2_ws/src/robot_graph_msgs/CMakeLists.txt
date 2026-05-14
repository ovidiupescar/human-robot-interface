cmake_minimum_required(VERSION 3.8)
project(robot_graph_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PersonIdentity.msg"
  "msg/LocationIdentity.msg"
  "msg/IdentifiedSpeech.msg"
  "msg/JournalEntry.msg"

  # Identity
  "srv/IdentifyVoice.srv"
  "srv/RegisterPerson.srv"
  "srv/RenamePerson.srv"
  "srv/ListPersons.srv"
  "srv/ForgetPerson.srv"

  # Locations
  "srv/IdentifyLocation.srv"
  "srv/LearnLocation.srv"
  "srv/ListLocations.srv"
  "srv/SetCurrentLocation.srv"
  "srv/RelateLocations.srv"

  # Memory / graph
  "srv/Remember.srv"
  "srv/Recall.srv"
  "srv/Relate.srv"
  "srv/FindRelated.srv"
  "srv/CypherQuery.srv"

  # Preferences
  "srv/SetLanguagePreference.srv"

  DEPENDENCIES std_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
