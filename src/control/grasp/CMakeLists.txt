add_executable (grasp can_communicator.cpp canAPI.cpp RockScissorsPaper.cpp controller.cpp)
target_link_libraries (grasp BHand pcanbasic pthread)
#install (TARGETS grasp DESTINATION ${PROJECT_SOURCE_DIR}/bin)
install (TARGETS grasp DESTINATION ${PROJECT_BINARY_DIR}/bin)
