# T1-specific addition (not part of upstream UMich-CURLY/drift).
add_executable(t1_sim_replay t1_sim_replay.cpp)
target_link_libraries(t1_sim_replay drift yaml-cpp)
