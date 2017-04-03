set(SOURCE_FILES
	main.cpp
	simulation.cpp
	visualisation.cpp
	particle.cpp)

add_executable(gravity ${SOURCE_FILES})
target_link_libraries(gravity ${VTK_LIBRARIES})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
