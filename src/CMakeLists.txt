set(SOURCE_FILES
	main.cpp
	simulation.cpp
	particle.cpp)

add_executable(gravity ${SOURCE_FILES})
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
