cmake_minimum_required(VERSION 2.6.0)

project(VehicleSimulator C)

include_directories(include/)

set (SimulatorFiles)
list(APPEND SimulatorFiles
	include/client.h
	include/simulator.h
	include/controller.h
	include/vehicle.h
	src/client.c
	src/simulator.c
	src/vehicle.c
	src/controller.c
)
add_definitions(-DPORTNUM=1635 -DIP="127.0.0.1")

# create the executable
add_executable(main src/main.c ${SimulatorFiles})
target_link_libraries(main m)

# note that we would normally create a library for the simulator files and
# link our tests and main function against that. for this assignment, to keep
# things simple, we'll compile separate binaries.

# now create our unit tests
enable_testing() # enables the CTEST framework

add_executable(test_controller tests/test_controller.c ${SimulatorFiles})
target_link_libraries(test_controller m)
## this created the executable, now lets add a test
add_test(TestController1 test_controller fail) # should fail
add_test(TestController2 test_controller pass) # should pass
