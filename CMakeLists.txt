cmake_minimum_required(VERSION 2.8.3)

project(fdo6)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	tf
)

include_directories(
	${catkin_INCLUDE_DIRS}
)

catkin_package()

include_directories(
	include
)

link_directories(include/lib)

add_executable(fdo6_driver src/fdo6_driver)
target_link_libraries(fdo6_driver
	${catkin_LIBRARIES}
	dhd
	drd
	usb-1.0 -lrt
	pthread
	)






