set(SOURCES rt_task_xeno.c)
add_executable(rt_task_xeno ${SOURCES})
target_link_libraries(rt_task_xeno soem alchemy copperplate)

install(TARGETS rt_task_xeno DESTINATION bin)

include_directories(
	/usr/xenomai/include
	/usr/xenomai/include/cobalt
	/usr/xenomai/include/alchemy
	)
	link_directories(/usr/xenomai/lib)
