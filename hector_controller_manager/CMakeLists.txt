cmake_minimum_required(VERSION 3.16)
project(hector_controller_manager LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(controller_interface REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_limits REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pal_statistics REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

# Pure core without ROS or threads
add_library(
  hector_controller_manager_core STATIC
  src/core/chain_rules.cpp src/core/model.cpp src/core/planner.cpp
  src/core/reconciler_apply.cpp src/core/reconciler_decide.cpp
  src/core/rollback.cpp)
target_include_directories(
  hector_controller_manager_core
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
set_target_properties(hector_controller_manager_core
                      PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(
  hector_controller_manager SHARED
  src/controller_lifecycle.cpp
  src/controller_manager.cpp
  src/diagnostics.cpp
  src/estop_monitor.cpp
  src/execution_time_registry.cpp
  src/executor_barrier.cpp
  src/hardware_preparation.cpp
  src/introspection.cpp
  src/leak.cpp
  src/node.cpp
  src/plugin_loader.cpp
  src/runtime/command_limiter.cpp
  src/runtime/controller_runtime.cpp
  src/runtime/doorbell.cpp
  src/runtime/hardware_runtime.cpp
  src/runtime/hardware_shutdown.cpp
  src/runtime/lifecycle_state.cpp
  src/runtime/owner_thread.cpp
  src/runtime/plan.cpp
  src/runtime/plan_builder.cpp
  src/runtime/rt_loop.cpp
  src/runtime/scheduling.cpp
  src/runtime/shadow_handles.cpp
  src/runtime/stop_batch.cpp
  src/request_queue.cpp
  src/request_threads.cpp
  src/startup.cpp
  src/status_messages.cpp
  src/step_executor.cpp
  src/supervisor.cpp)
target_link_libraries(
  hector_controller_manager
  PUBLIC hector_controller_manager_core
         ${builtin_interfaces_TARGETS}
         controller_interface::controller_interface
         ${controller_manager_msgs_TARGETS}
         diagnostic_updater::diagnostic_updater
         hardware_interface::hardware_interface
         joint_limits::joint_limits
         ${lifecycle_msgs_TARGETS}
         pluginlib::pluginlib
         rclcpp::rclcpp
         rclcpp_lifecycle::rclcpp_lifecycle
         realtime_tools::realtime_tools
         ${std_msgs_TARGETS})
target_link_libraries(
  hector_controller_manager
  PRIVATE ${diagnostic_msgs_TARGETS} ${pal_statistics_LIBRARIES} rcl::rcl
          ${rcl_interfaces_TARGETS})

add_executable(ros2_control_node src/ros2_control_node.cpp)
target_link_libraries(ros2_control_node hector_controller_manager)

install(
  TARGETS hector_controller_manager_core hector_controller_manager
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS ros2_control_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Tests load the library of this build tree, also when a sourced install of an
  # older one sets LD_LIBRARY_PATH, since DT_RPATH is searched before it and
  # DT_RUNPATH after it
  function(link_cm_test target)
    target_link_libraries(${target} hector_controller_manager ${ARGN})
    target_link_options(${target} PRIVATE "LINKER:--disable-new-dtags")
    # Only the build tree in DT_RPATH. Link directories there such as
    # /opt/ros/lyrical/lib would also be searched before LD_LIBRARY_PATH and
    # load a system copy of an underlay library
    set_target_properties(
      ${target} PROPERTIES BUILD_WITH_INSTALL_RPATH TRUE
                           INSTALL_RPATH "${CMAKE_CURRENT_BINARY_DIR}")
  endfunction()
  # In a release build test_handover takes 23 s, test_supervisor_failures,
  # test_fault_handling and test_supervisor_slow_callbacks about 15 s and the
  # other unit and node tests at most 13 s
  ament_add_gtest(
    test_core
    test/core/test_chain_rules.cpp
    test/core/test_planner.cpp
    test/core/test_reconciler.cpp
    test/core/test_rollback.cpp
    test/core/test_startup.cpp
    TIMEOUT
    60)
  target_link_libraries(test_core hector_controller_manager_core)

  # About 120 s in a release build and 410 s under ASan, which explores a sample
  set(EXPLORER_TIMEOUT 600)
  if(CMAKE_CXX_FLAGS MATCHES "-fsanitize")
    set(EXPLORER_TIMEOUT 1200)
  endif()
  ament_add_gtest(test_explorer test/core/test_explorer.cpp TIMEOUT
                  ${EXPLORER_TIMEOUT})
  target_link_libraries(test_explorer hector_controller_manager_core)

  ament_add_gtest(
    test_runtime
    test/runtime/test_controller_lifecycle.cpp
    test/runtime/test_controller_runtime.cpp
    test/runtime/test_estop_monitor.cpp
    test/runtime/test_estop_topics.cpp
    test/runtime/test_fault_latch.cpp
    test/runtime/test_hang_watch.cpp
    test/runtime/test_hardware_preparation.cpp
    test/runtime/test_hardware_runtime.cpp
    test/runtime/test_hardware_stats.cpp
    test/runtime/test_hardware_shutdown.cpp
    test/runtime/test_host_access.cpp
    test/runtime/test_io_gate.cpp
    test/runtime/test_lifecycle_state.cpp
    test/runtime/test_owner_cycle.cpp
    test/runtime/test_owner_thread.cpp
    test/runtime/test_plan_slot.cpp
    test/runtime/test_record_leak.cpp
    test/runtime/test_request_queue.cpp
    test/runtime/test_rt_loop.cpp
    test/runtime/test_shadow_handles.cpp
    test/runtime/test_sim_pacing.cpp
    test/runtime/test_stop_batch.cpp
    test/runtime/test_thread_role.cpp
    TIMEOUT
    300)
  link_cm_test(test_runtime)
  add_library(leak_probe MODULE test/runtime/leak_probe.cpp)
  target_compile_definitions(
    test_runtime PRIVATE LEAK_PROBE_PATH="$<TARGET_FILE:leak_probe>")
  add_dependencies(test_runtime leak_probe)

  # Separate, it replaces syscall to count futex wakes
  ament_add_gtest(test_wake_syscalls test/runtime/test_wake_syscalls.cpp
                  TIMEOUT 60)
  link_cm_test(test_wake_syscalls ${CMAKE_DL_LIBS})

  # Separate, it replaces operator new to count allocations of the control loop
  ament_add_gtest(test_supervisor test/runtime/test_supervisor.cpp
                  test/runtime/allocation_counter.cpp TIMEOUT 300)
  link_cm_test(test_supervisor)
  # Separate, it replaces operator new to count allocations of the command
  # limits
  ament_add_gtest(
    test_command_limiter test/runtime/test_command_limiter.cpp
    test/runtime/test_command_limits.cpp test/runtime/allocation_counter.cpp
    TIMEOUT 120)
  link_cm_test(test_command_limiter)
  ament_add_gtest(test_supervisor_failures
                  test/runtime/test_supervisor_failures.cpp TIMEOUT 300)
  link_cm_test(test_supervisor_failures)
  ament_add_gtest(test_supervisor_slow_callbacks
                  test/runtime/test_supervisor_slow_callbacks.cpp TIMEOUT 300)
  link_cm_test(test_supervisor_slow_callbacks)
  ament_add_gtest(test_controller_manager
                  test/runtime/test_controller_manager.cpp TIMEOUT 600)
  link_cm_test(test_controller_manager)
  ament_add_gtest(test_handover test/runtime/test_handover.cpp TIMEOUT 300)
  link_cm_test(test_handover)
  ament_add_gtest(test_fault_handling test/runtime/test_fault_handling.cpp
                  TIMEOUT 300)
  link_cm_test(test_fault_handling)
  ament_add_gtest(test_scheduling test/runtime/test_scheduling.cpp TIMEOUT 60)
  link_cm_test(test_scheduling)
  # Not registered with ctest, see its header for the options
  add_executable(bench_cycle test/runtime/bench_cycle.cpp)
  link_cm_test(bench_cycle)

  # The suite registers its tests at runtime, so the executable has its own main
  find_package(cm_test_suite REQUIRED)
  # The owner holds async hardware before its writer stops, so the suite gives
  # it no grace cycle
  set(suite_env CM_FUZZ_ASYNC_GRACE_CYCLES=0)
  ament_add_gtest(
    test_suite_cm
    test/suite/test_suite_cm.cpp
    SKIP_LINKING_MAIN_LIBRARIES
    TIMEOUT
    1800
    ENV
    ${suite_env})
  link_cm_test(test_suite_cm cm_test_suite::cm_test_suite)
  # Every rig component async, like Athena's Dynamixel buses, half the fuzz
  # seeds for its time
  set(result_file
      ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/test_suite_cm_all_async.gtest.xml
  )
  ament_add_test(
    test_suite_cm_all_async
    COMMAND
    $<TARGET_FILE:test_suite_cm>
    --gtest_output=xml:${result_file}
    RESULT_FILE
    ${result_file}
    ENV
    ${suite_env}
    CM_FUZZ_ALL_ASYNC=1
    CM_FUZZ_SEEDS=10
    CM_FUZZ_THREADED_SEEDS=2
    TIMEOUT
    1800)
  # The request services through DDS, with the suite's plugins
  ament_add_gtest(test_services test/suite/test_services.cpp TIMEOUT 120)
  link_cm_test(test_services cm_test_suite::cm_test_suite_core)
  ament_add_gtest(test_autostart test/suite/test_autostart.cpp TIMEOUT 120)
  link_cm_test(test_autostart cm_test_suite::cm_test_suite_core)
  # ros2_control_node's run in child processes ended by signals, its own main
  # runs a child
  ament_add_gtest(test_signals test/suite/test_signals.cpp
                  SKIP_LINKING_MAIN_LIBRARIES TIMEOUT 300)
  link_cm_test(test_signals cm_test_suite::cm_test_suite_core)
  # Recorded fuzz scripts, replayed in both modes
  file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/test/scripts/*.txt)
  foreach(script ${scripts})
    get_filename_component(name ${script} NAME_WE)
    set(result_file
        ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/replay_${name}.gtest.xml)
    ament_add_test(
      replay_${name}
      COMMAND
      $<TARGET_FILE:test_suite_cm>
      --gtest_filter=HectorReplay.*
      --gtest_output=xml:${result_file}
      RESULT_FILE
      ${result_file}
      ENV
      ${suite_env}
      CM_FUZZ_SCRIPT=${script}
      CM_FUZZ_REQUIRE_ALL=1
      TIMEOUT
      300)
  endforeach()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  builtin_interfaces
  controller_interface
  controller_manager_msgs
  diagnostic_updater
  hardware_interface
  joint_limits
  lifecycle_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs)
ament_package()
