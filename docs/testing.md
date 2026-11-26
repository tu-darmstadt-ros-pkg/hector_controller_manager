# Testing

```bash
colcon build --packages-select cm_test_suite hector_controller_manager \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
colcon test --packages-select hector_controller_manager && colcon test-result --verbose
```

`cm_test_suite` is a test dependency and must be in the workspace. The test targets are listed in
the [design notes](design.md#tests).

## The common test suite

`cm_test_suite` judges a controller manager only by what reaches its rig hardware. Every command a
controller writes is a unique token, and when hardware applies one, the suite walks the chain back
to its writer, which must be active, in the activation that wrote it and at most 5 cycles old. It
also checks mode switches against IO, settled consistency, the reported states, cycle times under 20
ms and power cuts. Its fuzzer generates scripts of requests, faults and hangs and shrinks a failure
to a minimal script. The README of the `cm_test_suite` package describes every check and variable.

```bash
source install/setup.bash
CM_FUZZ_SEEDS=200 ./build/hector_controller_manager/test_suite_cm --gtest_filter='*Fuzz.stepped*'
CM_FUZZ_SCRIPT=$PWD/fail.txt ./build/hector_controller_manager/test_suite_cm \
  --gtest_filter='HectorReplay.*'
```

ctest gives every test that uses ROS its own domain, so `colcon test --ctest-args -j8` runs them in
parallel. Test binaries started by hand in parallel need separate `ROS_DOMAIN_ID`s.

The CI also runs the threaded tests, labelled `concurrency`, under AddressSanitizer:

```bash
colcon build --packages-select hector_controller_manager --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  "-DCMAKE_CXX_FLAGS=-fsanitize=address -fno-omit-frame-pointer"
colcon test --packages-select hector_controller_manager --ctest-args -j8 -L concurrency
```
