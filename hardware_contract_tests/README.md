# hardware_contract_tests

A gtest suite that checks a ros2_control hardware plugin against what `hector_controller_manager`
expects from it. It runs on one joint and checks that

- a NaN position or velocity command stops and holds the joint,
- stopping an interface holds the joint and starting one takes commands,
- deactivation stops the joint and activation holds it,
- the component recovers from `on_error` through configure and activate.

Tests that need a position or velocity command the joint does not export are skipped.

## Use

```cmake
find_package(hardware_contract_tests REQUIRED)
ament_add_gtest(test_hardware_contract test/test_hardware_contract.cpp)
target_link_libraries(test_hardware_contract hardware_contract_tests::hardware_contract_tests)
```

```cpp
#include <hardware_contract_tests/hardware_contract_tests.hpp>

struct MyPlant {
  std::unique_ptr<hardware_interface::SystemInterface> make();
  hardware_interface::HardwareInfo info();
  std::string joint();
  void step(double period);  // physics that write() does not run
  void push(double delta);   // moves the joint by hand
};

namespace hardware_contract_tests
{
INSTANTIATE_TYPED_TEST_SUITE_P(MyPlugin, HardwareContract, MyPlant);
}
```

The joint needs room to move one radian either way and no gravity. Examples are
`dynamics_mock_hardware/test/test_hardware_contract.cpp` and `test_hw_contract.cpp` in
`dynamixel_ros_control`.
