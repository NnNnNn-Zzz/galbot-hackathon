#include <iostream>
#include "galbot_robot.hpp"

using namespace galbot::sdk;

int main() {
    std::cout << "[probe] 开始测试..." << std::endl;
    auto& robot = GalbotRobot::get_instance(MachineType::G1);

    bool init_ok = robot.init();
    std::cout << "[probe] init() = " << (init_ok ? "true" : "false") << std::endl;

    // 尝试读取真实关节数据（如果没连上真机，这里会返回空或者打印超时）
    std::cout << "[probe] 正在尝试读取关节状态..." << std::endl;
    auto states = robot.get_joint_states({"left_arm"});
    std::cout << "[probe] 读到了 " << states.size() << " 个左臂关节。" << std::endl;

    robot.request_shutdown();
    robot.wait_for_shutdown();
    robot.destroy();
    std::cout << "[probe] done" << std::endl;
    return 0;
}