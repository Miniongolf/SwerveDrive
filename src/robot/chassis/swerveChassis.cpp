#include "robot/chassis/swerveChassis.hpp"
#include "pros/rtos.hpp"
#include "swervePod.hpp"

SwerveChassis::SwerveChassis(std::vector<ChassisSwervePodPtr> swervePods)
    : m_pods(std::move(swervePods)) {}

void SwerveChassis::initialize() {
    for (ChassisSwervePodPtr& pod : m_pods) { pod->initialize(); }
    pros::Task task([&]() { taskFunct(); });
}

void SwerveChassis::taskFunct() {
    while (true) {
        for (ChassisSwervePodPtr& pod : m_pods) { pod->update(); }
    }
}