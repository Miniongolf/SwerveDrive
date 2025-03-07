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
        pros::delay(10); // Wait to save CPU while devices update
        Number maxSaturation = 1;
        // Calculate the max saturation of all the pods
        for (ChassisSwervePodPtr& pod : m_pods) { maxSaturation = units::max(pod->calcSaturation(), maxSaturation); }
        // Update all the pods, using the highest saturation value
        for (ChassisSwervePodPtr& pod : m_pods) { pod->update(maxSaturation); }
    }
}