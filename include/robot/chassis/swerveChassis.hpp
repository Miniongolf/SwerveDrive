#include "robot/chassis/swervePod.hpp"

class SwerveChassis {
    public:
        /**
         * @brief Construct a new Swerve Chassis object
         * @param swervePods The swerve pods that make up the chassis
         */
        SwerveChassis(std::vector<ChassisSwervePodPtr> swervePods);

        /**
         * @brief Initialize all pods and start the task
         * @param velocity The velocity to move the chassis at
         * @param spin The spin to move the chassis at
         */
        void initialize();
    protected:
        void taskFunct();
        std::vector<ChassisSwervePodPtr> m_pods;
};