#include "robot/chassis/swervePod.hpp"

class SwerveChassis {
    public:
        /**
         * @brief Construct a new Swerve Chassis object
         * @param swervePods The swerve pods that make up the chassis
         */
        SwerveChassis(std::vector<ChassisSwervePodPtr> swervePods);

        void initialize();
    protected:
        void taskFunct();
        std::vector<ChassisSwervePodPtr> m_pods;
};