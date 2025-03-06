#pragma once

#include "hardware/Encoder/V5RotationSensor.hpp"
#include "hardware/Motor/motor.hpp"
#include "units/units.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "lemlib/pid.hpp"
#include <optional>

class SwervePod {
    public:
        /**
         * @brief Construct a new Swerve Pod object
         * @param topMotor The motor that controls the top gear (output rpm of the top gear, ccw)
         * @param bottomMotor The motor that controls the bottom gear (output rpm of the bottom gear, ccw)
         * @param rotSens The rotation sensor that measures the angle of the pod (direction should be set to standard
         * angles, i.e. counterclockwise is positive, 0 is horizontal to the right)
         * @param offset The offset of the pod from the center of the robot
         * @param wheelDiameter The diameter of the wheel
         * @param diffyRatio The ratio of the top and bottom gears to the diffy wheel gear
         * @param spinPID The PID controller for the pod's spin
         */
        SwervePod(lemlib::Motor* topMotor, lemlib::Motor* bottomMotor, lemlib::V5RotationSensor* rotSens,
                  const Length wheelDiameter, const Number diffyRatio, lemlib::PID spinPID);

        /**
         * @brief Initializes the pod, set the angle to 90º first before initialization (pointing forwards)
         */
        void initialize();

        /**
         * @brief Get the angle of the pod
         * @return The angle of the pod
         */
        Angle getAngle() const;

        /**
         * @brief Move the pod to a certain angle without forwards motion
         * @warning Stops the pod's current motion
         * @warning Should only be used for pre-aligning the pod (e.g. before auton)
         */
        void setAngle(const Angle angle);

        /**
         * @brief Get the target velocity vector of the pod
         * @return The target velocity vector
         */
        units::V2Velocity getVelVector() const;

        /**
         * @brief Move the pod with a certain velocity vector relative to the bot
         */
        void setVelVector(const units::V2Velocity velocity);

        void update();
    protected:
        /**
         * @brief Move the pod with a certain wheel linear velocity and angular velocity
         * @note Private method that should only be called in the update function
         */
        void moveVelocity(const LinearVelocity speed, const AngularVelocity spin);

        // Devices
        lemlib::Motor* m_topMotor;
        lemlib::Motor* m_bottomMotor;
        lemlib::V5RotationSensor* m_rotSens;

        // Constants
        const Divided<Length, Angle> m_circumference;
        const Number m_diffyRatio;
        const LinearVelocity m_maxSpeed;
        const AngularVelocity m_maxSpin;

        // PID
        lemlib::PID m_spinPID;

        // State machine vars
        units::V2Velocity targetVelocity = {0_inps, 0_inps};
        std::optional<Angle> targetAngle = std::nullopt;

        bool reversedWheel = false;
};

using SwervePodPtr = std::unique_ptr<SwervePod>;

class ChassisSwervePod {
    public:
        ChassisSwervePod(SwervePodPtr swervePod, const units::V2Position chassisOffset);
        void move(LinearVelocity forward, LinearVelocity strafe, AngularVelocity turn);
    protected:
        SwervePodPtr m_pod;

        const units::V2Position m_offset; // The position of the pod relative to the center of the chassis
        const units::Vector2D<Divided<Length, Angle>> m_turnDirectionVec; // Direction vector to turn the chassis
        const units::Vector2D<Number> forwardDirVec = {0, 1}; // Normalized direction vector for forwards
        const units::Vector2D<Number> strafeDirVec = {1, 0}; // Normalized direction vector for strafing
};