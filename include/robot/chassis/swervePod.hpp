#pragma once

#include <optional>
#include <utility>
#include "units/units.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "hardware/Encoder/V5RotationSensor.hpp"
#include "hardware/Motor/motor.hpp"
#include "lemlib/pid.hpp"

struct PodMotorVels {
        AngularVelocity v_top;
        AngularVelocity v_bottom;
};

using VelocityPair = std::pair<LinearVelocity, AngularVelocity>;

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
         * @brief Get the target angle of the pod
         * @return The target angle
         */
        Angle getTargetAngle() const;

        /**
         * @brief Get the target velocity vector of the pod
         * @return The target velocity vector
         */
        units::V2Velocity getTargetVector() const;

        /**
         * @brief Move the pod with a certain velocity vector relative to the bot
         */
        void setTarget(const units::V2Velocity velVector);

        /**
         * @brief Move the pod to a certain angle without forwards motion
         * @warning Stops the pod's current motion
         * @warning Should only be used for pre-aligning the pod (e.g. before auton)
         */
        void setTarget(const Angle angle);

        VelocityPair calcVelocities();

        Number calcSaturation();

        /**
         * @brief Update the pod's movement
         * @param saturation The saturation factor for the pod's movement
         * @note Should be called in a loop to update the pod's movement
         * @warning Make sure saturation has been calculated first to update the cache, and then call this
         */
        void update(const std::optional<Number> saturation);
    protected:
        PodMotorVels toPodVelPair(const LinearVelocity speed, const AngularVelocity spin,
                                  const std::optional<Number> saturation);

        /**
         * @brief Move the pod with a certain wheel linear velocity and angular velocity
         * @note Private method that should only be called in the update function
         */
        void moveVelocity(const LinearVelocity speed, const AngularVelocity spin,
                          const std::optional<Number> saturation);

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

        // Cache
        VelocityPair m_velPairCache = {0_inps, 0_radps};

        // State machine vars
        units::V2Velocity m_targetVelocity = {0_inps, 0_inps};
        std::optional<Angle> m_targetAngle = 90_stDeg;

        bool reversedWheel = false;
};

using SwervePodPtr = std::unique_ptr<SwervePod>;

class ChassisSwervePod : public SwervePod {
    public:
        ChassisSwervePod(SwervePod swervePod, const units::V2Position chassisOffset);
        void move(LinearVelocity forward, LinearVelocity strafe, AngularVelocity turn);
    protected:
        const units::V2Position m_offset; // The position of the pod relative to the center of the chassis
        const units::Vector2D<Divided<Length, Angle>> m_turnDirectionVec; // Direction vector to turn the chassis
        const units::Vector2D<Number> forwardDirVec = {0, 1}; // Normalized direction vector for forwards
        const units::Vector2D<Number> strafeDirVec = {1, 0}; // Normalized direction vector for strafing
};

using ChassisSwervePodPtr = std::unique_ptr<ChassisSwervePod>;

/**
 * @brief Make a swerve pod for the chassis
 * @param topMotor The motor that controls the top gear (output rpm of the top gear, ccw)
 * @param bottomMotor The motor that controls the bottom gear (output rpm of the bottom gear, ccw)
 * @param rotSens The rotation sensor that measures the angle of the pod (direction should be set to standard
 * angles, i.e. counterclockwise is positive, 0 is horizontal to the right)
 * @param wheelDiameter The diameter of the wheel
 * @param diffyRatio The ratio of the top and bottom gears to the diffy wheel gear
 * @param spinPID The PID controller for the pod's spin
 * @param chassisOffset The offset of the pod from the center of the robot
 * @return Unique pointer to the swerve pod
 */
ChassisSwervePodPtr makeChassisPod(lemlib::Motor* topMotor, lemlib::Motor* bottomMotor,
                                   lemlib::V5RotationSensor* rotSens, const Length wheelDiameter,
                                   const Number diffyRatio, lemlib::PID spinPID, const units::V2Position chassisOffset);