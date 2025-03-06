#pragma once

#include "hardware/Encoder/V5RotationSensor.hpp"
#include "hardware/Motor/motor.hpp"
#include "units/units.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "lemlib/pid.hpp"

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
         * @brief Move the pod with a certain wheel linear velocity and angular velocity
         */
        void moveVelocity(const LinearVelocity speed, const AngularVelocity spin);

        /**
         * @brief Move the pod with a certain wheel speed and spin percentage [-1, 1]
         */
        void movePcnt(const Number speedPcnt, const Number spinPcnt);

        /**
         * @brief Move the pod with a certain velocity vector relative to the bot
         */
        void moveVelVector(const units::V2Velocity velocity);
    protected:
        lemlib::Motor* m_topMotor;
        lemlib::Motor* m_bottomMotor;
        lemlib::V5RotationSensor* m_rotSens;
        const Divided<Length, Angle> m_circumference;
        const Number m_diffyRatio;
        const LinearVelocity m_maxSpeed;
        const AngularVelocity m_maxSpin;

        lemlib::PID m_spinPID;

        bool reversedWheel = false;
};

class ChassisSwervePod : public SwervePod {
    public:
        ChassisSwervePod(SwervePod swervePod, const units::V2Position chassisOffset);
        void move(LinearVelocity forward, LinearVelocity strafe, AngularVelocity turn);
    private:
        const units::V2Position m_offset; // The position of the pod relative to the center of the chassis
        const units::Vector2D<Divided<Length, Angle>> m_turnDirectionVec; // Direction vector to turn the chassis
        const units::Vector2D<Number> forwardDirVec = {0, 1}; // Normalized direction vector for forwards
        const units::Vector2D<Number> strafeDirVec = {1, 0}; // Normalized direction vector for strafing
};