#include "robot/chassis/swervePod.hpp"
#include "units/Angle.hpp"
#include "units/units.hpp"

SwervePod::SwervePod(lemlib::Motor* topMotor, lemlib::Motor* bottomMotor, lemlib::V5RotationSensor* rotSens,
                     const Length wheelDiameter, const Number diffyRatio)
    : m_topMotor(topMotor),
      m_bottomMotor(bottomMotor),
      m_rotSens(rotSens),
      m_circumference(M_PI * wheelDiameter / rad),
      m_diffyRatio(diffyRatio),
      m_maxSpeed(2 * topMotor->getOutputVelocity() * m_diffyRatio * m_circumference),
      m_maxSpin(topMotor->getOutputVelocity()) {
    if (m_topMotor->getOutputVelocity() != m_bottomMotor->getOutputVelocity()) {
        throw std::invalid_argument("Diffy swerve pod motors must have equal output velocities");
    }
}

void SwervePod::initialize() {
    reversedWheel = false;
    m_rotSens->setAngle(90_stDeg);
}

Angle SwervePod::getAngle() const {
    Angle sensAngle = units::constrainAngle360(m_rotSens->getAngle());
    Angle reversedAngle = reversedWheel ? 180_stDeg - sensAngle : sensAngle;
    return units::constrainAngle360(reversedAngle);
}

void SwervePod::moveVelocity(const LinearVelocity speed, const AngularVelocity spin) {
    // Clamp speed and spin
    auto clampedSpeed = units::clamp(speed, -m_maxSpeed, m_maxSpeed);
    if (reversedWheel) { clampedSpeed = -clampedSpeed; }

    auto clampedSpin = units::clamp(spin, -m_maxSpin, m_maxSpin);

    auto v_top = clampedSpin + (clampedSpeed / (2 * m_diffyRatio * m_circumference));
    auto v_bottom = clampedSpin - (clampedSpeed / (2 * m_diffyRatio * m_circumference));

    // Normalize velocities to max motor speed
    AngularVelocity maxMotorSpeed = m_topMotor->getOutputVelocity();
    AngularVelocity highSpeed = units::max(units::abs(v_top), units::abs(v_bottom));

    if (highSpeed > maxMotorSpeed) {
        v_bottom = maxMotorSpeed * v_bottom / highSpeed;
        v_top = maxMotorSpeed * v_top / highSpeed;
    }

    m_topMotor->moveVelocity(v_top);
    m_bottomMotor->moveVelocity(v_bottom);
}

void SwervePod::movePcnt(const Number speedPcnt, const Number spinPcnt) {
    // v_linear = (v_top - v_bottom) * diffyRatio * wheelCircumference
    // v_rot = (v_top + v_bottom) / 2
    this->moveVelocity(speedPcnt * m_maxSpeed, spinPcnt * m_maxSpin);
}

void SwervePod::moveVelVector(const units::V2Velocity velocity) {
    Angle currentAngle = getAngle();
    Angle targetAngle = units::V2Velocity(0_inps, 0_inps).angleTo(velocity);
    Angle error = units::constrainAngle180(targetAngle - currentAngle);

    if (units::abs(error) > 90_stDeg) { reversedWheel = !reversedWheel; }

    AngularVelocity spinOutput = 0_radps;
    LinearVelocity speedOutput = units::sin(error) * velocity.magnitude();
    // pid update with error, then clamp to velocity
}

ChassisSwervePod::ChassisSwervePod(SwervePod swervePod, const units::V2Position chassisOffset)
    : SwervePod(swervePod),
      m_offset(chassisOffset),
      m_turnDirectionVec(m_offset.rotatedBy(-90_stDeg) / rad) {}

void ChassisSwervePod::move(LinearVelocity forward, LinearVelocity strafe, AngularVelocity turn) {
    units::V2Velocity podVelVector = (forward * forwardDirVec) + (strafe * strafeDirVec) + (turn * m_turnDirectionVec);
    this->moveVelVector(podVelVector);
}