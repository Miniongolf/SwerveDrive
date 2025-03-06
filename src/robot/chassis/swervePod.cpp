#include "robot/chassis/swervePod.hpp"
#include "lemlib/pid.hpp"
#include "units/Angle.hpp"
#include "units/Vector2D.hpp"
#include "units/units.hpp"

SwervePod::SwervePod(lemlib::Motor* topMotor, lemlib::Motor* bottomMotor, lemlib::V5RotationSensor* rotSens,
                     const Length wheelDiameter, const Number diffyRatio, lemlib::PID spinPID)
    : m_topMotor(topMotor),
      m_bottomMotor(bottomMotor),
      m_rotSens(rotSens),
      m_circumference(M_PI * wheelDiameter / rad),
      m_diffyRatio(diffyRatio),
      m_maxSpeed(2 * topMotor->getOutputVelocity() * m_diffyRatio * m_circumference),
      m_maxSpin(topMotor->getOutputVelocity()),
      m_spinPID(spinPID) {
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

Angle SwervePod::getTargetAngle() const {
    Angle vectorAngle = units::V2Velocity(0_inps, 0_inps).angleTo(targetVelocity);
    return this->targetAngle.value_or(vectorAngle);
}

void SwervePod::setAngle(const Angle angle) {
    this->targetVelocity = {0_inps, 0_inps};
    this->targetAngle = angle;
}

units::V2Velocity SwervePod::getVelVector() const { return this->targetVelocity; }

void SwervePod::setVelVector(const units::V2Velocity velocity) {
    if (velocity.magnitude() == 0_inps) {
        this->targetAngle = units::V2Velocity(0_inps, 0_inps).angleTo(targetVelocity);
    } else {
        this->targetAngle = std::nullopt;
    }
    this->targetVelocity = velocity;
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

void SwervePod::update() {
    Angle currentAngle = getAngle();
    Angle targetAngle = getTargetAngle();
    Angle error = units::constrainAngle180(targetAngle - currentAngle);

    if (units::abs(error) > 90_stDeg) { reversedWheel = !reversedWheel; }

    // pid update with error, then clamp to velocity
    AngularVelocity spinOutput = from_radps(m_spinPID.update(error.convert(rad)));
    spinOutput = units::clamp(spinOutput, -m_maxSpin, m_maxSpin);
    LinearVelocity speedOutput = units::sin(error) * this->targetVelocity.magnitude();
    speedOutput = units::clamp(speedOutput, -m_maxSpeed, m_maxSpeed);

    this->moveVelocity(speedOutput, spinOutput);
}

ChassisSwervePod::ChassisSwervePod(SwervePod swervePod, const units::V2Position chassisOffset)
    : SwervePod(swervePod),
      m_offset(chassisOffset),
      m_turnDirectionVec(m_offset.rotatedBy(-90_stDeg) / rad) {}

void ChassisSwervePod::move(LinearVelocity forward, LinearVelocity strafe, AngularVelocity turn) {
    units::V2Velocity podVelVector = (forward * forwardDirVec) + (strafe * strafeDirVec) + (turn * m_turnDirectionVec);
    this->setVelVector(podVelVector);
}

ChassisSwervePodPtr makeChassisPod(lemlib::Motor* topMotor, lemlib::Motor* bottomMotor,
                                   lemlib::V5RotationSensor* rotSens, const Length wheelDiameter,
                                   const Number diffyRatio, lemlib::PID spinPID,
                                   const units::V2Position chassisOffset) {
    return std::make_unique<ChassisSwervePod>(
        SwervePod(topMotor, bottomMotor, rotSens, wheelDiameter, diffyRatio, spinPID), chassisOffset);
}