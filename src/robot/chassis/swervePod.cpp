#include "robot/chassis/swervePod.hpp"
#include "units/Angle.hpp"
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
    Angle vectorAngle = units::V2Velocity(0_inps, 0_inps).angleTo(m_targetVelocity);
    return m_targetAngle.value_or(vectorAngle);
}

void SwervePod::setTarget(const Angle angle) {
    m_targetVelocity = {0_inps, 0_inps};
    m_targetAngle = angle;
}

units::V2Velocity SwervePod::getTargetVector() const { return m_targetVelocity; }

void SwervePod::setTarget(const units::V2Velocity velVector) {
    if (velVector.magnitude() == 0_inps) {
        m_targetAngle = units::V2Velocity(0_inps, 0_inps).angleTo(m_targetVelocity);
    } else {
        m_targetAngle = std::nullopt;
    }
    m_targetVelocity = velVector;
}

VelocityPair SwervePod::calcVelocities() {
    Angle currentAngle = getAngle();
    Angle targetAngle = getTargetAngle();
    Angle error = units::constrainAngle180(targetAngle - currentAngle);

    if (units::abs(error) > 90_stDeg) { reversedWheel = !reversedWheel; }

    // pid update with error, then clamp to velocity
    AngularVelocity spinOutput = from_radps(m_spinPID.update(error.convert(rad)));
    spinOutput = units::clamp(spinOutput, -m_maxSpin, m_maxSpin);
    LinearVelocity speedOutput = units::sin(error) * m_targetVelocity.magnitude();
    speedOutput = units::clamp(speedOutput, -m_maxSpeed, m_maxSpeed);

    m_velPairCache = {speedOutput, spinOutput};

    return {speedOutput, spinOutput};
}

Number SwervePod::calcSaturation() {
    auto [speed, spin] = this->calcVelocities();
    // Make sure NOT to use std::nullopt for saturation here to avoid infinite recursion
    auto [v_top, v_bottom] = toPodVelPair(speed, spin, 1);
    return units::max(units::abs(v_top), units::abs(v_bottom)) / m_topMotor->getOutputVelocity();
}

// Protected method
PodMotorVels SwervePod::toPodVelPair(const LinearVelocity speed, const AngularVelocity spin,
                                     const std::optional<Number> saturation) {
    // Do this instead of value_or to avoid side effects of updating cache when saturation is given
    const Number saturationVal = saturation.has_value() ? saturation.value() : calcSaturation();

    // Clamp speed and spin
    LinearVelocity clampedSpeed = units::clamp(speed, -m_maxSpeed, m_maxSpeed);
    if (reversedWheel) { clampedSpeed = -clampedSpeed; }

    AngularVelocity clampedSpin = units::clamp(spin, -m_maxSpin, m_maxSpin);

    AngularVelocity v_top = clampedSpin + (clampedSpeed / (2 * m_diffyRatio * m_circumference));
    AngularVelocity v_bottom = clampedSpin - (clampedSpeed / (2 * m_diffyRatio * m_circumference));
    return {v_top / saturationVal, v_bottom / saturationVal};
}

// Protected method
void SwervePod::moveVelocity(const LinearVelocity speed, const AngularVelocity spin,
                             const std::optional<Number> saturation) {
    // Do this instead of value_or to avoid side effects of updating cache when saturation is given
    Number saturationVal = saturation.has_value() ? saturation.value() : calcSaturation();
    auto [v_top, v_bottom] = toPodVelPair(speed, spin, saturationVal);

    m_topMotor->moveVelocity(v_top);
    m_bottomMotor->moveVelocity(v_bottom);
}

void SwervePod::update(const std::optional<Number> saturation) {
    auto [speed, spin] = m_velPairCache;
    this->moveVelocity(speed, spin, saturation);
}

ChassisSwervePod::ChassisSwervePod(SwervePod swervePod, const units::V2Position chassisOffset)
    : SwervePod(swervePod),
      m_offset(chassisOffset),
      m_turnDirectionVec(m_offset.rotatedBy(-90_stDeg) / rad) {}

void ChassisSwervePod::move(LinearVelocity forward, LinearVelocity strafe, AngularVelocity turn) {
    units::V2Velocity podVelVector = (forward * forwardDirVec) + (strafe * strafeDirVec) + (turn * m_turnDirectionVec);
    this->setTarget(podVelVector);
}

ChassisSwervePodPtr makeChassisPod(lemlib::Motor* topMotor, lemlib::Motor* bottomMotor,
                                   lemlib::V5RotationSensor* rotSens, const Length wheelDiameter,
                                   const Number diffyRatio, lemlib::PID spinPID,
                                   const units::V2Position chassisOffset) {
    return std::make_unique<ChassisSwervePod>(
        SwervePod(topMotor, bottomMotor, rotSens, wheelDiameter, diffyRatio, spinPID), chassisOffset);
}