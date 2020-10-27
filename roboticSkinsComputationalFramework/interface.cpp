#include "interface.h"
#include "sensor.h"
#include "actuator.h"
#include <Eigen/Dense>
#include <iostream>


/********************************
 * Constructor module
 *
 * The constructor module builds a default interface with default sensor
 * and actuator attached. The spring constant is set to 1800 N/m and the
 * sensor and actuators are set to the length of 100 mm. The interface is
 * set with attachment constants of beta_0 = 0 and beta_1 = 0 according
 * to Equation 15.
 *
 * Inputs: none
 * Output: an interface object
 *
 * Written by Jennifer Case on 4/2/2019
 * Tested on 4/4/2019
 ********************************/
interface::interface() {
    std::array<double, 2> tempConstants = { 0,0 };
    changeIsSensor(true);
    changeIsActuator(true);
    changeSpringConstant(1800);
    changeAttachmentConstants(tempConstants);
    changeSensorActuatorLength(0.1); // note that this may be an "unstable" configuration (ie the total length may not equal component length plus interface length)
}

/********************************
 * Destructor module
 *
 * In theory, this function should destroy the actuator
 * if initialized, but I don't know how it does that.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/2/2019
 * Tested on 4/4/2019
 ********************************/
interface::~interface() {}

/********************************
 * changeIsSensor module
 *
 * This function is used to change whether a sensor is attached to the
 * interface or not.
 *
 * Inputs: bool
 *      True -> sensor is attached
 *      False -> sensor is not attached
 * Output: none
 *
 * Written by Jennifer Case on 4/2/2019
 * Tested on 4/4/2019
 ********************************/
void interface::changeIsSensor(bool inSensor) {
    isSensor = inSensor;
}

/********************************
 * changeIsActuator module
 *
 * This function is used to change whether an actuator is attached to the
 * interface or not.
 *
 * Inputs: bool
 *      True -> actuator is attached
 *      False -> actuator is not attached
 * Output: none
 *
 * Written by Jennifer Case on 4/2/2019
 * Tested on 4/4/2019
 ********************************/
void interface::changeIsActuator(bool inActuator) {
    isActuator = inActuator;
}

/********************************
 * changeForce module
 *
 * This function is used to change the force exerted by the
 * attached sensor and actuator.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/2/2019
 * Tested on 4/4/2019
 ********************************/
void interface::changeForce(double inForce) {
    force = inForce;

    calculateInterfaceLength();
}

/********************************
 * changeSpringConstant module
 *
 * This function is used to change the spring constant of the
 * interface.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/2/2019
 * Tested on 4/4/2019
 ********************************/
void interface::changeSpringConstant(double inSpringConstant) {
    springConstant = inSpringConstant;

    calculateInterfaceLength();
}

/********************************
 * changeAttachmentConstants module
 *
 * This function is used to change the attachment constants of
 * attached sensor and actuator.
 *
 * Inputs: array of two doubles ({beta_0, beta_1})
 * Output: none
 *
 * Written by Jennifer Case on 4/2/2019
 * Tested on 4/4/2019
 ********************************/
void interface::changeAttachmentConstants(std::array<double, 2> inAttachmentConstants) {
    attachmentConstants = inAttachmentConstants;
}

/********************************
 * changeInterfaceLength module
 *
 * This function is used to change the interface length of the
 * spring interface.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/2/2019
 * Tested on 4/4/2019
 ********************************/
void interface::changeInterfaceLength(std::array<double, 2> inInterfaceLength) {
    interfaceLength = inInterfaceLength;
}

/********************************
 * changeSensorActuatorLength module
 *
 * This function is used to change the length of the
 * sensor and actuator attached to the interface.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/4/2019
 * Tested on 4/4/2019
 ********************************/
void interface::changeSensorActuatorLength(double inLength) {
    bool sensorTrue = getIsSensor();
    bool actuatorTrue = getIsActuator();

    if (sensorTrue) { sensor_changeCurrentLength(inLength); }
    if (actuatorTrue) { actuator_changeCurrentLength(inLength); }

    calculateForce();
}

/********************************
 * changeForceVector module
 *
 * This function is used to change the force vector coming from the
 * interface.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 4/3/2019
 * Tested on 4/4/2019
 ********************************/
void interface::changeForceVector(Eigen::Ref<Eigen::Vector4d> inForceVector) {
    for (int i = 0; i < 4; i++) {
        forceVector(i) = inForceVector(i);
    }
}

/********************************
 * calculateForce module
 *
 * This function is used to calculate the force of the interface.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/3/2019
 * Modified by Jennifer Case on 4/4/2019 to only account for attached components
 * Tested on 4/4/2019
 ********************************/
void interface::calculateForce() {
    bool sensorTrue = getIsSensor();
    bool actuatorTrue = getIsActuator();
    double sensorForce = 0;
    double actuatorForce = 0;

    if (sensorTrue) { sensorForce = sensor_getForce(); }
    if (actuatorTrue) { actuatorForce = actuator_getForce(); }

    changeForce(sensorForce + actuatorForce);
}

/********************************
 * calculateForceVector module
 *
 * This function is used to calculate the force vector coming
 * from the interface.
 *
 * Inputs: double (radius of the cylindrical structure), double (length of the cylindrical structure)
 * Output: none
 *
 * Written by Jennifer Case on 4/3/2019
 * Modified by Jennifer Case on 4/4/2019 to accept radius and lengthEnd externally
 * Tested on 4/4/2019
 ********************************/
void interface::calculateForceVector(double inRadius, double inLengthEnd) {
    double f = getForce();
    std::array<double, 2> constants = getAttachmentConstants();

    Eigen::Vector4d tangentVector;
    tangentVector(0) = -constants[0] * inRadius * sin(constants[0] * inLengthEnd + constants[1]);
    tangentVector(1) = constants[0] * inRadius * cos(constants[0] * inLengthEnd + constants[1]);
    tangentVector(2) = 1;
    tangentVector(3) = 0;

    Eigen::Vector4d normalizedVector;
    normalizedVector = -f * tangentVector.normalized();

    changeForceVector(normalizedVector);
}

/********************************
 * calculateInterfaceLength module
 *
 * This function is used to calculate the interface length. It
 * provides two interface lengths. One without a spring
 * and one with a spring.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/3/2019
 * Tested on 4/4/2019
 ********************************/
void interface::calculateInterfaceLength() {
    double sprConst = getSpringConstant();
    double noSprConst = 10000000.0;
    double f = getForce();
    std::array<double, 2> intLengths;
    intLengths[0] = f / noSprConst;
    intLengths[1] = f / sprConst;

    changeInterfaceLength(intLengths);
}

/********************************
 * getIsSensor module
 *
 * This function is allows the user to determine if the sensor
 * is attached to the interface.
 *
 * Inputs: none
 * Output: bool
 *      True -> sensor is attached
 *      False -> sensor is not attached
 *
 * Written by Jennifer Case on 4/3/2019
 * Tested on 4/4/2019
 ********************************/
bool interface::getIsSensor() {
    return isSensor;
}

/********************************
 * getIsActuator module
 *
 * This function is allows the user to determine if the actuator
 * is attached to the interface.
 *
 * Inputs: none
 * Output: bool
 *      True -> actuator is attached
 *      False -> actuator is not attached
 *
 * Written by Jennifer Case on 4/3/2019
 * Tested on 4/4/2019
 ********************************/
bool interface::getIsActuator() {
    return isActuator;
}

/********************************
 * getForce module
 *
 * This function is allows the user to determine the force from
 * the interface.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/3/2019
 * Tested on 4/4/2019
 ********************************/
double interface::getForce() {
    return force;
}

/********************************
 * getSpringConstant module
 *
 * This function is allows the user to determine the spring constant of
 * the interface.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/3/2019
 * Tested on 4/4/2019
 ********************************/
double interface::getSpringConstant() {
    return springConstant;
}

/********************************
 * getAttachmentConstants module
 *
 * This function is allows the user to determine the attachment constants of
 * the interface.
 *
 * Inputs: none
 * Output: array of two doubles ({beta_0, beta_1})
 *
 * Written by Jennifer Case on 4/3/2019
 * Tested on 4/4/2019
 ********************************/
std::array<double, 2> interface::getAttachmentConstants() {
    return attachmentConstants;
}

/********************************
 * getInterfaceLength module
 *
 * This function is allows the user to determine the interface length of
 * the interface.
 *
 * Inputs: none
 * Output: array of two doubles ({length with no spring, length with spring})
 *
 * Written by Jennifer Case on 4/3/2019
 * Tested on 4/4/2019
 ********************************/
std::array<double, 2> interface::getInterfaceLength() {
    return interfaceLength;
}

/********************************
 * getForceVector module
 *
 * This function is allows the user to determine the force vector from
 * the interface.
 *
 * Inputs: 4d Eigen vector (copies the force vector here), double (radius of the cylindrical structure),
 *      double (length of the cylindrical structure)
 * Output: none
 *
 * Written by Jennifer Case on 4/3/2019
 * Tested on 4/4/2019
 ********************************/
void interface::getForceVector(Eigen::Ref<Eigen::Vector4d> inVector, double inRadius, double inLengthEnd) {
    calculateForce();
    calculateForceVector(inRadius, inLengthEnd);
    for (int i = 0; i < 4; i++) {
        inVector(i) = forceVector(i);
    }
}

/********************************
 * getSensorActuatorLength module
 *
 * This function is allows the user to determine the sensor and actuator length.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/3/2019
 * Tested on 4/4/2019
 ********************************/
double interface::getSensorActuatorLength() {
    bool sensorTrue = getIsSensor();
    if (sensorTrue) { return sensor_getCurrentLength(); }

    bool actuatorTrue = getIsActuator();
    if (actuatorTrue) { return actuator_getCurrentLength(); }

    return 0;
}

/********************************
 * actuator_changeUseTheoretical module
 *
 * This function is used to change whether the theoretical
 * McKibben actuator model is used.
 *
 * Inputs: bool
 *      True -> use the theoretical model
 *      False -> use the empirical model
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void interface::actuator_changeUseTheoreticalModel(bool inBool) {
    attachedActuator.changeUseTheoreticalModel(inBool);
}

/********************************
 * actuator_changePressure module
 *
 * This function is used to change the pressure in the actuator.
 * Units: Pascals (Pa)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void interface::actuator_changePressure(double inPressure) {
    attachedActuator.changePressure(inPressure);
}

/********************************
 * actuator_changeInitialLength module
 *
 * This function is used to change the intial actuator length.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/22/2019
 ********************************/
void interface::actuator_changeInitialLength(double inInitialLength) {
    attachedActuator.changeInitialLength(inInitialLength);
}

/********************************
 * actuator_changeCurrentLength module
 *
 * This function is used to change the current actuator length.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void interface::actuator_changeCurrentLength(double inCurrentLength) {
    attachedActuator.changeCurrentLength(inCurrentLength);
}

/********************************
 * actuator_changeInitialAngle module
 *
 * This function is used to change the initial angle of the
 * fibers winding around the actuator.
 * Units: radians
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void interface::actuator_changeInitialAngle(double inInitialAngle) {
    attachedActuator.changeInitialAngle(inInitialAngle);
}

/********************************
 * actuator_changeBraidLength module
 *
 * This function is used to change the length of the
 * fiber braid.
 * Units: meters (m)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void interface::actuator_changeBraidLength(double inBraidLength) {
    attachedActuator.changeBraidLength(inBraidLength);
}

/********************************
 * actuator_changeBraidDiameter module
 *
 * This function is used to change the braid diameter of the
 * McKibben actuator.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void interface::actuator_changeBraidDiameter(double inBraidDiameter) {
    attachedActuator.changeBraidDiameter(inBraidDiameter);
}

/********************************
 * actuator_changeEmpiricalConstants module
 *
 * This function is used to change the constants of the empirically-
 * derived actuator model.
 * Units: c_0 - Newtons (N)
 *        c_1 - N/Pa
 *        c_2 - N/m
 *        c_3 - N/(Pa.m)
 *        c_4 - N/(m^2)
 *
 * Inputs: array of five doubles ({c_0, c_1, c_2, c_3, c_4})
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void interface::actuator_changeEmpiricalConstants(std::array<double, 5> inEmpiricalConstants) {
    attachedActuator.changeEmpiricalConstants(inEmpiricalConstants);
}

/********************************
 * actuator_getUseTheoreticalModel module
 *
 * This function allows users to read the value explaining if the
 * theoretical or empirical actuator model is being used.
 *
 * Inputs: none
 * Output: bool
 *      True -> use the theoretical model
 *      False -> use the empirical model
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
bool interface::actuator_getUseTheoreticalModel() {
    return attachedActuator.getUseTheoreticalModel();
}

/********************************
 * actuator_getForce module
 *
 * This function allows users to read the force exerted by
 * the actuator.
 * Units: Newtons (N)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::actuator_getForce() {
    return attachedActuator.getForce();
}

/********************************
 * actuator_getPressure module
 *
 * This function allows users to read the pressure applied to
 * the actuator.
 * Units: Pascals (Pa)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::actuator_getPressure() {
    return attachedActuator.getPressure();
}

/********************************
 * actuator_getNumberOfTimesFibersWound module
 *
 * This function allows users to read the number of times the
 * fibers are wound around the actuator.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::actuator_getNumberOfTimesFibersWound() {
    return attachedActuator.getNumberOfTimesFibersWound();
}

/********************************
 * actuator_getInitialLength module
 *
 * This function allows users to read the initial length of
 * the actuator.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::actuator_getInitialLength() {
    return attachedActuator.getInitialLength();
}

/********************************
 * actuator_getCurrentLength module
 *
 * This function allows users to read the current length of
 * the actuator.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::actuator_getCurrentLength() {
    return attachedActuator.getCurrentLength();
}

/********************************
 * actuator_getInitialAngle module
 *
 * This function allows users to read the initial angle of
 * the actuator braids.
 * Units: radians
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::actuator_getInitialAngle() {
    return attachedActuator.getInitialAngle();
}

/********************************
 * actuator_getBraidLength module
 *
 * This function allows users to read the braid length of the
 * actautor.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::actuator_getBraidLength() {
    return attachedActuator.getBraidLength();
}

/********************************
 * actuator_getBraidDiameter module
 *
 * This function allows users to read the braid diameter of the
 * actuator.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::actuator_getBraidDiameter() {
    return attachedActuator.getBraidDiameter();
}

/********************************
 * actuator_getEmpiricalConstants module
 *
 * This function allows users to read the constants of the
 * empirical model.
 * Units: c_0 - Newtons (N)
 *        c_1 - N/Pa
 *        c_2 - N/m
 *        c_3 - N/(Pa.m)
 *        c_4 - N/(m^2)
 *
 * Inputs: none
 * Output: array of five doubles ({c_0, c_1, c_2, c_3, c_4})
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
std::array<double, 5> interface::actuator_getEmpiricalConstants() {
    return attachedActuator.getEmpiricalConstants();
}

/********************************
 * sensor_changePlasticConstants module
 *
 * This function is used to change the plastic constants as defined by
 * Equation 7b in the paper.
 * Units: both constants are unitless (m/m)
 *
 * Inputs: array of two doubles ({d_0, d_1})
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void interface::sensor_changePlasticConstants(std::array<double, 2> inConstants) {
    attachedSensor.changePlasticConstants(inConstants);
}

/********************************
 * sensor_changeMaxLength module
 *
 * This function is used to change the maximum length experienced
 * by the sensor.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void interface::sensor_changeMaxLength(double inMaxLength) {
    attachedSensor.changeMaxLength(inMaxLength);
}

/********************************
 * sensor_changeLengthInitial module
 *
 * This function is used to change the initial length of the
 * sensor.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void interface::sensor_changeLengthInitial(double inLengthInitial) {
    attachedSensor.changeLengthInitial(inLengthInitial);
}

/********************************
 * sensor_changeWidth module
 *
 * This function is used to change the width of the sensor.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void interface::sensor_changeWidth(double inWidth) {
    attachedSensor.changeWidth(inWidth);
}

/********************************
 * sensor_changeThickness module
 *
 * This function is used to change the thickness of the sensor.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void interface::sensor_changeThickness(double inThickness) {
    attachedSensor.changeThickness(inThickness);
}

/********************************
 * sensor_changeCurrentLength module
 *
 * This function is used to change the current length of the
 * sensor.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void interface::sensor_changeCurrentLength(double inCurrentLength) {
    attachedSensor.changeCurrentLength(inCurrentLength);
}

/********************************
 * sensor_changeOgdenMu module
 *
 * This function is used to change the mu values of the Ogden
 * material model.
 *
 * Inputs: array of three doubles ({mu_1, mu_2, mu_3})
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void interface::sensor_changeOgdenMu(std::array<double, 3> inMu) {
    attachedSensor.changeOgdenMu(inMu);
}

/********************************
 * sensor_changeOgdenAlpha module
 *
 * This function is used to change the alpha values of the Ogden
 * material model.
 *
 * Inputs: array of three doubles ({alpha_1, alpha_2, alpha_3})
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void interface::sensor_changeOgdenAlpha(std::array<double, 3> inAlpha) {
    attachedSensor.changeOgdenAlpha(inAlpha);
}

/********************************
 * sensor_changePoissonRatio module
 *
 * This function is used the material's Poisson ratio.
 * Units: unitless
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void interface::sensor_changePoissonRatio(double inPoissonRatio) {
    attachedSensor.changePoissonRatio(inPoissonRatio);
}

/********************************
 * sensor_getForce module
 *
 * This function allows the user to read the force exerted by
 * the sensor.
 * Units: Newtons (N)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::sensor_getForce() {
    return attachedSensor.getForce();
}

/********************************
 * sensor_getWidth module
 *
 * This function allows the user to read the width of the sensor.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::sensor_getWidth() {
    return attachedSensor.getWidth();
}

/********************************
 * sensor_getThickness module
 *
 * This function allows the user to read the thickness of the sensor.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::sensor_getThickness() {
    return attachedSensor.getThickness();
}

/********************************
 * sensor_getCurrentLength module
 *
 * This function allows the user to read the current length of the
 * sensor.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::sensor_getCurrentLength() {
    return attachedSensor.getCurrentLength();
}

/********************************
 * sensor_getOgdenMu module
 *
 * This function allows the user to read the mu values of the Ogden
 * material model.
 *
 * Inputs: none
 * Output: array of three doubles ({mu_1, mu_2, mu_3})
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
std::array<double, 3> interface::sensor_getOgdenMu() {
    return attachedSensor.getOgdenMu();
}

/********************************
 * sensor_getOgdenAlpha module
 *
 * This function allows the user to read the alpha values of the
 * Ogden material model.
 *
 * Inputs: none
 * Output: array of three doubles ({alpha_1, alpha_2, alpha_3})
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
std::array<double, 3> interface::sensor_getOgdenAlpha() {
    return attachedSensor.getOgdenAlpha();
}

/********************************
 * sensor_getLength0 module
 *
 * This function allows the user to read unstrained sensor length after
 * it has been pre-strained to account for plastic deformation.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::sensor_getLength0() {
    return attachedSensor.getLength0();
}

/********************************
 * sensor_getLengthInitial module
 *
 * This function allows the user to read the initial length of the sensor.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::sensor_getLengthInitial() {
    return attachedSensor.getLengthInitial();
}

/********************************
 * sensor_getLengthPlastic module
 *
 * This function allows the user to read the change in sensor length due
 * to plastic deformation.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::sensor_getLengthPlastic() {
    return attachedSensor.getLengthPlastic();
}

/********************************
 * sensor_getStrainPlastic module
 *
 * This function allows the user to read the strain due to plastic
 * deformation.
 * Units: unitless (m/m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::sensor_getStrainPlastic() {
    return attachedSensor.getStrainPlastic();
}

/********************************
 * sensor_getStrainMax module
 *
 * This function allows the user to read the maximum strain experienced
 * by the sensor.
 * Units: unitless (m/m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::sensor_getStrainMax() {
    return attachedSensor.getStrainMax();
}

/********************************
 * sensor_getPlasticConstants module
 *
 * This function allows the user to read the constants for plastic
 * deformation.
 *
 * Inputs: none
 * Output: array of two doubles ({d_0, d_1})
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
std::array<double, 2> interface::sensor_getPlasticConstants() {
    return attachedSensor.getPlasticConstants();
}

/********************************
 * sensor_getMaxLength module
 *
 * This function allows the user to read the maximum length the sensor
 * experiences.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::sensor_getMaxLength() {
    return attachedSensor.getMaxLength();
}

/********************************
 * sensor_getPoissonRatio module
 *
 * This function allows the user to read the Poisson ratio of the material.
 * Units: unitless
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double interface::sensor_getPoissonRatio() {
    return attachedSensor.getPoissonRatio();
}