#include "roboticskin.h"
#include <vector>

#include <iostream>

/********************************
 * Constructor module
 *
 * The constructor module builds a default robotic skin. It requires the user
 * to provide the number of interfaces attached to the robotic skin, the
 * attachment constants for the interfaces, and whether or not actuators and
 * sensors are attached to the interfaces.
 *
 * Inputs: int (number of interfaces), vector of doubles (list of beta_0),
 *      vector of doubles (list of beta_1), vector of bools (list saying if an
 *      actuator is attached to the interfaces), vector of bools (list saying if
 *      a sensor is attached to the interfaces)
 * Output: an roboticSkin object
 *
 * Written by Jennifer Case on 4/8/2019
 * Tested on 4/8/2019
 ********************************/
roboticSkin::roboticSkin(int inSize, std::vector<double> inB0, std::vector<double> inB1, std::vector<bool> inIsActuator, std::vector<bool> inIsSensor) {
    changeNumInterfaces(inSize);
    for (int i = 0; i < inSize; i++) {
        interfaces.push_back(interface());
    }

    changeInterfaceAttachments(inB0, inB1, inIsActuator, inIsSensor);
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
 * Written by Jennifer Case on 4/8/2019
 ********************************/
roboticSkin::~roboticSkin() {}

/********************************
 * changeNumInterfaces module
 *
 * This function is used to change the number of interfaces
 * attached to the robotic skin.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: int
 * Output: none
 *
 * Written by Jennifer Case on 4/8/2019
 * Tested on 4/8/2019
 ********************************/
void roboticSkin::changeNumInterfaces(int inSize) {
    numInterfaces = inSize;
}

/********************************
 * changeInterfaceAttachments module
 *
 * This function is used to change the interface attachments.
 * The number of interfaces should not change from what is set
 * in the robotic skin.
 *
 * Inputs: vector of doubles (list of beta_0), vector of doubles (list of beta_1),
 *      vector of bools (list saying if an actuator is attached to the interfaces),
 *      vector of bools (list saying if a sensor is attached to the interfaces)
 * Output: none
 *
 * Written by Jennifer Case on 4/8/2019
 * Tested on 4/8/2019
 ********************************/
void roboticSkin::changeInterfaceAttachments(std::vector<double> inB0, std::vector<double> inB1, std::vector<bool> inIsActuator, std::vector<bool> inIsSensor) {
    int size = getNumInterfaces();

    for (int i = 0; i < size; i++) {
        std::array<double, 2> tempConstants = { inB0[static_cast<unsigned long>(i)], inB1[static_cast<unsigned long>(i)] };
        interface_changeAttachmentConstants(i, tempConstants);
        interface_changeIsSensor(i, inIsSensor[static_cast<unsigned long>(i)]);
        interface_changeIsActuator(i, inIsActuator[static_cast<unsigned long>(i)]);
    }
}

/********************************
 * getNumInterfaces module
 *
 * This function is used to change the number of interfaces
 * attached to the robotic skin.

 * Inputs: none
 * Output: int
 *
 * Written by Jennifer Case on 4/8/2019
 * Tested on 4/8/2019
 ********************************/
int roboticSkin::getNumInterfaces() {
    return numInterfaces;
}

/********************************
 * interface_changeIsSensor module
 *
 * This function is used to change whether a sensor is attached to the
 * interface or not.
 *
 * Inputs: bool
 *      True -> sensor is attached
 *      False -> sensor is not attached
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void roboticSkin::interface_changeIsSensor(int inNum, bool inBool) {
    interfaces[static_cast<unsigned long>(inNum)].changeIsSensor(inBool);
}

/********************************
 * interface_changeIsActuator module
 *
 * This function is used to change whether an actuator is attached to the
 * interface or not.
 *
 * Inputs: bool
 *      True -> actuator is attached
 *      False -> actuator is not attached
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void roboticSkin::interface_changeIsActuator(int inNum, bool inBool) {
    interfaces[static_cast<unsigned long>(inNum)].changeIsActuator(inBool);
}

/********************************
 * interface_changeSpringConstant module
 *
 * This function is used to change the spring constant of the
 * interface.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void roboticSkin::interface_changeSpringConstant(int inNum, double inConstant) {
    interfaces[static_cast<unsigned long>(inNum)].changeSpringConstant(inConstant);
}

/********************************
 * interface_changeAttachmentConstants module
 *
 * This function is used to change the attachment constants of
 * attached sensor and actuator.
 *
 * Inputs: array of two doubles ({beta_0, beta_1})
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void roboticSkin::interface_changeAttachmentConstants(int inNum, std::array<double, 2> inConstants) {
    interfaces[static_cast<unsigned long>(inNum)].changeAttachmentConstants(inConstants);
}

/********************************
 * interface_changeSensorActuatorLength module
 *
 * This function is used to change the length of the
 * sensor and actuator attached to the interface.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void roboticSkin::interface_changeSensorActuatorLength(int inNum, double inLength) {
    interfaces[static_cast<unsigned long>(inNum)].changeSensorActuatorLength(inLength);
}

/********************************
 * interface_getIsSensor module
 *
 * This function is allows the user to determine if the sensor
 * is attached to the interface.
 *
 * Inputs: none
 * Output: bool
 *      True -> sensor is attached
 *      False -> sensor is not attached
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
bool roboticSkin::interface_getIsSensor(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].getIsSensor();
}

/********************************
 * interface_getIsActuator module
 *
 * This function is allows the user to determine if the actuator
 * is attached to the interface.
 *
 * Inputs: none
 * Output: bool
 *      True -> actuator is attached
 *      False -> actuator is not attached
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
bool roboticSkin::interface_getIsActuator(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].getIsActuator();
}

/********************************
 * interface_getForce module
 *
 * This function is allows the user to determine the force from
 * the interface.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double roboticSkin::interface_getForce(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].getForce();
}

/********************************
 * interface_getForceVector module
 *
 * This function is allows the user to determine the force vector from
 * the interface.
 *
 * Inputs: 4d Eigen vector (copies the force vector here), double (radius of the cylindrical structure),
 *      double (length of the cylindrical structure)
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void roboticSkin::interface_getForceVector(int inNum, Eigen::Ref<Eigen::Vector4d> inVector, double inRadius, double inLengthEnd) {
    return interfaces[static_cast<unsigned long>(inNum)].getForceVector(inVector, inRadius, inLengthEnd);
}

/********************************
 * interface_getSpringConstant module
 *
 * This function is allows the user to determine the spring constant of
 * the interface.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double roboticSkin::interface_getSpringConstant(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].getSpringConstant();
}

/********************************
 * interface_getInterfaceLength module
 *
 * This function is allows the user to determine the interface length of
 * the interface.
 *
 * Inputs: none
 * Output: array of two doubles ({length with no spring, length with spring})
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
std::array<double, 2> roboticSkin::interface_getInterfaceLength(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].getInterfaceLength();
}

/********************************
 * interface_getAttachmentConstants module
 *
 * This function is allows the user to determine the attachment constants of
 * the interface.
 *
 * Inputs: none
 * Output: array of two doubles ({beta_0, beta_1})
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
std::array<double, 2> roboticSkin::interface_getAttachmentConstants(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].getAttachmentConstants();
}

/********************************
 * interface_getSensorActuatorLength module
 *
 * This function is allows the user to determine the sensor and actuator length.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double roboticSkin::interface_getSensorActuatorLength(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].getSensorActuatorLength();
}

/********************************
 * actuator_changeUseTheoreticalModel module
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
 * Tested on
 ********************************/
void roboticSkin::actuator_changeUseTheoreticalModel(int inNum, bool inBool) {
    interfaces[static_cast<unsigned long>(inNum)].actuator_changeUseTheoreticalModel(inBool);
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
void roboticSkin::actuator_changePressure(int inNum, double inPressure) {
    interfaces[static_cast<unsigned long>(inNum)].actuator_changePressure(inPressure);
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
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void roboticSkin::actuator_changeInitialLength(int inNum, double inInitialLength) {
    interfaces[static_cast<unsigned long>(inNum)].actuator_changeInitialLength(inInitialLength);
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
void roboticSkin::actuator_changeInitialAngle(int inNum, double inInitialAngle) {
    interfaces[static_cast<unsigned long>(inNum)].actuator_changeInitialAngle(inInitialAngle);
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
void roboticSkin::actuator_changeBraidLength(int inNum, double inBraidLength) {
    interfaces[static_cast<unsigned long>(inNum)].actuator_changeBraidLength(inBraidLength);
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
void roboticSkin::actuator_changeBraidDiameter(int inNum, double inBraidDiameter) {
    interfaces[static_cast<unsigned long>(inNum)].actuator_changeBraidDiameter(inBraidDiameter);
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
void roboticSkin::actuator_changeEmpiricalConstants(int inNum, std::array<double, 5> inConstants) {
    interfaces[static_cast<unsigned long>(inNum)].actuator_changeEmpiricalConstants(inConstants);
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
bool roboticSkin::actuator_getUseTheoreticalModel(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].actuator_getUseTheoreticalModel();
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
double roboticSkin::actuator_getForce(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].actuator_getForce();
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
double roboticSkin::actuator_getPressure(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].actuator_getPressure();
}

/********************************
 * actuator_getNumberOfTImesFibersWound module
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
double roboticSkin::actuator_getNumberOfTimesFibersWound(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].actuator_getNumberOfTimesFibersWound();
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
double roboticSkin::actuator_getInitialLength(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].actuator_getInitialLength();
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
double roboticSkin::actuator_getCurrentLength(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].actuator_getCurrentLength();
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
double roboticSkin::actuator_getInitialAngle(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].actuator_getInitialAngle();
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
double roboticSkin::actuator_getBraidLength(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].actuator_getBraidLength();
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
double roboticSkin::actuator_getBraidDiameter(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].actuator_getBraidDiameter();
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
std::array<double, 5> roboticSkin::actuator_getEmpiricalConstants(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].actuator_getEmpiricalConstants();
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
void roboticSkin::sensor_changePlasticConstants(int inNum, std::array<double, 2> inConstants) {
    interfaces[static_cast<unsigned long>(inNum)].sensor_changePlasticConstants(inConstants);
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
void roboticSkin::sensor_changeMaxLength(int inNum, double inMaxLength) {
    interfaces[static_cast<unsigned long>(inNum)].sensor_changeMaxLength(inMaxLength);
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
void roboticSkin::sensor_changeLengthInitial(int inNum, double inLengthInitial) {
    interfaces[static_cast<unsigned long>(inNum)].sensor_changeLengthInitial(inLengthInitial);
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
void roboticSkin::sensor_changeWidth(int inNum, double inWidth) {
    interfaces[static_cast<unsigned long>(inNum)].sensor_changeWidth(inWidth);
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
void roboticSkin::sensor_changeThickness(int inNum, double inThickness) {
    interfaces[static_cast<unsigned long>(inNum)].sensor_changeThickness(inThickness);
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
void roboticSkin::sensor_changeOgdenMu(int inNum, std::array<double, 3> inMu) {
    interfaces[static_cast<unsigned long>(inNum)].sensor_changeOgdenMu(inMu);
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
void roboticSkin::sensor_changeOgdenAlpha(int inNum, std::array<double, 3> inAlpha) {
    interfaces[static_cast<unsigned long>(inNum)].sensor_changeOgdenAlpha(inAlpha);
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
void roboticSkin::sensor_changePoissonRatio(int inNum, double inPoissonRatio) {
    interfaces[static_cast<unsigned long>(inNum)].sensor_changePoissonRatio(inPoissonRatio);
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
double roboticSkin::sensor_getForce(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].sensor_getForce();
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
double roboticSkin::sensor_getWidth(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].sensor_getWidth();
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
double roboticSkin::sensor_getThickness(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].sensor_getThickness();
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
double roboticSkin::sensor_getCurrentLength(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].sensor_getCurrentLength();
}

/********************************
 * sensor_getOdgenMu module
 *
 * This function allows the user to read the mu values of the Ogden
 * material model.
 *
 * Inputs: none
 * Output: array of three doubles ({mu_1, mu_2, mu_3})
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
std::array<double, 3> roboticSkin::sensor_getOgdenMu(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].sensor_getOgdenMu();
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
std::array<double, 3> roboticSkin::sensor_getOgdenAlpha(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].sensor_getOgdenAlpha();
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
double roboticSkin::sensor_getLength0(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].sensor_getLength0();
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
double roboticSkin::sensor_getLengthInitial(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].sensor_getLengthInitial();
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
double roboticSkin::sensor_getLengthPlastic(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].sensor_getLengthPlastic();
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
double roboticSkin::sensor_getStrainPlastic(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].sensor_getStrainPlastic();
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
double roboticSkin::sensor_getStrainMax(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].sensor_getStrainMax();
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
std::array<double, 2> roboticSkin::sensor_getPlasticConstants(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].sensor_getPlasticConstants();
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
double roboticSkin::sensor_getMaxLength(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].sensor_getMaxLength();
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
double roboticSkin::sensor_getPoissonRatio(int inNum) {
    return interfaces[static_cast<unsigned long>(inNum)].sensor_getPoissonRatio();
}

/********************************
 * substrate_changeInitialLength module
 *
 * This function is used to change the initial length of the
 * substrate.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void roboticSkin::substrate_changeInitialLength(double inLength) {
    spandex.changeInitialLength(inLength);
}

/********************************
 * substrate_changeInitialWidth module
 *
 * This function is used to change the initial width of the
 * substrate.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void roboticSkin::substrate_changeInitialWidth(double inWidth) {
    spandex.changeInitialWidth(inWidth);
}

/********************************
 * substrate_changeCurrentLength module
 *
 * This function is used to change the current length of the
 * substrate.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void roboticSkin::substrate_changeCurrentLength(double inLength) {
    spandex.changeCurrentLength(inLength);
}

/********************************
 * substrate_changeCurrentWidth module
 *
 * This function is used to change the current width of the
 * substrate.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void roboticSkin::substrate_changeCurrentWidth(double inWidth) {
    spandex.changeCurrentWidth(inWidth);
}

/********************************
 * substrate_changeSpringConstants module
 *
 * This function is used to change the spring constants of the
 * substrate.
 * Units: N/m
 *
 * Inputs: array of two doubles ({k_1, k_3})
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void roboticSkin::substrate_changeSpringConstants(std::array<double, 2> inConstants) {
    spandex.changeSpringConstants(inConstants);
}

/********************************
 * substrate_changeStiffnessConstants module
 *
 * This function is used to change the stiffness constants of the
 * substrate.
 *
 * This function refers to a concept of changing the bending stiffness
 * of the system due to strain on the substrate. This concept was not
 * explored in this work, but information on it can be found in:
 *
 * J. C. Case, M. C. Yuen, J. Jacobs and R. Kramer-Bottiglio, "Robotic Skins That Learn to Control Passive
 *     Structures," in IEEE Robotics and Automation Letters, vol. 4, no. 3, pp. 2485-2492, July 2019.
 *
 * Inputs: array of three doubles ({a_0, a_1, a_2})
 * Output: none
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void roboticSkin::substrate_changeStiffnessConstants(std::array<double, 3> inConstants) {
    spandex.changeStiffnessConstants(inConstants);
}

/********************************
 * substrate_getInitialLength module
 *
 * This function allows the user to read the initial length
 * of the substrate.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double roboticSkin::substrate_getInitialLength() {
    return spandex.getInitialLength();
}

/********************************
 * substrate_getInitialWidth module
 *
 * This function allows the user to read the initial width
 * of the substrate.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double roboticSkin::substrate_getInitialWidth() {
    return spandex.getInitialWidth();
}

/********************************
 * substrate_getCurrentLength module
 *
 * This function allows the user to read the current length
 * of the substrate.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double roboticSkin::substrate_getCurrentLength() {
    return spandex.getCurrentLength();
}

/********************************
 * substrate_getCurrentWidth module
 *
 * This function allows the user to read the current width
 * of the substrate.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double roboticSkin::substrate_getCurrentWidth() {
    return spandex.getCurrentWidth();
}

/********************************
 * substrate_getSpringConstants module
 *
 * This function allows the user to read the spring constants used by
 * the substrate.
 * Units: N/m
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
std::array<double, 2> roboticSkin::substrate_getSpringConstants() {
    return spandex.getSpringConstants();
}

/********************************
 * substrate_getStiffnessConstants module
 *
 * This function allows the user to read the stiffness constants
 * used for the substrate.
 *
 * This function refers to a concept of changing the bending stiffness
 * of the system due to strain on the substrate. This concept was not
 * explored in this work, but information on it can be found in:
 *
 * J. C. Case, M. C. Yuen, J. Jacobs and R. Kramer-Bottiglio, "Robotic Skins That Learn to Control Passive
 *     Structures," in IEEE Robotics and Automation Letters, vol. 4, no. 3, pp. 2485-2492, July 2019.
 *
 * Inputs: none
 * Output: array of three doubles ({a_0, a_1, a_2})
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
std::array<double, 3> roboticSkin::substrate_getStiffnessConstants() {
    return spandex.getStiffnessConstants();
}

/********************************
 * substrate_getChangeInLength module
 *
 * This function allows the user to read the changes in lengths
 * of the substrate.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: array of two doubles ({k_1, k_3})
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
std::array<double, 2> roboticSkin::substrate_getChangeInLengths() {
    return spandex.getChangeInLengths();
}

/********************************
 * substrate_getBendingStiffness module
 *
 * This function allows the user to read the bending stiffness that is
 * due to the substrate stretching over a cylindrical substrate.
 * Units: Pascals (Pa)
 *
 * This function refers to a concept of changing the bending stiffness
 * of the system due to strain on the substrate. This concept was not
 * explored in this work, but information on it can be found in:
 *
 * J. C. Case, M. C. Yuen, J. Jacobs and R. Kramer-Bottiglio, "Robotic Skins That Learn to Control Passive
 *     Structures," in IEEE Robotics and Automation Letters, vol. 4, no. 3, pp. 2485-2492, July 2019.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
double roboticSkin::substrate_getBendingStiffness() {
    return spandex.getBendingStiffness();
}

/********************************
 * substrate_getForceVector module
 *
 * This function allows the user to read the force vector
 * of the substrate.
 * Units: Newtons (N)
 *
 * Inputs: none
 * Output: 4d Eigen vector
 *
 * Written by Jennifer Case on 5/22/2019
 ********************************/
void roboticSkin::substrate_getForceVector(Eigen::Ref<Eigen::Vector4d> inVector) {
    spandex.getForceVector(inVector);
}