#include "actuator.h"
#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>
#include <iostream>

/********************************
 * Constructor module
 *
 * The constructor module builds a default actuator with no pressure,
 * a braid angle of 29 degrees, an initial length of 120 mm, a current
 * length of 100 mm, a braid diameter of 9 mm, and inaccurate empirical
 * constants. Because the empirical constants are wrong, the actuator
 * is initialized to use the theoretical model.
 *
 * Inputs: none
 * Output: an actuator object
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
actuator::actuator() {
    std::array<double, 5> tempEmpiricalConstants = { 1, 2, 3, 4, 5 };

    changeUseTheoreticalModel(true);
    changePressure(0.0);
    changeInitialAngle(29.0 * M_PI / 180.0);
    changeCurrentLength(0.1); // meters
    changeBraidDiameter(0.009);
    changeInitialLength(0.12); // meters
    changeEmpiricalConstants(tempEmpiricalConstants);
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
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
actuator::~actuator() {}

/********************************
 * changeUseTheoreticalModel module
 *
 * This function is used to change whether the theoretical
 * McKibben actuator model is used.
 *
 * Inputs: bool
 *      True -> use the theoretical model
 *      False -> use the empirical model
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void actuator::changeUseTheoreticalModel(bool inBoolean) {
    useTheoreticalModel = inBoolean;

    calculateForce();
}

/********************************
 * changePressure module
 *
 * This function is used to change the pressure in the actuator.
 * Units: Pascals (Pa)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void actuator::changePressure(double inPressure) {
    pressure = inPressure;

    calculateForce();
}

/********************************
 * changeNumberOfTimesFibersWound module
 *
 * This function is used to change the number of times
 * the fibers are wound around the actuator.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void actuator::changeNumberOfTimesFibersWound(double inNumberOfTimesFibersWound) {
    numberOfTimesFibersWound = inNumberOfTimesFibersWound;

    calculateForce();
}

/********************************
 * changeInitialLength module
 *
 * This function is used to change the intial actuator length.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void actuator::changeInitialLength(double inInitialLength) {
    initialLength = inInitialLength;

    calculateBraidLength();
    calculateNumberOfTimesFibersWound();
}

/********************************
 * changeCurrentLength module
 *
 * This function is used to change the current actuator length.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void actuator::changeCurrentLength(double inCurrentLength) {
    currentLength = inCurrentLength;

    calculateForce();
}

/********************************
 * changeInitialAngle module
 *
 * This function is used to change the initial angle of the
 * fibers winding around the actuator.
 * Units: radians
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void actuator::changeInitialAngle(double inInitialAngle) {
    initialAngle = inInitialAngle;

    calculateBraidLength();
    calculateNumberOfTimesFibersWound();
}

/********************************
 * changeBraidLength module
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
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void actuator::changeBraidLength(double inBraidLength) {
    braidLength = inBraidLength;

    calculateForce();
}

/********************************
 * changeBraidDiameter module
 *
 * This function is used to change the braid diameter of the
 * McKibben actuator.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void actuator::changeBraidDiameter(double inBraidDiameter) {
    braidDiameter = inBraidDiameter;

    calculateNumberOfTimesFibersWound();
}

/********************************
 * changeForce module
 *
 * This function is used to change the force exerted by the
 * actuator.
 * Units: Newtons (N)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void actuator::changeForce(double inForce) {
    force = inForce;
}

/********************************
 * changeEmpiricalConstants module
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
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void actuator::changeEmpiricalConstants(std::array<double, 5> inConstants) {
    empiricalConstants = inConstants;

    calculateForce();
}

/********************************
 * calculateBraidLength module
 *
 * This function calculates the braid length from the stored parameters.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void actuator::calculateBraidLength() {
    double lengthInitial = getInitialLength();
    double angleInitial = getInitialAngle();
    double calculatedBraidLength;

    calculatedBraidLength = lengthInitial * cos(angleInitial);
    changeBraidLength(calculatedBraidLength);
}

/********************************
 * calculateNumberOfTimesFibersWound module
 *
 * This function calculates the number of times the fiber is wound
 * around the actuator from the stored parameters.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void actuator::calculateNumberOfTimesFibersWound() {
    double lengthInitial = getInitialLength();
    double angleInitial = getInitialAngle();
    double diameter = getBraidDiameter();
    double calculatedNumberOfTimesFibersWound;

    //cout << lengthInitial << ", " << angleInitial << ", " << diameter << endl;

    calculatedNumberOfTimesFibersWound = lengthInitial * tan(angleInitial) / (M_PI * diameter);
    changeNumberOfTimesFibersWound(calculatedNumberOfTimesFibersWound);
}

/********************************
 * calculateForce module
 *
 * This function calculate the force exerted by the actuator
 * from the stored parameters.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void actuator::calculateForce() {
    bool model = getUseTheoreticalModel();
    double p = getPressure();
    double l = getCurrentLength();
    double calculatedForce;

    if (model) {
        // theory
        double l0 = getInitialLength();
        double d = getBraidDiameter();
        double theta = getInitialAngle();
        double strain = 1 - l / l0;

        calculatedForce = p * M_PI * pow(d, 2.0) * (3 * pow(1 - strain, 2.0) / pow(tan(theta), 2.0) - 1 / pow(sin(theta), 2.0)) / 4.0;
    }
    else {
        std::array<double, 5> constants = getEmpiricalConstants();

        if (p > 0) {
            calculatedForce = constants[0] + constants[1] * p + constants[2] * l + constants[3] * p * l + constants[4] * pow(l, 2.0);
        }
        else { calculatedForce = 0; }
        if (calculatedForce < 0) { calculatedForce = 0; }
    }

    changeForce(calculatedForce);
}

/********************************
 * getUseTheoreticalModel module
 *
 * This function allows users to read the value explaining if the
 * theoretical or empirical actuator model is being used.
 *
 * Inputs: none
 * Output: bool
 *      True -> use the theoretical model
 *      False -> use the empirical model
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
bool actuator::getUseTheoreticalModel() {
    return useTheoreticalModel;
}

/********************************
 * getForce module
 *
 * This function allows users to read the force exerted by
 * the actuator.
 * Units: Newtons (N)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double actuator::getForce() {
    return force;
}

/********************************
 * getPressure module
 *
 * This function allows users to read the pressure applied to
 * the actuator.
 * Units: Pascals (Pa)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double actuator::getPressure() {
    return pressure;
}

/********************************
 * getNumberOfTimesFibersWound module
 *
 * This function allows users to read the number of times the
 * fibers are wound around the actuator.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double actuator::getNumberOfTimesFibersWound() {
    return numberOfTimesFibersWound;
}

/********************************
 * getInitialLength module
 *
 * This function allows users to read the initial length of
 * the actuator.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double actuator::getInitialLength() {
    return initialLength;
}

/********************************
 * getCurrentLength module
 *
 * This function allows users to read the current length of
 * the actuator.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double actuator::getCurrentLength() {
    return currentLength;
}

/********************************
 * getInitialAngle module
 *
 * This function allows users to read the initial angle of
 * the actuator braids.
 * Units: radians
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double actuator::getInitialAngle() {
    return initialAngle;
}

/********************************
 * getBraidLength module
 *
 * This function allows users to read the braid length of the
 * actautor.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double actuator::getBraidLength() {
    return braidLength;
}

/********************************
 * getBraidDiameter module
 *
 * This function allows users to read the braid diameter of the
 * actuator.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double actuator::getBraidDiameter() {
    return braidDiameter;
}

/********************************
 * getEmpiricalConstants module
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
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
std::array<double, 5> actuator::getEmpiricalConstants() {
    return empiricalConstants;
}