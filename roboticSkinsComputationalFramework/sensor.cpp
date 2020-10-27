#include "sensor.h"
#include "math.h"
#include <iostream>

// Note: separate header and cpp files means that this portion does not need to be recompiled

/********************************
 * Constructor module
 *
 * The constructor module builds a default sensor with a width of 10 mm,
 * thickness of 1.5 mm, initial length of 90 mm, current length of 125 mm,
 * maximum deformed length of 125 mm, and a Poisson's ratio of 0.5. The
 * parameters of the Ogden model and plastic deformation constants are set
 * to the defined in the paper.
 *
 * Inputs: none
 * Output: a sensor object
 *
 * Written by Jennifer Case on 3/29/2019
 * Tested on 4/1/2019
 ********************************/
sensor::sensor() {
    //std::cout << "Sensor Initializing..." << std::endl;
    std::array<double, 3> tempOgdenMu = { 2961,50215,75698 };
    std::array<double, 3> tempOgdenAlpha = { 7.8766,0.6886,0.6886 };
    std::array<double, 2> tempPlasticConstants = { 0.0679,-0.0093 };

    changePlasticConstants(tempPlasticConstants);
    changeOgdenMu(tempOgdenMu);
    changeOgdenAlpha(tempOgdenAlpha);

    changeWidth(0.01); // meters
    changeThickness(0.0015); // meters
    changeMaxLength(0.125); // meters
    changePoissonRatio(0.5);
    changeLengthInitial(0.09); // meters
    changeCurrentLength(0.1); // meters
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
 ********************************/
sensor::~sensor() {}

/********************************
 * changeStrainMax module
 *
 * This function is used to change the maximum strain experienced
 * by the sensor.
 * Units: unitless (m/m)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 3/29/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::changeStrainMax(double inStrainMax) {
    strainMax = inStrainMax;

    calculateStrainPlastic();
}

/********************************
 * changePlasticConstants module
 *
 * This function is used to change the plastic constants as defined by
 * Equation 7b in the paper.
 * Units: both constants are unitless (m/m)
 *
 * Inputs: array of two doubles ({d_0, d_1})
 * Output: none
 *
 * Written by Jennifer Case on 3/29/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::changePlasticConstants(std::array<double, 2> inPlasticConstants) {
    plasticConstants = inPlasticConstants;

    calculateStrainPlastic();
}

/********************************
 * changeMaxLength module
 *
 * This function is used to change the maximum length experienced
 * by the sensor.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 3/29/2019
 * Tested on 4/1/2019
 * Edited on 5/9/2019 to fix poor initialization
 ********************************/
void sensor::changeMaxLength(double inMaxLength) {
    maxLength = inMaxLength;

    //calculateLengthInitial(length0);
    calculateStrainMax();
    calculateLengthPlastic();
    calculateLength0();
}

/********************************
 * changeLengthInitial module
 *
 * This function is used to change the initial length of the
 * sensor.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 3/29/2019
 * Modified by Jennifer Case on 4/4/2019 to calculate the length0 when the initial length is changed
 * Tested on 4/1/2019
 ********************************/
void sensor::changeLengthInitial(double inLengthInitial) {
    lengthInitial = inLengthInitial;

    calculateStrainMax();
    calculateLengthPlastic();
    calculateLength0();
    calculateForce();
}

/********************************
 * changeWidth module
 *
 * This function is used to change the width of the sensor.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 3/29/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::changeWidth(double inWidth) {
    width = inWidth;

    calculateForce();
}

/********************************
 * changeThickness module
 *
 * This function is used to change the thickness of the sensor.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 3/29/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::changeThickness(double inThickness) {
    thickness = inThickness;

    calculateForce();
}

/********************************
 * changeCurrentLength module
 *
 * This function is used to change the current length of the
 * sensor.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 3/29/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::changeCurrentLength(double inCurrentLength) {
    currentLength = inCurrentLength;

    calculateForce();
}

/********************************
 * changeOgdenMu module
 *
 * This function is used to change the mu values of the Ogden
 * material model.
 *
 * Inputs: array of three doubles ({mu_1, mu_2, mu_3})
 * Output: none
 *
 * Written by Jennifer Case on 3/29/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::changeOgdenMu(std::array<double, 3> inOgdenMu) {
    ogdenMu = inOgdenMu;

    calculateForce();
}

/********************************
 * changeOgdenAlpha module
 *
 * This function is used to change the alpha values of the Ogden
 * material model.
 *
 * Inputs: array of three doubles ({alpha_1, alpha_2, alpha_3})
 * Output: none
 *
 * Written by Jennifer Case on 3/29/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::changeOgdenAlpha(std::array<double, 3> inOgdenAlpha) {
    ogdenAlpha = inOgdenAlpha;

    calculateForce();
}

/********************************
 * changeForce module
 *
 * This function is used to change the force exerted by the sensor.
 * Units: Newtons (N)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 3/29/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::changeForce(double inForce) {
    force = inForce;
}

/********************************
 * changeLength0 module
 *
 * This function is used to change the unstrained sensor length
 * after it has been pre-strained to account for plastic deformation.
 * Units: meters (m)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 3/29/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::changeLength0(double inLength0) {
    length0 = inLength0;
}

/********************************
 * changeLengthPlastic module
 *
 * This function is used to change the change in sensor length due
 * to plastic deformation.
 * Units: meters (m)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 3/29/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::changeLengthPlastic(double inLengthPlastic) {
    lengthPlastic = inLengthPlastic;

    calculateForce();
}

/********************************
 * changeStrainPlastic module
 *
 * This function is used to change the strain caused by plastic
 * deformation.
 * Units: unitless (m/m)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::changeStrainPlastic(double inStrainPlastic) {
    strainPlastic = inStrainPlastic;

    calculateLengthPlastic();
}

/********************************
 * changePoissonRatio module
 *
 * This function is used the material's Poisson ratio.
 * Units: unitless
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::changePoissonRatio(double inPoissonRatio) {
    poissonRatio = inPoissonRatio;

    calculateForce();
}

/********************************
 * calculateStrainMax module
 *
 * This function is used to calculate the maximum strain experienced
 * by the sensor.
 * Units: unitless (m/m)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::calculateStrainMax() {
    double calculatedStrainMax;
    double lengthMax = getMaxLength();
    double initialLength = getLengthInitial();

    calculatedStrainMax = (lengthMax - initialLength) / initialLength;
    changeStrainMax(calculatedStrainMax);
}

/********************************
 * calculateStrainPlastic module
 *
 * This function is used to calculate the strain due to plastic
 * deformation.
 * Units: unitless (m/m)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::calculateStrainPlastic() {
    double calculatedStrainPlastic;
    std::array<double, 2> constants = getPlasticConstants();
    double maxStrain = getStrainMax();

    if (maxStrain > -constants[1] / constants[0]) {
        calculatedStrainPlastic = constants[0] * maxStrain + constants[1];
    }
    else {
        calculatedStrainPlastic = 0;
    }

    changeStrainPlastic(calculatedStrainPlastic);
}

/********************************
 * calculateLengthInitial module
 *
 * This function is used to calculate the initial length of the
 * sensor.
 * Units: meters (m)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double (the desired natural length)
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::calculateLengthInitial(double inDesiredLength0) {
    double calculatedLengthInitial;
    std::array<double, 2> constants = getPlasticConstants();
    double lengthMax = getMaxLength();

    calculatedLengthInitial = (inDesiredLength0 - constants[0] * lengthMax) / (1 - constants[0] + constants[1]);
    changeLengthInitial(calculatedLengthInitial);
}

/********************************
 * calculateLengthPlastic module
 *
 * This function is used to calculate the change of length due to plastic
 * deformation.
 * Units: meters (m)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::calculateLengthPlastic() {
    double calculatedLengthPlastic;
    double plasticStrain = getStrainPlastic();
    double initialLength = getLengthInitial();

    calculatedLengthPlastic = plasticStrain * initialLength;
    changeLengthPlastic(calculatedLengthPlastic);
}

/********************************
 * calculateLength0 module
 *
 * This function is used to calculate the unstrained sensor length
 * after the sensor has been pre-strained to account for plastic deformation.
 * Units: meters (m)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::calculateLength0() {
    double calculatedLength0;
    double initialLength = getLengthInitial();
    double plasticLength = getLengthPlastic();

    calculatedLength0 = initialLength + plasticLength;
    changeLength0(calculatedLength0);
}

/********************************
 * calculateForce module
 *
 * This function is used to calculate the force exerted by the sensor.
 * Units: Newtons (N)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
void sensor::calculateForce() {
    double calculatedForce;
    double lengthCurrent = getCurrentLength();
    //calculateLength0();
    double lengthZ = getLength0();

    //std::cout << "Current Length: " << lengthCurrent << ", Zero Length: " << lengthZ << std::endl;

    if (lengthCurrent <= lengthZ) {
        calculatedForce = 0;
    }
    else {
        double pRatio = getPoissonRatio();
        std::array<double, 3> mu = getOgdenMu();
        std::array<double, 3> alpha = getOgdenAlpha();
        double w = getWidth();
        double t = getThickness();
        double stretch = 1 + (lengthCurrent - lengthZ) / lengthZ;
        double delta = 1 - pow(stretch, pRatio);

        double stress = 0;
        for (unsigned long i = 0; i < 3; i++) {
            stress += mu[i] * (pow(stretch, alpha[i] - 1) - pow(stretch, -2) * (2 * pow(stretch, -(1 + alpha[i]) / 2) + pow(stretch, 1 + alpha[i])) / 3);
        }
        calculatedForce = w * t * pow(1 - delta, 2) * stress;
    }

    changeForce(calculatedForce);
}

/********************************
 * getForce module
 *
 * This function allows the user to read the force exerted by
 * the sensor.
 * Units: Newtons (N)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double sensor::getForce() {
    return force;
}

/********************************
 * getWidth module
 *
 * This function allows the user to read the width of the sensor.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double sensor::getWidth() {
    return width;
}

/********************************
 * getThickness module
 *
 * This function allows the user to read the thickness of the sensor.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double sensor::getThickness() {
    return thickness;
}

/********************************
 * getCurrentLength module
 *
 * This function allows the user to read the current length of the
 * sensor.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double sensor::getCurrentLength() {
    return currentLength;
}

/********************************
 * getOgdenMu module
 *
 * This function allows the user to read the mu values of the Ogden
 * material model.
 *
 * Inputs: none
 * Output: array of three doubles ({mu_1, mu_2, mu_3})
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
std::array<double, 3> sensor::getOgdenMu() {
    return ogdenMu;
}

/********************************
 * getOgdenAlpha module
 *
 * This function allows the user to read the alpha values of the
 * Ogden material model.
 *
 * Inputs: none
 * Output: array of three doubles ({alpha_1, alpha_2, alpha_3})
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
std::array<double, 3> sensor::getOgdenAlpha() {
    return ogdenAlpha;
}

/********************************
 * getPlasticConstants module
 *
 * This function allows the user to read the constants for plastic
 * deformation.
 *
 * Inputs: none
 * Output: array of two doubles ({d_0, d_1})
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
std::array<double, 2> sensor::getPlasticConstants() {
    return plasticConstants;
}

/********************************
 * getLength0 module
 *
 * This function allows the user to read unstrained sensor length after
 * it has been pre-strained to account for plastic deformation.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double sensor::getLength0() {
    return length0;
}

/********************************
 * getLengthInitial module
 *
 * This function allows the user to read the initial length of the sensor.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double sensor::getLengthInitial() {
    return lengthInitial;
}

/********************************
 * getLengthPlastic module
 *
 * This function allows the user to read the change in sensor length due
 * to plastic deformation.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double sensor::getLengthPlastic() {
    return lengthPlastic;
}

/********************************
 * getStrainPlastic module
 *
 * This function allows the user to read the strain due to plastic
 * deformation.
 * Units: unitless (m/m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double sensor::getStrainPlastic() {
    return strainPlastic;
}

/********************************
 * getStrainMax module
 *
 * This function allows the user to read the maximum strain experienced
 * by the sensor.
 * Units: unitless (m/m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double sensor::getStrainMax() {
    return strainMax;
}

/********************************
 * getMaxLength module
 *
 * This function allows the user to read the maximum length the sensor
 * experiences.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double sensor::getMaxLength() {
    return maxLength;
}

/********************************
 * getPoissonRatio module
 *
 * This function allows the user to read the Poisson ratio of the material.
 * Units: unitless
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/1/2019
 * Tested on 4/1/2019
 ********************************/
double sensor::getPoissonRatio() {
    return poissonRatio;
}