#include "cylindricalstructure.h"
#include <iostream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>

/********************************
 * Constructor module
 *
 * The constructor module builds a default cylindrical structure with a
 * neutral state of q={0,0,1,0}, an outer radius of 15 mm, an inner radius
 * of 0 mm, a length of 100 mm, an elastic modulus of 265 kPa, and a
 * Poisson ratio of 0.5.
 *
 * Inputs: none
 * Output: an cylindrical structure object
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
cylindricalStructure::cylindricalStructure() {
    Eigen::Vector4d tempState = { 0.0,0.0,1.0,0.0 };
    changeOuterRadius(0.015); // m
    changeInnerRadius(0.0); // m
    changeLength(0.1); // m
    changeElasticModulus(265000); // Pa
    changePoissonRatio(0.5);
    changeState(tempState);
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
 * Tested on 5/1/2019
 ********************************/
cylindricalStructure::~cylindricalStructure() {}

/********************************
 * changeOuterRadius module
 *
 * This function is used to change the outer radius of the
 * cylindrical structure.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::changeOuterRadius(double inRadius) {
    outerRadius = inRadius;

    calculateBendingStiffness();
    calculateAxialStiffness();
    calculateTorsionalStiffness();
    calculateBucklingForce();
}

/********************************
 * changeInnerRadius module
 *
 * This function is used to change the inner radius of the
 * cylindrical structure.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::changeInnerRadius(double inRadius) {
    innerRadius = inRadius;

    calculateBendingStiffness();
    calculateAxialStiffness();
    calculateTorsionalStiffness();
    calculateBucklingForce();
}

/********************************
 * changeLength module
 *
 * This function is used to change the length of the cylindrical
 * structure.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::changeLength(double inLength) {
    length = inLength;

    Eigen::Matrix4d tempTransform;
    calculateBucklingForce();
    //std::cout << "Buckling force" << std::endl;
    calculateTransform(inLength, tempTransform);
    //std::cout << "Transform calculated" << std::endl;
    changeTransform(tempTransform);
    //std::cout << "Transform changed" << std::endl;
}

/********************************
 * changeElasticModulus module
 *
 * This function is used to change the elastic modulus of the
 * cylindrical structure.
 * Units: Pascals (Pa)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::changeElasticModulus(double inElasticModulus) {
    elasticModulus = inElasticModulus;

    calculateBendingStiffness();
    calculateAxialStiffness();
    calculateTorsionalStiffness();
    calculateBucklingForce();
}

/********************************
 * changePoissonRatio module
 *
 * This function is used to change the Poisson ratio of the
 * cylindrical structure.
 * Units: unitless
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::changePoissonRatio(double inPoissonRatio) {
    poissonRatio = inPoissonRatio;

    calculateTorsionalStiffness();
    calculateBucklingForce();
}

/********************************
 * changeState module
 *
 * This function is used to change the state of the cylindrical
 * structure.
 *
 * Inputs: 4d Eigen vector ({kappa_{e_1}, kappa_{e_2}, lambda, alpha})
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::changeState(Eigen::Ref<Eigen::Vector4d> inState) {
    for (int i = 0; i < 4; i++) {
        state(i) = inState(i);
    }

    double len = getLength();
    Eigen::Matrix4d tempTransform;
    calculateTransform(len, tempTransform);
    changeTransform(tempTransform);
}

/********************************
 * changeBendingStiffness module
 *
 * This function is used to change the bending stiffness of
 * the cylindrical structure.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::changeBendingStiffness(double inBendingStiffness) {
    bendingStiffness = inBendingStiffness;
}

/********************************
 * changeAxialStiffness module
 *
 * This function is used to change the axial stiffness of the
 * cylindrical structure.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::changeAxialStiffness(double inAxialStiffness) {
    axialStiffness = inAxialStiffness;
}

/********************************
 * changeTorsionalStiffness module
 *
 * This function is used to change the torsional stiffness of
 * the cylindrical structure.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::changeTorsionalStiffness(double inTorsionalStiffness) {
    torsionalStiffness = inTorsionalStiffness;
}

/********************************
 * changeBucklingForce module
 *
 * This function is used to change the buckling force of the
 * cylindrical structure.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::changeBucklingForce(double inBucklingForce) {
    bucklingForce = inBucklingForce;
}

/********************************
 * changeTransform module
 *
 * This function is used to change the homogeneous transform of the
 * cylindrical structure.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: 4d Eigen matrix
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::changeTransform(Eigen::Ref<Eigen::Matrix4d> inTransform) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            transform(i, j) = inTransform(i, j);
        }
    }
}

/********************************
 * calculateBendingStiffness module
 *
 * This function is used to calculate the bending stiffness of
 * the cylindrical structure.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::calculateBendingStiffness() {
    double eMod = getElasticModulus();
    double inertia = calculateSecondMomentOfInertia();

    double bendStiff = eMod * inertia;
    changeBendingStiffness(bendStiff);
}

/********************************
 * calculateAxialStiffness module
 *
 * This function is used to calculate the axial stiffness of the
 * cylindrical structure.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::calculateAxialStiffness() {
    double eMod = getElasticModulus();
    double area = calculateCrossSectionalArea();

    double axialStiff = eMod * area;
    changeAxialStiffness(axialStiff);
}

/********************************
 * calculateTorsionalStiffness module
 *
 * This function is used to change the torsional stiffness of
 * the cylindrical stiffness.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::calculateTorsionalStiffness() {
    double rigidity = calculateRigidityModulus();
    double momentArea = calculateSecondMomentOfArea();

    double torsionStiff = rigidity * momentArea;
    changeTorsionalStiffness(torsionStiff);
}

/********************************
 * calculateTransform module
 *
 * This function is used to calculate the homogeneous transform
 * of the cylindrical structure.
 *
 * Inputs: double (length along the cylindrical structure),
        4d Eigen matrix (transform)
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::calculateTransform(double currLength, Eigen::Ref<Eigen::Matrix4d> inTransform) {
    Eigen::Vector4d currState;
    getState(currState);
    double endLength = getLength();

    double curvature = sqrt(pow(currState(0), 2.0) + pow(currState(1), 2.0));
    double phi = atan2(currState(1), currState(0));
    double s = currState(2) * currLength;
    double sEnd = currState(2) * endLength;
    double alpha = currState(3) * s / sEnd;

    double c_ks = cos(curvature * s);
    double s_ks = sin(curvature * s);
    double v_ks = c_ks - 1;
    double c_aphi = cos(alpha + phi);
    double s_aphi = sin(alpha + phi);
    double c_a = cos(alpha);
    double s_a = sin(alpha);
    double c_phi = cos(phi);

    inTransform.setIdentity();

    if (fabs(curvature) < 1e-8) {
        inTransform(0, 0) = c_a;
        inTransform(1, 1) = c_a;
        inTransform(0, 1) = -s_a;
        inTransform(1, 0) = s_a;
        inTransform(2, 3) = s;
    }
    else {
        inTransform(0, 0) = c_phi * v_ks * c_aphi + c_a;
        inTransform(0, 1) = c_phi * v_ks * s_aphi - s_a * c_ks;
        inTransform(0, 2) = s_ks * c_aphi;
        inTransform(0, 3) = -v_ks * c_aphi / curvature;
        inTransform(1, 0) = c_phi * v_ks * s_aphi + s_a;
        inTransform(1, 1) = -c_phi * v_ks * c_aphi + c_a * c_ks;
        inTransform(1, 2) = s_ks * s_aphi;
        inTransform(1, 3) = -v_ks * s_aphi / curvature;
        inTransform(2, 0) = -c_phi * s_ks;
        inTransform(2, 1) = -sin(phi) * s_ks;
        inTransform(2, 2) = c_ks;
        inTransform(2, 3) = s_ks / curvature;
    }
}

/********************************
 * calculateBucklingForce module
 *
 * This function is used to calculate the buckling force of the
 * cylindrical structure.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::calculateBucklingForce() {
    double bendStiff = getBendingStiffness();
    double endLength = getLength();
    double rigidity = calculateRigidityModulus();
    double area = calculateCrossSectionalArea();
    double correctionFactor = calculateShearCorrectionFactor();

    double buckling = ((pow(M_PI, 2) * bendStiff) / (4 * pow(endLength, 2))) / (1 + (pow(M_PI, 2) * bendStiff) / (4 * correctionFactor * rigidity * area * pow(endLength, 2)));
    changeBucklingForce(buckling);
}

/********************************
 * calculateSecondMomentOfInertia module
 *
 * This function is used to calculate the second moment of
 * inertia of the cyliindrical structure.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
double cylindricalStructure::calculateSecondMomentOfInertia() {
    double R = getOuterRadius();
    double r = getInnerRadius();

    double inertia = M_PI * (pow(R, 4) - pow(r, 4)) / 4.0;
    return inertia;
}

/********************************
 * calculateCrossSectionalArea module
 *
 * This function is used to change the cross-sectional area of
 * the cylindrical structure.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
double cylindricalStructure::calculateCrossSectionalArea() {
    double R = getOuterRadius();
    double r = getInnerRadius();

    double area = M_PI * (pow(R, 2) - pow(r, 2));
    return area;
}

/********************************
 * calculateRigidityModulus module
 *
 * This function is used to calculate the rigidity modulus of the
 * cylindrical structure.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
double cylindricalStructure::calculateRigidityModulus() {
    double eMod = getElasticModulus();
    double poisson = getPoissonRatio();

    double rigidityModulus = eMod / (2 + 2 * poisson);
    return rigidityModulus;
}

/********************************
 * calculateSecondMomentOfArea module
 *
 * This function is used to calculate the second moment of area
 * of the cylindrical structure.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
double cylindricalStructure::calculateSecondMomentOfArea() {
    double R = getOuterRadius();
    double r = getInnerRadius();

    double momentArea = M_PI * (pow(R, 4) - pow(r, 4)) / 2.0;
    return momentArea;
}

/********************************
 * calculateShearCorrectionFactor module
 *
 * This function is used to calculate shear correction factor for
 * the cylindrical structure.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
double cylindricalStructure::calculateShearCorrectionFactor() {
    double poisson = getPoissonRatio();
    double ri = getInnerRadius();
    double r = getOuterRadius();

    double correctionFactor = 6 * (1 + poisson) * pow(1 + pow(ri / r,2),2) / ((7 + 6 * poisson) * pow(1 + pow(ri / r, 2), 2) + (20 + 12 * poisson) * pow(ri / r, 2));
    return correctionFactor;
}

/********************************
 * getOuterRadius module
 *
 * This function allows the user to read the outer radius of the
 * cylindrical structure.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
double cylindricalStructure::getOuterRadius() {
    return outerRadius;
}

/********************************
 * getInnerRadius module
 *
 * This function allows the user to read the inner radius of the
 * cylindrical structure.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
double cylindricalStructure::getInnerRadius() {
    return innerRadius;
}

/********************************
 * getLength module
 *
 * This function allows the user to read the length of the
 * cylindrical structure.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
double cylindricalStructure::getLength() {
    return length;
}

/********************************
 * getElasticModulus module
 *
 * This function allows the user to read the elastic modulus of the
 * cylindrical structure.
 * Units: Pascals (Pa)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
double cylindricalStructure::getElasticModulus() {
    return elasticModulus;
}

/********************************
 * getPoissonRatio module
 *
 * This function allows the user to read the Poisson ratio of the
 * cylindrical structure.
 * Units: unitless
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
double cylindricalStructure::getPoissonRatio() {
    return poissonRatio;
}

/********************************
 * getState module
 *
 * This function allows the user to read the state of the
 * cylindrical structure.
 * Units: meters (m)
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
void cylindricalStructure::getState(Eigen::Ref<Eigen::Vector4d> outState) {
    for (int i = 0; i < 4; i++) {
        outState(i) = state(i);
    }
}

/********************************
 * getBendingStiffness module
 *
 * This function allows the user to read the bending stiffness of the
 * cylindrical structure.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
double cylindricalStructure::getBendingStiffness() {
    return bendingStiffness;
}

/********************************
 * getAxialStiffness module
 *
 * This function allows the user to read the axial stiffness of the
 * cylindrical structure.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
double cylindricalStructure::getAxialStiffness() {
    return axialStiffness;
}

/********************************
 * getTorsionalStiffness module
 *
 * This function allows the user to read the torsional stiffness of the
 * cylindrical structure.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
double cylindricalStructure::getTorsionalStiffness() {
    return torsionalStiffness;
}

/********************************
 * getBucklingForce module
 *
 * This function allows the user to read the buckling force of the
 * cylindrical structure.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/30/2019
 * Tested on 5/1/2019
 ********************************/
double cylindricalStructure::getBucklingForce() {
    return bucklingForce;
}