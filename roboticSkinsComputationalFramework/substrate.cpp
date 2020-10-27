#include "substrate.h"
#include <Eigen/Dense>

/********************************
 * Constructor module
 *
 * The constructor module builds a default substrate with default spring and
 * stiffness constants, an initial length of 100 mm, an initial width of 120 mm,
 * a current length of 100 mm, and a current width of 120 mm.
 *
 * Inputs: none
 * Output: a substrate object
 *
 * Written by Jennifer Case on 4/4/2019
 * Tested on 4/5/2019
 ********************************/
substrate::substrate() {
    std::array<double, 2> sprConstants = { 1800.0, 1800.0 }; // not sure if this is correct....
    std::array<double, 3> stiffConstants = { 0, 0, 0 }; // also don't really know this....
    changeInitialLength(0.1);
    changeInitialWidth(0.12);
    changeCurrentLength(0.1);
    changeCurrentWidth(0.12);
    changeSpringConstants(sprConstants);
    changeStiffnessConstants(stiffConstants);
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
 * Written by Jennifer Case on 4/4/2019
 ********************************/
substrate::~substrate() {}

/********************************
 * changeInitialLength module
 *
 * This function is used to change the initial length of the
 * substrate.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/4/2019
 * Tested on 4/5/2019
 ********************************/
void substrate::changeInitialLength(double inInitialLength) {
    initialLength = inInitialLength;

    calculateChangeInLengths();
}

/********************************
 * changeInitialWidth module
 *
 * This function is used to change the initial width of the
 * substrate.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/4/2019
 * Tested on 4/5/2019
 ********************************/
void substrate::changeInitialWidth(double inInitialWidth) {
    initialWidth = inInitialWidth;

    calculateChangeInLengths();
}

/********************************
 * changeCurrentLength module
 *
 * This function is used to change the current length of the
 * substrate.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/4/2019
 * Tested on 4/5/2019
 ********************************/
void substrate::changeCurrentLength(double inCurrentLength) {
    currentLength = inCurrentLength;

    calculateChangeInLengths();
}

/********************************
 * changeCurrentWidth module
 *
 * This function is used to change the current width of the
 * substrate.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/4/2019
 * Tested on 4/5/2019
 ********************************/
void substrate::changeCurrentWidth(double inCurrentWidth) {
    currentWidth = inCurrentWidth;

    calculateChangeInLengths();
}

/********************************
 * changeSpringConstants module
 *
 * This function is used to change the spring constants of the
 * substrate.
 * Units: N/m
 *
 * Inputs: array of two doubles ({k_1, k_3})
 * Output: none
 *
 * Written by Jennifer Case on 4/5/2019
 * Tested on 4/5/2019
 ********************************/
void substrate::changeSpringConstants(std::array<double, 2> inSpringConstants) {
    springConstants = inSpringConstants;

    calculateForceVector();
}

/********************************
 * changeStiffnessConstants module
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
 * Written by Jennifer Case on 4/5/2019
 * Tested on 4/5/2019
 ********************************/
void substrate::changeStiffnessConstants(std::array<double, 3> inStiffnessConstants) {
    stiffnessConstants = inStiffnessConstants;

    calculateBendingStiffness();
}

/********************************
 * changeChangeInLengths module
 *
 * This function is used to change the changes in lengths experienced by the
 * sensor.
 * Units: meters (m)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: array of two doubles ({change in length of length, change in length of width})
 * Output: none
 *
 * Written by Jennifer Case on 4/4/2019
 * Tested on 4/5/2019
 ********************************/
void substrate::changeChangeInLengths(std::array<double, 2> inLengths) {
    changeInLengths = inLengths;

    calculateForceVector();
    calculateBendingStiffness();
}

/********************************
 * changeBendingStiffness module
 *
 * This function is used to change the bending stiffness due to a strained
 * substrate.
 * Units: Pascals (Pa)
 *
 * This function refers to a concept of changing the bending stiffness
 * of the system due to strain on the substrate. This concept was not
 * explored in this work, but information on it can be found in:
 *
 * J. C. Case, M. C. Yuen, J. Jacobs and R. Kramer-Bottiglio, "Robotic Skins That Learn to Control Passive
 *     Structures," in IEEE Robotics and Automation Letters, vol. 4, no. 3, pp. 2485-2492, July 2019.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/4/2019
 * Tested on 4/5/2019
 ********************************/
void substrate::changeBendingStiffness(double inBendingStiffness) {
    bendingStiffness = inBendingStiffness;
}

/********************************
 * changeForceVector module
 *
 * This function is used to change the force vector coming from the
 * substrate.
 * Units: Newtons (N)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 4/4/2019
 * Tested on 4/5/2019
 ********************************/
void substrate::changeForceVector(Eigen::Ref<Eigen::Vector4d> inForceVector) {
    for (int i = 0; i < 4; i++) {
        forceVector(i) = inForceVector(i);
    }
}

/********************************
 * calculateChangeInLengths module
 *
 * This function is used to calculate the changes in length
 * experienced by the substrate.
 * Units: meters (m)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/4/2019
 * Tested on 4/5/2019
 ********************************/
void substrate::calculateChangeInLengths() {
    double initLength = getInitialLength();
    double initWidth = getInitialWidth();
    double currLength = getCurrentLength();
    double currWidth = getCurrentWidth();
    std::array<double, 2> changes;

    changes[0] = currLength - initLength;
    changes[1] = currWidth - initWidth;

    changeChangeInLengths(changes);
}

/********************************
 * calculateForceVector module
 *
 * This function is used to calculate the force vector of the
 * substrate.
 * Units: Newtons (N)
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/5/2019
 * Tested on 4/5/2019
 ********************************/
void substrate::calculateForceVector() {
    std::array<double, 2> springs = getSpringConstants();
    std::array<double, 2> lengthChanges = getChangeInLengths();
    Eigen::Vector4d force;

    for (int i = 0; i < 4; i++) { force(i) = 0.0; }

    if (lengthChanges[0] > 0) { force(2) = -springs[0] * lengthChanges[0]; } // remember this is the length which affects the e_3 axis
    if (lengthChanges[1] > 0) { force(0) = -springs[1] * lengthChanges[1]; } // remember this is the width which affects the e_1 axis

    changeForceVector(force);
}

/********************************
 * calculateBendingStiffness module
 *
 * This function is used to calculate the bending stiffness due to the
 * substrate being stretched and wrapped around a cylindrical structure.
 * Units: Pascals (Pa)
 *
 * This function refers to a concept of changing the bending stiffness
 * of the system due to strain on the substrate. This concept was not
 * explored in this work, but information on it can be found in:
 *
 * J. C. Case, M. C. Yuen, J. Jacobs and R. Kramer-Bottiglio, "Robotic Skins That Learn to Control Passive
 *     Structures," in IEEE Robotics and Automation Letters, vol. 4, no. 3, pp. 2485-2492, July 2019.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/5/2019
 * Tested on 4/5/2019
 ********************************/
void substrate::calculateBendingStiffness() {
    std::array<double, 3> constants = getStiffnessConstants();
    std::array<double, 2> lengthChanges = getChangeInLengths();
    double bendingK = constants[0];

    if (lengthChanges[0] > 0) { bendingK += constants[1] * lengthChanges[0]; }
    if (lengthChanges[1] > 0) { bendingK += constants[2] * lengthChanges[1]; }

    changeBendingStiffness(bendingK);
}

/********************************
 * getInitialLength module
 *
 * This function allows the user to read the initial length
 * of the substrate.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/5/2019
 * Tested on 4/5/2019
 ********************************/
double substrate::getInitialLength() {
    return initialLength;
}

/********************************
 * getInitialWidth module
 *
 * This function allows the user to read the initial width
 * of the substrate.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/5/2019
 * Tested on 4/5/2019
 ********************************/
double substrate::getInitialWidth() {
    return initialWidth;
}

/********************************
 * getCurrentLength module
 *
 * This function allows the user to read the current length
 * of the substrate.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/5/2019
 * Tested on 4/5/2019
 ********************************/
double substrate::getCurrentLength() {
    return currentLength;
}

/********************************
 * getCurrentWidth module
 *
 * This function allows the user to read the current width
 * of the substrate.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/5/2019
 * Tested on 4/5/2019
 ********************************/
double substrate::getCurrentWidth() {
    return currentWidth;
}

/********************************
 * getSpringConstants module
 *
 * This function allows the user to read the spring constants used by
 * the substrate.
 * Units: N/m
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/5/2019
 * Tested on 4/5/2019
 ********************************/
std::array<double, 2> substrate::getSpringConstants() {
    return springConstants;
}

/********************************
 * getStiffnessConstants module
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
 * Written by Jennifer Case on 4/5/2019
 * Tested on 4/5/2019
 ********************************/
std::array<double, 3> substrate::getStiffnessConstants() {
    return stiffnessConstants;
}

/********************************
 * getChangesInLength module
 *
 * This function allows the user to read the changes in lengths
 * of the substrate.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: array of two doubles ({k_1, k_3})
 *
 * Written by Jennifer Case on 4/5/2019
 * Tested on 4/5/2019
 ********************************/
std::array<double, 2> substrate::getChangeInLengths() {
    return changeInLengths;
}

/********************************
 * getBendingStiffness module
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
 * Written by Jennifer Case on 4/5/2019
 * Tested on 4/5/2019
 ********************************/
double substrate::getBendingStiffness() {
    return bendingStiffness;
}

/********************************
 * getForceVector module
 *
 * This function allows the user to read the force vector
 * of the substrate.
 * Units: Newtons (N)
 *
 * Inputs: none
 * Output: 4d Eigen vector
 *
 * Written by Jennifer Case on 4/5/2019
 * Tested on 4/5/2019
 ********************************/
void substrate::getForceVector(Eigen::Ref<Eigen::Vector4d> inVector) {
    for (int i = 0; i < 4; i++) {
        inVector(i) = forceVector(i);
    }
}