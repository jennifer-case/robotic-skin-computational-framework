#include "endcap.h"

/********************************
 * Constructor module
 *
 * The constructor module builds a default end cap with a
 * length of 43 mm.
 *
 * Inputs: none
 * Output: an end cap object
 *
 * Written by Jennifer Case on 4/8/2019
 * Tested on 5/7/2019
 ********************************/
endCap::endCap() {
    changeLength(0.043);
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
 * Tested on 5/7/2019
 ********************************/
endCap::~endCap() {}

/********************************
 * changeLength module
 *
 * This function is used to change the length of the end cap.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 4/8/2019
 * Tested on 5/7/2019
 ********************************/
void endCap::changeLength(double inLength) {
    length = inLength;

    calculateTransform();
}

/********************************
 * changeTransform module
 *
 * This function is used to change the transformation matrix
 * of the end cap.
 *
 * Inputs: 4d Eigen matrix
 * Output: none
 *
 * Written by Jennifer Case on 4/8/2019
 * Tested on 5/7/2019
 ********************************/
void endCap::changeTransform(Eigen::Ref<Eigen::Matrix4d> inTransform) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            transform(i, j) = inTransform(i, j);
        }
    }
}

/********************************
 * calculateTransform module
 *
 * This function is used to calculate the transformation matrix
 * of the end cap.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 4/8/2019
 * Tested on 5/7/2019
 ********************************/
void endCap::calculateTransform() {
    double len = getLength();
    Eigen::Matrix4d tran;
    tran.setIdentity();

    tran(2, 3) = len;
    changeTransform(tran);
}

/********************************
 * getLength module
 *
 * This function allows the user to read the length of the
 * end cap.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 4/8/2019
 * Tested on 5/7/2019
 ********************************/
double endCap::getLength() {
    return length;
}

/********************************
 * getTransform module
 *
 * This function allows the user to read the transformation matrix
 * of the end cap.
 *
 * Inputs: 4d Eigen matrix
 * Output: none
 *
 * Written by Jennifer Case on 4/8/2019
 * Tested on 5/7/2019
 ********************************/
void endCap::getTransform(Eigen::Ref<Eigen::Matrix4d> inMatrix) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            inMatrix(i, j) = transform(i, j);
        }
    }
}