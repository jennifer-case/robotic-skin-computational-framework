#include "wholesystem.h"
#include <iostream>

/********************************
 * Constructor module
 *
 * The constructor module builds a default system based on the number of segments, a vector
 * of segments, and the global gravity direction.
 *
 * Inputs: none
 * Output: a system object
 *
 * Written by Jennifer Case on 5/20/2019
 * Tested on 5/20/2019
 ********************************/
wholeSystem::wholeSystem(int inNumSegments, std::vector<segment> inSegments, Eigen::Ref<Eigen::Vector4d> inGravityDirection) {
    changeNumSegments(inNumSegments);
    changeGravityDirection(inGravityDirection);
    segments = inSegments;
}

/********************************
 * Deconstructor module
 *
 * In theory, this function should destroy the actuator
 * if initialized, but I don't know how it does that.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 5/20/2019
 ********************************/
wholeSystem::~wholeSystem() {}

/********************************
 * changeNumSegments module
 *
 * This function is used to change the number of segments in the system.
 *
 * Inputs: int
 * Output: none
 *
 * Written by Jennifer Case on 5/20/2019
 * Tested on 5/20/2019
 ********************************/
void wholeSystem::changeNumSegments(int inNum) {
    numSegments = inNum;
}

/********************************
 * changeGravityDirection module
 *
 * This function is used to change the direction of gravity.
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 5/20/2019
 * Tested on 5/20/2019
 ********************************/
void wholeSystem::changeGravityDirection(Eigen::Ref<Eigen::Vector4d> inDirection) {
    for (int i = 0; i < 4; i++) {
        globalGravityDirection(i) = -inDirection(i);
    }
}

/********************************
 * adjustWeightDirections module
 *
 * This function is used to adjust the weight directions of each segment.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 5/20/2019
 * Tested on 5/20/2019
 ********************************/
void wholeSystem::adjustWeightDirections() {
    int numSeg = getNumSegments();
    Eigen::Vector4d gravityDirection;
    getGravityDirection(gravityDirection);

    Eigen::Matrix4d tempMatrix;
    tempMatrix.setIdentity();

    for (int i = 0; i < numSeg; i++) {
        Eigen::Vector4d currDirection;
        Eigen::Matrix4d transposedMatrix;
        transposedMatrix = tempMatrix.transpose();
        // make sure to set bottom rows to zero since they shouldn't matter
        transposedMatrix(3, 0) = 0; transposedMatrix(3, 1) = 0; transposedMatrix(3, 2) = 0;
        currDirection = transposedMatrix * gravityDirection;
        currDirection.normalize();
        segment_changeWeightDirectionVector(i, currDirection);
        Eigen::Matrix4d cylinderMatrix;
        double length = cylinder_getLength(i);
        cylinder_calculateTransform(i, length, cylinderMatrix);
        tempMatrix = tempMatrix * cylinderMatrix;
    }
}

/********************************
 * deformSystem module
 *
 * This function is used to deform the whole system.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 5/21/2019
 * Tested on 5/21/2019
 ********************************/
void wholeSystem::deformSystem() {
    int numSegs = getNumSegments();
    Eigen::Vector4d prevForce = { 0,0,0,0 };
    Eigen::Vector4d prevMoment = { 0,0,0,0 };
    Eigen::Matrix4d endCapTransform;
    endCapTransform.setIdentity();

    adjustWeightDirections();
    for (int i = numSegs - 1; i >= 0; i--) {
        if (i < numSegs - 1) {
            segment_getForceVector(i + 1, prevForce);
            segment_getMomentVector(i + 1, prevMoment);
            endcap_getTransform(i + 1, 1, endCapTransform);
        }
        segment_optimizeState(i, endCapTransform, prevForce, prevMoment);
    }
}

/********************************
 * optimizeSystemPressure module
 *
 * This function is used to optimize the pressure throughout the system.
 *
 * Inputs: vector of 4d Eigen vectors (states of each segment)
 * Output: none
 *
 * Written by Jennifer Case on 5/21/2019
 * Tested on 5/21/2019
 ********************************/
void wholeSystem::optimizeSystemPressure(std::vector<Eigen::Ref<Eigen::Vector4d>> inStates) {
    int numSegs = getNumSegments();
    std::vector<Eigen::Vector4d> prevWeightDirections(static_cast<unsigned long>(numSegs));
    std::vector<Eigen::Vector4d> currWeightDirections(static_cast<unsigned long>(numSegs));
    std::vector<Eigen::Vector4d> errorWeightDirections(static_cast<unsigned long>(numSegs));

    double currError;

    int cnt = 0;
    while (true) {
        cnt++;
        if (cnt > 100) {
            std::cout << "Failed to optimize system" << std::endl;
            break;
        }
        // grab previous weight directions
        for (int i = 0; i < numSegs; i++) {
            segment_getWeightDirectionVector(i, prevWeightDirections[static_cast<unsigned long>(i)]);
        }
        Eigen::Matrix4d endCapTransform;
        endCapTransform.setIdentity();
        Eigen::Vector4d tempForce = { 0,0,0,0 };
        Eigen::Vector4d tempMoment = { 0,0,0,0 };
        for (int i = numSegs - 1; i >= 0; i--) {
            segment_optimizePressure(i, endCapTransform, tempForce, tempMoment, inStates[static_cast<unsigned long>(i)]);
            segment_getForceVector(i, tempForce);
            segment_getMomentVector(i, tempMoment);
            endcap_getTransform(i, 0, endCapTransform);
        }
        adjustWeightDirections();
        // grab current weight directions
        for (int i = 0; i < numSegs; i++) {
            segment_getWeightDirectionVector(i, currWeightDirections[static_cast<unsigned long>(i)]);
            // calculate error for each weight direction
            errorWeightDirections[static_cast<unsigned long>(i)] = currWeightDirections[static_cast<unsigned long>(i)] - prevWeightDirections[static_cast<unsigned long>(i)];
        }
        // calculate error
        currError = 0;
        for (int i = 0; i < numSegs; i++) {
            for (int j = 0; j < 4; j++) {
                currError += fabs(errorWeightDirections[static_cast<unsigned long>(i)](j));
            }
        }

        if (currError < 1e-2) {
            break;
        }
    }
}

/********************************
 * optimizeSystemState module
 *
 * This function is used to optimize the state throughout the system.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 5/21/2019
 * Tested on 5/21/2019
 ********************************/
void wholeSystem::optimizeSystemState() {
    int numSegs = getNumSegments();
    std::vector<Eigen::Vector4d> prevWeightDirections(static_cast<unsigned long>(numSegs));
    std::vector<Eigen::Vector4d> currWeightDirections(static_cast<unsigned long>(numSegs));
    std::vector<Eigen::Vector4d> errorWeightDirections(static_cast<unsigned long>(numSegs));
    double currError;

    int cnt = 0;
    while (true) {
        cnt++;
        if (cnt > 100) {
            std::cout << "Failed to optimize system" << std::endl;
            break;
        }
        // grab previous weight directions
        for (int i = 0; i < numSegs; i++) {
            segment_getWeightDirectionVector(i, prevWeightDirections[static_cast<unsigned long>(i)]);
        }
        deformSystem();
        adjustWeightDirections();
        // grab current weight directions
        for (int i = 0; i < numSegs; i++) {
            segment_getWeightDirectionVector(i, currWeightDirections[static_cast<unsigned long>(i)]);
            // calculate error for each weight direction
            errorWeightDirections[static_cast<unsigned long>(i)] = currWeightDirections[static_cast<unsigned long>(i)] - prevWeightDirections[static_cast<unsigned long>(i)];
        }
        // calculate error
        currError = 0;
        for (int i = 0; i < numSegs; i++) {
            for (int j = 0; j < 4; j++) {
                currError += fabs(errorWeightDirections[static_cast<unsigned long>(i)](j));
            }
        }

        if (currError < 1e-2) {
            break;
        }
    }
}

/********************************
 * estimateSystemState module
 *
 * This function is used to estimate the state of the system given sensor values.
 * It currently can only handle a single segment, but can be expanded to account for
 * multiple segments.
 *
 * Inputs: vector of doubles (reported sensor lengths in meters)
 * Output: none
 *
 * Written by Jennifer Case on 3/1/2020
 * Tested on 3/15/2020
 ********************************/
void wholeSystem::estimateSystemState(std::vector<double> inSensorLengths) {
    // if you have more than one segment, this should be adapted to account for that
    int numSegs = getNumSegments();

    adjustWeightDirections();
    for (int i = numSegs - 1; i >= 0; i--) {
        segment_stateEstimation(i, inSensorLengths);
    }
}

/********************************
 * getNumSegments module
 *
 * This function allows the user to read the number of segments in the system.
 *
 * Inputs: none
 * Output: int
 *
 * Written by Jennifer Case on 5/20/2019
 * Tested on 5/20/2019
 ********************************/
int wholeSystem::getNumSegments() {
    return numSegments;
}

/********************************
 * getGravityDirection module
 *
 * This function allows the user to read the direction of gravity.
 *
 * Inputs: none
 * Output: int
 *
 * Written by Jennifer Case on 5/20/2019
 * Tested on 5/20/2019
 ********************************/
void wholeSystem::getGravityDirection(Eigen::Ref<Eigen::Vector4d> inVector) {
    inVector = globalGravityDirection;
}

/********************************
 * segment_changeMass module
 *
 * This function is used to change the mass of the segment.
 * Units: kilograms (kg)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 * Tested on 5/24/2019
 ********************************/
void wholeSystem::segment_changeMass(int inSegID, double inMass) {
    segments[static_cast<unsigned long>(inSegID)].changeMass(inMass);
}

/********************************
 * segment_changeGravity module
 *
 * This function is used to change the gravity experienced by
 * the segment.
 * Units: m/s^2
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 * Tested on 5/24/2019
 ********************************/
void wholeSystem::segment_changeGravity(int inSegID, double inGravity) {
    segments[static_cast<unsigned long>(inSegID)].changeGravity(inGravity);
}

/********************************
 * segment_changeWeightDirectionVector module
 *
 * This function is used to change the weight direction
 * of the segment.
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 * Tested on 5/24/2019
 ********************************/
void wholeSystem::segment_changeWeightDirectionVector(int inSegID, Eigen::Ref<Eigen::Vector4d> inVector) {
    segments[static_cast<unsigned long>(inSegID)].changeWeightDirectionVector(inVector);
}

/********************************
 * segment_optimizeState module
 *
 * This function optimizes for state given the parameters of the segment
 *
 * Inputs: 4d Eigen matrix (transform from previous segment), 4d Eigen vector (force from previous segment),
 *      4d Eigen vector (moment from previous segment)
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 * Tested on 5/24/2019
 ********************************/
void wholeSystem::segment_optimizeState(int inSegID, Eigen::Ref<Eigen::Matrix4d> inEndCapTransform, Eigen::Ref<Eigen::Vector4d> inPrevForce, Eigen::Ref<Eigen::Vector4d> inPrevMoment) {
    segments[static_cast<unsigned long>(inSegID)].optimizeState(inEndCapTransform, inPrevForce, inPrevMoment);
}

/********************************
 * segment_stateEstimation module
 *
 * This function determines the state of the segment given the reported
 * sensor lengths.
 *
 * Inputs: vector of doubles (reported sensor lengths)
 * Output: none
 *
 * Written by Jennifer Case on 3/5/2020
 * Tested on 3/15/2020
 ********************************/
void wholeSystem::segment_stateEstimation(int inSegID, std::vector<double> inSensorLengths) {
    segments[static_cast<unsigned long>(inSegID)].stateEstimation(inSensorLengths);
}
/********************************
 * segment_optimizePressure module
 *
 * This function optimizes for actuator pressures given the parameters of the segment
 *
 * Inputs: 4d Eigen matrix (transform from previous segment), 4d Eigen vector (force from previous segment),
 *      4d Eigen vector (moment from previous segment), 4d Eigen vector (desired state for the segment)
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 * Tested on 5/24/2019
 ********************************/
void wholeSystem::segment_optimizePressure(int inSegID, Eigen::Ref<Eigen::Matrix4d> inEndCapTransform, Eigen::Ref<Eigen::Vector4d> inPrevForce, Eigen::Ref<Eigen::Vector4d> inPrevMoment, Eigen::Ref<Eigen::Vector4d> inDesiredState) {
    segments[static_cast<unsigned long>(inSegID)].optimizePressure(inEndCapTransform, inPrevForce, inPrevMoment, inDesiredState);
}

/********************************
 * segment_getMass module
 *
 * This function allows users to read the mass of the segment.
 * Units: kilograms (kg)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/24/2019
 * Tested on 5/24/2019
 ********************************/
double wholeSystem::segment_getMass(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].getMass();
}

/********************************
 * segment_getGravity module
 *
 * This function allows users to read the gravity experienced by the segment.
 * Units: m/s^2
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/24/2019
 * Tested on 5/24/2019
 ********************************/
double wholeSystem::segment_getGravity(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].getGravity();
}

/********************************
 * segment_getWeightDirectionVector module
 *
 * This function allows users to read the direction of the weight vector.
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 * Tested on 5/24/2019
 ********************************/
void wholeSystem::segment_getWeightDirectionVector(int inSegID, Eigen::Ref<Eigen::Vector4d> inVector) {
    segments[static_cast<unsigned long>(inSegID)].getWeightDirectionVector(inVector);
}

/********************************
 * segment_getBendingStiffness module
 *
 * This function allows users to read the bending stiffness of the segment.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/24/2019
 * Tested on 5/24/2019
 ********************************/
double wholeSystem::segment_getBendingStiffness(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].getBendingStiffness();
}

/********************************
 * segment_getAxialStiffness module
 *
 * This function allows users to read the axial stiffness of the segment.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/24/2019
 * Tested on 5/24/2019
 ********************************/
double wholeSystem::segment_getAxialStiffness(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].getAxialStiffness();
}

/********************************
 * segment_getTorsionalStiffness module
 *
 * This function allows users to read the torsional stiffness of the segment.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/24/2019
 * Tested on 5/24/2019
 ********************************/
double wholeSystem::segment_getTorsionalStiffness(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].getTorsionalStiffness();
}

/********************************
 * segment_getForceVector module
 *
 * This function allows users to read the force vector of the segment.
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 * Tested on 5/24/2019
 ********************************/
void wholeSystem::segment_getForceVector(int inSegID, Eigen::Ref<Eigen::Vector4d> inVector) {
    segments[static_cast<unsigned long>(inSegID)].getForceVector(inVector);
}

/********************************
 * segment_getMomentVector module
 *
 * This function allows users to read the moment vector of the segment.
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 * Tested on 5/24/2019
 ********************************/
void wholeSystem::segment_getMomentVector(int inSegID, Eigen::Ref<Eigen::Vector4d> inVector) {
    segments[static_cast<unsigned long>(inSegID)].getMomentVector(inVector);
}

/********************************
 * getError module
 *
 * This function allows users to read the error of the segment.
 *
 * Inputs: none
 * Output: bool
 *
 * Written by Jennifer Case on 2/13/2020
 * Tested on 2/13/2020
 ********************************/
bool wholeSystem::segment_getError(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].getError();
}

/********************************
 * cylinder_changeOuterRadius module
 *
 * This function is used to change the outer radius of the
 * cylindrical structure.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::cylinder_changeOuterRadius(int inSegID, double inRadius) {
    segments[static_cast<unsigned long>(inSegID)].cylinder_changeOuterRadius(inRadius);
}

/********************************
 * cylinder_changeInnerRadius module
 *
 * This function is used to change the inner radius of the
 * cylindrical structure.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::cylinder_changeInnerRadius(int inSegID, double inRadius) {
    segments[static_cast<unsigned long>(inSegID)].cylinder_changeInnerRadius(inRadius);
}

/********************************
 * cylinder_changeLength module
 *
 * This function is used to change the length of the cylindrical
 * structure.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::cylinder_changeLength(int inSegID, double inLength) {
    segments[static_cast<unsigned long>(inSegID)].cylinder_changeLength(inLength);
}

/********************************
 * cylinder_changeElasticModulus module
 *
 * This function is used to change the elastic modulus of the
 * cylindrical structure.
 * Units: Pascals (Pa)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::cylinder_changeElasticModulus(int inSegID, double inModulus) {
    segments[static_cast<unsigned long>(inSegID)].cylinder_changeElasticModulus(inModulus);
}

/********************************
 * cylinder_changePoissonRatio module
 *
 * This function is used to change the Poisson ratio of the
 * cylindrical structure.
 * Units: unitless
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::cylinder_changePoissonRatio(int inSegID, double inPoissonRatio) {
    segments[static_cast<unsigned long>(inSegID)].cylinder_changePoissonRatio(inPoissonRatio);
}

/********************************
 * cylinder_changeState module
 *
 * This function is used to change the state of the cylindrical
 * structure.
 *
 * Inputs: 4d Eigen vector ({kappa_{e_1}, kappa_{e_2}, lambda, alpha})
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::cylinder_changeState(int inSegID, Eigen::Ref<Eigen::Vector4d> inVector) {
    segments[static_cast<unsigned long>(inSegID)].cylinder_changeState(inVector);
}

/********************************
 * cylinder_calculateTransform module
 *
 * This function is used to calculate the homogeneous transform
 * of the cylindrical structure.
 *
 * Inputs: double (length along the cylindrical structure),
        4d Eigen matrix (transform)
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::cylinder_calculateTransform(int inSegID, double inH, Eigen::Ref<Eigen::Matrix4d> inMatrix) {
    segments[static_cast<unsigned long>(inSegID)].cylinder_calculateTransform(inH, inMatrix);
}

/********************************
 * cylinder_getOuterRadius module
 *
 * This function allows the user to read the outer radius of the
 * cylindrical structure.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
double wholeSystem::cylinder_getOuterRadius(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].cylinder_getOuterRadius();
}

/********************************
 * cylinder_getInnerRadius module
 *
 * This function allows the user to read the inner radius of the
 * cylindrical structure.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
double wholeSystem::cylinder_getInnerRadius(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].cylinder_getInnerRadius();
}

/********************************
 * cylinder_getLength module
 *
 * This function allows the user to read the length of the
 * cylindrical structure.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
double wholeSystem::cylinder_getLength(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].cylinder_getLength();
}

/********************************
 * cylinder_getElasticModulus module
 *
 * This function allows the user to read the elastic modulus of the
 * cylindrical structure.
 * Units: Pascals (Pa)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
double wholeSystem::cylinder_getElasticModulus(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].cylinder_getElasticModulus();
}

/********************************
 * cylinder_getPoissonRatio module
 *
 * This function allows the user to read the Poisson ratio of the
 * cylindrical structure.
 * Units: unitless
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
double wholeSystem::cylinder_getPoissonRatio(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].cylinder_getPoissonRatio();
}

/********************************
 * cylinder_getState module
 *
 * This function allows the user to read the state of the
 * cylindrical structure.
 * Units: meters (m)
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::cylinder_getState(int inSegID, Eigen::Ref<Eigen::Vector4d> inVector) {
    segments[static_cast<unsigned long>(inSegID)].cylinder_getState(inVector);
}

/********************************
 * cylinder_getBendingStiffness module
 *
 * This function allows the user to read the bending stiffness of the
 * cylindrical structure.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
double wholeSystem::cylinder_getBendingStiffness(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].cylinder_getBendingStiffness();
}

/********************************
 * cylinder_getAxialStiffness module
 *
 * This function allows the user to read the axial stiffness of the
 * cylindrical structure.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
double wholeSystem::cylinder_getAxialStiffness(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].cylinder_getAxialStiffness();
}

/********************************
 * cylinder_getTorsionalStiffness module
 *
 * This function allows the user to read the torsional stiffness of the
 * cylindrical structure.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
double wholeSystem::cylinder_getTorsionalStiffness(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].cylinder_getTorsionalStiffness();
}

/********************************
 * cylinder_getBucklingForce module
 *
 * This function allows the user to read the buckling force of the
 * cylindrical structure.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
double wholeSystem::cylinder_getBucklingForce(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].cylinder_getBucklingForce();
}

/********************************
 * endcap_changeLength module
 *
 * This function is used to change the length of the end cap.
 * Units: meters (m)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::endcap_changeLength(int inSegID, int inInterfaceID, double inLength) {
    segments[static_cast<unsigned long>(inSegID)].endcap_changeLength(inInterfaceID, inLength);
}

/********************************
 * endcap_getLength module
 *
 * This function allows the user to read the length of the
 * end cap.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
double wholeSystem::endcap_getLength(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].endcap_getLength(inInterfaceID);
}

/********************************
 * endcap_getTransform module
 *
 * This function allows the user to read the transformation matrix
 * of the end cap.
 *
 * Inputs: 4d Eigen matrix
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::endcap_getTransform(int inSegID, int inEndCapID, Eigen::Ref<Eigen::Matrix4d> inMatrix) {
    segments[static_cast<unsigned long>(inSegID)].endcap_getTransform(inEndCapID, inMatrix);
}

/********************************
 * skin_changeInterfaceAttachments module
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
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::skin_changeInterfaceAttachments(int inSegID, std::vector<double> inB0, std::vector<double> inB1, std::vector<bool> inIsActuator, std::vector<bool> inIsSensor) {
    segments[static_cast<unsigned long>(inSegID)].skin_changeInterfaceAttachments(inB0, inB1, inIsActuator, inIsSensor);
}

/********************************
 * skin_getNumInterfaces module
 *
 * This function is used to change the number of interfaces
 * attached to the robotic skin.

 * Inputs: none
 * Output: int
 *
 * Written by Jennifer Case on 5/24/2019
 ********************************/
int wholeSystem::skin_getNumInterfaces(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].skin_getNumInterfaces();
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
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::substrate_changeInitialLength(int inSegID, double inLength) {
    segments[static_cast<unsigned long>(inSegID)].substrate_changeInitialLength(inLength);
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
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::substrate_changeInitialWidth(int inSegID, double inWidth) {
    segments[static_cast<unsigned long>(inSegID)].substrate_changeInitialWidth(inWidth);
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
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::substrate_changeCurrentLength(int inSegID, double inLength) {
    segments[static_cast<unsigned long>(inSegID)].substrate_changeCurrentLength(inLength);
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
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::substrate_changeCurrentWidth(int inSegID, double inWidth) {
    segments[static_cast<unsigned long>(inSegID)].substrate_changeCurrentWidth(inWidth);
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
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::substrate_changeSpringConstants(int inSegID, std::array<double, 2> inConstants) {
    segments[static_cast<unsigned long>(inSegID)].substrate_changeSpringConstants(inConstants);
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
 * Written by Jennifer Case on 5/24/2019
 ********************************/
void wholeSystem::substrate_changeStiffnessConstants(int inSegID, std::array<double, 3> inConstants) {
    segments[static_cast<unsigned long>(inSegID)].substrate_changeStiffnessConstants(inConstants);
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
 * Written by Jennifer Case on 5/24/2019
 ********************************/
double wholeSystem::substrate_getInitialLength(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].substrate_getInitialLength();
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
 * Written by Jennifer Case on 5/24/2019
 ********************************/
double wholeSystem::substrate_getInitialWidth(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].substrate_getInitialWidth();
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
 * Written by Jennifer Case on 5/24/2019
 ********************************/
double wholeSystem::substrate_getCurrentLength(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].substrate_getCurrentLength();
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
 * Written by Jennifer Case on 5/24/2019
 ********************************/
double wholeSystem::substrate_getCurrentWidth(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].substrate_getCurrentWidth();
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
std::array<double, 2> wholeSystem::substrate_getSpringConstants(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].substrate_getSpringConstants();
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
std::array<double, 3> wholeSystem::substrate_getStiffnessConstants(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].substrate_getStiffnessConstants();
}

/********************************
 * substrate_getChangeInLengths module
 *
 * This function allows the user to read the changes in lengths
 * of the substrate.
 * Units: meters (m)
 *
 * Inputs: none
 * Output: array of two doubles ({k_1, k_3})
 *
 * Written by Jennifer Case on 5/25/2019
 ********************************/
std::array<double, 2> wholeSystem::substrate_getChangeInLengths(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].substrate_getChangeInLengths();
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::substrate_getBendingStiffness(int inSegID) {
    return segments[static_cast<unsigned long>(inSegID)].substrate_getBendingStiffness();
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::substrate_getForceVector(int inSegID, Eigen::Ref<Eigen::Vector4d> inVector) {
    segments[static_cast<unsigned long>(inSegID)].substrate_getForceVector(inVector);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::interface_changeIsSensor(int inSegID, int inInterfaceID, bool inBool) {
    segments[static_cast<unsigned long>(inSegID)].interface_changeIsSensor(inInterfaceID, inBool);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::interface_changeIsActuator(int inSegID, int inInterfaceID, bool inBool) {
    segments[static_cast<unsigned long>(inSegID)].interface_changeIsActuator(inInterfaceID, inBool);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::interface_changeSpringConstant(int inSegID, int inInterfaceID, double inConstant) {
    segments[static_cast<unsigned long>(inSegID)].interface_changeSpringConstant(inInterfaceID, inConstant);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::interface_changeSensorActuatorLength(int inSegID, int inInterfaceID, double inLength) {
    segments[static_cast<unsigned long>(inSegID)].interface_changeSensorActuatorLength(inInterfaceID, inLength);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
bool wholeSystem::interface_getIsSensor(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].interface_getIsSensor(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
bool wholeSystem::interface_getIsActuator(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].interface_getIsActuator(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::interface_getForce(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].interface_getForce(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::interface_getForceVector(int inSegID, int inInterfaceID, Eigen::Ref<Eigen::Vector4d> inVector) {
    segments[static_cast<unsigned long>(inSegID)].interface_getForceVector(inInterfaceID, inVector);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::interface_getSpringConstant(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].interface_getSpringConstant(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
std::array<double, 2> wholeSystem::interface_getInterfaceLength(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].interface_getInterfaceLength(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
std::array<double, 2> wholeSystem::interface_getAttachmentConstants(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].interface_getAttachmentConstants(inInterfaceID);
}

/********************************
 * interface_getSensorActuatorLength module
 *
 * This function is allows the user to determine the sensor and actuator length.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::interface_getSensorActuatorLength(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].interface_getSensorActuatorLength(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::actuator_changeUseTheoreticalModel(int inSegID, int inInterfaceID, bool inBool) {
    segments[static_cast<unsigned long>(inSegID)].actuator_changeUseTheoreticalModel(inInterfaceID, inBool);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::actuator_changePressure(int inSegID, int inInterfaceID, double inPressure) {
    segments[static_cast<unsigned long>(inSegID)].actuator_changePressure(inInterfaceID, inPressure);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::actuator_changeInitialLength(int inSegID, int inInterfaceID, double inLength) {
    segments[static_cast<unsigned long>(inSegID)].actuator_changeInitialLength(inInterfaceID, inLength);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::actuator_changeBraidLength(int inSegID, int inInterfaceID, double inLength) {
    segments[static_cast<unsigned long>(inSegID)].actuator_changeBraidLength(inInterfaceID, inLength);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::actuator_changeInitialAngle(int inSegID, int inInterfaceID, double inAngle) {
    segments[static_cast<unsigned long>(inSegID)].actuator_changeInitialAngle(inInterfaceID, inAngle);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::actuator_changeBraidDiameter(int inSegID, int inInterfaceID, double inDiameter) {
    segments[static_cast<unsigned long>(inSegID)].actuator_changeBraidDiameter(inInterfaceID, inDiameter);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::actuator_changeEmpiricalConstants(int inSegID, int inInterfaceID, std::array<double, 5> inConstants) {
    segments[static_cast<unsigned long>(inSegID)].actuator_changeEmpiricalConstants(inInterfaceID, inConstants);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
bool wholeSystem::actuator_getUseTheoreticalModel(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].actuator_getUseTheoreticalModel(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::actuator_getForce(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].actuator_getForce(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::actuator_getPressure(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].actuator_getPressure(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::actuator_getNumberOfTimesFibersWound(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].actuator_getNumberOfTimesFibersWound(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::actuator_getInitialLength(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].actuator_getInitialLength(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::actuator_getCurrentLength(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].actuator_getCurrentLength(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::actuator_getInitialAngle(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].actuator_getInitialAngle(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::actuator_getBraidLength(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].actuator_getBraidLength(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 * Tested on
 ********************************/
double wholeSystem::actuator_getBraidDiameter(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].actuator_getBraidDiameter(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
std::array<double, 5> wholeSystem::actuator_getEmpiricalConstants(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].actuator_getEmpiricalConstants(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::sensor_changePlasticConstants(int inSegID, int inInterfaceID, std::array<double, 2> inConstants) {
    segments[static_cast<unsigned long>(inSegID)].sensor_changePlasticConstants(inInterfaceID, inConstants);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::sensor_changeMaxLength(int inSegID, int inInterfaceID, double inLength) {
    segments[static_cast<unsigned long>(inSegID)].sensor_changeMaxLength(inInterfaceID, inLength);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::sensor_changeLengthInitial(int inSegID, int inInterfaceID, double inlength) {
    segments[static_cast<unsigned long>(inSegID)].sensor_changeLengthInitial(inInterfaceID, inlength);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::sensor_changeWidth(int inSegID, int inInterfaceID, double inWidth) {
    segments[static_cast<unsigned long>(inSegID)].sensor_changeWidth(inInterfaceID, inWidth);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::sensor_changeThickness(int inSegID, int inInterfaceID, double inThickness) {
    segments[static_cast<unsigned long>(inSegID)].sensor_changeThickness(inInterfaceID, inThickness);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::sensor_changeOgdenMu(int inSegID, int inInterfaceID, std::array<double, 3> inMus) {
    segments[static_cast<unsigned long>(inSegID)].sensor_changeOgdenMu(inInterfaceID, inMus);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::sensor_changeOgdenAlpha(int inSegID, int inInterfaceID, std::array<double, 3> inAlphas) {
    segments[static_cast<unsigned long>(inSegID)].sensor_changeOgdenAlpha(inInterfaceID, inAlphas);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
void wholeSystem::sensor_changePoissonRatio(int inSegID, int inInterfaceID, double inPoissonRatio) {
    segments[static_cast<unsigned long>(inSegID)].sensor_changePoissonRatio(inInterfaceID, inPoissonRatio);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::sensor_getForce(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].sensor_getForce(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::sensor_getWidth(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].sensor_getWidth(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::sensor_getThickness(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].sensor_getThickness(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::sensor_getCurrentLength(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].sensor_getCurrentLength(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
std::array<double, 3> wholeSystem::sensor_getOgdenMu(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].sensor_getOgdenMu(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
std::array<double, 3> wholeSystem::sensor_getOgdenAlpha(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].sensor_getOgdenAlpha(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::sensor_getLength0(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].sensor_getLength0(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::sensor_getLengthInitial(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].sensor_getLengthInitial(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::sensor_getLengthPlastic(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].sensor_getLengthPlastic(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::sensor_getStrainPlastic(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].sensor_getStrainPlastic(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::sensor_getStrainMax(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].sensor_getStrainMax(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
std::array<double, 2> wholeSystem::sensor_getPlasticConstants(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].sensor_getPlasticConstants(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::sensor_getMaxLength(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].sensor_getMaxLength(inInterfaceID);
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
 * Written by Jennifer Case on 5/25/2019
 ********************************/
double wholeSystem::sensor_getPoissonRatio(int inSegID, int inInterfaceID) {
    return segments[static_cast<unsigned long>(inSegID)].sensor_getPoissonRatio(inInterfaceID);
}