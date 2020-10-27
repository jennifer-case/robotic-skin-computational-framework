#include "segment.h"
#include <vector>
#include <iostream>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>

/********************************
 * Constructor module
 *
 * The constructor module builds a default segment with a mass of 110 g,
 * gravity of 9.81 m/s^2, weight direction of {0,0,-1}, no force or
 * moment vectors.
 *
 * Inputs: none
 * Output: a segment object
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
segment::segment(int inSize, std::vector<double> inB0, std::vector<double> inB1, std::vector<bool> inIsActuator, std::vector<bool> inIsSensor) : skin(inSize, inB0, inB1, inIsActuator, inIsSensor) {
    changeMass(0.11); // kg
    changeGravity(9.81); // m/s^2

    Eigen::Vector4d tempWeight = { 0,0,-1,0 };
    changeWeightDirectionVector(tempWeight);

    Eigen::Vector4d tempVector = { 0,0,0,0 };
    changeForceVector(tempVector);
    changeMomentVector(tempVector);

    changeError(true);

    // stretch the skin onto the segment
    double length = cylinder_getLength();
    double radius = cylinder_getOuterRadius();
    double circumference = 2 * M_PI * radius;
    substrate_changeCurrentLength(length);
    substrate_changeCurrentWidth(circumference);

    // note that these only grab the values from the cylinder and do not include the affects of adding the skin
    // this will need to be found experimentally, but for purely simulation, we can assume the effects are negligible
    calculateStiffnesses();
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
 * Written by Jennifer Case on 5/6/2019
 ********************************/
segment::~segment() {}

/********************************
 * changeMass module
 *
 * This function is used to change the mass of the segment.
 * Units: kilograms (kg)
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
void segment::changeMass(double inMass) {
    mass = inMass;
}

/********************************
 * changeGravity module
 *
 * This function is used to change the gravity experienced by
 * the segment.
 * Units: m/s^2
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
void segment::changeGravity(double inGravity) {
    gravity = inGravity;
}

/********************************
 * changeWeightDirectionVector module
 *
 * This function is used to change the weight direction
 * of the segment.
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
void segment::changeWeightDirectionVector(Eigen::Ref<Eigen::Vector4d> inVector) {
    for (int i = 0; i < 4; i++) {
        weightDirectionVector(i) = inVector(i);
    }
}

/********************************
 * changeBendingStiffness module
 *
 * This function is used to change the bending stiffness of the segment.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
void segment::changeBendingStiffness(double inStiffness) {
    bendingStiffness = inStiffness;
}

/********************************
 * changeAxialStiffness module
 *
 * This function is used to change the axial stiffness of the segment.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
void segment::changeAxialStiffness(double inStiffness) {
    axialStiffness = inStiffness;
}

/********************************
 * changeTorsionalStiffness module
 *
 * This function is used to change the torsional stiffness of the segment.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double
 * Output: none
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
void segment::changeTorsionalStiffness(double inStiffness) {
    torsionalStiffness = inStiffness;
}

/********************************
 * changeForceVector module
 *
 * This function is used to change the force vector of the segment.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
void segment::changeForceVector(Eigen::Ref<Eigen::Vector4d> inVector) {
    for (int i = 0; i < 4; i++) {
        forceVector(i) = inVector(i);
    }
}

/********************************
 * changeMomentVector module
 *
 * This function is used to change the moment vector of the segment.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on
 ********************************/
void segment::changeMomentVector(Eigen::Ref<Eigen::Vector4d> inVector) {
    for (int i = 0; i < 4; i++) {
        momentVector(i) = inVector(i);
    }
}

/********************************
 * changeError module
 *
 * This function is used to change the error of the segment.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: bool
 * Output: none
 *
 * Written by Jennifer Case on 2/13/2020
 * Tested on 2/13/2020
 ********************************/
void segment::changeError(bool inError) {
    segmentError = inError;
}


/********************************
 * calculateForceVector module
 *
 * This sums together the forces from the weight, interfaces, substrate, and the previous segment to get
 * the total force experienced by this segment
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
void segment::calculateForceVector(Eigen::Ref<Eigen::Vector4d> inPreviousForce) {
    Eigen::Vector4d tempForceVector = { 0,0,0,0 };

    // weight
    double weight = getMass() * getGravity();
    Eigen::Vector4d updatedWeightVector;
    getWeightDirectionVector(updatedWeightVector);
    tempForceVector += -weight * updatedWeightVector;

    // interfaces (sensors & actuators)
    int numInterfaces = skin_getNumInterfaces();
    for (int i = 0; i < numInterfaces; i++) {
        Eigen::Vector4d tempForce;
        interface_getForceVector(i, tempForce);
        tempForceVector += -tempForce;
    }

    // substrate
    Eigen::Vector4d substrateForce;
    substrate_getForceVector(substrateForce);
    tempForceVector(2) += -substrateForce(2);

    // previous segment
    tempForceVector += inPreviousForce;

    changeForceVector(tempForceVector);
}

/********************************
 * calculateMomentVector module
 *
 * This sums together moments generated from the weight, interfaces, and previous segments
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: 4d Eigen vector (force from previous segment), 4d Eigen vector (moment from previous segment)
 * Output: none
 *
 * Written by Jennifer Case on 5/7/2019
 * Tested on 5/7/2019
 ********************************/
void segment::calculateMomentVector(Eigen::Ref<Eigen::Vector4d> inPreviousForce, Eigen::Ref<Eigen::Vector4d> inPreviousMoment) {
    Eigen::Vector4d tempMomentVector = { 0,0,0,0 };

    Eigen::Vector4d currState;
    cylinder_getState(currState);

    // weight
    double weight = getMass() * getGravity();
    Eigen::Vector4d updatedWeightVector;
    getWeightDirectionVector(updatedWeightVector);
    double length = cylinder_getLength();
    Eigen::Matrix4d midPlane;
    cylinder_calculateTransform(0.5 * length, midPlane);
    Eigen::Vector3d dVector;
    dVector(0) = midPlane(0, 3); dVector(1) = midPlane(1, 3); dVector(2) = midPlane(2, 3);
    Eigen::Vector3d weightVector;
    weightVector(0) = updatedWeightVector(0); weightVector(1) = updatedWeightVector(1); weightVector(2) = updatedWeightVector(2);
    Eigen::Vector3d crossProduct;
    crossProduct = dVector.cross(weightVector);
    tempMomentVector(0) += -weight * crossProduct(0);
    tempMomentVector(1) += -weight * crossProduct(1);
    tempMomentVector(2) += -weight * crossProduct(2);

    // interfaces
    int numInterfaces = skin_getNumInterfaces();
    for (int i = 0; i < numInterfaces; i++) {
        std::array<double, 2> attachmentConstants = interface_getAttachmentConstants(i);
        double radius = cylinder_getOuterRadius();
        double force = interface_getForce(i);
        Eigen::Vector3d directionVector = { -attachmentConstants[0] * radius * sin(attachmentConstants[0] * length + attachmentConstants[1]), attachmentConstants[0] * radius * cos(attachmentConstants[0] * length + attachmentConstants[1]), 1 };
        Eigen::Vector3d normalizedDirection = directionVector.normalized();
        Eigen::Vector3d momentArm = { radius * cos(attachmentConstants[0] * length + attachmentConstants[1]), radius * sin(attachmentConstants[0] * length + attachmentConstants[1]), 0 };
        crossProduct = momentArm.cross(normalizedDirection);
        tempMomentVector(0) += -force * crossProduct(0);
        tempMomentVector(1) += -force * crossProduct(1);
        tempMomentVector(2) += -force * crossProduct(2);
    }

    // previous segment - moment
    tempMomentVector += inPreviousMoment;

    // previous segment - force
    Eigen::Matrix4d endPlane;
    cylinder_calculateTransform(length, endPlane);
    Eigen::Vector3d endVector;
    endVector(0) = endPlane(0, 3); endVector(1) = endPlane(1, 3); endVector(2) = endPlane(2, 3);
    Eigen::Vector3d previousForce;
    previousForce(0) = inPreviousForce(0); previousForce(1) = inPreviousForce(1); previousForce(2) = inPreviousForce(2);
    crossProduct = endVector.cross(previousForce);
    tempMomentVector(0) += crossProduct(0);
    tempMomentVector(1) += crossProduct(1);
    tempMomentVector(2) += crossProduct(2);

    changeMomentVector(tempMomentVector);
}

/********************************
 * calculateState module
 *
 * This function is used to calculate the state of the segment.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: 4d Eigen matrix (transform from previous segment), 4d Eigen vector (force from previous segment),
 *      4d Eigen vector (moment from previous segment), 4d Eigen vector (state of this segment)
 * Output: none
 *
 * Written by Jennifer Case on 5/7/2019
 * Tested on 5/7/2019
 ********************************/
void segment::calculateState(Eigen::Ref<Eigen::Matrix4d> inPrevTransform, Eigen::Ref<Eigen::Vector4d> inPreviousForce, Eigen::Ref<Eigen::Vector4d> inPreviousMoment, Eigen::Ref<Eigen::Vector4d> inState) {
    Eigen::Matrix4d endCapTransform;
    Eigen::Matrix4d cylinderTransform;
    double length = cylinder_getLength();
    endcap_getTransform(1, endCapTransform);
    cylinder_calculateTransform(length, cylinderTransform);
    Eigen::Matrix4d totalTransform = cylinderTransform * endCapTransform * inPrevTransform;
    Eigen::Vector4d adjustedPrevForce = totalTransform * inPreviousForce;
    Eigen::Vector4d adjustedPrevMoment = totalTransform * inPreviousMoment;

    calculateForceVector(adjustedPrevForce);
    calculateMomentVector(adjustedPrevForce, adjustedPrevMoment);

    Eigen::Vector4d force;
    Eigen::Vector4d moment;
    getForceVector(force);
    getMomentVector(moment);
    double bStiff = getBendingStiffness();
    double aStiff = getAxialStiffness();
    double tStiff = getTorsionalStiffness();

    inState(0) = moment(1) / bStiff;
    inState(1) = -moment(0) / bStiff;
    inState(2) = 1 - force(2) / aStiff;
    inState(3) = length * moment(2) / tStiff;
}

/********************************
 * optimizeState module
 *
 * This function optimizes for state given the parameters of the segment
 *
 * Inputs: 4d Eigen matrix (transform from previous segment), 4d Eigen vector (force from previous segment),
 *      4d Eigen vector (moment from previous segment)
 * Output: none
 *
 * Written by Jennifer Case on 5/13/2019
 * Tested on 5/13/2019
 * Modified on 2/13/2020 to track error in the segment
 ********************************/
void segment::optimizeState(Eigen::Ref<Eigen::Matrix4d> inPrevTransform, Eigen::Ref<Eigen::Vector4d> inPreviousForce, Eigen::Ref<Eigen::Vector4d> inPreviousMoment) {

    Eigen::Vector4d prevError;
    Eigen::Vector4d currError;
    Eigen::Vector4d prevState;
    cylinder_getState(prevState);
    Eigen::Vector4d currState;
    cylinder_getState(currState);
    prevState += 0.1 * Eigen::Vector4d::Ones();
    Eigen::Vector4d predictedState;
    Eigen::Vector4d tempState;
    Eigen::Vector4d diff;
    Eigen::Vector4d bestState;
    double bestVal = 10000;

    deformSegment(prevState);
    calculateState(inPrevTransform, inPreviousForce, inPreviousMoment, predictedState);
    prevError = predictedState - prevState;

    deformSegment(currState);

    int cnt = 0;
    bool isError = false;
    changeError(isError); // start optimization with no error

    while (true) {
        cnt++;
        isError = getError();
        if (cnt > 500 || isError) {
            if (isError) {
                std::cout << "Error Triggered" << std::endl;
                break;
            }
            std::cout << "Failed to optimize state" << std::endl;
            deformSegment(bestState);
            changeError(true);
            break;
        }

        calculateState(inPrevTransform, inPreviousForce, inPreviousMoment, predictedState);

        currError = predictedState - currState;

        double sum = 0;
        for (int i = 0; i < 4; i++) {
            sum += fabs(currError(i));
        }
        if (sum < bestVal) {
            bestState = currState;
            bestVal = sum;
        }
        if (sum < 1e-2) {
            break;
        }

        diff = currError - prevError;
        for (int i = 0; i < 4; i++) {
            if (fabs(diff(i)) < 1e-3) {
                if (diff(i) < 0) { diff(i) = -1e-3; }
                else { diff(i) = 1e-3; }
            }
        }

        for (int i = 0; i < 4; i++) {
            tempState(i) = currState(i) + 0.1 * currError(i);
        }

        prevState = currState;
        currState = tempState;
        prevError = currError;
        deformSegment(currState);
    }

}

/********************************
 * optimizePressure module
 *
 * This function optimizes for actuator pressures given the parameters of the segment
 *
 * Inputs: 4d Eigen matrix (transform from previous segment), 4d Eigen vector (force from previous segment),
 *      4d Eigen vector (moment from previous segment), 4d Eigen vector (desired state for the segment)
 * Output: none
 *
 * Written by Jennifer Case on 5/13/2019
 * Tested on
 ********************************/
void segment::optimizePressure(Eigen::Ref<Eigen::Matrix4d> inPrevTransform, Eigen::Ref<Eigen::Vector4d> inPreviousForce, Eigen::Ref<Eigen::Vector4d> inPreviousMoment, Eigen::Ref<Eigen::Vector4d> inDesiredState) {

    int numInterfaces = skin_getNumInterfaces();
    int numActuators = 0;
    std::vector<int> actuatorIdx;
    for (int i = 0; i < numInterfaces; i++) {
        if (interface_getIsActuator(i)) {
            numActuators++;
            actuatorIdx.push_back(i);
        }
    }

    std::vector<double> currPressures(static_cast<unsigned long>(numActuators));
    std::vector<double> bestPressures(static_cast<unsigned long>(numActuators));
    double bestError = 100;
    std::vector<double> inverseJacobian(static_cast<unsigned long>(numActuators));
    Eigen::Vector4d currState;
    Eigen::Vector4d offsetState;
    Eigen::Vector4d stateError;
    double prevError = 0;

    // get current pressures
    for (int i = 0; i < numActuators; i++) {
        currPressures[static_cast<unsigned long>(i)] = actuator_getPressure(i);
    }

    int cnt = 0;
    bool actuatorCapped = false;
    while (true) {
        cnt++;
        if (cnt > 100) {
            std::cout << "Failed to optimize pressure" << std::endl;
            for (int j = 0; j < numActuators; j++) {
                actuator_changePressure(j, bestPressures[static_cast<unsigned long>(j)]);
            }
            break;
        }
        std::cout << "Optimize Pressure: " << cnt << std::endl;
        // get the current state
        optimizeState(inPrevTransform, inPreviousForce, inPreviousMoment);
        cylinder_getState(currState);
        std::cout << "State (" << cnt << "): " << std::endl << currState << std::endl;

        //get the error
        stateError = inDesiredState - currState;
        double summedError = 0;
        for (int i = 0; i < 4; i++) {
            if (inDesiredState(i) < 999) { summedError += fabs(stateError(i)); }
        }
        std::cout << "Summed Error (" << cnt << "): " << summedError << std::endl;
        if (summedError < bestError) {
            bestError = summedError;
            bestPressures = currPressures;
        }

        // see if it is time to exit the loop
        if (fabs(summedError - prevError) < 1e-4 || summedError < 1e-1 || (actuatorCapped && fabs(summedError - prevError) < 0.06)) {
            std::cout << "Exiting..." << std::endl;
            break;
        }

        // get the inverse Jacobian
        for (int i = 0; i < numActuators; i++) {
            double h = 1000; // slightly less than 0.25 psi
            // get the offset state
            // first, we need to adjust all the pressures
            for (int j = 0; j < numActuators; j++) {
                actuator_changePressure(j, currPressures[static_cast<unsigned long>(j)]);
            }
            // next, adjust the pressure we want to change (i.e. the ith pressure)
            actuator_changePressure(i, currPressures[static_cast<unsigned long>(i)] + h);
            double actuatorForce = actuator_getForce(i);
            while (currPressures[static_cast<unsigned long>(i)] + h > 0 && actuatorForce <= 0) {
                h += 1000;
                actuator_changePressure(i, currPressures[static_cast<unsigned long>(i)] + h);
                actuatorForce = actuator_getForce(i);
            }
            // now we can calculate the state
            optimizeState(inPrevTransform, inPreviousForce, inPreviousMoment);
            cylinder_getState(offsetState);

            // get the error from the offset state
            stateError = inDesiredState - offsetState;
            double offsetError = 0;
            for (int j = 0; j < 4; j++) {
                if (inDesiredState(j) < 999) { offsetError += fabs(stateError(j)); }
            }

            // get the initial Jacobian value
            double initialJacobian = (offsetError - summedError) / h;
            if (fabs(initialJacobian) < 1e-5) {
                if (initialJacobian < 0) { initialJacobian = -1e-5; }
                else { initialJacobian = 1e-5; }
            }

            // plug into inverse Jacobian
            inverseJacobian[static_cast<unsigned long>(i)] = 1.0 / initialJacobian;
        }
        std::cout << std::endl;

        // calculate the new pressures
        std::cout << "Pressures (" << cnt << "): ";
        for (int i = 0; i < numActuators; i++) {
            currPressures[static_cast<unsigned long>(i)] = currPressures[static_cast<unsigned long>(i)] - 0.1 * inverseJacobian[static_cast<unsigned long>(i)] * summedError;

            // adjust pressure to be within boundaries
            if (currPressures[static_cast<unsigned long>(i)] < 0) { currPressures[static_cast<unsigned long>(i)] = 0; }
            if (currPressures[static_cast<unsigned long>(i)] > 172375) { currPressures[static_cast<unsigned long>(i)] = 172375; actuatorCapped = true; }

            // adjust the pressure in the actuator
            actuator_changePressure(i, currPressures[static_cast<unsigned long>(i)]);
            // check that calculated force is reasonable
            double actuatorForce = actuator_getForce(i);
            while (currPressures[static_cast<unsigned long>(i)] > 0 && actuatorForce <= 0) {
                currPressures[static_cast<unsigned long>(i)] += 100.0;
                actuator_changePressure(i, currPressures[static_cast<unsigned long>(i)]);
                actuatorForce = actuator_getForce(i);
            }
            std::cout << currPressures[static_cast<unsigned long>(i)] / 6895.0 << " ";
        }
        std::cout << std::endl;

        prevError = summedError;
    }
}


/********************************
 * deformSegment module
 *
 * This function is used to deform the segment.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 5/7/2019
 * Tested on 5/13/2019
 ********************************/
void segment::deformSegment(Eigen::Ref<Eigen::Vector4d> inState) {
    cylinder_changeState(inState);

    int numInterfaces = skin_getNumInterfaces();
    for (int i = 0; i < numInterfaces; i++) {
        optimizeLength(i);
    }
}

/********************************
 * getPointLocationsForLength module
 *
 * This function is used to get points for measure the lengths along the actuators and sensors.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double (length along the cylindrical structure), array of two doubles (attachment constants),
 *      3d Eigen vector (point location)
 * Output: none
 *
 * Written by Jennifer Case on 5/13/2019
 * Tested on 5/13/2019
 ********************************/
void segment::getPointLocationsForLength(double inH, std::array<double, 2> inAttachmentConstants, Eigen::Ref<Eigen::Vector3d> inOutputVector) {
    Eigen::Matrix4d transformation;
    cylinder_calculateTransform(inH, transformation);

    double radius = cylinder_getOuterRadius();

    Eigen::Vector4d spaceCurvePoint = { 0,0,0,1 };
    spaceCurvePoint(0) = radius * cos(inAttachmentConstants[0] * inH + inAttachmentConstants[1]);
    spaceCurvePoint(1) = radius * sin(inAttachmentConstants[0] * inH + inAttachmentConstants[1]);
    spaceCurvePoint(2) = 0;

    Eigen::Vector4d deformedCurve;
    deformedCurve = transformation * spaceCurvePoint;

    inOutputVector(0) = deformedCurve(0);
    inOutputVector(1) = deformedCurve(1);
    inOutputVector(2) = deformedCurve(2);
}

/********************************
 * getNormalizedrPrime module
 *
 * This function returns the normalized r' vector.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: double, double, array of two doubles (attachment constants)
 * Output: none
 *
 * Written by Jennifer Case on 5/7/2019
 * Tested on 5/13/2019
 ********************************/
double segment::getNormalizedrPrime(double inH0, double inH1, std::array<double, 2> inAttachmentConstants) {
    Eigen::Vector3d f0;
    Eigen::Vector3d f1;

    getPointLocationsForLength(inH0, inAttachmentConstants, f0);
    getPointLocationsForLength(inH1, inAttachmentConstants, f1);

    Eigen::Vector3d fPrime;
    fPrime = (f1 - f0) / (inH1 - inH0);

    double output = fPrime.norm();
    return output;
}

/********************************
 * getLength module
 *
 * This function returns the length of a sensor or actuator.
 *
 * This is a private function that is not accessible to users.
 *
 * Inputs: int, array of two doubles (attachment constants)
 * Output: none
 *
 * Written by Jennifer Case on 5/7/2019
 * Tested on 5/13/2019
 ********************************/
double segment::getLength(int inNum, std::array<double, 2> inAttachmentConstants) {
    double length = cylinder_getLength();
    //std::cout << length << std::endl;

    std::vector<double> h(static_cast<unsigned long>(inNum + 1));
    for (int i = 0; i < inNum + 1; i++) {
        h[static_cast<unsigned long>(i)] = i * length / inNum;
    }

    std::vector<double> sols(static_cast<unsigned long>(inNum));
    for (int i = 0; i < inNum; i++) {
        sols[static_cast<unsigned long>(i)] = getNormalizedrPrime(h[static_cast<unsigned long>(i)], h[static_cast<unsigned long>(i + 1)], inAttachmentConstants);
    }

    double output;
    output = (length / static_cast<double>(inNum)) * (sols[0] + sols[static_cast<unsigned long>(inNum - 1)]) / 2.0;

    for (int i = 1; i < inNum - 1; i++) {
        output += (length / static_cast<double>(inNum)) * sols[static_cast<unsigned long>(i)];
    }

    return output;
}

/********************************
 * optimizeLength module
 *
 * This function optimizes the length of the the sensors and actuators according
 * to their exerted forces and the state of the segment.
 *
 * Inputs: int (interface index number)
 * Output: none
 *
 * Written by Jennifer Case on 5/7/2019
 * Tested on 5/13/2019
 ********************************/
void segment::optimizeLength(int interfaceIdx) {
    int n = 500;
    std::array<double, 2> inAttachmentConstants = interface_getAttachmentConstants(interfaceIdx);
    double deformedLength = getLength(n, inAttachmentConstants);
    double currLength = deformedLength;
    std::array<double, 2> intLengths;
    double interfaceLength = 0;
    double error = 0;
    double offsetError = 0;
    double jacobian = 0;

    int cnt = 0;
    while (true) {
        cnt++;
        if (cnt > 300) {
            std::cout << "Failed to optimize length" << std::endl;
            changeError(true);
            break;
        }

        interface_changeSensorActuatorLength(interfaceIdx, currLength);
        intLengths = interface_getInterfaceLength(interfaceIdx);

        interfaceLength = intLengths[1];
        error = deformedLength - (currLength + interfaceLength);

        if (fabs(error) < 1e-3) {
            break;
        }

        // calculate the Jacobian
        double h = 0.0005; // 1/2 millimeter
        interface_changeSensorActuatorLength(interfaceIdx, currLength + h);

        double offsetInterface = 0;
        intLengths = interface_getInterfaceLength(interfaceIdx);

        offsetInterface = intLengths[1];

        offsetError = deformedLength - (currLength + h + offsetInterface);
        jacobian = (offsetError - error) / h;

        if (fabs(jacobian) < 1e-3) {
            if (jacobian < 0) { jacobian = -1e-3; }
            else { jacobian = 1e-3; }
        }

        currLength = currLength - 0.05 * error / jacobian;
    }
}

/********************************
 * stateEstimation module
 *
 * This function determines the state of the segment given the reported
 * sensor lengths.
 *
 * Inputs: vector of doubles (reported sensor lengths)
 * Output: none
 *
 * Written by Jennifer Case on 3/2/2020
 * Tested on 3/15/2020
 ********************************/
void segment::stateEstimation(std::vector<double> reportedSensorLengths) {
    double currError, bestError, tempError;
    Eigen::Vector4d currState, tempState;
    cylinder_getState(currState);
    deformSegment(currState);
    Eigen::Vector4d bestState;

    int numInterfaces = skin_getNumInterfaces();

    // calculate Error
    currError = 0;
    int sensorCnt = 0;
    for (int i = 0; i < numInterfaces; i++) {
        if (interface_getIsSensor(i)) {
            if (reportedSensorLengths[static_cast<unsigned long>(sensorCnt)] > 0.050) {
                currError += pow(reportedSensorLengths[static_cast<unsigned long>(sensorCnt)] - sensor_getCurrentLength(i), 2.0);
            }
            sensorCnt++;
        }
    }
    bestError = currError;
    bestState = currState;
    tempState = currState;

    int cnt = 0;
    bool madeChange = true;
    while (true) {
        if (bestError < 1e-7 || cnt > 10000 || !madeChange) {
            deformSegment(bestState);
            break;
        }

        tempState = currState;

        std::array<double, 4> hSteps = { 0.05,0.05,0.0025,0.025 };
        std::array<double, 4> changeLimit = { 1,1,0.01,0.25 };
        std::array<double, 4> a = { 100000,100000,10,10 };
        madeChange = false;
        // cycle through all the states
        for (int j = 0; j < 4; j++) {
            tempState(j) += hSteps[static_cast<unsigned long>(j)];
            deformSegment(tempState);

            sensorCnt = 0;
            tempError = 0;
            for (int i = 0; i < numInterfaces; i++) {
                if (interface_getIsSensor(i)) {
                    if (reportedSensorLengths[static_cast<unsigned long>(sensorCnt)] > 0.050) {
                        tempError += pow(reportedSensorLengths[static_cast<unsigned long>(sensorCnt)] - sensor_getCurrentLength(i), 2.0);
                    }
                    sensorCnt++;
                }
            }

            double changeVal = -a[static_cast<unsigned long>(j)] * (tempError - currError) / (tempState(j) - currState(j));
            if (std::abs(changeVal) > changeLimit[static_cast<unsigned long>(j)]) {
                if (changeVal > 0) { changeVal = changeLimit[static_cast<unsigned long>(j)]; }
                else { changeVal = -changeLimit[static_cast<unsigned long>(j)]; }
            }

            tempState(j) = currState(j) + changeVal;
            deformSegment(tempState);

            sensorCnt = 0;
            tempError = 0;
            for (int i = 0; i < numInterfaces; i++) {
                if (interface_getIsSensor(i)) {
                    if (reportedSensorLengths[static_cast<unsigned long>(sensorCnt)] > 0.050) {
                        tempError += pow(reportedSensorLengths[static_cast<unsigned long>(sensorCnt)] - sensor_getCurrentLength(i), 2.0);
                    }
                    sensorCnt++;
                }
            }
            if (tempError < currError) {
                currError = tempError;
                currState = tempState;
                madeChange = true;
            }
            else {
                tempState = currState; // return to better values
            }
        }

        if (currError < bestError) {
            bestError = currError;
            bestState = currState;
        }
        cnt++;
    }
}

/********************************
 * getMass module
 *
 * This function allows users to read the mass of the segment.
 * Units: kilograms (kg)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
double segment::getMass() {
    return mass;
}

/********************************
 * getGravity module
 *
 * This function allows users to read the gravity experienced by the segment.
 * Units: m/s^2
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
double segment::getGravity() {
    return gravity;
}

/********************************
 * getWeightDirectionVector module
 *
 * This function allows users to read the direction of the weight vector.
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
void segment::getWeightDirectionVector(Eigen::Ref<Eigen::Vector4d> inVector) {
    for (int i = 0; i < 4; i++) {
        inVector(i) = weightDirectionVector(i);
    }
}

/********************************
 * calculateStiffnesses module
 *
 * This function is used to calculate the stiffnesses of the segments.
 *
 * Inputs: none
 * Output: none
 *
 * Written by Jennifer Case on 5/24/2019
 * Tested on 5/24/2019
 ********************************/
void segment::calculateStiffnesses() {
    double temp = cylinder_getBendingStiffness();
    double skinTemp = substrate_getBendingStiffness();
    changeBendingStiffness(temp + skinTemp);
    temp = cylinder_getAxialStiffness();
    changeAxialStiffness(temp);
    temp = cylinder_getTorsionalStiffness();
    changeTorsionalStiffness(temp);
}

/********************************
 * getBendingStiffness module
 *
 * This function allows users to read the bending stiffness of the segment.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
double segment::getBendingStiffness() {
    return bendingStiffness;
}

/********************************
 * getAxialStiffness module
 *
 * This function allows users to read the axial stiffness of the segment.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
double segment::getAxialStiffness() {
    return axialStiffness;
}

/********************************
 * getTorsionalStiffness module
 *
 * This function allows users to read the torsional stiffness of the segment.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
double segment::getTorsionalStiffness() {
    return torsionalStiffness;
}

/********************************
 * getForceVector module
 *
 * This function allows users to read the force vector of the segment.
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
void segment::getForceVector(Eigen::Ref<Eigen::Vector4d> inVector) {
    for (int i = 0; i < 4; i++) {
        inVector(i) = forceVector(i);
    }
}

/********************************
 * getMomentVector module
 *
 * This function allows users to read the moment vector of the segment.
 *
 * Inputs: 4d Eigen vector
 * Output: none
 *
 * Written by Jennifer Case on 5/6/2019
 * Tested on 5/7/2019
 ********************************/
void segment::getMomentVector(Eigen::Ref<Eigen::Vector4d> inVector) {
    for (int i = 0; i < 4; i++) {
        inVector(i) = momentVector(i);
    }
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
bool segment::getError() {
    return segmentError;
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::cylinder_changeOuterRadius(double inRadius) {
    cylinder.changeOuterRadius(inRadius);
    calculateStiffnesses();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::cylinder_changeInnerRadius(double inRadius) {
    cylinder.changeInnerRadius(inRadius);
    calculateStiffnesses();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::cylinder_changeLength(double inLength) {
    cylinder.changeLength(inLength);

    int numInterfaces = skin_getNumInterfaces();
    for (int i = 0; i < numInterfaces; i++) {
        interface_changeSensorActuatorLength(i, inLength);
    }
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::cylinder_changeElasticModulus(double inModulus) {
    cylinder.changeElasticModulus(inModulus);
    calculateStiffnesses();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::cylinder_changePoissonRatio(double inPoissonRatio) {
    cylinder.changePoissonRatio(inPoissonRatio);
    calculateStiffnesses();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::cylinder_changeState(Eigen::Ref<Eigen::Vector4d> inState) {
    cylinder.changeState(inState);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::cylinder_calculateTransform(double inLength, Eigen::Ref<Eigen::Matrix4d> inMatrix) {
    cylinder.calculateTransform(inLength, inMatrix);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::cylinder_getOuterRadius() {
    return cylinder.getOuterRadius();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::cylinder_getInnerRadius() {
    return cylinder.getInnerRadius();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::cylinder_getLength() {
    return cylinder.getLength();
}

/********************************
 * cylinder_changeElasticModulus module
 *
 * This function allows the user to read the elastic modulus of the
 * cylindrical structure.
 * Units: Pascals (Pa)
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::cylinder_getElasticModulus() {
    return cylinder.getElasticModulus();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::cylinder_getPoissonRatio() {
    return cylinder.getPoissonRatio();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::cylinder_getState(Eigen::Ref<Eigen::Vector4d> inVector) {
    cylinder.getState(inVector);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::cylinder_getBendingStiffness() {
    return cylinder.getBendingStiffness();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::cylinder_getAxialStiffness() {
    return cylinder.getAxialStiffness();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::cylinder_getTorsionalStiffness() {
    return cylinder.getTorsionalStiffness();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::cylinder_getBucklingForce() {
    return cylinder.getBucklingForce();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::endcap_changeLength(int inNum, double inLength) {
    endCaps[static_cast<unsigned long>(inNum)].changeLength(inLength);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::endcap_getLength(int inNum) {
    return endCaps[static_cast<unsigned long>(inNum)].getLength();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::endcap_getTransform(int inNum, Eigen::Ref<Eigen::Matrix4d> inMatrix) {
    endCaps[static_cast<unsigned long>(inNum)].getTransform(inMatrix);
}

/********************************
 * skin_changeInterfaceAttachements module
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::skin_changeInterfaceAttachments(std::vector<double> inB0, std::vector<double> inB1, std::vector<bool> inIsActuator, std::vector<bool> inIsSensor) {
    skin.changeInterfaceAttachments(inB0, inB1, inIsActuator, inIsSensor);
}

/********************************
 * skin_getNumInterfaces module
 *
 * This function is used to change the number of interfaces
 * attached to the robotic skin.

 * Inputs: none
 * Output: int
 *
 * Written by Jennifer Case on 5/23/2019
 ********************************/
int segment::skin_getNumInterfaces() {
    return skin.getNumInterfaces();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::substrate_changeInitialLength(double inLength) {
    skin.substrate_changeInitialLength(inLength);
    calculateStiffnesses();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::substrate_changeInitialWidth(double inWidth) {
    skin.substrate_changeInitialWidth(inWidth);
    calculateStiffnesses();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::substrate_changeCurrentLength(double inLength) {
    skin.substrate_changeCurrentLength(inLength);
    calculateStiffnesses();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::substrate_changeCurrentWidth(double inWidth) {
    skin.substrate_changeCurrentWidth(inWidth);
    calculateStiffnesses();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::substrate_changeSpringConstants(std::array<double, 2> inConstants) {
    skin.substrate_changeSpringConstants(inConstants);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::substrate_changeStiffnessConstants(std::array<double, 3> inConstants) {
    skin.substrate_changeStiffnessConstants(inConstants);
    calculateStiffnesses();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::substrate_getInitialLength() {
    return skin.substrate_getInitialLength();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::substrate_getInitialWidth() {
    return skin.substrate_getInitialWidth();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::substrate_getCurrentLength() {
    return skin.substrate_getCurrentLength();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::substrate_getCurrentWidth() {
    return skin.substrate_getCurrentWidth();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
std::array<double, 2> segment::substrate_getSpringConstants() {
    return skin.substrate_getSpringConstants();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
std::array<double, 3> segment::substrate_getStiffnessConstants() {
    return skin.substrate_getStiffnessConstants();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
std::array<double, 2> segment::substrate_getChangeInLengths() {
    return skin.substrate_getChangeInLengths();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::substrate_getBendingStiffness() {
    return skin.substrate_getBendingStiffness();
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::substrate_getForceVector(Eigen::Ref<Eigen::Vector4d> inVector) {
    skin.substrate_getForceVector(inVector);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::interface_changeIsSensor(int inNum, bool inBool) {
    skin.interface_changeIsSensor(inNum, inBool);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::interface_changeIsActuator(int inNum, bool inBool) {
    skin.interface_changeIsActuator(inNum, inBool);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::interface_changeSpringConstant(int inNum, double inConstant) {
    skin.interface_changeSpringConstant(inNum, inConstant);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::interface_changeSensorActuatorLength(int inNum, double inLength) {
    skin.interface_changeSensorActuatorLength(inNum, inLength);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
bool segment::interface_getIsSensor(int inNum) {
    return skin.interface_getIsSensor(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
bool segment::interface_getIsActuator(int inNum) {
    return skin.interface_getIsActuator(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::interface_getForce(int inNum) {
    return skin.interface_getForce(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::interface_getForceVector(int inNum, Eigen::Ref<Eigen::Vector4d> inVector) {
    double radius = cylinder_getOuterRadius();
    double length = cylinder_getLength();
    skin.interface_getForceVector(inNum, inVector, radius, length);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::interface_getSpringConstant(int inNum) {
    return skin.interface_getSpringConstant(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
std::array<double, 2> segment::interface_getInterfaceLength(int inNum) {
    return skin.interface_getInterfaceLength(inNum);
}

/********************************
 * interface_getAttachmentConstant module
 *
 * This function is allows the user to determine the attachment constants of
 * the interface.
 *
 * Inputs: none
 * Output: array of two doubles ({beta_0, beta_1})
 *
 * Written by Jennifer Case on 5/23/2019
 ********************************/
std::array<double, 2> segment::interface_getAttachmentConstants(int inNum) {
    return skin.interface_getAttachmentConstants(inNum);
}

/********************************
 * interface_getSensorActuatorLength module
 *
 * This function is allows the user to determine the sensor and actuator length.
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::interface_getSensorActuatorLength(int inNum) {
    return skin.interface_getSensorActuatorLength(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::actuator_changeUseTheoreticalModel(int inNum, bool inBool) {
    skin.actuator_changeUseTheoreticalModel(inNum, inBool);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::actuator_changePressure(int inNum, double inPressure) {
    skin.actuator_changePressure(inNum, inPressure);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::actuator_changeInitialLength(int inNum, double inLength) {
    skin.actuator_changeInitialLength(inNum, inLength);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::actuator_changeInitialAngle(int inNum, double inAngle) {
    skin.actuator_changeInitialAngle(inNum, inAngle);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::actuator_changeBraidLength(int inNum, double inLength) {
    skin.actuator_changeBraidLength(inNum, inLength);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::actuator_changeBraidDiameter(int inNum, double inDiameter) {
    skin.actuator_changeBraidDiameter(inNum, inDiameter);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::actuator_changeEmpiricalConstants(int inNum, std::array<double, 5> inConstants) {
    skin.actuator_changeEmpiricalConstants(inNum, inConstants);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
bool segment::actuator_getUseTheoreticalModel(int inNum) {
    return skin.actuator_getUseTheoreticalModel(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::actuator_getForce(int inNum) {
    return skin.actuator_getForce(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::actuator_getPressure(int inNum) {
    return skin.actuator_getPressure(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::actuator_getNumberOfTimesFibersWound(int inNum) {
    return skin.actuator_getNumberOfTimesFibersWound(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::actuator_getInitialLength(int inNum) {
    return skin.actuator_getInitialLength(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::actuator_getCurrentLength(int inNum) {
    return skin.actuator_getCurrentLength(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::actuator_getInitialAngle(int inNum) {
    return skin.actuator_getInitialAngle(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::actuator_getBraidLength(int inNum) {
    return skin.actuator_getBraidLength(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 * Tested on
 ********************************/
double segment::actuator_getBraidDiameter(int inNum) {
    return skin.actuator_getBraidDiameter(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
std::array<double, 5> segment::actuator_getEmpiricalConstants(int inNum) {
    return skin.actuator_getEmpiricalConstants(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::sensor_changePlasticConstants(int inNum, std::array<double, 2> inConstants) {
    skin.sensor_changePlasticConstants(inNum, inConstants);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::sensor_changeMaxLength(int inNum, double inLength) {
    skin.sensor_changeMaxLength(inNum, inLength);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::sensor_changeLengthInitial(int inNum, double inLength) {
    skin.sensor_changeLengthInitial(inNum, inLength);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::sensor_changeWidth(int inNum, double inWidth) {
    skin.sensor_changeWidth(inNum, inWidth);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::sensor_changeThickness(int inNum, double inThickness) {
    skin.sensor_changeThickness(inNum, inThickness);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::sensor_changeOgdenMu(int inNum, std::array<double, 3> inMu) {
    skin.sensor_changeOgdenMu(inNum, inMu);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::sensor_changeOgdenAlpha(int inNum, std::array<double, 3> inAlpha) {
    skin.sensor_changeOgdenAlpha(inNum, inAlpha);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
void segment::sensor_changePoissonRatio(int inNum, double inPoissonRatio) {
    skin.sensor_changePoissonRatio(inNum, inPoissonRatio);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::sensor_getForce(int inNum) {
    return skin.sensor_getForce(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::sensor_getWidth(int inNum) {
    return skin.sensor_getWidth(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::sensor_getThickness(int inNum) {
    return skin.sensor_getThickness(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::sensor_getCurrentLength(int inNum) {
    return skin.sensor_getCurrentLength(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
std::array<double, 3> segment::sensor_getOgdenMu(int inNum) {
    return skin.sensor_getOgdenMu(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
std::array<double, 3> segment::sensor_getOgdenAlpha(int inNum) {
    return skin.sensor_getOgdenAlpha(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::sensor_getLength0(int inNum) {
    return skin.sensor_getLength0(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::sensor_getLengthInitial(int inNum) {
    return skin.sensor_getLengthInitial(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::sensor_getLengthPlastic(int inNum) {
    return skin.sensor_getLengthPlastic(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::sensor_getStrainPlastic(int inNum) {
    return skin.sensor_getStrainPlastic(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::sensor_getStrainMax(int inNum) {
    return skin.sensor_getStrainMax(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
std::array<double, 2> segment::sensor_getPlasticConstants(int inNum) {
    return skin.sensor_getPlasticConstants(inNum);
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
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::sensor_getMaxLength(int inNum) {
    return skin.sensor_getMaxLength(inNum);
}

/********************************
 * sensor_PoissonRatio module
 *
 * This function allows the user to read the Poisson ratio of the material.
 * Units: unitless
 *
 * Inputs: none
 * Output: double
 *
 * Written by Jennifer Case on 5/23/2019
 ********************************/
double segment::sensor_getPoissonRatio(int inNum) {
    return skin.sensor_getPoissonRatio(inNum);
}