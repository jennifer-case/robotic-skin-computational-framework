#include <iostream>
#include <fstream>
#define _USE_MATH_DEFINES
#include <cmath>
#include <math.h>
#include <random>
#include <numeric>
#include "sensor.h"
#include "actuator.h"
#include "interface.h"
#include "substrate.h"
#include "roboticskin.h"
#include "endcap.h"
#include "cylindricalstructure.h"
#include "segment.h"
#include "wholesystem.h"
#include <Eigen/Dense>

using namespace std;

void readPSFile(string, std::array<std::array<double, 5>, 32>&);
void processSimulationPoints(std::array<std::array<double, 3>, 4>&, std::array<std::array<double, 3>, 4>&, wholeSystem&);
void getInitPoints(std::array<std::array<double, 5>, 32>&, std::array<std::array<double, 3>, 32>&);
double calculateError(std::array<std::array<double, 3>, 4>&, std::array<std::array<double, 3>, 4>&, int&);
void runAnalysis(wholeSystem&, std::array<std::array<double, 5>, 32>&, std::array<std::array<double, 5>, 32>&, std::array<std::array<double, 5>, 32>&, std::array<std::array<double, 5>, 32>&, std::array<std::array<double, 3>, 32>&, std::array<std::array<double, 3>, 32>&, std::array<std::array<double, 3>, 32>&, std::array<std::array<double, 3>, 32>&, std::array<double, 28>&, std::array<double, 28>&, std::array<std::array<double, 4>, 28>&, std::array<double, 28>&, std::array<int, 28>&);

int main() {
    // When making the segments, the interfaces must first be defined.
    // We will define an example skin with 4 interfaces with both actuators and sensors
    // using the twisting skin design since this is the more complicated of the skins.
    int numInterfaces = 4;
    double l = 0.1; // the length of the segment is 0.1 m (or 100 mm)
    vector<double> b0(static_cast<unsigned long>(numInterfaces)); // define a vector for b_0 that is the length of the number of interfaces
    b0[0] = -M_PI / l; b0[1] = M_PI / l; b0[2] = -M_PI / l; b0[3] = M_PI / l; // define the b_0 values for each interface
    vector<double> b1(static_cast<unsigned long>(numInterfaces)); // define a vector for b_1 that is the length of the number of interfaces
    b1[0] = M_PI; b1[1] = M_PI; b1[2] = 0; b1[3] = 0; // define the b_1 values for each interface
    vector<bool> actuators(static_cast<unsigned long>(numInterfaces)); // define the boolean vector describing actuator attachment to the interfaces
    for (int i = 0; i < 4; i++) { actuators[static_cast<unsigned long>(i)] = true; } // place an actuator at each interface
    vector<bool> sensors(static_cast<unsigned long>(numInterfaces)); // define the boolean vector describing sensor attachment to the interfaces
    for (int i = 0; i < 4; i++) { sensors[static_cast<unsigned long>(i)] = true; } // place a sensor at each interface
    segment testSeg(numInterfaces, b0, b1, actuators, sensors); // generate a segment using these interface parameters

    // We can change parameters of the system either using the segment class or the whole system class.
    // Here, we will show how to adjust actuators from both.
    // We'll start with the segment class.
    array<double, 5> tempActConstants = { 204.0, -6.97e-4, -4.25e3, 8.19e-3, 2.15e4 }; // define approximate actuator constants for a 140 mm long actuator described in the paper
    // We are going to assume that all actuators have these same parameters
    for (int i = 0; i < 4; i++) {
        testSeg.actuator_changeEmpiricalConstants(i, tempActConstants); // change the ith actuator to have these constants
        testSeg.actuator_changeUseTheoreticalModel(i, false); // change the ith actuator to use the empirical model
        testSeg.actuator_changeInitialLength(i, 0.14); // change the ith actuator length to 0.14 m (or 140 mm)
    }
    // We must put together the whole system first before we show how the actuators are adjusted
    std::vector<segment> segments; // Make a vector of segments
    segments.push_back(testSeg); // put the first segment into vector (if there were other segments, they would be added in order)
    Eigen::Vector4d gravityDirection = { 0,0,-1,0 }; // define the direction of gravity
    wholeSystem testSystem(static_cast<int>(segments.size()), segments, gravityDirection); // create the whole system from the vector of segments and gravity direction
    // We will assume that the actuators have the same parameters as before
    for (int i = 0; i < 4; i++) {
        testSystem.actuator_changeEmpiricalConstants(0, i, tempActConstants); // change the 0th segment's ith actuator to have these constants
        testSystem.actuator_changeUseTheoreticalModel(0, i, false); // change the 0th segment's ith actuator to use the empirical model
        testSystem.actuator_changeInitialLength(0, i, 0.14); // change the 0th segment's ith actuator length to 0.14 m (or 140 mm)
    }
    
    // We will show the possible changes to the other classes using testSystem so that users are aware of everything that they have access to.
    // Sensor class
    int j = 0; // for clarity, j will refer to the segment within the test system
    int i = 0; // for clarity, i will refer to the interface within the jth segment in the test system
    array<double, 2> newPlasticConstants = { 0.0679,-0.0093 }; // define plastic constants d_0 and d_1
    testSystem.sensor_changePlasticConstants(j, i, newPlasticConstants); // change the plastic constants of the sensor
    array<double, 2> checkConstants = testSystem.sensor_getPlasticConstants(j, i); // retrieve the plastic constants
    cout << "Plastic constants: " << checkConstants[0] << " " << checkConstants[1] << endl; // print out the plastic constants
    testSystem.sensor_changeMaxLength(j, i, 0.125); // change the maximum length of deformation to 0.125 m (125 mm)
    cout << "Sensor Max Length: " << testSystem.sensor_getMaxLength(j, i) << " m" << endl; // retrieve and print out the max length the sensor was stretched to
    testSystem.sensor_changeLengthInitial(j, i, 0.9); // change the initial length of the sensor to 0.9 m (90 mm)
    cout << "Sensor Initial Length: " << testSystem.sensor_getLengthInitial(j, i) << " m" << endl; // retrieve and print out the initial sensor length
    testSystem.sensor_changeWidth(j, i, 0.1); // change the sensor width to 0.01 m (10 mm)
    cout << "Sensor Width: " << testSystem.sensor_getWidth(j, i) << " m" << endl; // retrieve and print out the sensor width
    testSystem.sensor_changeThickness(j, i, 0.0015); // change the sensor thickness to 0.0015 m (1.5 mm)
    cout << "Sensor Thickness: " << testSystem.sensor_getThickness(j, i) << " m" << endl; // retrieve and print out the sensor thickness
    array<double, 3> newOgdenMu = { 2961,50215,75698 }; // define new mu values for the Ogden model to match the sensor
    testSystem.sensor_changeOgdenMu(j, i, newOgdenMu); // change the mu values for the sensor's Ogden model
    array<double, 3> checkOgdenMu = testSystem.sensor_getOgdenMu(j, i); // retrieve the mu values for the Ogden model
    cout << "Sensor Ogden Mu Values: " << checkOgdenMu[0] << " " << checkOgdenMu[1] << ", " << checkOgdenMu[2] << endl; // print the mu values for the Ogden model
    array<double, 3> newOgdenAlpha = { 7.8766,0.6886,0.6886 }; // define new alpha values for the Ogden model to match the sensor
    testSystem.sensor_changeOgdenAlpha(j, i, newOgdenAlpha); // change the alpha values for the sensor's Ogden model
    array<double, 3> checkOgdenAlpha = testSystem.sensor_getOgdenAlpha(j, i); // retrieve the alpha values for the Ogden model
    cout << "Sensor Ogden Alpha Values: " << checkOgdenAlpha[0] << " " << checkOgdenAlpha[1] << " " << checkOgdenAlpha[2] << endl; // print the alpha values for the Ogden model
    testSystem.sensor_changePoissonRatio(j, i, 0.5); // change the Poisson ratio of the sensor model (0.5 = incompressible material)
    cout << "Sensor Poisson Ratio: " << testSystem.sensor_getPoissonRatio(j, i) << endl; // retrieve and print the sensor's poisson ratio
    cout << "Sensor Current Length: " << testSystem.sensor_getCurrentLength(j, i) << " m" << endl; // retrieve and print the sensor's current length
    cout << "Sensor Force: " << testSystem.sensor_getForce(j, i) << " N" << endl; // retrieve and print the sensor's force
    cout << "Sensor L_0: " << testSystem.sensor_getLength0(j, i) << " m" << endl; // retrieve and print the sensor's L_0 value
    cout << "Sensor Plastic Length: " << testSystem.sensor_getLengthPlastic(j, i) << " m" << endl; // retrieve and print the sensor's length of plastic deformation
    cout << "Sensor Plastic Strain: " << testSystem.sensor_getStrainPlastic(j, i) << endl; // retrieve and print the sensor's strain of plastic deformation
    cout << "Sensor Maximum Strain: " << testSystem.sensor_getStrainMax(j, i) << endl << endl; // retrieve and print the sensor's maximum strain experienced
    // Note that values like Maximum Length will not be auto-updated if the maximum length is exceeded during operation of the skin because changing the maximum length
    // will cause a change in the sensor model. Thus, the maximum length used in sensor characterization should exceed expected operation of the skin.

    // Actuator class
    // We have already shown how to use the empirical model above, so that won't be repeated here. Instead, we will highlight what you can change in general
    // and then what can be changed for the theoretical model.
    testSystem.actuator_changePressure(j, i, 0); // change the pressure in the actuator to 0 kPa
    cout << "Actuator Pressure: " << testSystem.actuator_getPressure(j, i) << " kPa" << endl; // retrieve and print the pressure in the actuator
    cout << "Actuator Current Length: " << testSystem.actuator_getCurrentLength(j, i) << " m" << endl; // retrieve and print the current length of the actuator
    cout << "Actuator Force: " << testSystem.actuator_getForce(j, i) << " N" << endl; // retrieve and print the force of the actuator
    cout << "Actuator Initial Length: " << testSystem.actuator_getInitialLength(j, i) << " m" << endl; // retrieve and print the initial length of the actuator
    array<double, 5> checkEmpiricalConstants = testSystem.actuator_getEmpiricalConstants(j, i);
    cout << "Actuator Empirical Constants: ";
    for (int i = 0; i < 5; i++) { cout << checkEmpiricalConstants[i] << " "; }
    cout << endl;
    // Parameters that affect the theoretical model (if used)
    testSystem.actuator_changeInitialAngle(j, i, 29.0 * M_PI / 180.0); // change the initial braid angle of the McKibben actuator
    cout << "Actuator Initial Braid Angle: " << testSystem.actuator_getInitialAngle(j, i) << " rad" << endl; // retrieve and print the actuator's initial braid angle
    testSystem.actuator_changeBraidDiameter(j, i, 0.009); // change the braid diameter for the McKibben actuator
    cout << "Actuator Braid Diameter: " << testSystem.actuator_getBraidDiameter(j, i) << " m" << endl; // retrieve and print the actuator's braid diameter
    cout << "Actuator Braid Length: " << testSystem.actuator_getBraidLength(j, i) << " m" << endl; // retrieve and print the actuator's braid length
    cout << "Number of times braid is wound around the Actuator: " << testSystem.actuator_getNumberOfTimesFibersWound(j, i) << endl; // retrieve and print the number of windings
    cout << "Actuator Theoretical Model in Use? " << testSystem.actuator_getUseTheoreticalModel(j, i) << endl << endl; // retrieve and print which actuator model is used
 
    // Interface class
    testSystem.interface_changeIsSensor(j, i, true); // change the interface to have a sensor
    cout << "Is there a sensor on the interface? " << testSystem.interface_getIsSensor(j, i) << endl; // retrieve and print if a sensor is attached to the interface
    testSystem.interface_changeIsActuator(j, i, true); // change the interface to have an actuator
    cout << "Is there an actuator on the interface? " << testSystem.interface_getIsActuator(j, i) << endl; // retrieve and print if an actuator is attached to the interface
    testSystem.interface_changeSpringConstant(j, i, 200); // change the interface's spring constant
    cout << "Interface Spring Constant: " << testSystem.interface_getSpringConstant(j, i) << " N/m" << endl; // retrieve and print the interface's spring constant
    cout << "Interface Force: " << testSystem.interface_getForce(j, i) << " N" << endl; // retrieve and print the interface's force
    Eigen::Vector4d forceVector;
    testSystem.interface_getForceVector(j, i, forceVector); // retrieve the force vector from the interface
    cout << "Interface Force Vector: " << forceVector << " N" << endl; // print the force vector from the interface
    cout << "Interface Length: " << testSystem.interface_getInterfaceLength(j, i)[1] << " m" << endl; // retrieve and print the interface length
    cout << "Attached Sensor/Actuator Length: " << testSystem.interface_getSensorActuatorLength(j, i) << " m" << endl; // retrieve and print the attached sensor/actuator length
    array<double, 2> attachmentConstants = testSystem.interface_getAttachmentConstants(j, i); // retrieve the interface attachment constants (b_0, b_1)
    cout << "Interface constant b_0: " << attachmentConstants[0] << " Interface constant b_1: " << attachmentConstants[1] << endl << endl;
    
    // Substrate class
    testSystem.substrate_changeInitialLength(j, 0.1); // change the initial length of the substrate
    cout << "Substrate Initial Length: " << testSystem.substrate_getInitialLength(j) << " m" << endl; // retrieve and print the initial length of the substrate
    testSystem.substrate_changeInitialWidth(j, 0.12); // change the initial width of the substrate
    cout << "Substrate Initial Width: " << testSystem.substrate_getInitialWidth(j) << " m" << endl; // retrieve and print the initial width of the substrate
    testSystem.substrate_changeCurrentLength(j, 0.1); // change the current length of the substrate
    cout << "Substrate Current Length: " << testSystem.substrate_getCurrentLength(j) << " m" << endl; // retrieve and print the current length of the substrate
    testSystem.substrate_changeCurrentWidth(j, 0.12); // change the current width of the substrate
    cout << "Substrate Current Width: " << testSystem.substrate_getCurrentWidth(j) << " m" << endl; // retrieve and print the current width of the substrate
    array<double, 2> newSpringConstants = { 1800, 1800 };
    testSystem.substrate_changeSpringConstants(j, newSpringConstants); // change the spring constants for the substrate
    array<double, 2> checkSpringConstants = testSystem.substrate_getSpringConstants(j); // retrieve the spring constants for the substrate
    cout << "Substrate Spring Constants: " << checkSpringConstants[0] << " " << checkSpringConstants[1] << " N/m" << endl; // print the spring constants of the substrate
    array<double, 2> checkChangeInLengths = testSystem.substrate_getChangeInLengths(j); // retrieve the change in lengths of the substrate
    cout << "Substrate Change in Lengths (Length Width): " << checkChangeInLengths[0] << " " << checkChangeInLengths[1] << " m" << endl; // print the change in lengths of the substrate
    Eigen::Vector4d checkForceVector;
    testSystem.substrate_getForceVector(j, checkForceVector); // retrieve the force vector of the substrate
    cout << "Substrate Force Vector: " << checkForceVector << " N" << endl << endl; // print the force vector of the substrate

    // Robotic Skin class
    testSystem.skin_changeInterfaceAttachments(j, b0, b1, actuators, sensors); // change the interface attachments and whether actuators and sensors are attached
    cout << "Number of Interfaces on Robotic Skin: " << testSystem.skin_getNumInterfaces(j) << endl << endl; // retrieve and print the number of interfaces on the robotic skin

    // End Cap class
    testSystem.endcap_changeLength(j, 0, 0.043); // change the end cap length
    cout << "End Cap Length: " << testSystem.endcap_getLength(j, 0) << " m" << endl; // retrieve and print the end cap length
    Eigen::Matrix4d endCapTransform;
    testSystem.endcap_getTransform(j, 0, endCapTransform); // retrieve the homogeneous tranform of the end cap
    cout << "End Cap Transform: " << endCapTransform << endl << endl; // print the end cap transform

    // Cylindrical Structure class
    testSystem.cylinder_changeOuterRadius(j, 0.015); // change the outer radius of the cylindrical structure
    cout << "Cylindrical Structure Outer Radius: " << testSystem.cylinder_getOuterRadius(j) << " m" << endl; // retrieve and print the outer radius of the cylindrical structure
    testSystem.cylinder_changeInnerRadius(j, 0); // change the inner radius of the cylindrical structure
    cout << "Cylindrical Structure Inner Radius: " << testSystem.cylinder_getInnerRadius(j) << " m" << endl; // retrieve and print the inner radius of the cylindrical structure
    testSystem.cylinder_changeLength(j, 0.1); // change the length of the cylindrical structure
    cout << "Cylindrical Structure Length: " << testSystem.cylinder_getLength(j) << " m" << endl; // retrieve and print the length of the cylindrical structure
    testSystem.cylinder_changeElasticModulus(j, 130000); // change the elastic modulus of the cylindrical structure
    cout << "Cylindrical Structure Elastic Modulus: " << testSystem.cylinder_getElasticModulus(j) << " Pa" << endl; // retrieve and print the elastic modulus of the cylindrical structure
    testSystem.cylinder_changePoissonRatio(j, 0.5); // change the Poisson ratio of the cylindrical structure
    cout << "Cylindrical Structure Poisson Ratio: " << testSystem.cylinder_getPoissonRatio(j) << endl; // retrieve and print the Poisson ratio of the cylindrical structure
    Eigen::Vector4d newState = { 0.0, 0.0, 1.0, 0.0 };
    testSystem.cylinder_changeState(j, newState); // change the state of the cylindrical structure (this can be helpful when resetting it to a neutral position)
    Eigen::Vector4d checkState;
    testSystem.cylinder_getState(j, checkState); // retrieve the state of the cylindrical structure
    cout << "Cylindrical Structure State: " << checkState << endl; // print the state of the cylindrical structure
    Eigen::Matrix4d checkTransform;
    testSystem.cylinder_calculateTransform(j, 0.05, checkTransform); // calculate the transform 1/2 up the cylindrical structure (recall length was set to 0.1)
    cout << "Cylindrical Structure Calculated Transform: " << checkTransform << endl; // print the calculated transform for the cylindrical structure (based on the state)
    cout << "Cylindrical Structure Buckling Force: " << testSystem.cylinder_getBucklingForce(j) << " N" << endl << endl; // retrieve and print the buckling force of the cylindrical structure

    // Segment class
    testSystem.segment_changeMass(j, 0.17); // change the mass of the segment (cylindrical structure + skin)
    cout << "Segment Mass: " << testSystem.segment_getMass(j) << " kg" << endl; // retrieve and print the mass of the segment
    testSystem.segment_changeGravity(j, 9.81); // change the gravity experienced by the segment
    cout << "Segment Gravity: " << testSystem.segment_getGravity(j) << " m/s^2" << endl; // retrieve and print the gravity experienced by the segment
    Eigen::Vector4d checkVector;
    testSystem.segment_getWeightDirectionVector(j, checkVector); // retrieve weight direction vector of the segment
    cout << "Segment Weight Direction: " << checkVector << endl; // print the weight direction vector of the segment
    testSystem.segment_getForceVector(j, checkVector); // retrieve the force vector experienced by the segment
    cout << "Segment Force Vector: " << checkVector << endl; // print the force vector experienced by the segment
    testSystem.segment_getMomentVector(j, checkVector); // retrieve the moment vector experienced by the segment
    cout << "Segment Moment Vector: " << checkVector << endl; // print the moment vector experienced by the segment
    cout << "Is the segment in a valid position? " << testSystem.segment_getError(j) << endl; // retrieve and print if the segment is in a valid position
    cout << "Press enter to continue: "; cin.get(); cout << endl;
    
    // Mechanics Model example
    testSystem.actuator_changePressure(0, 1, 100000); // change the pressure in Actuator 1 (from 0, 1, 2, 3) to 100 kPa
    testSystem.optimizeSystemState();
    testSystem.cylinder_getState(0, checkState);
    cout << "System's new state: " << checkState << endl;
    cout << "Press enter to continue: "; cin.get(); cout << endl;

    // reset the model
    testSystem.actuator_changePressure(0, 1, 0);
    testSystem.cylinder_changeState(j, newState); // recall newState = {0, 0, 1, 0}

    // Controls Model example
    Eigen::Vector4d desiredState = { 4, 0.3, 1000, 0.15 }; // enter the desired state (1000+ values are ignored by the optimization)
    vector<Eigen::Ref<Eigen::Vector4d>> desiredSegmentState;
    desiredSegmentState.push_back(desiredState);
    testSystem.optimizeSystemPressure(desiredSegmentState);
    testSystem.cylinder_getState(0, checkState);
    cout << "Desired State: " << desiredState << endl << "Control Model State: " << checkState << endl;
    cout << "Press enter to continue: "; cin.get(); cout << endl;
    cout << "Actuator Pressures: " << testSystem.actuator_getPressure(0, 0) << " " << testSystem.actuator_getPressure(0, 1) << " " << testSystem.actuator_getPressure(0, 2) << " " << testSystem.actuator_getPressure(0, 3) << " kPa" << endl << endl;
    
    // reset the model
    for (int i = 0; i < 4; i++) { testSystem.actuator_changePressure(0, i, 0); }
    testSystem.cylinder_changeState(j, newState); // recall newState = {0, 0, 1, 0}

    // State Estimation example (currently only functional for single segment systems)
    vector<double> sensorLengths;
    sensorLengths.push_back(0.0946); sensorLengths.push_back(0.102); sensorLengths.push_back(0.100); sensorLengths.push_back(0.1027);
    testSystem.estimateSystemState(sensorLengths);
    cout << "Sensor Lengths: " << sensorLengths[0] << " " << sensorLengths[1] << " " << sensorLengths[2] << " " << sensorLengths[3] << " m" << endl;
    cout << "Predicted Sensor Lengths: " << testSystem.sensor_getCurrentLength(0, 0) << " " << testSystem.sensor_getCurrentLength(0, 1) << " " << testSystem.sensor_getCurrentLength(0, 2) << " " << testSystem.sensor_getCurrentLength(0, 3) << " m" << endl;
    testSystem.cylinder_getState(0, checkState);
    cout << "State Estimation Model State: " << checkState << endl;

    return 0;
}