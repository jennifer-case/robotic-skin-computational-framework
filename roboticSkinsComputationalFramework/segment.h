#pragma once
#include "cylindricalstructure.h"
#include "endcap.h"
#include "roboticskin.h"
#include <vector>
#include <array>
#include <Eigen/Dense>

class segment
{
public:
    segment(int, std::vector<double>, std::vector<double>, std::vector<bool>, std::vector<bool>);
    ~segment();

    void changeMass(double);
    void changeGravity(double);
    void changeWeightDirectionVector(Eigen::Ref<Eigen::Vector4d>);

    void optimizeState(Eigen::Ref<Eigen::Matrix4d>, Eigen::Ref<Eigen::Vector4d>, Eigen::Ref<Eigen::Vector4d>);
    void optimizePressure(Eigen::Ref<Eigen::Matrix4d>, Eigen::Ref<Eigen::Vector4d>, Eigen::Ref<Eigen::Vector4d>, Eigen::Ref<Eigen::Vector4d>);
    void stateEstimation(std::vector<double>);

    double getMass(void);
    double getGravity(void);
    void getWeightDirectionVector(Eigen::Ref<Eigen::Vector4d>);
    double getBendingStiffness(void);
    double getAxialStiffness(void);
    double getTorsionalStiffness(void);
    void getForceVector(Eigen::Ref<Eigen::Vector4d>);
    void getMomentVector(Eigen::Ref<Eigen::Vector4d>);
    bool getError(void);

    // for the cylinder
    void cylinder_changeOuterRadius(double);
    void cylinder_changeInnerRadius(double);
    void cylinder_changeLength(double);
    void cylinder_changeElasticModulus(double);
    void cylinder_changePoissonRatio(double);
    void cylinder_changeState(Eigen::Ref<Eigen::Vector4d>);
    void cylinder_calculateTransform(double, Eigen::Ref<Eigen::Matrix4d>);
    double cylinder_getOuterRadius(void);
    double cylinder_getInnerRadius(void);
    double cylinder_getLength(void);
    double cylinder_getElasticModulus(void);
    double cylinder_getPoissonRatio(void);
    void cylinder_getState(Eigen::Ref<Eigen::Vector4d>);
    double cylinder_getBendingStiffness(void);
    double cylinder_getAxialStiffness(void);
    double cylinder_getTorsionalStiffness(void);
    double cylinder_getBucklingForce(void);

    // for the endcap
    void endcap_changeLength(int, double);
    double endcap_getLength(int);
    void endcap_getTransform(int, Eigen::Ref<Eigen::Matrix4d>);

    // for the skin
    void skin_changeInterfaceAttachments(std::vector<double>, std::vector<double>, std::vector<bool>, std::vector<bool>);
    int skin_getNumInterfaces();

    // for the skin->substrate class
    void substrate_changeInitialLength(double);
    void substrate_changeInitialWidth(double);
    void substrate_changeCurrentLength(double);
    void substrate_changeCurrentWidth(double);
    void substrate_changeSpringConstants(std::array<double, 2>);
    void substrate_changeStiffnessConstants(std::array<double, 3>);
    double substrate_getInitialLength(void);
    double substrate_getInitialWidth(void);
    double substrate_getCurrentLength(void);
    double substrate_getCurrentWidth(void);
    std::array<double, 2> substrate_getSpringConstants();
    std::array<double, 3> substrate_getStiffnessConstants();
    std::array<double, 2> substrate_getChangeInLengths();
    double substrate_getBendingStiffness(void);
    void substrate_getForceVector(Eigen::Ref<Eigen::Vector4d>);

    // for skin->interface class
    void interface_changeIsSensor(int, bool);
    void interface_changeIsActuator(int, bool);
    void interface_changeSpringConstant(int, double);
    void interface_changeSensorActuatorLength(int, double);
    bool interface_getIsSensor(int);
    bool interface_getIsActuator(int);
    double interface_getForce(int);
    void interface_getForceVector(int, Eigen::Ref<Eigen::Vector4d>);
    double interface_getSpringConstant(int);
    std::array<double, 2> interface_getInterfaceLength(int);
    std::array<double, 2> interface_getAttachmentConstants(int);
    double interface_getSensorActuatorLength(int);

    // for the interface->actuator
    void actuator_changeUseTheoreticalModel(int, bool);
    void actuator_changePressure(int, double);
    void actuator_changeInitialLength(int, double);
    void actuator_changeInitialAngle(int, double);
    void actuator_changeBraidLength(int, double);
    void actuator_changeBraidDiameter(int, double);
    void actuator_changeEmpiricalConstants(int, std::array<double, 5>);
    bool actuator_getUseTheoreticalModel(int);
    double actuator_getForce(int);
    double actuator_getPressure(int);
    double actuator_getNumberOfTimesFibersWound(int);
    double actuator_getInitialLength(int);
    double actuator_getCurrentLength(int);
    double actuator_getInitialAngle(int);
    double actuator_getBraidLength(int);
    double actuator_getBraidDiameter(int);
    std::array<double, 5> actuator_getEmpiricalConstants(int);

    // for the interface->sensor
    void sensor_changePlasticConstants(int, std::array<double, 2>);
    void sensor_changeMaxLength(int, double);
    void sensor_changeLengthInitial(int, double);
    void sensor_changeWidth(int, double);
    void sensor_changeThickness(int, double);
    void sensor_changeOgdenMu(int, std::array<double, 3>);
    void sensor_changeOgdenAlpha(int, std::array<double, 3>);
    void sensor_changePoissonRatio(int, double);
    double sensor_getForce(int);
    double sensor_getWidth(int);
    double sensor_getThickness(int);
    double sensor_getCurrentLength(int);
    std::array<double, 3> sensor_getOgdenMu(int);
    std::array<double, 3> sensor_getOgdenAlpha(int);
    double sensor_getLength0(int);
    double sensor_getLengthInitial(int);
    double sensor_getLengthPlastic(int);
    double sensor_getStrainPlastic(int);
    double sensor_getStrainMax(int);
    std::array<double, 2> sensor_getPlasticConstants(int);
    double sensor_getMaxLength(int);
    double sensor_getPoissonRatio(int);


private:
    double mass;
    double gravity;
    Eigen::Vector4d weightDirectionVector;
    double bendingStiffness;
    double axialStiffness;
    double torsionalStiffness;
    Eigen::Vector4d forceVector;
    Eigen::Vector4d momentVector;
    cylindricalStructure cylinder;
    std::array<endCap, 2> endCaps;
    roboticSkin skin;
    bool segmentError;

    void changeBendingStiffness(double);
    void changeAxialStiffness(double);
    void changeTorsionalStiffness(double);
    void changeForceVector(Eigen::Ref<Eigen::Vector4d>);
    void changeMomentVector(Eigen::Ref<Eigen::Vector4d>);
    void changeError(bool);
    void calculateStiffnesses(void);
    void calculateForceVector(Eigen::Ref<Eigen::Vector4d>);
    void calculateMomentVector(Eigen::Ref<Eigen::Vector4d>, Eigen::Ref<Eigen::Vector4d>);
    void calculateState(Eigen::Ref<Eigen::Matrix4d>, Eigen::Ref<Eigen::Vector4d>, Eigen::Ref<Eigen::Vector4d>, Eigen::Ref<Eigen::Vector4d>);
    void deformSegment(Eigen::Ref<Eigen::Vector4d>);
    void getPointLocationsForLength(double, std::array<double, 2>, Eigen::Ref<Eigen::Vector3d>);
    double getNormalizedrPrime(double, double, std::array<double, 2>);
    double getLength(int, std::array<double, 2>);
    void optimizeLength(int);
};