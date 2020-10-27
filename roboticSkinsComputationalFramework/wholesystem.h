#pragma once
#include <vector>
#include <array>
#include "segment.h"
#include <Eigen/Dense>

class wholeSystem
{
public:
    std::vector<segment> segments;

    wholeSystem(int, std::vector<segment>, Eigen::Ref<Eigen::Vector4d>);
    ~wholeSystem();

    void deformSystem(void);
    void optimizeSystemState(void);
    void optimizeSystemPressure(std::vector<Eigen::Ref<Eigen::Vector4d>>);
    void estimateSystemState(std::vector<double>); // requires a vector of a vector to allow multi-segment estimation
    void adjustWeightDirections(void);

    int getNumSegments(void);
    void getGravityDirection(Eigen::Ref<Eigen::Vector4d>);

    // segment
    void segment_changeMass(int, double);
    void segment_changeGravity(int, double);
    void segment_changeWeightDirectionVector(int, Eigen::Ref<Eigen::Vector4d>);
    void segment_optimizeState(int, Eigen::Ref<Eigen::Matrix4d>, Eigen::Ref<Eigen::Vector4d>, Eigen::Ref<Eigen::Vector4d>);
    void segment_optimizePressure(int, Eigen::Ref<Eigen::Matrix4d>, Eigen::Ref<Eigen::Vector4d>, Eigen::Ref<Eigen::Vector4d>, Eigen::Ref<Eigen::Vector4d>);
    void segment_stateEstimation(int, std::vector<double>);
    double segment_getMass(int);
    double segment_getGravity(int);
    void segment_getWeightDirectionVector(int, Eigen::Ref<Eigen::Vector4d>);
    double segment_getBendingStiffness(int);
    double segment_getAxialStiffness(int);
    double segment_getTorsionalStiffness(int);
    void segment_getForceVector(int, Eigen::Ref<Eigen::Vector4d>);
    void segment_getMomentVector(int, Eigen::Ref<Eigen::Vector4d>);
    bool segment_getError(int);

    // for the cylinder
    void cylinder_changeOuterRadius(int, double);
    void cylinder_changeInnerRadius(int, double);
    void cylinder_changeLength(int, double);
    void cylinder_changeElasticModulus(int, double);
    void cylinder_changePoissonRatio(int, double);
    void cylinder_changeState(int, Eigen::Ref<Eigen::Vector4d>);
    void cylinder_calculateTransform(int, double, Eigen::Ref<Eigen::Matrix4d>);
    double cylinder_getOuterRadius(int);
    double cylinder_getInnerRadius(int);
    double cylinder_getLength(int);
    double cylinder_getElasticModulus(int);
    double cylinder_getPoissonRatio(int);
    void cylinder_getState(int, Eigen::Ref<Eigen::Vector4d>);
    double cylinder_getBendingStiffness(int);
    double cylinder_getAxialStiffness(int);
    double cylinder_getTorsionalStiffness(int);
    double cylinder_getBucklingForce(int);

    // for the endcap
    void endcap_changeLength(int, int, double);
    double endcap_getLength(int, int);
    void endcap_getTransform(int, int, Eigen::Ref<Eigen::Matrix4d>);

    // for the skin
    void skin_changeInterfaceAttachments(int, std::vector<double>, std::vector<double>, std::vector<bool>, std::vector<bool>);
    int skin_getNumInterfaces(int);

    // for the skin->substrate class
    void substrate_changeInitialLength(int, double);
    void substrate_changeInitialWidth(int, double);
    void substrate_changeCurrentLength(int, double);
    void substrate_changeCurrentWidth(int, double);
    void substrate_changeSpringConstants(int, std::array<double, 2>);
    void substrate_changeStiffnessConstants(int, std::array<double, 3>);
    double substrate_getInitialLength(int);
    double substrate_getInitialWidth(int);
    double substrate_getCurrentLength(int);
    double substrate_getCurrentWidth(int);
    std::array<double, 2> substrate_getSpringConstants(int);
    std::array<double, 3> substrate_getStiffnessConstants(int);
    std::array<double, 2> substrate_getChangeInLengths(int);
    double substrate_getBendingStiffness(int);
    void substrate_getForceVector(int, Eigen::Ref<Eigen::Vector4d>);

    // for skin->interface class
    void interface_changeIsSensor(int, int, bool);
    void interface_changeIsActuator(int, int, bool);
    void interface_changeSpringConstant(int, int, double);
    void interface_changeSensorActuatorLength(int, int, double);
    bool interface_getIsSensor(int, int);
    bool interface_getIsActuator(int, int);
    double interface_getForce(int, int);
    void interface_getForceVector(int, int, Eigen::Ref<Eigen::Vector4d>);
    double interface_getSpringConstant(int, int);
    std::array<double, 2> interface_getInterfaceLength(int, int);
    std::array<double, 2> interface_getAttachmentConstants(int, int);
    double interface_getSensorActuatorLength(int, int);

    // for the interface->actuator
    void actuator_changeUseTheoreticalModel(int, int, bool);
    void actuator_changePressure(int, int, double);
    void actuator_changeInitialLength(int, int, double);
    void actuator_changeInitialAngle(int, int, double);
    void actuator_changeBraidLength(int, int, double);
    void actuator_changeBraidDiameter(int, int, double);
    void actuator_changeEmpiricalConstants(int, int, std::array<double, 5>);
    bool actuator_getUseTheoreticalModel(int, int);
    double actuator_getForce(int, int);
    double actuator_getPressure(int, int);
    double actuator_getNumberOfTimesFibersWound(int, int);
    double actuator_getInitialLength(int, int);
    double actuator_getCurrentLength(int, int);
    double actuator_getInitialAngle(int, int);
    double actuator_getBraidLength(int, int);
    double actuator_getBraidDiameter(int, int);
    std::array<double, 5> actuator_getEmpiricalConstants(int, int);

    // for the interface->sensor
    void sensor_changePlasticConstants(int, int, std::array<double, 2>);
    void sensor_changeMaxLength(int, int, double);
    void sensor_changeLengthInitial(int, int, double);
    void sensor_changeWidth(int, int, double);
    void sensor_changeThickness(int, int, double);
    void sensor_changeOgdenMu(int, int, std::array<double, 3>);
    void sensor_changeOgdenAlpha(int, int, std::array<double, 3>);
    void sensor_changePoissonRatio(int, int, double);
    double sensor_getForce(int, int);
    double sensor_getWidth(int, int);
    double sensor_getThickness(int, int);
    double sensor_getCurrentLength(int, int);
    std::array<double, 3> sensor_getOgdenMu(int, int);
    std::array<double, 3> sensor_getOgdenAlpha(int, int);
    double sensor_getLength0(int, int);
    double sensor_getLengthInitial(int, int);
    double sensor_getLengthPlastic(int, int);
    double sensor_getStrainPlastic(int, int);
    double sensor_getStrainMax(int, int);
    std::array<double, 2> sensor_getPlasticConstants(int, int);
    double sensor_getMaxLength(int, int);
    double sensor_getPoissonRatio(int, int);
private:
    int numSegments;
    Eigen::Vector4d globalGravityDirection;

    void changeNumSegments(int);
    void changeGravityDirection(Eigen::Ref<Eigen::Vector4d>);
};

