#pragma once
#include "substrate.h"
#include "interface.h"
#include <vector>
#include <array>

class roboticSkin
{
public:
    roboticSkin(int, std::vector<double>, std::vector<double>, std::vector<bool>, std::vector<bool>);
    ~roboticSkin();

    void changeInterfaceAttachments(std::vector<double>, std::vector<double>, std::vector<bool>, std::vector<bool>);
    int getNumInterfaces();

    substrate spandex;
    std::vector<interface> interfaces;

    // for the substrate class
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

    // for interface class
    void interface_changeIsSensor(int, bool);
    void interface_changeIsActuator(int, bool);
    void interface_changeSpringConstant(int, double);
    void interface_changeAttachmentConstants(int, std::array<double, 2>);
    void interface_changeSensorActuatorLength(int, double);
    bool interface_getIsSensor(int);
    bool interface_getIsActuator(int);
    double interface_getForce(int);
    void interface_getForceVector(int, Eigen::Ref<Eigen::Vector4d>, double, double);
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
    int numInterfaces;

    void changeNumInterfaces(int);
};

