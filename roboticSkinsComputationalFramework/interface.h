#pragma once
#include "sensor.h"
#include "actuator.h"
#include <Eigen/Dense>
#include <array>

class interface
{
public:
    interface();
    ~interface();

    void changeIsSensor(bool);
    void changeIsActuator(bool);
    void changeSpringConstant(double);
    void changeAttachmentConstants(std::array<double, 2>);
    void changeSensorActuatorLength(double);

    bool getIsSensor(void);
    bool getIsActuator(void);
    double getForce(void);
    void getForceVector(Eigen::Ref<Eigen::Vector4d>, double, double);
    double getSpringConstant(void);
    std::array<double, 2> getInterfaceLength(void);
    std::array<double, 2> getAttachmentConstants(void);
    double getSensorActuatorLength(void);

    // for the actuator
    void actuator_changeUseTheoreticalModel(bool);
    void actuator_changePressure(double);
    void actuator_changeInitialLength(double);
    void actuator_changeInitialAngle(double);
    void actuator_changeBraidLength(double);
    void actuator_changeBraidDiameter(double);
    void actuator_changeEmpiricalConstants(std::array<double, 5>);
    bool actuator_getUseTheoreticalModel(void);
    double actuator_getForce(void);
    double actuator_getPressure(void);
    double actuator_getNumberOfTimesFibersWound(void);
    double actuator_getInitialLength(void);
    double actuator_getCurrentLength(void);
    double actuator_getInitialAngle(void);
    double actuator_getBraidLength(void);
    double actuator_getBraidDiameter(void);
    std::array<double, 5> actuator_getEmpiricalConstants(void);

    // for the sensor
    void sensor_changePlasticConstants(std::array<double, 2>);
    void sensor_changeMaxLength(double);
    void sensor_changeLengthInitial(double);
    void sensor_changeWidth(double);
    void sensor_changeThickness(double);
    void sensor_changeOgdenMu(std::array<double, 3>);
    void sensor_changeOgdenAlpha(std::array<double, 3>);
    void sensor_changePoissonRatio(double);
    double sensor_getForce(void);
    double sensor_getWidth(void);
    double sensor_getThickness(void);
    double sensor_getCurrentLength(void);
    std::array<double, 3> sensor_getOgdenMu();
    std::array<double, 3> sensor_getOgdenAlpha();
    double sensor_getLength0(void);
    double sensor_getLengthInitial(void);
    double sensor_getLengthPlastic(void);
    double sensor_getStrainPlastic(void);
    double sensor_getStrainMax(void);
    std::array<double, 2> sensor_getPlasticConstants();
    double sensor_getMaxLength(void);
    double sensor_getPoissonRatio(void);

private:
    bool isSensor;
    bool isActuator;
    sensor attachedSensor;
    actuator attachedActuator;

    double force;
    Eigen::Vector4d forceVector;
    double springConstant;
    std::array<double, 2> interfaceLength;
    std::array<double, 2> attachmentConstants;

    // for the interface
    void changeForce(double);
    void changeForceVector(Eigen::Ref<Eigen::Vector4d>);
    void changeInterfaceLength(std::array<double, 2>);
    void calculateForce(void);
    void calculateForceVector(double, double);
    void calculateInterfaceLength(void);

    // for actuator and sensor
    void actuator_changeCurrentLength(double);
    void sensor_changeCurrentLength(double);
};