#pragma once
#include<array>

class actuator
{
public:
    actuator();
    ~actuator();

    void changeUseTheoreticalModel(bool);
    void changePressure(double);
    void changeInitialLength(double);
    void changeCurrentLength(double);
    void changeInitialAngle(double);
    void changeBraidLength(double);
    void changeBraidDiameter(double);
    void changeEmpiricalConstants(std::array<double, 5>);

    bool getUseTheoreticalModel(void);
    double getForce(void);
    double getPressure(void);
    double getNumberOfTimesFibersWound(void);
    double getInitialLength(void);
    double getCurrentLength(void);
    double getInitialAngle(void);
    double getBraidLength(void);
    double getBraidDiameter(void);
    std::array<double, 5> getEmpiricalConstants(void);

private:
    bool useTheoreticalModel;
    double force;
    double pressure;
    double numberOfTimesFibersWound;
    double initialLength;
    double currentLength;
    double initialAngle;
    double braidLength;
    double braidDiameter;
    std::array<double, 5> empiricalConstants;

    void changeForce(double);
    void changeNumberOfTimesFibersWound(double);
    void calculateForce(void);
    void calculateBraidLength(void);
    void calculateNumberOfTimesFibersWound(void);
};