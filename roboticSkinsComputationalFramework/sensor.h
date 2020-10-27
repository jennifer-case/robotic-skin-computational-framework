#pragma once

/***************************
 * Sensor class
 *
 * Contains information about sensor and determines force in the sensor.
 *
 * Written by Jennifer C. Case on 3/29/2019
 ***************************/

#include <array>

class sensor
{
public:
    sensor(void);
    ~sensor(void);

    void changePlasticConstants(std::array<double, 2>);
    void changeMaxLength(double);
    void changeLengthInitial(double);
    void changeWidth(double);
    void changeThickness(double);
    void changeCurrentLength(double);
    void changeOgdenMu(std::array<double, 3>);
    void changeOgdenAlpha(std::array<double, 3>);
    void changePoissonRatio(double);

    double getForce(void);
    double getWidth(void);
    double getThickness(void);
    double getCurrentLength(void);
    std::array<double, 3> getOgdenMu();
    std::array<double, 3> getOgdenAlpha();
    double getLength0(void);
    double getLengthInitial(void);
    double getLengthPlastic(void);
    double getStrainPlastic(void);
    double getStrainMax(void);
    std::array<double, 2> getPlasticConstants();
    double getMaxLength(void);
    double getPoissonRatio(void);

private:
    double force;
    double width;
    double thickness;
    double currentLength;
    std::array<double, 3> ogdenMu;
    std::array<double, 3> ogdenAlpha;
    double length0;
    double lengthInitial;
    double lengthPlastic;
    double strainPlastic;
    double strainMax;
    std::array<double, 2> plasticConstants;
    double maxLength;
    double poissonRatio;

    void changeForce(double);
    void changeLength0(double);
    void changeLengthPlastic(double);
    void changeStrainPlastic(double);
    void changeStrainMax(double);
    void calculateLengthInitial(double);
    void calculateStrainPlastic(void);
    void calculateLengthPlastic(void);
    void calculateForce(void);
    void calculateLength0(void);
    void calculateStrainMax(void);
};