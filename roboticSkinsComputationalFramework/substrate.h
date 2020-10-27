#pragma once
#include <Eigen/Dense>
#include <array>

class substrate
{
public:
    substrate();
    ~substrate();

    void changeInitialLength(double);
    void changeInitialWidth(double);
    void changeCurrentLength(double);
    void changeCurrentWidth(double);
    void changeSpringConstants(std::array<double, 2>);
    void changeStiffnessConstants(std::array<double, 3>);

    double getInitialLength(void);
    double getInitialWidth(void);
    double getCurrentLength(void);
    double getCurrentWidth(void);
    std::array<double, 2> getSpringConstants();
    std::array<double, 3> getStiffnessConstants();
    std::array<double, 2> getChangeInLengths();
    double getBendingStiffness(void);
    void getForceVector(Eigen::Ref<Eigen::Vector4d>);

private:
    double initialLength;
    double initialWidth;
    double currentLength;
    double currentWidth;
    std::array<double, 2> springConstants;
    std::array<double, 2> changeInLengths;
    double bendingStiffness;
    std::array<double, 3> stiffnessConstants;
    Eigen::Vector4d forceVector;

    void changeChangeInLengths(std::array<double, 2>);
    void changeBendingStiffness(double);
    void changeForceVector(Eigen::Ref<Eigen::Vector4d>);

    void calculateChangeInLengths(void);
    void calculateBendingStiffness(void);
    void calculateForceVector(void);
};