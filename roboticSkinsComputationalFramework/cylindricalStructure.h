#pragma once
#include <Eigen/Dense>

class cylindricalStructure
{
public:
    cylindricalStructure();
    ~cylindricalStructure();

    void changeOuterRadius(double);
    void changeInnerRadius(double);
    void changeLength(double);
    void changeElasticModulus(double);
    void changePoissonRatio(double);
    void changeState(Eigen::Ref<Eigen::Vector4d>);
    void calculateTransform(double, Eigen::Ref<Eigen::Matrix4d>);

    double getOuterRadius(void);
    double getInnerRadius(void);
    double getLength(void);
    double getElasticModulus(void);
    double getPoissonRatio(void);
    void getState(Eigen::Ref<Eigen::Vector4d>);
    double getBendingStiffness(void);
    double getAxialStiffness(void);
    double getTorsionalStiffness(void);
    double getBucklingForce(void);

private:
    double outerRadius;
    double innerRadius;
    double length;
    double elasticModulus;
    double poissonRatio;
    double bendingStiffness;
    double axialStiffness;
    double torsionalStiffness;
    Eigen::Vector4d state;
    Eigen::Matrix4d transform;
    double bucklingForce;

    void calculateBendingStiffness(void);
    void calculateAxialStiffness(void);
    void calculateTorsionalStiffness(void);
    void calculateBucklingForce(void);
    double calculateSecondMomentOfInertia(void);
    double calculateCrossSectionalArea(void);
    double calculateRigidityModulus(void);
    double calculateSecondMomentOfArea(void);
    double calculateShearCorrectionFactor(void);
    void changeBendingStiffness(double);
    void changeAxialStiffness(double);
    void changeTorsionalStiffness(double);
    void changeTransform(Eigen::Ref<Eigen::Matrix4d>);
    void changeBucklingForce(double);
};
