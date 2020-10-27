#pragma once
#include <Eigen/Dense>

class endCap
{
public:
    endCap();
    ~endCap();

    void changeLength(double);
    double getLength(void);
    void getTransform(Eigen::Ref<Eigen::Matrix4d>);

private:
    double length;
    Eigen::Matrix4d transform;

    void calculateTransform(void);
    void changeTransform(Eigen::Ref<Eigen::Matrix4d>);
};