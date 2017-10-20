#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size() || estimations.empty()) {
        cout << "Ivalid estimation or ground_truth data" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for (int i = 0; i < estimations.size(); i++) {
        VectorXd residual = estimations[i] - ground_truth[i];

        residual = residual.array() * residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    return rmse;
}