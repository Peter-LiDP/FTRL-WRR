#include <fstream>
#include <functional>
#include <string>
#include <algorithm> 
#include <iterator>
#include <vector>
#include <numeric>
#include <random>
#include <cmath>
#include <iostream>
#include "ftrl.h"
#include <queue>
#include <map> 
#include "globals.h"
#include "wrapper.h"
extern "C" {
    bool updated = false;  
} 
bool clearQueue = false;

FTRL::FTRL(int numPaths)
    : numPaths(numPaths), lr(0.99), r(1), t(1), reset_t(1), updated_t(1), sum_g(0), sum_g_without_lr(0), currentWeights(numPaths, 0){
    effectiveWeight.resize(numPaths);
    gt = 0;
    totalWeight = 1;
    lower_xt = 1.0 / numPaths;
    R = R_second_derivative();
    Ut = sampleUnitSphere();
    b = (r/2)*Ut*(1/sqrt(R));
    upper_Xt = lower_xt + b;
    RAtTimeStep[1] = R;
    bAtTimeStep[1] = b;
    resetWRR = true;
}

double FTRL::sampleUnitSphere() {
    double point = ((t / 2) % 2 * 2 - 1);  
    return point;
}

double calculate_lr(double t) {
    double logt = log(t);
    double eta = pow(2, -0.5) * pow(logt, 0.75) * pow(t, -0.75);
    return eta;
}

double calculate_r(double t) {
    double logt = log(t);
    double radius = pow(2, 0.5)*pow(t, -0.25)*pow(logt, 0.25);
    return std::min(radius, 1.0);
}

void FTRL::calculate_lower_xt() { 
    if (sum_g == 0) {
        lower_xt = 0.5;
    }
    else {
        lower_xt = (sum_g + 2 - sqrt(sum_g*sum_g + 4)) / (2*sum_g);
    }
}

double FTRL::R_second_derivative() {
    double R_t = (2*pow(lower_xt, 2) - 2*lower_xt + 1) / (pow((lower_xt - pow(lower_xt, 2)), 2));
    return R_t;
}

void FTRL::second_timestep_update() {
    t++;
    Ut = sampleUnitSphere();
    b = (r/2)*Ut*(1/sqrt(R));
    upper_Xt = lower_xt + b;
    bAtTimeStep[t] = b;
    RAtTimeStep[t] = R;
    resetWRR = true;
    updated = true;
    clearQueue = true;
    updated_t = t;
}

void FTRL::update(const double loss, int updating_timestep) {
    t++;
    double update_b, update_R;
    update_b = bAtTimeStep[updating_timestep];
    update_R = RAtTimeStep[updating_timestep];
    bAtTimeStep.erase(updating_timestep);
    RAtTimeStep.erase(updating_timestep);
    lr = calculate_lr(reset_t + 1);
    r = calculate_r(reset_t + 1);
    gt = (4*loss*update_R*update_b) / (r * r);
    sum_g_without_lr += gt;
    sum_g = sum_g_without_lr*lr;
    calculate_lower_xt();
        
    R = R_second_derivative(); 
    Ut = sampleUnitSphere();
    b = (r/2)*Ut*pow(R, -0.5);
    upper_Xt = lower_xt + b;  
    bAtTimeStep[t] = b;
    RAtTimeStep[t] = R;
    XtAtTimeStep[t] = upper_Xt;    
    resetWRR = true;
    updated = true;
    clearQueue = true;
    updated_t = t;
    reset_t++;
}

std::pair<int, int> FTRL::drawAction() {
    if (resetWRR) {
        effectiveWeight = {upper_Xt, 1 - upper_Xt}; 
        if (probing_finished) {
            double path0_weight = (double)path0mtu/(path0mtu + path1mtu);
            double path1_weight = (double)path1mtu/(path0mtu + path1mtu);
            double path_0_scaled_weight = upper_Xt / path0_weight;
            double path_1_scaled_weight = (1 - upper_Xt) / path1_weight;
            double effective_weight_0 = path_0_scaled_weight / (path_0_scaled_weight + path_1_scaled_weight);
            double effective_weight_1 = path_1_scaled_weight / (path_0_scaled_weight + path_1_scaled_weight);
            effectiveWeight = {effective_weight_0, effective_weight_1};
        }
        currentWeights.assign(numPaths, 0);
        resetWRR = false;  
    }
    int select = -1;
    double maxCurrentWeight = std::numeric_limits<double>::lowest();

    for (int i = 0; i < numPaths; ++i) {
        currentWeights[i] += effectiveWeight[i]; 
        if (currentWeights[i] > maxCurrentWeight) {
            maxCurrentWeight = currentWeights[i];
            select = i;
        }
    }

    if (select != -1) { 
        currentWeights[select] -= totalWeight;
    }

    return std::make_pair(select, updated_t);
}


