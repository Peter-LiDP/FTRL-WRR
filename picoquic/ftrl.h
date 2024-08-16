// ftrl.h
#ifndef FTRL_H
#define FTRL_H

#include <vector> 
#include <functional>
#include <queue>
#include <map>  
#include <random>
#include "wrapper.h"

class FTRL {
public: 
    FTRL(int numPaths);     
    void update(const double loss, int updating_timestep);
    void second_timestep_update(); 
    std::pair<int, int> drawAction();  
    int numPaths; 
    double lr;
    double r; 
    int updated_t;
    std::map<uint64_t, std::tuple<int, double, bool, uint64_t>> packetNumbertoTimestepMap0;
    std::map<uint64_t, std::tuple<int, double, bool, uint64_t>> packetNumbertoTimestepMap1;
    std::map<int, std::tuple<uint64_t, bool, bool, double, uint64_t, uint64_t, double, double>> TimestepMap; 
    std::map<int, double> XtAtTimeStep;

private:
    double b;
    double R;
    double lower_xt;
    double upper_Xt;
    double Ut;
    double gt;
    double sampleUnitSphere();
    void calculate_lower_xt();
    double R_second_derivative();
    double sum_g;
    std::vector<double> currentWeights;
    std::vector<double> effectiveWeight;
    std::map<int, double> bAtTimeStep;
    std::map<int, double> RAtTimeStep;
    double totalWeight;
    int t;
    bool resetWRR; 
};

#endif // FTRL_H

