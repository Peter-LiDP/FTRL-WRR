// linucb.h
#ifndef LINUCB_H
#define LINUCB_H
  
#include <vector>
#include <functional>
#include <queue>
#include <map>  
#include <random>
#include "wrapper.h"
#include <Eigen/Dense>
using namespace Eigen; 

struct VectorComparator {
    bool operator()(const std::vector<double>& lhs, const std::vector<double>& rhs) const {
        return std::lexicographical_compare(lhs.begin(), lhs.end(), rhs.begin(), rhs.end());
    }
};

typedef std::map<std::vector<double>, double, VectorComparator> FeatureRewardMap;

class LINUCB {
public:  
    LINUCB(int numPaths);  
    Eigen::MatrixXd A_t;
    Eigen::VectorXd b_t;
    Eigen::VectorXd theta_t;
    Eigen::MatrixXd A_w;
    Eigen::VectorXd b_w;
    Eigen::VectorXd theta_w;
    FeatureRewardMap feature_reward_map_t;
    FeatureRewardMap feature_reward_map_w;
    std::vector<double> calculate_q();
    double e;
    double f;
    double p_indiff_t;
    double p_indiff_w; 
    double p_t;
    double p_w;
    double count_t;
    double count_w;
    double q_t;
    double q_w;
    double qd_t;
    double qd_w;
    double score_t;
    double score_w;
    double total;
    std::queue<std::vector<double>> feature_t;
    std::queue<std::vector<double>> feature_w;
    int select_action(const std::vector<double>& x_t_vec);
    void update(const double reward, int updating_timestep, const std::vector<double>& x_t_vec);   
    void calculateAlpha(int currentTimeStep);
    void manual_update();
    int numPaths; 
    int get_timestep();
    std::map<uint64_t, std::tuple<int, double, bool, uint64_t, double>> packetNumbertoTimestepMap0;
    std::map<uint64_t, std::tuple<int, double, bool, uint64_t, double>> packetNumbertoTimestepMap1;
    std::map<int, std::tuple<uint64_t, bool, bool, double>> TimestepMap; 
    std::map<int, double> XtAtTimeStep;
    Eigen::MatrixXd computeCovarianceMatrix(const std::vector<std::vector<double>>& vectors);
    double computeMahalanobisDistance(const std::vector<double>& vec1, const std::vector<double>& vec2, const Eigen::MatrixXd& covInv);
    std::vector<std::pair<std::vector<double>, std::vector<double>>> findClosestPairs(const FeatureRewardMap& map1, const FeatureRewardMap& map2);


private:
    double alpha;
    int t;
};

#endif // LINUCB_H

