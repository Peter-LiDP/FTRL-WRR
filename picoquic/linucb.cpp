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
#include "linucb.h"
#include <queue>
#include <map> 
#include "globals.h"
#include "wrapper.h"
#include <Eigen/Dense>
using namespace Eigen; 

extern "C" {
    bool learning = true;
    bool deployment = false;
    int current_timestep = 0;
}

LINUCB::LINUCB(int numPaths)
    : numPaths(numPaths), alpha(0.8), t(0),
      A_w(Eigen::MatrixXd::Identity(6, 6)),
      b_w(Eigen::VectorXd::Zero(6)),
      A_t(Eigen::MatrixXd::Identity(6, 6)),
      b_t(Eigen::VectorXd::Zero(6)),
      theta_w(Eigen::VectorXd::Zero(6)),
      theta_t(Eigen::VectorXd::Zero(6)){}
 
void LINUCB::manual_update() {
    t++;
    current_timestep = t;
}

void LINUCB::update(const double reward, int updating_timestep, const std::vector<double>& x_t_vec) {
    Eigen::Map<const Eigen::VectorXd> x_t(x_t_vec.data(), x_t_vec.size());

    int action = updating_timestep % 2 == 0 ? 0 : 1;
    if (action == 0) {
        feature_reward_map_t[x_t_vec] = reward;
        feature_t.push(x_t_vec);
        A_t += x_t * x_t.transpose();
        b_t += reward * x_t;
        theta_t = A_t.inverse() * b_t;
        count_t++;
        double expected_reward_t = x_t.transpose() * theta_t + alpha * std::sqrt(x_t.transpose() * A_t.inverse() * x_t);
        e += std::abs(expected_reward_t - reward);
    }
    if (action == 1) {
        feature_reward_map_w[x_t_vec] = reward;
        feature_w.push(x_t_vec);
        A_w += x_t * x_t.transpose();
        b_w += reward * x_t;
        theta_w = A_w.inverse() * b_w;
        count_w++;
        double expected_reward_w = x_t.transpose() * theta_w + alpha * std::sqrt(x_t.transpose() * A_w.inverse() * x_t);
        f += std::abs(expected_reward_w - reward);
    }
}

int LINUCB::select_action(const std::vector<double>& x_t_vec) {
    Eigen::Map<const Eigen::VectorXd> x_t(x_t_vec.data(), x_t_vec.size());
    double expected_reward_t = x_t.transpose() * theta_t + alpha * std::sqrt(x_t.transpose() * A_t.inverse() * x_t);
    double expected_reward_w = x_t.transpose() * theta_w + alpha * std::sqrt(x_t.transpose() * A_w.inverse() * x_t);
    if (total >= 800) {
        if (total != 0) {
            qd_t = score_t / total;
            qd_w = score_w / total;
        }
        else {
            qd_t = 0;
            qd_w = 0;
        }
        if (std::abs(qd_t - q_t) >= 0.15 || std::abs(qd_w - q_w) >= 0.15) {
            learning = true;
            deployment = false;
            stop_collect = false;
            learning_bytes = 0;
            total = 0;
            score_t = 0;
            score_w = 0;
        }
    }
    
    if (expected_reward_t >= expected_reward_w) {
        score_t++;
        total++;
        return 0;
    } else {
        score_w++;
        total++;
        return 1;
    }
}

int LINUCB::get_timestep() {
    return t; 
}

MatrixXd LINUCB::computeCovarianceMatrix(const std::vector<std::vector<double>>& vectors) {
    int n = vectors.size();
    int d = vectors[0].size();
    MatrixXd data(n, d);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < d; ++j) {
            data(i, j) = vectors[i][j];
        }
    }
    VectorXd mean = data.colwise().mean();
    MatrixXd centered = data.rowwise() - mean.transpose();
    MatrixXd cov = (centered.adjoint() * centered) / double(data.rows() - 1);
    return cov;
}

double LINUCB::computeMahalanobisDistance(const std::vector<double>& vec1, const std::vector<double>& vec2, const MatrixXd& covInv) {
    VectorXd diff = Map<const VectorXd>(vec1.data(), vec1.size()) - Map<const VectorXd>(vec2.data(), vec2.size());
    return std::sqrt(diff.transpose() * covInv * diff);
}

std::vector<std::pair<std::vector<double>, std::vector<double>>> LINUCB::findClosestPairs(const FeatureRewardMap& map1, const FeatureRewardMap& map2) {
    std::vector<std::vector<double>> vectors1, vectors2;
    for (const auto& item : map1) {
        vectors1.push_back(item.first);
    }
    for (const auto& item : map2) {
        vectors2.push_back(item.first);
    }
    std::vector<std::vector<double>> combinedVectors = vectors1;
    combinedVectors.insert(combinedVectors.end(), vectors2.begin(), vectors2.end());
    MatrixXd cov = computeCovarianceMatrix(combinedVectors);
    MatrixXd covInv = cov.inverse();
    std::vector<std::pair<std::vector<double>, std::vector<double>>> pairs;
    std::vector<bool> used(vectors2.size(), false);
    for (const auto& vec1 : vectors1) {
        double minDistance = std::numeric_limits<double>::max();
        int minIndex = -1;
        for (int i = 0; i < vectors2.size(); ++i) {
            if (!used[i]) {
                double distance = computeMahalanobisDistance(vec1, vectors2[i], covInv);
                if (distance < minDistance) {
                    minDistance = distance;
                    minIndex = i;
                }
            }
        }
        if (minIndex != -1) {
            pairs.emplace_back(vec1, vectors2[minIndex]);
            used[minIndex] = true;
        }
    }
    return pairs;
}

std::vector<double> LINUCB::calculate_q() {
    std::vector<std::pair<std::vector<double>, std::vector<double>>> pairs = LINUCB::findClosestPairs(feature_reward_map_t, feature_reward_map_w);
    int trans = 0;
    int wait = 0;
    
    for (const auto& pair : pairs) {
        double reward_t = feature_reward_map_t.at(pair.first);
        double reward_w = feature_reward_map_w.at(pair.second);
        if (reward_t >= reward_w) {
            trans++;
        } else {
            wait++;
        }
    }
    
    int total_q = trans + wait;
    double score_t_q = 0;
    double score_w_q = 0;
    
    if (total_q != 0) {
       score_t_q = static_cast<double>(trans) / total_q;
       score_w_q = static_cast<double>(wait) / total_q;
    }
    else {
       score_t_q = 0;
       score_w_q = 0;
    }
    
    return std::vector<double>{score_t_q, score_w_q};
}



