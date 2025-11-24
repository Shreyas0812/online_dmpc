//
// Created by raorane on 24/11/25.
//

#include "avoidance.h"
#include <iostream>

using namespace std;
using namespace Eigen;

BVCAvoider::BVCAvoider (const std::vector<Eigen::MatrixXd>& horizon,
                        const Eigen::MatrixXd& Phi_pos,
                        const std::vector<EllipseParams>& p,
                        int deg_poly) :
    _horizon(horizon),
    _Phi_pos(Phi_pos),
    _deg_poly(deg_poly)
{
    Ellipse tmp;
    MatrixXd E;
    for (int i = 0; i < p.size(); i++) {
        tmp.order = p[i].order;
        tmp.rmin = p[i].rmin;
        E = p[i].c.asDiagonal();
        tmp.E1 = E.inverse();
        tmp.E2 = tmp.E1.array().pow(2);
        _ellipse.push_back(tmp);
    }

    _k_hor = _horizon[0].cols();
    _dim = _horizon[0].rows();
    _N = _horizon.size();
}

Constraint BVCAvoider::getCollisionConstraint(const State3D& state, int agent_id) {
    return buildBVCConstraintForAgent(agent_id, state);
}

Constraint BVCAvoider::buildBVCConstraintForAgent(int agent_id, const State3D& state) {
    using namespace Eigen;

    int num_variables = _Phi_pos.cols();
    
    // BVC: Check entire horizon for all neighbours (proactive, not reactive)
    std::vector<std::pair<int, int>> active_pairs;  // (timestep, neighbour)
    
    for (int k = 0; k < _k_hor; k++) {
        VectorXd pi_k = _horizon[agent_id].col(k);
        
        for (int j = 0; j < _N; j++) {
            if (j == agent_id) continue;
            
            VectorXd pj_k = _horizon[j].col(k);
            VectorXd diff = _ellipse[agent_id].E1 * (pi_k - pj_k);
            int order = _ellipse[agent_id].order;
            double dist = pow(((diff.array().pow(order)).sum()), 1.0 / order);
            
            // Add constraint if within a larger safety region for BVC partitioning
            if (dist < _ellipse[agent_id].rmin * 3.0) {
                active_pairs.push_back({k, j});
            }
        }
    }
    
    // If no close neighbours, return empty constraint
    if (active_pairs.empty()) {
        return {MatrixXd::Zero(0, num_variables), VectorXd::Zero(0)};
    }

    int num_neighbours = active_pairs.size();
    MatrixXd Ain(num_neighbours, num_variables);
    VectorXd bin(num_neighbours);
    VectorXd distance(num_neighbours);

    for (int idx = 0; idx < active_pairs.size(); idx++) {
        int k = active_pairs[idx].first;
        int j = active_pairs[idx].second;

        // Predicted positions from old horizon
        VectorXd pi_k = _horizon[agent_id].col(k);
        VectorXd pj_k = _horizon[j].col(k);

        // Compute ellipsoidal distance
        int order = _ellipse[agent_id].order;
        VectorXd diff_raw = pi_k - pj_k;
        VectorXd diff_scaled = _ellipse[agent_id].E1 * diff_raw;
        double d_ij = pow(((diff_scaled.array().pow(order)).sum()), 1.0 / order);

        // Compute gradient for constraint
        VectorXd diff_grad = (_ellipse[agent_id].E2 * diff_raw).array().pow(order - 1);
        double dist_pow = pow(d_ij, order - 1);

        // Build constraint: Keep agent i on its side of the hyperplane
        MatrixXd diff_row = MatrixXd::Zero(1, _dim * _k_hor);
        diff_row.middleCols(_dim * k, _dim) = diff_grad.transpose();

        Ain.row(idx) = -diff_row * _Phi_pos;
        bin(idx) = -dist_pow * (_ellipse[agent_id].rmin - d_ij) - diff_grad.dot(pi_k);
        distance(idx) = dist_pow;
    }

    // Build soft constraint with slack variables (same format as OnDemand)
    MatrixXd Ain_soft = MatrixXd::Zero(2 * num_neighbours, num_variables + num_neighbours);
    VectorXd bin_soft = VectorXd::Zero(2 * num_neighbours);

    MatrixXd diag_distance = distance.asDiagonal();

    Ain_soft << Ain, diag_distance,
                MatrixXd::Zero(num_neighbours, num_variables), MatrixXd::Identity(num_neighbours, num_neighbours);

    bin_soft << bin, VectorXd::Zero(num_neighbours);

    return {Ain_soft, bin_soft};
}