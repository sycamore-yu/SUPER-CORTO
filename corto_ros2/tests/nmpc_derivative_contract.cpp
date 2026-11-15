#include <cmath>
#include <iostream>
#include <vector>
#include <nlopt.hpp>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#define private public
#include "nmpc/nominal_nmpc_cor.hpp"
#undef private

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("nmpc_derivative_contract");
    for (const auto& name : {"nmpc.maxVel", "nmpc.real_maxVel", "nmpc.contour_weight",
                             "nmpc.yaw_w"})
        node->declare_parameter(name, 10.0);
    NominalNMPC nmpc(0.6, "LD_SLSQP", 200, 0.05, *node);
    Eigen::Matrix<double, 11, 1> weights;
    weights << 0,0,.005,.2,.2,.2,.2,60,100,10,5;
    nmpc.set_w(weights);
    nmpc.x0_ << 0,0,.49, 1.1,.45,-.15, std::cos(.12),0,0,std::sin(.12);
    nmpc.u0_ << .12,-.09,.08,.59;

    RefData ref;
    for (int k = 0; k <= NominalNMPC::n_step_; ++k) {
        ref.ref_pos.emplace_back(.085*k,.02*k,.49);
        ref.ref_fsf.push_back(Eigen::Matrix3d::Identity());
        ref.ref_coeffs.push_back({3.1,3.1,0.,0.,0.});
    }
    ref.TerminalPos << 10.,0.,.49;
    nmpc.set_ref_data(ref);

    constexpr unsigned n = NominalNMPC::n_step_ * NominalQuadDynamic::u_dim_;
    constexpr unsigned m = NominalNMPC::n_step_;
    std::vector<double> u(n);
    for (unsigned k=0;k<m;++k) {
        u[4*k]=.12 + .014*k;u[4*k+1]=-.11 + .005*k;
        u[4*k+2]=.08;u[4*k+3]=.58 + .002*k;
    }
    std::vector<double> g(m), dg(m*n);
    nmpc.ellipsoid_constraint_impl(m,g.data(),n,u.data(),dg.data());
    std::vector<double> cg(n);
    nmpc.cost_func_impl(u,cg);
    double worst_ellipse=0, worst_cost=0;
    constexpr double h=1e-6;
    for (unsigned j=0;j<n;++j) {
        auto up=u, down=u;
        up[j]+=h;down[j]-=h;
        std::vector<double> gu(m),gd(m);
        nmpc.ellipsoid_constraint_impl(m,gu.data(),n,up.data(),nullptr);
        nmpc.ellipsoid_constraint_impl(m,gd.data(),n,down.data(),nullptr);
        for (unsigned k=0;k<m;++k) {
            const double numeric=(gu[k]-gd[k])/(2*h);
            const double error=std::abs(numeric-dg[k*n+j])/(1.+std::abs(numeric));
            worst_ellipse=std::max(worst_ellipse,error);
        }
        std::vector<double> unused_gradient;
        const double cp=nmpc.cost_func_impl(up,unused_gradient);
        const double cm=nmpc.cost_func_impl(down,unused_gradient);
        const double numeric=(cp-cm)/(2*h);
        worst_cost=std::max(worst_cost,std::abs(numeric-cg[j])/(1.+std::abs(numeric)));
    }
    std::cout << "max_ellipse_relative_error=" << worst_ellipse
              << " max_objective_relative_error=" << worst_cost << "\n";
    rclcpp::shutdown();
    return worst_ellipse < 2e-3 && worst_cost < 2e-3 ? 0 : 1;
}
