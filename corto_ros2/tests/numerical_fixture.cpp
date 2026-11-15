#include <Eigen/Eigen>
#include <random>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <memory>
#include <map>
#include <vector>
#include "map/map.hpp"
#include "raycast/raycast.hpp"
#ifdef CORTO_UPSTREAM
#include <ros/ros.h>
#else
#include <rclcpp/rclcpp.hpp>
#endif
// Seed only the original PRM engine; no numerical or search function is changed.
#define private public
#include "path_searching/topo_prm.hpp"
#undef private
#include "cubic_spline/arc_length_spline.hpp"
#include "optimization/ref_traj_opt.hpp"
#include "corridor/corGen.hpp"
#include "nmpc/nominal_nmpc_cor.hpp"

template<typename Derived>
void dump(std::ostream& out, const std::string& name, const Eigen::MatrixBase<Derived>& v) {
    out << name << " " << v.size();
    for (Eigen::Index row = 0; row < v.rows(); ++row)
        for (Eigen::Index col = 0; col < v.cols(); ++col) out << " " << v(row,col);
    out << "\n";
}
void scalar(std::ostream& out, const std::string& name, double v) { out << name << " 1 " << v << "\n"; }

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    std::ofstream out(argv[1]);
    out << std::setprecision(17);
#ifdef CORTO_UPSTREAM
    ros::NodeHandle node;
    GridMap map(0.1, Eigen::Vector3d(30,15,3));
    for (int x=145; x<155; ++x) for (int y=70; y<80; ++y) for (int z=0; z<30; ++z)
        map.set_grid(Eigen::Vector3i(x,y,z), true);
#else
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<rclcpp::Node>("numerical_fixture");
    auto& node = *node_ptr;
    node.declare_parameter("n_sweep", 7);
    node.declare_parameter("MaxPointCloud", 10000);
    node.declare_parameter("nmpc.maxVel", 10.0);
    node.declare_parameter("nmpc.real_maxVel", 10.0);
    GridMap map;
    map.init(node);
#endif
    TopologyPRM prm;
    prm.setGridMap(&map);
    prm.init(node);
    prm.eng_.seed(20261115);
    std::list<GraphNode::Ptr> graph;
    std::vector<std::vector<Eigen::Vector3d>> raw, filtered;
    prm.findTopoPaths(Eigen::Vector3d(-4,0,1.5), Eigen::Vector3d(4,0,1.5), {}, {}, graph, raw, filtered);
    scalar(out, "topo.filtered_count", filtered.size());
    for (size_t i=0; i<filtered.size(); ++i) {
        Eigen::MatrixXd points(filtered[i].size(),3);
        for(size_t j=0;j<filtered[i].size();++j) points.row(j)=filtered[i][j];
        dump(out, "topo.path."+std::to_string(i), points);
    }
    if (filtered.empty()) return 3;

    std::vector<Eigen::Vector3d> anchors = {{-4,-1,1.3},{-2,-1.4,1.4},{0,-1.3,1.7},{2,-.8,1.6},{4,.2,1.5}};
    std::vector<Eigen::Vector3d> dense;
    for(size_t i=0;i+1<anchors.size();++i)
        for(int j=0;j<30;++j) dense.push_back(anchors[i]+(j/30.0)*(anchors[i+1]-anchors[i]));
    dense.push_back(anchors.back());
    Eigen::VectorXd X(dense.size()),Y(dense.size()),Z(dense.size());
    for(size_t i=0;i<dense.size();++i){X(i)=dense[i].x();Y(i)=dense[i].y();Z(i)=dense[i].z();}
    ArcLengthSpline spline;
    spline.gen3DSpline(X,Y,Z);
    scalar(out,"spline.length",spline.getLength_3D());
    Eigen::MatrixXd spline_samples(100,9);
    for(int i=0;i<100;++i){
        double s=i*spline.getLength_3D()/99;
        spline_samples.row(i).segment<3>(0)=spline.getPosition_3D(s);
        spline_samples.row(i).segment<3>(3)=spline.getDerivative_3D(s);
        spline_samples.row(i).segment<3>(6)=spline.getSecondDerivative_3D(s);
    }
    dump(out,"spline.samples",spline_samples);

    RefTrajOpt timing;
    timing.socp_interface(100,0,0,spline,10,5,true,100);
    scalar(out,"socp.total_time",timing.get_total_time());
    Eigen::Map<const Eigen::VectorXd> times(timing.get_result_t().data(),timing.get_result_t().size());
    dump(out,"socp.times",times);
    if(timing.get_result_t().size()!=100) return 4;

    // The physical initial speed is 2 m/s; upstream's boundary argument is v^2.
    RefTrajOpt moving_start;
#ifdef CORTO_UPSTREAM
    moving_start.socp_interface(100,4.0,0,spline,10,5,true,100);
#else
    moving_start.socp_interface(100,2.0,0,spline,10,5,true,100);
#endif
    scalar(out,"socp.moving_start_time",moving_start.get_total_time());
    Eigen::Map<const Eigen::VectorXd> moving_times(moving_start.get_result_t().data(),moving_start.get_result_t().size());
    dump(out,"socp.moving_start_samples",moving_times);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    for(int ix=0;ix<=80;++ix) for(int iz=0;iz<=24;++iz) {
        float x=-4.0f+0.1f*ix, z=0.3f+0.1f*iz;
        cloud.points.emplace_back(x,-2.7f,z);
        cloud.points.emplace_back(x,2.0f,z);
    }
    cloud.width=cloud.size();cloud.height=1;cloud.is_dense=true;
    corGen corridor;
    corridor.init(node);
    corridor.setCloudMap(cloud);
    auto coeff=corridor.corGeneratorPTF(spline);
    scalar(out,"sfc.success",coeff.success);
    scalar(out,"sfc.volume",coeff.volume);
    dump(out,"sfc.a",coeff.coeffs_a);dump(out,"sfc.b",coeff.coeffs_b);
    dump(out,"sfc.c",coeff.coeffs_c);dump(out,"sfc.d",coeff.coeffs_d);dump(out,"sfc.e",coeff.coeffs_e);
    if(!coeff.success) return 5;

    NominalNMPC nmpc(.6,"LD_SLSQP",200,.05,node);
    Eigen::Matrix<double,11,1> weights;
    weights << 0,0,.005,.2,.2,.2,.2,60,100,10,5;
    nmpc.set_w(weights);
    RefData ref;
    ref.TerminalPos=Eigen::Vector3d(10,0,1.5);
    for(int k=0;k<=10;++k) {
        ref.ref_s.push_back(.1*k);
        ref.ref_pos.emplace_back(.1*k,0,1.5);
        ref.ref_fsf.push_back(Eigen::Matrix3d::Identity());
        ref.ref_coeffs.push_back({1,1,0,0,0});
    }
    nmpc.set_ref_data(ref);
    Eigen::Matrix<double,10,1> state;
    state << 0,0,1.5,2,0,0,1,0,0,0;
    Eigen::Vector4d u0(0,0,0,.6);
    Eigen::Matrix<double,10,4> U=Eigen::Matrix<double,10,4>::Zero();
    U.col(3).setConstant(.6);
    Eigen::Matrix<double,10,10> pred;
    const int status=nmpc.solve(state,u0,U,pred);
    scalar(out,"nmpc.status",status);
    dump(out,"nmpc.controls",U);dump(out,"nmpc.prediction",pred);
    if(status<=0) return 6;
#ifndef CORTO_UPSTREAM
    auto executed = state;
    for(int k=0;k<10;++k) {
        Eigen::Matrix<double,10,1> next;
        nmpc.quad_dynamic_.rk4_func(executed,U.row(k).transpose(),.05,next);
        if((next-pred.row(k).transpose()).cwiseAbs().maxCoeff()>1e-9) return 7;
        executed=next;
    }
#endif

    Eigen::Matrix<double,10,1> next;
    Eigen::Matrix<double,10,10> gx;
    Eigen::Matrix<double,10,4> gu;
    Eigen::Vector3d acc;
    Eigen::Matrix<double,3,10> ax;
    Eigen::Matrix<double,3,4> au;
    Eigen::Vector4d input(.3,-.2,.1,.65);
    nmpc.quad_dynamic_.rk4_func(state,input,.05,next,gx,gu,acc,ax,au);
    dump(out,"rk4.next",next);dump(out,"rk4.gx",gx);dump(out,"rk4.gu",gu);
    dump(out,"rk4.acc",acc);dump(out,"rk4.acc_gx",ax);dump(out,"rk4.acc_gu",au);
    out.close();
#ifndef CORTO_UPSTREAM
    rclcpp::shutdown();
#endif
    return 0;
}
