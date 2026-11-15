#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include "cubic_spline/arc_length_spline.hpp"
#include "fusion.h"
#define private public
#include "corridor/corGen.hpp"
#undef private
#include "corridor/corridor_lbfgs.hpp"
#include <iostream>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("corridor_contract");
    node->declare_parameter("n_sweep", 7);
    node->declare_parameter("MaxPointCloud", 10000);
    corGen gen;
    gen.init(*node);
    Eigen::VectorXd x(5), y(5), z(5);
    x << -4,-2,0,2,4;
    y << -1,-1.4,-1.3,-.8,.2;
    z << 1.3,1.4,1.7,1.6,1.5;
    ArcLengthSpline spline;
    spline.gen3DSpline(x,y,z);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for(int ix=0;ix<=80;++ix) for(int iz=0;iz<=24;++iz) {
        cloud.points.emplace_back(-4.f+.1f*ix,-2.7f,.3f+.1f*iz);
        cloud.points.emplace_back(-4.f+.1f*ix,2.f,.3f+.1f*iz);
    }
    cloud.width=cloud.size(); cloud.height=1; cloud.is_dense=true;
    gen.setCloudMap(cloud);
    const auto coeff=gen.corGeneratorPTF(spline);
    const int segments=coeff.coeffs_a.rows(), degree=gen.getPolyDegree();
    corridor_lbfgs::CorridorProblem prob;
    prob.n_segments=segments; prob.degree=degree;
    prob.total_arc_length=spline.getLength_3D(); prob.eps_psd=.1;
    const int count=prob.ncp();
    std::vector<double> raw(6*count,0.0);
    for(int seg=0;seg<segments;++seg) for(int k=0;k<=degree;++k) {
        const int i=seg*(degree+1)+k;
        const double l11=std::sqrt(coeff.coeffs_a(seg,k)-.1);
        const double l21=coeff.coeffs_c(seg,k)/l11;
        const double l22=std::sqrt(coeff.coeffs_b(seg,k)-.1-l21*l21);
        const double l31=coeff.coeffs_d(seg,k)/(2*l11);
        const double l32=(coeff.coeffs_e(seg,k)/2-l21*l31)/l22;
        raw[i]=corridor_lbfgs::inv_softplus(l11);
        raw[count+i]=l21;
        raw[2*count+i]=corridor_lbfgs::inv_softplus(l22);
        raw[3*count+i]=l31; raw[4*count+i]=l32;
    }
    double maximum=0;
    for(int i=1;i<100;++i) {
        const double s=i*prob.total_arc_length/100;
        double a,b,c,d,e;
        prob.evalEllipseAt(raw.data(),s,a,b,c,d,e);
        const std::vector<double> expected{a,b,c,d,e};
        const auto actual=gen.polynomial_eval(s,coeff,degree);
        for(int k=0;k<5;++k) maximum=std::max(maximum,std::abs(actual[k]-expected[k]));
    }
    std::cout << "ellipse_representation_max_error=" << maximum << '\n';
    bool equal=std::isfinite(maximum) && maximum<1e-8;
    std::cout << (equal?"PASS ":"FAIL ") << "optimizer and NMPC use the same interpolated ellipse\n";

    // An occupied point between coarse path anchors must survive projection.
    x.resize(2);y.resize(2);z.resize(2);
    x<<-4,4; y<<0,0; z<<1.5,1.5;
    ArcLengthSpline straight;
    straight.gen3DSpline(x,y,z);
    gen.updateSpline(straight); gen.PTF_evaluate(straight);
    pcl::PointCloud<pcl::PointXYZ> single;
    single.points.emplace_back(.1234f,.2f,1.5f);
    single.width=1;single.height=1;single.is_dense=true;
    gen.setCloudMap(single);
    const auto projected=gen.processPointCloud("PTF");
    bool retained=false;
    for(size_t i=0;i<projected.all_raw_pos.size();++i) {
        const auto& p=projected.all_raw_pos[i];
        if(std::abs(projected.all_raw_s[i]-4.1234)<.003 &&
           std::abs(p.tail<2>().norm()-.2)<.003) retained=true;
    }
    std::cout << (retained?"PASS ":"FAIL ") << "obstacle between path anchors retained at its projected arc length\n";
    // A low-altitude spline must not create a PTF corridor that includes
    // points below the ROG-Map floor.
    z << .49,.49;
    ArcLengthSpline low;
    low.gen3DSpline(x,y,z);
    gen.setCloudMap({});
    const auto low_coeff = gen.corGeneratorPTF(low);
    bool floor_excluded = low_coeff.success;
    for (int i=1;i<20;++i) {
        const double s = i * low.getLength_3D() / 20.;
        const auto pt = low.getPosition_3D(s);
        const auto R = gen.PTF_at_s(s, low);
        const Eigen::Vector3d below(pt.x(),pt.y(),-.05);
        const Eigen::Vector2d yz = (R.transpose() * (below-pt)).tail<2>();
        const auto e = gen.polynomial_eval(s,low_coeff,degree);
        const double g = e[0]*yz.x()*yz.x()+e[1]*yz.y()*yz.y()
                       +2*e[2]*yz.x()*yz.y()+e[3]*yz.x()+e[4]*yz.y()-1.;
        if (!(g > 0)) floor_excluded=false;
    }
    std::cout << (floor_excluded?"PASS ":"FAIL ") << "low-altitude corridor excludes points below z=0\n";
    rclcpp::shutdown();
    return equal && retained && floor_excluded ? 0:1;
}
