#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>

// 点到线的残差计算
struct LidarEdgeFactor
{

public:
    Eigen::Vector3d curr_point,last_point_a,last_point_b;
    double s;



    LidarEdgeFactor(Eigen::Vector3d curr_point_,Eigen::Vector3d last_point_a_,
                    Eigen::Vector3d last_point_b_,double s_)
        :curr_point(curr_point_),last_point_a(last_point_a_),last_point_b(last_point_b_),s(s_)
    {}

    // q 旋转  t 移动  residual 残差
    template<typename T>
    bool operator()(const T *q,const T *t,T* residual)const
    {
        Eigen::Matrix<T,3,1> cp{T(curr_point.x()),T(curr_point.y()),T(curr_point.z())};
        Eigen::Matrix<T,3,1> lpa{T(last_point_a.x()),T(last_point_a.y()),T(last_point_a.z())};
        Eigen::Matrix<T,3,1> lpb{T(last_point_b.x()),T(last_point_b.y()),T(last_point_b.z())};

        Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]}
        Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};

        q_last_curr = q_identity.slerp(T(s),q_last_curr);
        Eigen::Matrix<T,3,1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2], T(s) * t[3]};

        Eigen::Matrix<T,3,1> lp;
        lp = q_last_curr * cp + t_last_curr;
        Eigen::Matrix<T,3,1> nu = (lp - lpa).cross(lp - lpb);
        Eigen::Matrix<T,3,1> de = lpa - lpb;

        residual[0] = nu.x() / de.norm();
        residual[1] = nu.y() / de.norm();
        residual[2] = nu.z() / de.norm();
        return true;
    }



};







