#ifndef ROBOT_MODEL_H
#define ROBOT_MODEL_H

#include "robot_common.h"
#include <vector>
#include <memory>

/**
 * @brief 机器人类 - 包含运动学、动力学和控制算法
 */
class RobotModel
{
public:
    /**
     * @brief 构造函数
     * @param params 机器人参数
     */
    explicit RobotModel(const RobotParams& params = RobotParams());

    /**
     * @brief 设置机器人参数
     */
    void setParameters(const RobotParams& params);

    /**
     * @brief 获取机器人参数
     */
    const RobotParams& getParameters() const { return params_; }

    // ==================== 运动学计算 ====================

    /**
     * @brief 正运动学：关节空间 -> 工作空间
     * @param q 关节角度 [q1, q2, q3] (rad)
     * @param position 输出末端位置 (x, y, z) (m)
     * @return 是否成功
     */
    bool forwardKinematics(const Vector3f& q, Vector3f& position) ;

    /**
     * @brief 逆运动学：工作空间 -> 关节空间
     * @param position 末端位置 (x, y, z) (m)
     * @param q 输出关节角度 [q1, q2, q3] (rad)
     * @param elbow 肘部配置 (+1 或 -1)
     * @return 是否成功
     */
    bool inverseKinematics(const Vector3f& position, Vector3f& q, int elbow = -1);

    /**
     * @brief 计算雅可比矩阵
     * @param q 关节角度 [q1, q2, q3] (rad)
     * @return 3x3 雅可比矩阵
     */
    Matrix3f computeJacobian(const Vector3f& q);

    /**
    * @brief 计算变换矩阵
    * @param q 关节角度 [q1, q2, q3] (rad)
    */
    void computeTransforms(const Vector3f& q);

    /**
     * @brief 计算雅可比矩阵的导数
     * @param q 关节角度 [q1, q2, q3] (rad)
     * @param qd 关节速度 [qd1, qd2, qd3] (rad/s)
     * @return 3x3 雅可比矩阵导数
     */
    Matrix3f computeJacobianDerivative(const Vector3f& q, const Vector3f& qd) ;

    // ==================== 动力学计算 ====================


    /**
     * @brief 从C#代码移植的力矩计算函数 (Closed_Arm_Modle_decoup)
     * @param jointstate [q1, q2, q3, qd1, qd2, qd3, v1, v2, v3]
     * @return [tau1, tau2, tau3] 力矩
     */
    Vector3f computeTorqueDecoupled(const std::vector<float>& jointstate) ;

    /**
     * @brief 重力补偿力矩计算 (Closed_Arm_Modle_thetch)
     * @param q 关节角度 [q1, q2, q3] (rad)
     * @return [tau1, tau2, tau3] 重力补偿力矩
     */
    Vector3f computeGravityCompensation(const Vector3f& q) ;

        // 逆速度计算（直接传入雅可比矩阵）
    bool inverseVelocity(const Matrix3f& J, const Vector3f& end_vel, Vector3f& qd) ;
    
    // 使用QR分解的逆速度计算
    bool inverseVelocityQR(const Matrix3f& J, const Vector3f& end_vel, Vector3f& qd) ;
    
    // 使用阻尼最小二乘法的逆速度计算
    bool inverseVelocityDamped(const Matrix3f& J, const Vector3f& end_vel, Vector3f& qd, float lambda = 0.01f) ;
    
    Vector3f forwardVelocity(const Vector3f& q, const Vector3f& qd) ;

        // 逆加速度计算
    bool inverseAcceleration(const Vector3f& q, const Vector3f& qd, 
                            const Vector3f& end_acc, Vector3f& qdd) ;
    
    bool inverseAccelerationQR(const Vector3f& q, const Vector3f& qd, 
                              const Vector3f& end_acc, Vector3f& qdd) ;


    // 逆向动力学计算
    Eigen::Vector3f inverseDynamics(const Eigen::Vector3f& q,
                                   const Eigen::Vector3f& qd,
                                   const Eigen::Vector3f& qdd);


    Eigen::Vector3f inverseDynamicsL(const Eigen::Vector3f& q,
                                     const Eigen::Vector3f& qd,
                                     const Eigen::Vector3f& qdd);
    
    // 设置末端外力
    void setEndEffectorWrench(const Eigen::VectorXd& F_tip);
    
    // 获取关节力矩
    const Eigen::Vector3f& getJointTorques() const { return tau_; }

    void setFlag(bool flag) { flag_ = flag; }

    /**
     * @brief 计算重力补偿力矩 (Closed_Arm_Modle_thetch)
     * @param q 关节角度 [q1, q2, q3] (rad)
     * @return [tau1, tau2, tau3] 重力补偿力矩
     */
    Eigen::Vector3f gravityCompensation(const Eigen::Vector3f& q) const;



private:
    RobotParams params_;  // 机器人参数

    Matrix4d zero_config_pose_M_;  // 零位姿的齐次变换矩阵
    // 计算零位姿的齐次变换矩阵
    void calculateZeroConfigPoseM();

    Matrix4d dh_transform(double a, double alpha, double d, double theta);
    void Initialize() ;

    Eigen::Matrix3d skew(const Eigen::Vector3d& w) ;
    Eigen::Matrix4d expm_screw(const Eigen::VectorXd& S, double theta) ;
    Matrix6d adjoint(const Eigen::Matrix4d& T) ;
    void computeSpatialAccelerations(const Vector3f& q, const Vector3f& qd, const Vector3f& qdd);
    Eigen::Matrix<double, 6, 6> ad(const Eigen::VectorXd& A) ;//李括号运算

    void CalculateSList();
        // 计算伴随变换缓存
    void updateAdjointTransforms();

        // 计算零位变换矩阵
    void computeZeroTransforms();



    // 计算空间惯量矩阵
    void computeSpatialInertias();
    
    // 计算单个连杆的空间惯量矩阵
    Matrix6d computeSpatialInertia(int link_index) ;
    
    // 计算力旋量逆向迭代
    void computeWrenches();


    // 原有的成员变量
    std::vector<Eigen::VectorXd> S_list_;
    
    // 新增：旋量变换后的关节轴
    std::vector<Eigen::VectorXd> A_list_;  // A1, A2, A3
    
    // 新增：各关节的空间速度
    std::vector<Eigen::VectorXd> V_list_;  // V0, V1, V2, V3

    std::vector<Eigen::VectorXd> Vd_list_;  // Vd0, Vd1, Vd2, Vd3
    
    // 变换矩阵成员变量
    Eigen::Matrix4d T0_1_;
    Eigen::Matrix4d T1_2_;
    Eigen::Matrix4d T2_3_;
    Eigen::Matrix4d T3_4_;
    
    // 伴随变换矩阵缓存
    Matrix6d Ad_T0_1_inv_;
    Matrix6d Ad_T1_2_inv_;
    Matrix6d Ad_T2_3_inv_;
    Matrix6d Ad_T3_4_inv_;
    
    // 为了兼容现有代码，添加引用成员
    Eigen::VectorXd& S1;
    Eigen::VectorXd& S2;
    Eigen::VectorXd& S3;
    
    Eigen::VectorXd& A1;
    Eigen::VectorXd& A2;
    Eigen::VectorXd& A3;
    
    Eigen::VectorXd& V0;
    Eigen::VectorXd& V1;
    Eigen::VectorXd& V2;
    Eigen::VectorXd& V3;

    Eigen::VectorXd& Vd0;
    Eigen::VectorXd& Vd1;
    Eigen::VectorXd& Vd2;
    Eigen::VectorXd& Vd3;




    // 零位变换矩阵（固定不变）
    Eigen::Matrix4d M1_;  // T0_1_zero
    Eigen::Matrix4d M2_;  // T0_1_zero * T1_2_zero
    Eigen::Matrix4d M3_;  // T0_1_zero * T1_2_zero * T2_3_zero
    
    // 逆变换缓存
    Eigen::Matrix4d M1_inv_;
    Eigen::Matrix4d M2_inv_;
    Eigen::Matrix4d M3_inv_;
    
    // 伴随变换缓存
    Matrix6d Ad_M1_inv_;
    Matrix6d Ad_M2_inv_;
    Matrix6d Ad_M3_inv_;


        // 新增：力旋量（wrench）存储
    std::vector<Eigen::VectorXd> F_list_;  // 4个力旋量 (F1-F4)
    Eigen::VectorXd& F1 = F_list_[0];
    Eigen::VectorXd& F2 = F_list_[1];
    Eigen::VectorXd& F3 = F_list_[2];
    Eigen::VectorXd& F4 = F_list_[3];
    
    // 新增：空间惯量矩阵 G
    std::vector<Matrix6d> G_list_;  // 3个连杆的空间惯量矩阵
    
    // 新增：关节力矩
    Eigen::Vector3f tau_;  // [τ1, τ2, τ3]^T
    bool flag_ = false; 




};

#endif // ROBOT_MODEL_H