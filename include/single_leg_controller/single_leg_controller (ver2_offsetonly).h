#ifndef SINGLE_LEG_CONTROLLER_H
#define SINGLE_LEG_CONTROLLER_H

#include <ros/ros.h>
#include <single_leg_controller/LegCommand.h>
#include <single_leg_controller/LegPosition.h>
#include "single_leg_controller/dynamixel_sync_controller.h"
#include <cmath>

class SingleLegController {
private:
    // ROSインターフェース
    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_;
    ros::Publisher state_pub_;
    ros::Publisher fk_pos_pub_;
    ros::Publisher ik_angle_pub_;
    ros::Subscriber pos_cmd_sub_;
    
    // Dynamixel制御用
    DynamixelSyncController* dxl_controller_;
    
    // モーター設定
    std::string device_name_;
    int baud_rate_;
    int protocol_version_;
    uint8_t coxa_id_;
    uint8_t femur_id_;
    uint8_t tibia_id_;
    
    // 制御パラメータ
    struct ControlParams {
        uint32_t position_max;
        uint32_t position_min;
        uint32_t velocity_limit;
        double update_rate;
    } control_params_;

    // ゼロ点オフセット
    struct JointZeroPosition {
        uint32_t coxa_offset;
        uint32_t femur_offset;
        uint32_t tibia_offset;
    } zero_positions_;

    // リンクパラメータ
    double coxa_length_;
    double femur_length_;
    double tibia_length_;

    // コールバック関数
    void commandCallback(const single_leg_controller::LegCommand::ConstPtr& msg);
    void positionCommandCallback(const single_leg_controller::LegPosition::ConstPtr& msg);
    
    // 運動学計算
    bool calculateInverseKinematics(double x, double y, double z,
                                  double& coxa_angle, double& femur_angle, double& tibia_angle);
    bool calculateForwardKinematics(double coxa_angle, double femur_angle, double tibia_angle,
                                  double& x, double& y, double& z);
    
    // 角度変換
    uint32_t angleToPosition(double angle, uint32_t offset);
    double positionToAngle(uint32_t position, uint32_t offset);

    // 角度の検証と正規化
    bool validateAngle(double angle);
    void normalizeAngle(double& angle);

    // パラメータ検証
    bool validateParameters();
    bool isWithinLimits(uint32_t position, uint32_t velocity);

public:
    SingleLegController();
    ~SingleLegController();
    
    bool initialize();
    void run();
};

#endif // SINGLE_LEG_CONTROLLER_H
