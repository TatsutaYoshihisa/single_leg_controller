#ifndef SINGLE_LEG_CONTROLLER_H
#define SINGLE_LEG_CONTROLLER_H

#include <ros/ros.h>
#include <single_leg_controller/LegCommand.h>
#include "single_leg_controller/dynamixel_sync_controller.h"
#include <cmath>

class SingleLegController {
private:
    // ROSインターフェース
    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_;
    ros::Publisher state_pub_;
    
    // Dynamixel制御用
    DynamixelSyncController* dxl_controller_;
    
    // モーター設定
    std::string device_name_;
    int baud_rate_;
    int protocol_version_;
    uint8_t coxa_id_;   // 股関節ID
    uint8_t femur_id_;  // 大腿部ID
    uint8_t tibia_id_;  // 脛部ID
    
    // 制御パラメータ
    struct ControlParams {
        uint32_t position_max;
        uint32_t position_min;
        uint32_t velocity_limit;
        double update_rate;
    } control_params_;

    // リンクパラメータ
    double coxa_length_;   // mm
    double femur_length_;  // mm
    double tibia_length_;  // mm

    // コールバック関数
    void commandCallback(const single_leg_controller::LegCommand::ConstPtr& msg);
    
    // 運動学計算
    bool calculateInverseKinematics(double x, double y, double z,
                                  double& coxa_angle, double& femur_angle, double& tibia_angle);
    bool calculateForwardKinematics(double coxa_angle, double femur_angle, double tibia_angle,
                                  double& x, double& y, double& z);
    
    // 角度変換
    uint32_t angleToPosition(double angle);
    double positionToAngle(uint32_t position);

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
