#include "single_leg_controller/single_leg_controller.h"

SingleLegController::SingleLegController() : nh_("~") {
    // Dynamixelの基本設定を読み込み
    nh_.param<std::string>("dynamixel/device_name", device_name_, "/dev/ttyUSB0");
    nh_.param<int>("dynamixel/baud_rate", baud_rate_, 57600);
    nh_.param<int>("dynamixel/protocol_version", protocol_version_, 2);

    // モーターIDの読み込み
    std::vector<int> motor_ids;
    if (!nh_.getParam("dynamixel/ids", motor_ids)) {
        ROS_ERROR("Failed to get motor IDs from parameter server");
        return;
    }
    if (motor_ids.size() != 3) {
        ROS_ERROR("Expected 3 motor IDs, got %zu", motor_ids.size());
        return;
    }
    coxa_id_ = motor_ids[0];
    femur_id_ = motor_ids[1];
    tibia_id_ = motor_ids[2];

    // 制御パラメータの読み込み
    int position_max_temp, position_min_temp, velocity_limit_temp;
    nh_.param("control/position_limit/max", position_max_temp, 4095);
    nh_.param("control/position_limit/min", position_min_temp, 0);
    nh_.param("control/velocity_limit", velocity_limit_temp, 1023);
    control_params_.position_max = static_cast<uint32_t>(position_max_temp);
    control_params_.position_min = static_cast<uint32_t>(position_min_temp);
    control_params_.velocity_limit = static_cast<uint32_t>(velocity_limit_temp);
    
    nh_.param<double>("control/update_rate", control_params_.update_rate, 50.0);

    // リンクパラメータの読み込み
    nh_.param<double>("leg_geometry/coxa_length", coxa_length_, 60.0);
    nh_.param<double>("leg_geometry/femur_length", femur_length_, 93.0);
    nh_.param<double>("leg_geometry/tibia_length", tibia_length_, 137.0);

    // パラメータの検証
    if (!validateParameters()) {
        ROS_ERROR("Parameter validation failed");
        return;
    }

    // Dynamixelコントローラーの初期化
    dxl_controller_ = new DynamixelSyncController(device_name_, baud_rate_);

    // ROSインターフェースの設定
    cmd_sub_ = nh_.subscribe("command", 1, &SingleLegController::commandCallback, this);
    state_pub_ = nh_.advertise<single_leg_controller::LegCommand>("state", 1);

    ROS_INFO("SingleLegController initialized successfully");
}

SingleLegController::~SingleLegController() {
    if (dxl_controller_) {
        delete dxl_controller_;
    }
}

bool SingleLegController::initialize() {
    if (!dxl_controller_->initialize()) {
        ROS_ERROR("Failed to initialize Dynamixel controller");
        return false;
    }

    // トルクの有効化
    std::vector<uint8_t> ids = {coxa_id_, femur_id_, tibia_id_};
    if (!dxl_controller_->enableTorques(ids, true)) {
        ROS_ERROR("Failed to enable torques");
        return false;
    }

    ROS_INFO("Initialization completed successfully");
    return true;
}

void SingleLegController::commandCallback(
    const single_leg_controller::LegCommand::ConstPtr& msg) {
    
    std::vector<uint8_t> ids = {coxa_id_, femur_id_, tibia_id_};
    std::vector<uint32_t> positions;
    std::vector<uint32_t> velocities;

    // 角度を位置値に変換
    uint32_t coxa_pos = angleToPosition(msg->coxa_angle);
    uint32_t femur_pos = angleToPosition(msg->femur_angle);
    uint32_t tibia_pos = angleToPosition(msg->tibia_angle);

    // 位置値の範囲チェック
    if (!isWithinLimits(coxa_pos, msg->velocity) ||
        !isWithinLimits(femur_pos, msg->velocity) ||
        !isWithinLimits(tibia_pos, msg->velocity)) {
        ROS_ERROR("Command values out of limits");
        return;
    }

    positions = {coxa_pos, femur_pos, tibia_pos};
    
    // 速度値の設定（0-1の値を0-1023に変換）
    uint32_t velocity = static_cast<uint32_t>(msg->velocity * control_params_.velocity_limit);
    velocities = {velocity, velocity, velocity};

    // 同期制御の実行
    if (!dxl_controller_->syncWritePositionVelocity(ids, positions, velocities)) {
        ROS_ERROR("Failed to execute command");
    }
}

bool SingleLegController::calculateInverseKinematics(
    double x, double y, double z,
    double& coxa_angle, double& femur_angle, double& tibia_angle) {
    
    // 股関節角度の計算
    coxa_angle = atan2(y, x);
    
    // 脚の水平面への投影長の計算
    double L = sqrt(x*x + y*y) - coxa_length_;
    double L2 = sqrt(L*L + z*z);
    
    // 到達可能性のチェック
    if (L2 > (femur_length_ + tibia_length_)) {
        ROS_ERROR("Target position out of reach");
        return false;
    }
    
    // 脛部角度の計算（余弦定理）
    double cos_tibia = (L2*L2 - femur_length_*femur_length_ 
                       - tibia_length_*tibia_length_) /
                      (2 * femur_length_ * tibia_length_);
    
    if (cos_tibia < -1 || cos_tibia > 1) {
        ROS_ERROR("Invalid tibia angle calculation");
        return false;
    }
    
    tibia_angle = acos(cos_tibia);
    
    // 大腿部角度の計算
    double gamma = atan2(z, L);
    double alpha = acos((L2*L2 + femur_length_*femur_length_ 
                        - tibia_length_*tibia_length_) /
                       (2 * L2 * femur_length_));
    femur_angle = gamma + alpha;
    
    return true;
}

bool SingleLegController::calculateForwardKinematics(
    double coxa_angle, double femur_angle, double tibia_angle,
    double& x, double& y, double& z) {
    
    // 股関節位置での座標計算
    x = coxa_length_ * cos(coxa_angle);
    y = coxa_length_ * sin(coxa_angle);
    
    // 大腿部の寄与
    double femur_x = femur_length_ * cos(femur_angle) * cos(coxa_angle);
    double femur_y = femur_length_ * cos(femur_angle) * sin(coxa_angle);
    double femur_z = femur_length_ * sin(femur_angle);
    
    // 脛部の寄与
    double tibia_x = tibia_length_ * cos(femur_angle + tibia_angle) * cos(coxa_angle);
    double tibia_y = tibia_length_ * cos(femur_angle + tibia_angle) * sin(coxa_angle);
    double tibia_z = tibia_length_ * sin(femur_angle + tibia_angle);
    
    // 最終位置の計算
    x += femur_x + tibia_x;
    y += femur_y + tibia_y;
    z = femur_z + tibia_z;
    
    return true;
}

uint32_t SingleLegController::angleToPosition(double angle) {
    // 角度（ラジアン）をDynamixelの位置値（0-4095）に変換
    return static_cast<uint32_t>((angle + M_PI) * 2048.0 / M_PI);
}

double SingleLegController::positionToAngle(uint32_t position) {
    // Dynamixelの位置値（0-4095）を角度（ラジアン）に変換
    return (static_cast<double>(position) * M_PI / 2048.0) - M_PI;
}

bool SingleLegController::validateParameters() {
    // 位置制限値のチェック
    if (control_params_.position_min >= control_params_.position_max) {
        ROS_ERROR("Invalid position limits: min >= max");
        return false;
    }

    // 速度制限値のチェック
    if (control_params_.velocity_limit > 1023) {
        ROS_ERROR("Invalid velocity limit: must be <= 1023");
        return false;
    }

    // リンク長のチェック
    if (coxa_length_ <= 0 || femur_length_ <= 0 || tibia_length_ <= 0) {
        ROS_ERROR("Invalid link lengths: must be positive");
        return false;
    }

    return true;
}

bool SingleLegController::isWithinLimits(uint32_t position, uint32_t velocity) {
    if (position < control_params_.position_min || 
        position > control_params_.position_max) {
        ROS_ERROR("Position %d out of limits [%d, %d]", 
                  position, control_params_.position_min, control_params_.position_max);
        return false;
    }

    if (velocity > control_params_.velocity_limit) {
        ROS_ERROR("Velocity %d exceeds limit %d", 
                  velocity, control_params_.velocity_limit);
        return false;
    }

    return true;
}

void SingleLegController::run() {
    ros::Rate rate(control_params_.update_rate);
    
    while (ros::ok()) {
        // 現在位置の読み取りと公開
        std::vector<uint8_t> ids = {coxa_id_, femur_id_, tibia_id_};
        std::vector<uint32_t> positions;
        
        if (dxl_controller_->syncReadPositions(ids, positions)) {
            single_leg_controller::LegCommand state_msg;
            state_msg.coxa_angle = positionToAngle(positions[0]);
            state_msg.femur_angle = positionToAngle(positions[1]);
            state_msg.tibia_angle = positionToAngle(positions[2]);
            state_pub_.publish(state_msg);
        }
        
        ros::spinOnce();
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "single_leg_controller");
    
    SingleLegController controller;
    if (!controller.initialize()) {
        ROS_ERROR("Failed to initialize controller");
        return 1;
    }
    
    controller.run();
    return 0;
}
