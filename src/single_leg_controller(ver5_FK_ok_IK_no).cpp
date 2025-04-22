#include "single_leg_controller/single_leg_controller.h"
#include <boost/make_shared.hpp>
//コンストラクタ（初期化処理）
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
    nh_.param<double>("leg_geometry/coxa_length", coxa_length_, 25.0);
    nh_.param<double>("leg_geometry/femur_length", femur_length_, 105.0);
    nh_.param<double>("leg_geometry/tibia_length", tibia_length_, 105.0);

    // ゼロ点オフセットの読み込み
    double coxa_offset_temp, femur_offset_temp, tibia_offset_temp;
    nh_.param("joint_zero_position/coxa_offset", coxa_offset_temp, 0.0);
    nh_.param("joint_zero_position/femur_offset", femur_offset_temp, 0.0);
    nh_.param("joint_zero_position/tibia_offset", tibia_offset_temp, 0.0);
    
    
    zero_positions_.coxa_offset = degToPosition(coxa_offset_temp);
    zero_positions_.femur_offset = degToPosition(femur_offset_temp);
    zero_positions_.tibia_offset = degToPosition(tibia_offset_temp);

    // 回転方向の読み込み
    int coxa_dir_temp, femur_dir_temp, tibia_dir_temp;
    nh_.param("joint_direction/coxa_direction", coxa_dir_temp, 1);
    nh_.param("joint_direction/femur_direction", femur_dir_temp, 1);
    nh_.param("joint_direction/tibia_direction", tibia_dir_temp, 1);
    
    joint_directions_.coxa_direction = coxa_dir_temp;
    joint_directions_.femur_direction = femur_dir_temp;
    joint_directions_.tibia_direction = tibia_dir_temp;

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
    fk_pos_pub_ = nh_.advertise<single_leg_controller::LegPosition>("foot_position", 1);
    ik_angle_pub_ = nh_.advertise<single_leg_controller::LegCommand>("joint_angles", 1);
    pos_cmd_sub_ = nh_.subscribe("position_command", 1, 
                                &SingleLegController::positionCommandCallback, this);

    ROS_INFO("SingleLegController initialized successfully");
}

SingleLegController::~SingleLegController() {
    if (dxl_controller_) {
        delete dxl_controller_;
    }
}
//initialize()による初期化処理（ハード面の初期化処理）
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

//--------------------------<検証系の関数>--------------------------

//角度の範囲が、(-π~+π)なのか確認用関数
bool SingleLegController::validateAngle(double angle) {
    //angleが-π~+πの範囲内であればtrue(1),範囲外であれば、false(0)が戻り値が返る。
    return angle >= -M_PI && angle <= M_PI;
}

//角度の範囲が-π~+πの範囲外である場合、範囲を内に角度の値を整える関数
void SingleLegController::normalizeAngle(double& angle) {
    //角度の範囲が-π~+πの範囲外である時に実行
    if(!validateAngle(angle)){
       //angleにπ足して、範囲を-π~+πから0~2πに変更する。それを2πで割り、余りを求める。 
       angle = fmod(angle + M_PI, 2.0 * M_PI);
       //余りからπ引けば、範囲内に変換できる。ただし、余りが負の値の時引くのではなく足す必要があるため、2π足すことで実質余りにπを足している。
       if (angle < 0) angle += 2.0 * M_PI;
       angle -= M_PI;
       ROS_INFO("Angle convrted");
    }
}

//--------------------------<変換系の関数>--------------------------

//angleをdynamixelが対応している角度値に変更
uint32_t SingleLegController::angleToPosition(double angle, int32_t offset, int direction) {
    // 角度を-πからπの範囲に正規化
    normalizeAngle(angle);
    // 回転方向を考慮して角度を変換
    angle *= direction;
    // 角度を位置値に変換してオフセットを適用
    int32_t cal_position = static_cast<int32_t>((angle + M_PI) * 2048.0 / M_PI) + offset;
    // 0-4095の範囲に正規化
    uint32_t position = static_cast<int32_t>(((cal_position % 4096) + 4096) % 4096);
    ROS_INFO("angleToPosition() : Angle Converted %f to %d",angle,position);
    return position;
}

double SingleLegController::positionToAngle(uint32_t position, int32_t offset, int direction) {
    // オフセットを考慮した位置値を0-4095の範囲に正規化
    int32_t normalized_position = static_cast<int32_t>(position - offset);
    
    // 4096で割った余りを計算（負の値も適切に処理）
    normalized_position = ((normalized_position % 4096) + 4096) % 4096;
    
    // 正規化された位置値を角度に変換（-πからπの範囲）
    double angle = (static_cast<double>(normalized_position) * 2.0 * M_PI / 4096.0) - M_PI;
    
    // 回転方向を考慮
    angle *= direction;
    ROS_INFO("positionToAngle() : Angle Converted %d to %f",position,angle);
    return angle;
}

int32_t SingleLegController::degToPosition(double angle){
	int32_t position = static_cast<int32_t>((angle) * 2048.0 / 180.0);
	ROS_INFO("degToPosition() : Angle Converted %f to %d",angle,position);
	return position;
}

//void SinglelegController::displayValue(double dou_var , uint32_t uint_var){}

//--------------------------<Callback関数>--------------------------
//FK
void SingleLegController::commandCallback(
    const single_leg_controller::LegCommand::ConstPtr& msg) {
    
    std::vector<uint8_t> ids = {coxa_id_, femur_id_, tibia_id_};
    std::vector<uint32_t> positions;
    std::vector<uint32_t> velocities;

    // 角度値の正規化
    double coxa_angle = msg->coxa_angle;
    double femur_angle = msg->femur_angle;
    double tibia_angle = msg->tibia_angle;
    
    normalizeAngle(coxa_angle);
    normalizeAngle(femur_angle);
    normalizeAngle(tibia_angle);

    // 正規化された角度を位置値に変換（オフセットと方向を考慮）
    uint32_t coxa_pos = angleToPosition(coxa_angle, zero_positions_.coxa_offset, 
                                      joint_directions_.coxa_direction);
    uint32_t femur_pos = angleToPosition(femur_angle, zero_positions_.femur_offset,
                                       joint_directions_.femur_direction);
    uint32_t tibia_pos = angleToPosition(tibia_angle, zero_positions_.tibia_offset,
                                       joint_directions_.tibia_direction);

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
        return;
    }

    // 順運動学計算と位置のパブリッシュ
    double x, y, z;
    if (calculateForwardKinematics(coxa_angle, femur_angle, tibia_angle, x, y, z)) {
        single_leg_controller::LegPosition pos_msg;
        pos_msg.x = x;
        pos_msg.y = y;
        pos_msg.z = z;
        fk_pos_pub_.publish(pos_msg);
    }
}

//IK

void SingleLegController::positionCommandCallback(
    const single_leg_controller::LegPosition::ConstPtr& msg) {
    
    double coxa_angle, femur_angle, tibia_angle;
    
    // 逆運動学計算
    if (calculateInverseKinematics(msg->x, msg->y, msg->z,
                                 coxa_angle, femur_angle, tibia_angle)) {
        // 角度の正規化
        normalizeAngle(coxa_angle);
        normalizeAngle(femur_angle);
        normalizeAngle(tibia_angle);
        
        // 関節角度のパブリッシュ
        single_leg_controller::LegCommand angle_msg;
        angle_msg.coxa_angle = coxa_angle;
        angle_msg.femur_angle = femur_angle;
        angle_msg.tibia_angle = tibia_angle;
        angle_msg.velocity = 0.5;  // デフォルト速度を設定
        ik_angle_pub_.publish(angle_msg);
        
        // 実際のモーター制御用にcommandCallbackを利用
        commandCallback(boost::make_shared<single_leg_controller::LegCommand>(angle_msg));
    } else {
        ROS_ERROR("Inverse kinematics calculation failed");
    }
}

//--------------------------<計算系の関数>--------------------------
//IK
bool SingleLegController::calculateInverseKinematics(
    double x, double y, double z,
    double& coxa_angle, double& femur_angle, double& tibia_angle) {
    
    const double EPSILON = 1e-6;  // 数値計算の誤差許容範囲

    // Step 1: coxa角度の計算
    coxa_angle = atan2(y, x);
    
    // Step 2: coxaジョイントからの水平距離を計算
    double L = sqrt(x*x + y*y) - coxa_length_;
    // z座標はそのまま使用
    
    // Step 3: femur-tibia平面での2リンク問題を解く
    double L2 = sqrt(L*L + z*z);  // 目標点までの距離
    
    // 到達可能性チェック
    if (L2 > (femur_length_ + tibia_length_ - EPSILON)) {
        ROS_ERROR("Target position out of reach (too far): %f > %f", 
                  L2, femur_length_ + tibia_length_);
        return false;
    }
    //fabsは浮動小数点数の絶対値（abs()のflaot版）
    if (L2 < fabs(femur_length_ - tibia_length_) + EPSILON) {
        ROS_ERROR("Target position out of reach (too close): %f < %f", 
                  L2, fabs(femur_length_ - tibia_length_));
        return false;
    }
    
    // Step 4: 余弦定理を使用してtibia角度を計算
    double cos_tibia = (L2*L2 - femur_length_*femur_length_ 
                       - tibia_length_*tibia_length_) /
                      (2.0 * femur_length_ * tibia_length_);
                      
    // 数値誤差の処理
    if (cos_tibia > 1.0 - EPSILON) {
        tibia_angle = 0.0;
    } else if (cos_tibia < -1.0 + EPSILON) {
        tibia_angle = M_PI;
    } else {
        tibia_angle = acos(cos_tibia);
    }
    
    // elbow-down設定の場合は符号を反転
    // bool use_elbow_up = true;  // 設定によって変更可能
    // if (!use_elbow_up) {
    //     tibia_angle = -tibia_angle;
    // }
    
    // Step 5: femur角度を計算
    double gamma = atan2(z, L);  // 目標点の仰角
    double alpha = atan2(tibia_length_ * sin(tibia_angle),
                        femur_length_ + tibia_length_ * cos(tibia_angle));
    femur_angle = gamma + alpha;
    
    // Step 6: 関節角度の範囲チェック
    if (!validateAngle(coxa_angle) || 
        !validateAngle(femur_angle) || 
        !validateAngle(tibia_angle)) {
        ROS_ERROR("Joint angles out of valid range");
        return false;
    }
    
    // Step 7: 特異点チェック（必要に応じて）
    if (fabs(L2) < EPSILON) {
        ROS_WARN("Near singularity: target point close to coxa axis");
        // 特異点での特別な処理が必要な場合はここで実装
    }
    
    return true;
}

// 補助関数：特異点チェック
bool SingleLegController::isNearSingularity(
    double coxa_angle, double femur_angle, double tibia_angle) {
    const double SINGULARITY_THRESHOLD = 0.01;  // ラジアン
    
    // 完全伸展位置のチェック
    if (fabs(tibia_angle) < SINGULARITY_THRESHOLD ||
        fabs(tibia_angle - M_PI) < SINGULARITY_THRESHOLD) {
        return true;
    }
    
    return false;
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
//--------------------------<計算結果確認用関数>--------------------------
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

    // 方向パラメータの検証
    if (abs(joint_directions_.coxa_direction) != 1 ||
        abs(joint_directions_.femur_direction) != 1 ||
        abs(joint_directions_.tibia_direction) != 1) {
        ROS_ERROR("Invalid joint direction values: must be 1 or -1");
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

//--------------------------<ループ処理（角度の読み取り）>--------------------------

void SingleLegController::run() {
    ros::Rate rate(control_params_.update_rate);
    
    while (ros::ok()) {
        // 現在位置の読み取りと公開
        std::vector<uint8_t> ids = {coxa_id_, femur_id_, tibia_id_};
        std::vector<uint32_t> positions;
        
        if (dxl_controller_->syncReadPositions(ids, positions)) {
            single_leg_controller::LegCommand state_msg;
            state_msg.coxa_angle = positionToAngle(positions[0], zero_positions_.coxa_offset,
                                                joint_directions_.coxa_direction);
            state_msg.femur_angle = positionToAngle(positions[1], zero_positions_.femur_offset,
                                                 joint_directions_.femur_direction);
            state_msg.tibia_angle = positionToAngle(positions[2], zero_positions_.tibia_offset,
                                                 joint_directions_.tibia_direction);
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
