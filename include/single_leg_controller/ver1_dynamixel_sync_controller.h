#ifndef DYNAMIXEL_SYNC_CONTROLLER_H
#define DYNAMIXEL_SYNC_CONTROLLER_H

#include <ros/ros.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <vector>

class DynamixelSyncController {
private:
    // Dynamixelのアドレステーブル
    enum {
        ADDR_TORQUE_ENABLE          = 64,
        ADDR_GOAL_POSITION          = 116,
        ADDR_GOAL_VELOCITY          = 104,
        ADDR_PRESENT_POSITION       = 132,
        ADDR_PRESENT_VELOCITY       = 128,
        
        LEN_GOAL_POSITION          = 4,
        LEN_GOAL_VELOCITY          = 4,
        LEN_PRESENT_POSITION       = 4
    };

    // 通信関連のメンバー変数
    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    dynamixel::GroupSyncWrite *syncWritePosition;
    dynamixel::GroupSyncWrite *syncWriteVelocity;
    dynamixel::GroupSyncRead *syncReadPosition;

    // 通信設定
    int baudRate;
    uint8_t dxl_error;

public:
    DynamixelSyncController(const std::string &port_name, int baud_rate);
    ~DynamixelSyncController();

    // 初期化と終了処理
    bool initialize();
    void close();

    // トルク制御
    bool enableTorque(uint8_t id, bool enable);
    bool enableTorques(const std::vector<uint8_t>& ids, bool enable);

    // 同期制御機能
    bool syncWritePositionVelocity(const std::vector<uint8_t>& ids,
                                  const std::vector<uint32_t>& positions,
                                  const std::vector<uint32_t>& velocities);
    bool syncReadPositions(const std::vector<uint8_t>& ids,
                          std::vector<uint32_t>& positions);

    // 単一モーター制御（デバッグ用）
    bool writePosition(uint8_t id, uint32_t position);
    bool writeVelocity(uint8_t id, uint32_t velocity);
    bool readPosition(uint8_t id, uint32_t& position);
};

#endif // DYNAMIXEL_SYNC_CONTROLLER_H
