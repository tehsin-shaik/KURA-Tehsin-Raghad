#pragma once

#include <tuple>
#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>
#include <algorithm>

#include "types.h"

class Brain;

using namespace std;
using namespace BT;

class BrainTree
{
public:
    BrainTree(Brain *argBrain) : brain(argBrain) {}

    void init();

    void tick();

    // get entry on blackboard
    template <typename T>
    inline T getEntry(const string &key)
    {
        T value;
        [[maybe_unused]] auto res = tree.rootBlackboard()->get<T>(key, value);
        return value;
    }

    // set entry on blackboard
    template <typename T>
    inline void setEntry(const string &key, const T &value)
    {
        tree.rootBlackboard()->set<T>(key, value);
    }

private:
    Tree tree;
    Brain *brain;

    /**
     * 初始化 blackboard 里的 entry，注意新加字段，在这里设置个默认值
     */
    void initEntry();
};


class CalcKickDir : public SyncActionNode 
{
public:
    CalcKickDir(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("cross_threshold", 0.2, "可进门的角度范围小于这个值时, 则传中")
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class StrikerDecide : public SyncActionNode
{
public:
    StrikerDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "超过这个距离, 执行追球动作"),
            InputPort<string>("decision_in", "", "用于读取上一次的 decision"),
            InputPort<string>("position", "offense", "offense | defense, 决定了向哪个方向踢球"),
            OutputPort<string>("decision_out")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    double lastDeltaDir; 
    rclcpp::Time timeLastTick; 
};


class GoalieDecide : public SyncActionNode
{
public:
    GoalieDecide(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "超过这个距离, 执行追球动作"),
            InputPort<double>("adjust_angle_tolerance", 0.1, "小于这个角度, 认为 adjust 已经成功"),
            InputPort<double>("adjust_y_tolerance", 0.1, "y 方向偏移小于这个值, 认为 y 方向 adjust 成功"),
            InputPort<string>("decision_in", "", "用于读取上一次的 decision"),
            InputPort<double>("auto_visual_kick_enable_dist_min", 2.0, "自动视觉踢球启用时球的最小距离"),
            InputPort<double>("auto_visual_kick_enable_dist_max", 3.0, "自动视觉踢球启用时球的最大距离"),
            InputPort<double>("auto_visual_kick_enable_angle", 0.785, "自动视觉踢球启用时球的角度范围"),
            InputPort<double>("auto_visual_kick_obstacle_dist_threshold", 3.0, "自动视觉踢球障碍物距离阈值，如果该距离内有障碍物，则不执行自动视觉踢球"),
            InputPort<double>("auto_visual_kick_obstacle_angle_threshold", 1.744, "自动视觉踢球障碍物在前方角度范围内的阈值，如果该角度内有障碍物，则不执行自动视觉踢球"),
            OutputPort<string>("decision_out"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


class CamTrackBall : public SyncActionNode
{
public:
    CamTrackBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {};
    }
    NodeStatus tick() override;

private:
    Brain *brain;
};


class CamFindBall : public SyncActionNode
{
public:
    CamFindBall(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;

private:
    double _cmdSequence[6][2];    
    rclcpp::Time _timeLastCmd;   
    int _cmdIndex;                
    long _cmdIntervalMSec;        
    long _cmdRestartIntervalMSec; 

    Brain *brain;

};


class RobotFindBall : public StatefulActionNode
{
public:
    RobotFindBall(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vyaw_limit", 1.0, "转向的速度上限"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    double _turnDir; 
    Brain *brain;
};


class CamFastScan : public StatefulActionNode
{
public:
    CamFastScan(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("msecs_interval", 300, "在同一个位置停留多少毫秒"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override {};

private:
    double _cmdSequence[7][2] = {
        {0.45, 1.1},
        {0.45, 0.0},
        {0.45, -1.1},
        {1.0, -1.1},
        {1.0, 0.0},
        {1.0, 1.1},
        {0.45, 0.0},
    };    
    rclcpp::Time _timeLastCmd;    
    int _cmdIndex = 0;               
    Brain *brain;
};

class TurnOnSpot : public StatefulActionNode
{
public:
    TurnOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("rad", 0, "转多少弧度, 向左为正"),
            InputPort<bool>("towards_ball", false, "为 true 时, 不考虑 rad 的正负号, 而是转向上一次看到不球的方向.")
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override {};

private:
    double _lastAngle; 
    double _angle;
    double _cumAngle; 
    double _msecLimit = 5000;  
    rclcpp::Time _timeStart;
    Brain *brain;
};


class Chase : public SyncActionNode
{
public:
    Chase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("vx_limit", 0.6, "追球的最大 x 速度"),
            InputPort<double>("vy_limit", 0.4, "追球的最大 y 速度"),
            InputPort<double>("speed", 1.0, "追球的最大 y 速度"),
            InputPort<double>("vtheta_limit", 1.0, "追球时, 实时调整方向的速度不大于这个值"),
            InputPort<double>("dist", 0.1, "追球的目标是球后面多少距离"),
            InputPort<double>("safe_dist", 4.0, "circle back 时, 保持的安全距离"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    string _state;     
    double _dir = 1.0; 
};


class Adjust : public SyncActionNode
{
public:
    Adjust(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("turn_threshold", 3.25, "球的角度大于这个值, 机器人先转身面向球, 直线运动先暂停"),
            InputPort<double>("vx_limit", 0.05, "调整过过程中 vx 的限制 [-limit, limit]"),
            InputPort<double>("vy_limit", 0.05, "调整过过程中 vy 的限制 [-limit, limit]"),
            InputPort<double>("vtheta_limit", 0.1, "调整过过程中 vtheta 的限制 [-limit, limit]"),
            InputPort<double>("range", 2.25, "ball  range 保持这个值"),
            InputPort<double>("vtheta_factor", 3.0, "调整角度时, vtheta 的乘数, 越大转向越快"),
            InputPort<double>("tangential_speed_far", 0.2, "调整角度时, 较远时的切线速度"),
            InputPort<double>("tangential_speed_near", 0.15, "调整角度时, 较近时的切线速度"),
            InputPort<double>("near_threshold", 0.8, "距离目标小于这个值时, 使用 near speed"),
            InputPort<double>("no_turn_threshold", 0.02, "角度差小于这个值时, 不转身"),
            InputPort<double>("turn_first_threshold", 0.5, "角度差大于这个值时, 先转身, 不移动"),

        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};

// 执行踢球动作
class Kick : public StatefulActionNode
{
public:
    Kick(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("min_msec_kick", 500, "踢球动作最少执行多少毫秒"),
            InputPort<double>("msecs_stablize", 1000, "稳定多少毫秒"),
            InputPort<double>("speed_limit", 0.8, "速度最大值"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime; 
    string _state = "kick"; // stablize | kick
    int _msecKick = 1000;    
    double _speed; 
    double _minRange; 
    tuple<double, double, double> _calcSpeed();
};

// 触发机械踢球（调用底层 Loco API），快速返回
class KickLeg : public SyncActionNode
{
public:
    KickLeg(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return { };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class StandStill : public StatefulActionNode
{
public:
    StandStill(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<int>("msecs", 1000, "站立多少毫秒"),
        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

private:
    Brain *brain;
    rclcpp::Time _startTime; 
};


class CamScanField : public SyncActionNode
{
public:
    CamScanField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            // InputPort<double>("low_pitch", 0.45, "向下看时的最大 pitch"),
            // InputPort<double>("high_pitch", 0.15, "向上看时的最小 pitch"),
            InputPort<double>("low_pitch", 0.6, "向下看时的最大 pitch"),
            InputPort<double>("high_pitch", 0.45, "向上看时的最小 pitch"),
            InputPort<double>("left_yaw", 0.8, "向左看时的最大 yaw"),
            InputPort<double>("right_yaw", -0.8, "向右看时的最小 yaw"),
            InputPort<int>("msec_cycle", 4000, "多少毫秒转一圈"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class MoveToPoseOnField : public SyncActionNode
{
public:
    MoveToPoseOnField(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "目标 x 坐标, Field 坐标系"),
            InputPort<double>("y", 0, "目标 y 坐标, Field 坐标系"),
            InputPort<double>("theta", 0, "目标最终朝向, Field 坐标系"),
            InputPort<double>("long_range_threshold", 1.5, "目标点的距离超过这个值时, 优先走过去, 而不是细调位置和方向"),
            InputPort<double>("turn_threshold", 0.4, "长距离时, 目标点的方向超这个数值时, 先转向目标点"),
            InputPort<double>("vx_limit", 0.8, "x 限速"),
            InputPort<double>("vy_limit", 0.5, "y 限速"),
            InputPort<double>("vtheta_limit", 0.2, "theta 限速"),
            InputPort<double>("x_tolerance", 0.5, "x 容差"),
            InputPort<double>("y_tolerance", 0.5, "y 容差"),
            InputPort<double>("theta_tolerance", 0.5, "theta 容差"),
            InputPort<bool>("avoid_obstacle", false, "是否避障")
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};

class GoToReadyPosition : public SyncActionNode
{
public:
    GoToReadyPosition(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_tolerance", 0.8, "x tolerance"),
            InputPort<double>("theta_tolerance", 0.5, "theta tolerance"),
            InputPort<double>("vx_limit", 0.8, "vx limit"),
            InputPort<double>("vy_limit", 0.5, "vy limit"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


class GoToGoalBlockingPosition : public SyncActionNode
{
public:
    GoToGoalBlockingPosition(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_tolerance", 0.8, "dist tolerance, within which considered arrived."),
            InputPort<double>("theta_tolerance", 0.8, "theta tolerance, winin which considered arrived."),
            InputPort<double>("vx_limit", 0.1, "x speed limit"),
            InputPort<double>("vy_limit", 0.1, "y speed limit"),
            InputPort<double>("dist_to_goalline", 2.5, "机器人站在门前多少距离"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


class Assist : public SyncActionNode
{
public:
    Assist(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("dist_tolerance", 0.8, "dist tolerance, within which considered arrived."),
            InputPort<double>("theta_tolerance", 0.8, "theta tolerance, winin which considered arrived."),
            InputPort<double>("vx_limit", 0.1, "x speed limit"),
            InputPort<double>("vy_limit", 0.1, "y speed limit"),
            InputPort<double>("dist_to_goalline", 2.5, "机器人站在门前多少距离"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


/**
 * @brief 设置机器人的速度
 *
 * @param x,y,theta double, 机器人在 x，y 方向上的速度（m/s）和逆时针转动的角速度（rad/s), 默认值为 0. 全为 0 时，即相当于给出站立不动指令
 *
 */
class SetVelocity : public SyncActionNode
{
public:
    SetVelocity(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "Default x is 0"),
            InputPort<double>("y", 0, "Default y is 0"),
            InputPort<double>("theta", 0, "Default  theta is 0"),
        };
    }

private:
    Brain *brain;
};

// 原地踏步
class StepOnSpot : public SyncActionNode
{
public:
    StepOnSpot(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    NodeStatus tick() override;
    static PortsList providedPorts()
    {
        return {};
    }

private:
    Brain *brain;
};

class WaveHand : public SyncActionNode
{
public:
    WaveHand(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<string>("action", "start", "start | stop"),
        };
    }

private:
    Brain *brain;
};

class MoveHead : public SyncActionNode
{
public:
    MoveHead(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("pitch", 0, "target head pitch"),
            InputPort<double>("yaw", 0, "target head yaw"),
        };
    }

private:
    Brain *brain;
};


class CheckAndStandUp : public SyncActionNode
{
public:
CheckAndStandUp(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts() {
        return {};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class GoToFreekickPosition : public StatefulActionNode
{
public:
    GoToFreekickPosition(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<string>("side", "attack", "attack | defense"),
            InputPort<double>("attack_dist", 0.7, "attack side target dist to ball"),
            InputPort<double>("defense_dist", 1.9, "defense side target dist to ball"),
            InputPort<double>("vx_limit", 1.2, "vx limit"),
            InputPort<double>("vy_limit", 0.5, "vy limit"),
            InputPort<bool>("start_at_origin", false, "Go to (0,0) first before freekick positioning")

        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    Brain *brain;
    bool _isInFinalAdjust = false; 
    bool _wentToOrigin = false;

};

class GoToFreekickPositionDefender : public StatefulActionNode
{
public:
    GoToFreekickPositionDefender(const string &name, const NodeConfig &config, Brain *_brain) : StatefulActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<string>("side", "attack", "attack | defense"),
            InputPort<double>("attack_dist", 0.7, "attack side target dist to ball"),
            InputPort<double>("defense_dist", 1.9, "defense side target dist to ball"),
            InputPort<double>("vx_limit", 1.2, "vx limit"),
            InputPort<double>("vy_limit", 0.5, "vy limit"),

        };
    }

    NodeStatus onStart() override;

    NodeStatus onRunning() override;

    void onHalted() override;

private:
    Brain *brain;
    bool _isInFinalAdjust = false; 
};
// 回到场地内
class GoBackInField : public SyncActionNode
{
public:
    GoBackInField(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("valve", 0.5, "回到场内距离边界多远可以停止"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class SimpleChase : public SyncActionNode
{
public:
    SimpleChase(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("stop_dist", 1.0, "在距离球多远的距离, 就不再走向球了"),
            InputPort<double>("stop_angle", 0.1, "球的角度在多少时, 就不再转向球了"),
            InputPort<double>("vy_limit", 0.2, "限制 Y 方向速度, 以防止走路不稳定. 要起作用需要小于机器本身的限速 0.4"),
            InputPort<double>("vx_limit", 0.6, "限制 X 方向速度, 以防止走路不稳定. 要起作用需要小于机器本身的限速 1.2"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


class CalibrateOdom : public SyncActionNode
{
public:
    CalibrateOdom(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("x", 0, "x"),
            InputPort<double>("y", 0, "y"),
            InputPort<double>("theta", 0, "theta"),
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
};


// 向 cout 打印文字
class PrintMsg : public SyncActionNode
{
public:
    PrintMsg(const std::string &name, const NodeConfig &config, Brain *_brain)
        : SyncActionNode(name, config)
    {
    }

    NodeStatus tick() override;

    static PortsList providedPorts()
    {
        return {InputPort<std::string>("msg")};
    }

private:
    Brain *brain;
};

// 播放一些预定义的声音
class PlaySound : public SyncActionNode
{
public:
    PlaySound(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<string>("sound", "cheerful", "播放声音的名称"),
            InputPort<bool>("allow_repeat", false, "是否允许重复播放同一个声音"),
        };
    }

private:
    Brain *brain;
};

// 使用本地 tts (espeak) 朗读文本
class Speak : public SyncActionNode
{
public:
    Speak(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
    {
    }

    NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return {
            InputPort<string>("text", "", "朗读的文本内容, 必须是英文"),
        };
    }

private:
    Brain *brain;
};

class newGoalieDecide : public SyncActionNode
{
public:
   newGoalieDecide(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
 
    static BT::PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "Perform the chasing action if the distance exceeds this threshold"),
            InputPort<double>("adjust_angle_tolerance", 0.1, "Consider the adjustment successful if the angle is smaller than this value"),
            InputPort<double>("adjust_y_tolerance", 0.1, "Consider the y-direction adjustment successful if the offset is smaller than this value"),
            InputPort<string>("decision_in", "", "Used to read the last decision"),
            OutputPort<string>("decision_out"),
        };
    }
 
    BT::NodeStatus tick() override;
 
private:
    Brain *brain;
};
 

class Y_Keeper : public SyncActionNode
{
public:
    Y_Keeper(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
 
    static BT::PortsList providedPorts()
    {
        return {            
            InputPort<double>("theta", 0, "Final orientation of the target in the Field coordinate system"),
            InputPort<double>("long_range_threshold", 1.5, "When the distance to the target point exceeds this value, prioritize moving towards it rather than fine-tuning position and orientation"),
            InputPort<double>("turn_threshold", 0.4, "For long distances, if the angle to the target point exceeds this threshold, turn towards the target point first"),
            InputPort<double>("vx_limit", 1.0, "x limit"),
            InputPort<double>("vy_limit", 0.5, "y limit"),
            InputPort<double>("vtheta_limit", 0.4, "theta limit"),
            InputPort<double>("x_tolerance", 0.2, "X tolerance"),
            InputPort<double>("y_tolerance", 0.2, "y tolerance"),
            InputPort<double>("theta_tolerance", 0.1, "theta tolerance"),

            // // NEW: Goalie box + kick offset (with defaults)
            // InputPort<double>("goalie_box_x_min", -7.5, "Back boundary of goalie work zone"),
            // InputPort<double>("goalie_box_x_max", -5.0, "Front boundary of goalie work zone"),
            // InputPort<double>("goalie_box_half_y", 1.2,  "Half-height of goalie work zone"),
            // InputPort<double>("kick_offset", 0.28,       "Stand-off behind ball before kick")

        };
    }
 
    BT::NodeStatus tick() override;
 
private:
    Brain *brain;
};


// class Y_Keeper : public SyncActionNode
// {
// public:
//     Y_Keeper(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}
 
//     static BT::PortsList providedPorts()
//     {
//         return {            
//             InputPort<double>("theta", 0, "Final orientation of the target in the Field coordinate system"),
//             InputPort<double>("long_range_threshold", 1.5, "When the distance to the target point exceeds this value, prioritize moving towards it rather than fine-tuning position and orientation"),
//             InputPort<double>("turn_threshold", 0.4, "For long distances, if the angle to the target point exceeds this threshold, turn towards the target point first"),
//             InputPort<double>("vx_limit", 1.0, "x limit"),
//             InputPort<double>("vy_limit", 0.5, "y limit"),
//             InputPort<double>("vtheta_limit", 0.4, "theta limit"),
//             InputPort<double>("x_tolerance", 0.2, "X tolerance"),
//             InputPort<double>("y_tolerance", 0.2, "y tolerance"),
//             InputPort<double>("theta_tolerance", 0.1, "theta tolerance"),
//         };
//     }
 
//     BT::NodeStatus tick() override;
 
// private:
//     Brain *brain;
// };


class DefenderDecide : public SyncActionNode
{
public:
    DefenderDecide(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts()
    {
        return {
            InputPort<double>("chase_threshold", 1.0, "超过这个距离, 执行追球动作"),
            InputPort<string>("decision_in", "", "用于读取上一次的 decision"),
            InputPort<string>("position", "offense", "offense | defense, 决定了向哪个方向踢球"),
            OutputPort<string>("decision_out")};
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    double lastDeltaDir; 
    rclcpp::Time timeLastTick; 
};



class DefenceChase : public SyncActionNode
{
public:
    DefenceChase(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static BT::PortsList providedPorts()
    {
        return {            
            InputPort<double>("theta", 0, "Final orientation of the target in the Field coordinate system"),
            InputPort<double>("long_range_threshold", 1.5, "When the distance to the target point exceeds this value, prioritize moving towards it rather than fine-tuning position and orientation"),
            InputPort<double>("turn_threshold", 0.4, "For long distances, if the angle to the target point exceeds this threshold, turn towards the target point first"),
            InputPort<double>("vx_limit", 1.0, "x limit"),
            InputPort<double>("vy_limit", 0.5, "y limit"),
            InputPort<double>("vtheta_limit", 0.4, "theta limit"),
            InputPort<double>("x_tolerance", 0.2, "X tolerance"),
            InputPort<double>("y_tolerance", 0.2, "y tolerance"),
            InputPort<double>("theta_tolerance", 0.1, "theta tolerance"),
        };
    }

    BT::NodeStatus tick() override;

private:
    Brain *brain;
};


class CalcKickDirPF : public SyncActionNode {
public:
    CalcKickDirPF(const string &name, const NodeConfig &config, Brain *_brain)
    : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts() {
        return {
            // --- Your legacy PF knobs (kept for compatibility; ignored by this implementation) ---
            InputPort<double>("goal_attract_gain",     3.0, "IGNORED"),
            InputPort<double>("teammate_repulse_gain", 0.8, "IGNORED"),
            InputPort<double>("teammate_facing_gain",  1.0, "IGNORED"),
            InputPort<double>("opponent_repulse_gain", 1.5, "IGNORED"),
            InputPort<double>("edge_repulse_gain",     1.0, "IGNORED"),
            InputPort<double>("edge_influence_dist",   1.0, "IGNORED"),
            InputPort<double>("obstacle_clearance",    0.30,"IGNORED"),
            InputPort<double>("obstacle_check_dist",   2.00,"IGNORED"),
            InputPort<double>("search_angle_step",     0.15,"IGNORED"),
            InputPort<double>("search_max_angle",      1.00,"IGNORED"),
            InputPort<double>("min_forward_dot",       0.20,"IGNORED"),
            InputPort<double>("min_force",             0.01,"IGNORED"),
            InputPort<double>("saturate_force",        5.00,"IGNORED"),

            // --- Python-logic knobs (these map to the scorer below; all optional) ---
            InputPort<double>("goal_half",           0.6,  "Half goal height (m)"),
            InputPort<double>("ball_speed",          8.0,  "v_ball (m/s)"),
            InputPort<double>("open_space_D",        3.2,  "Open-space cap distance (m)"),
            InputPort<double>("w_open_space",        2.6,  "Open-space weight"),
            InputPort<double>("w_flank",             3.8,  "Flank weight"),
            InputPort<double>("goal_reward_gain",    10.0,  "Goal reward gain"),
            InputPort<double>("out_penalty",         8.0,  "Out-of-bounds penalty"),
            InputPort<double>("near_post_repulse",   0.18, "Near-post repulse range (m)"),
            InputPort<double>("min_kick_clearance",  0.30, "Min lateral clearance (m)"),
            InputPort<double>("jitter_deg",          7.0,  "Angle jitter sigma (deg)"),
            InputPort<int   >("jitter_samples",      5,    "Samples for robust avg"),
            InputPort<bool  >("use_smoothed_peak",   true, "Smooth scores before peak pick"),
            InputPort<double>("smooth_sigma_deg",    8.0,  "Gaussian smooth sigma (deg)"),
            InputPort<double>("peak_margin",         0.5,  "Accept within (max - margin)"),

            // (Optional) publish on BT blackboard
            OutputPort<double>("kick_dir")
        };
    }

    NodeStatus tick() override;

private:
    Brain *brain;
    rclcpp::Time lastExecutionTime;
    double prev_angle_rad_ = 0.0;  // sticky memory across ticks
};