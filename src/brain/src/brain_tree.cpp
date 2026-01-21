#include <cmath>
#include <cstdlib>
#include "brain_tree.h"
#include "locator.h"
#include "brain.h"
#include "utils/math.h"
#include "utils/print.h"
#include "utils/misc.h"
#include "locator.h"
#include "std_msgs/msg/string.hpp"
#include <fstream>
#include <ios>
#include <vector>
#include <algorithm>
#include <numeric>
#include <limits>

/**
 * 这里使用宏定义来缩减 RegisterBuilder _的代码量
 * REGISTER_BUILDER(Test) 展开后的效果是
 * factory.registerBuilder<Test>(  \
 *      "Test",                    \
 *     [this](const string& name, const NodeConfig& config) { return make_unique<Test>(name, config, brain); });
 */
#define REGISTER_BUILDER(Name)     \
    factory.registerBuilder<Name>( \
        #Name,                     \
        [this](const string &name, const NodeConfig &config) { return make_unique<Name>(name, config, brain); });

void BrainTree::init()
{
    BehaviorTreeFactory factory;

    // Action Nodes
    REGISTER_BUILDER(RobotFindBall)
    REGISTER_BUILDER(Chase)
    REGISTER_BUILDER(SimpleChase)
    REGISTER_BUILDER(Adjust)
    REGISTER_BUILDER(Kick)
    REGISTER_BUILDER(KickLeg)
    REGISTER_BUILDER(StandStill)
    REGISTER_BUILDER(CalcKickDir)
    REGISTER_BUILDER(CalcKickDirPF)
    REGISTER_BUILDER(StrikerDecide)
    REGISTER_BUILDER(CamTrackBall)
    REGISTER_BUILDER(CamFindBall)
    REGISTER_BUILDER(CamFastScan)
    REGISTER_BUILDER(CamScanField)
    REGISTER_BUILDER(newGoalieDecide)
    REGISTER_BUILDER(Y_Keeper)
    // REGISTER_BUILDER(SelfLocate)
    // REGISTER_BUILDER(SelfLocateEnterField)
    // REGISTER_BUILDER(SelfLocate1M)
    // REGISTER_BUILDER(SelfLocateBorder)
    // REGISTER_BUILDER(SelfLocate2T)
    // REGISTER_BUILDER(SelfLocateLT)
    // REGISTER_BUILDER(SelfLocatePT)
    // REGISTER_BUILDER(SelfLocate2X)
    REGISTER_BUILDER(SetVelocity)
    REGISTER_BUILDER(StepOnSpot)
    REGISTER_BUILDER(GoToFreekickPosition)
    REGISTER_BUILDER(GoToFreekickPositionDefender)
    
    REGISTER_BUILDER(GoToReadyPosition)
    REGISTER_BUILDER(GoToGoalBlockingPosition)
    REGISTER_BUILDER(TurnOnSpot)
    REGISTER_BUILDER(MoveToPoseOnField)
    REGISTER_BUILDER(GoBackInField)
    REGISTER_BUILDER(GoalieDecide)
    REGISTER_BUILDER(WaveHand)
    REGISTER_BUILDER(MoveHead)
    REGISTER_BUILDER(CheckAndStandUp)
    REGISTER_BUILDER(Assist)
    REGISTER_BUILDER(DefenderDecide)
    REGISTER_BUILDER(DefenceChase)

    // 注册 Locator 相关的节点
    brain->registerLocatorNodes(factory);

    // Action Nodes for debug
    REGISTER_BUILDER(CalibrateOdom)
    REGISTER_BUILDER(PrintMsg)
    REGISTER_BUILDER(PlaySound)
    REGISTER_BUILDER(Speak)
    

    factory.registerBehaviorTreeFromFile(brain->config->treeFilePath);
    tree = factory.createTree("MainTree");

    // 构造完成后，初始化 blackboard entry
    initEntry();
}

void BrainTree::initEntry()
{
    setEntry<string>("player_role", brain->config->playerRole);
    setEntry<int>("player_id", brain->config->playerId);
    setEntry<bool>("ball_location_known", false);
    setEntry<bool>("tm_ball_pos_reliable", false);
    setEntry<bool>("ball_out", false);
    setEntry<bool>("track_ball", true);
    setEntry<bool>("odom_calibrated", false);
    setEntry<string>("decision", "");
    setEntry<string>("defend_decision", "chase");
    setEntry<double>("ball_range", 0);

    setEntry<int>("attacker_id", -1);
    setEntry<int>("shadow_id", -1);

    setEntry<bool>("gamecontroller_isKickOff", true);
    setEntry<string>("gc_game_state", "");
    setEntry<string>("gc_game_sub_state_type", "NONE");
    setEntry<string>("gc_game_sub_state", "");
    setEntry<bool>("gc_is_kickoff_side", false);
    setEntry<bool>("gc_is_sub_state_kickoff_side", false);
    setEntry<bool>("gc_is_under_penalty", false);
    setEntry<bool>("is_defender", true);
    setEntry<bool>("is_ready", false); // yaman

    setEntry<bool>("need_check_behind", false);

    setEntry<bool>("is_lead", true); 
    setEntry<string>("goalie_mode", "attack"); 

    setEntry<int>("test_choice", 0);
    setEntry<int>("control_state", 0);
    setEntry<bool>("assist_chase", false);
    setEntry<bool>("assist_kick", false);
    setEntry<bool>("go_manual", false);

    setEntry<bool>("we_just_scored", false);
    setEntry<bool>("wait_for_opponent_kickoff", false);
    setEntry<bool>("is_defender", true);


    // 自动视觉校准相关
    setEntry<string>("calibrate_state", "pitch");
    setEntry<double>("calibrate_pitch_center", 0.0);
    setEntry<double>("calibrate_pitch_step", 1.0);
    setEntry<double>("calibrate_yaw_center", 0.0);
    setEntry<double>("calibrate_yaw_step", 1.0);
    setEntry<double>("calibrate_z_center", 0.0);
    setEntry<double>("calibrate_z_step", 0.01);
}

void BrainTree::tick()
{
    tree.tickOnce();
}

NodeStatus SetVelocity::tick()
{
    double x, y, theta;
    vector<double> targetVec;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    auto res = brain->client->setVelocity(x, y, theta);
    return NodeStatus::SUCCESS;
}

NodeStatus StepOnSpot::tick()
{
    std::srand(std::time(0));
    double vx = (std::rand() / (RAND_MAX / 0.02)) - 0.01;

    auto res = brain->client->setVelocity(vx, 0, 0);
    return NodeStatus::SUCCESS;
}

NodeStatus KickLeg::tick()
{

    brain->client->kickLeg();
    return NodeStatus::SUCCESS;
}

NodeStatus CamTrackBall::tick()
{
    double pitch, yaw, ballX, ballY, deltaX, deltaY;
    const double pixToleranceX = brain->config->camPixX / 4.; 
    const double pixToleranceY = brain->config->camPixY / 4.;
    const double xCenter = brain->config->camPixX / 2;
    const double yCenter = brain->config->camPixY / 2; 

    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/CamTrackBall", rerun::TextLog(msg));
    };
    auto logTrackingBox = [=](int color, string label) {
        brain->log->setTimeNow();
        vector<rerun::Vec2D> mins;
        vector<rerun::Vec2D> sizes;
        mins.push_back(rerun::Vec2D{xCenter - pixToleranceX, yCenter - pixToleranceY});
        sizes.push_back(rerun::Vec2D{pixToleranceX * 2, pixToleranceY * 2});
        brain->log->log(
            "image/track_ball",
            rerun::Boxes2D::from_mins_and_sizes(mins, sizes)
                .with_labels({label})
                .with_colors(color)
        );   

    };

    bool iSeeBall = brain->data->ballDetected;
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
        return NodeStatus::SUCCESS;

    if (!iSeeBall)
    { 
        if (iKnowBallPos) {
            pitch = brain->data->ball.pitchToRobot;
            yaw = brain->data->ball.yawToRobot;
        } else if (tmBallPosReliable) {
            pitch = brain->data->tmBall.pitchToRobot;
            yaw = brain->data->tmBall.yawToRobot;
        } else {
            log("reached impossible condition");
        }
        logTrackingBox(0x000000FF, "ball not detected"); 
    }
    else {      
        ballX = mean(brain->data->ball.boundingBox.xmax, brain->data->ball.boundingBox.xmin);
        ballY = mean(brain->data->ball.boundingBox.ymax, brain->data->ball.boundingBox.ymin);
        deltaX = ballX - xCenter;
        deltaY = ballY - yCenter; 
        
        if (std::fabs(deltaX) < pixToleranceX && std::fabs(deltaY) < pixToleranceY)
        {
            auto label = format("ballX: %.1f, ballY: %.1f, deltaX: %.1f, deltaY: %.1f", ballX, ballY, deltaX, deltaY);
            logTrackingBox(0x00FF00FF, label);
            return NodeStatus::SUCCESS;
        }

        double smoother = 1.5;
        double deltaYaw = deltaX / brain->config->camPixX * brain->config->camAngleX / smoother;
        double deltaPitch = deltaY / brain->config->camPixY * brain->config->camAngleY / smoother;

        pitch = brain->data->headPitch + deltaPitch;
        yaw = brain->data->headYaw - deltaYaw;
        auto label = format("ballX: %.1f, ballY: %.1f, deltaX: %.1f, deltaY: %.1f, pitch: %.1f, yaw: %.1f", ballX, ballY, deltaX, deltaY, pitch, yaw);
        logTrackingBox(0xFF0000FF, label);
    }

    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

CamFindBall::CamFindBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain)
{
    double lowPitch = 1.0;
    double highPitch = 0.45;
    double leftYaw = 1.1;
    double rightYaw = -1.1;

    _cmdSequence[0][0] = lowPitch;
    _cmdSequence[0][1] = leftYaw;
    _cmdSequence[1][0] = lowPitch;
    _cmdSequence[1][1] = 0;
    _cmdSequence[2][0] = lowPitch;
    _cmdSequence[2][1] = rightYaw;
    _cmdSequence[3][0] = highPitch;
    _cmdSequence[3][1] = rightYaw;
    _cmdSequence[4][0] = highPitch;
    _cmdSequence[4][1] = 0;
    _cmdSequence[5][0] = highPitch;
    _cmdSequence[5][1] = leftYaw;

    _cmdIndex = 0;
    _cmdIntervalMSec = 800;
    _cmdRestartIntervalMSec = 50000;
    _timeLastCmd = brain->get_clock()->now();
}

NodeStatus CamFindBall::tick()
{
    if (brain->data->ballDetected)
    {
        return NodeStatus::SUCCESS;
    }

    auto curTime = brain->get_clock()->now();
    auto timeSinceLastCmd = (curTime - _timeLastCmd).nanoseconds() / 1e6;
    if (timeSinceLastCmd < _cmdIntervalMSec)
    {
        return NodeStatus::SUCCESS;
    } 
    else if (timeSinceLastCmd > _cmdRestartIntervalMSec)
    {                 
        _cmdIndex = 0; 
    }
    else
    { 
        _cmdIndex = (_cmdIndex + 1) % (sizeof(_cmdSequence) / sizeof(_cmdSequence[0]));
    }

    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    _timeLastCmd = brain->get_clock()->now();
    return NodeStatus::SUCCESS;
}

NodeStatus CamScanField::tick()
{
    auto sec = brain->get_clock()->now().seconds();
    auto msec = static_cast<unsigned long long>(sec * 1000);
    double lowPitch, highPitch, leftYaw, rightYaw;
    getInput("low_pitch", lowPitch);
    getInput("high_pitch", highPitch);
    getInput("left_yaw", leftYaw);
    getInput("right_yaw", rightYaw);
    int msecCycle;
    getInput("msec_cycle", msecCycle);

    int cycleTime = msec % msecCycle;
    double pitch = cycleTime > (msecCycle / 2.0) ? lowPitch : highPitch;
    double yaw = cycleTime < (msecCycle / 2.0) ? (leftYaw - rightYaw) * (2.0 * cycleTime / msecCycle) + rightYaw : (leftYaw - rightYaw) * (2.0 * (msecCycle - cycleTime) / msecCycle) + rightYaw;

    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

NodeStatus Chase::tick()
{
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/Chase4", rerun::TextLog(msg));
    };
    log("ticked");
    
    double vxLimit, vyLimit, vthetaLimit, dist, safeDist, v;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("dist", dist);
    getInput("safe_dist", safeDist);
    getInput("speed", v);
    bool avoidObstacle;
    brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle);
    double oaSafeDist;
    brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist);

    if (
        brain->config->limitNearBallSpeed
        && brain->data->ball.range < brain->config->nearBallRange
    ) {
        vxLimit = min(brain->config->nearBallSpeedLimit, vxLimit);
    }

    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
    double kickDir = brain->data->kickDir;
    double theta_br = atan2(
        brain->data->robotPoseToField.y - brain->data->ball.posToField.y,
        brain->data->robotPoseToField.x - brain->data->ball.posToField.x
    );
    double theta_rb = brain->data->robotBallAngleToField;
    auto ballPos = brain->data->ball.posToField;


    double vx, vy, vtheta;
    Pose2D target_f, target_r; 
    static string targetType = "direct"; 
    static double circleBackDir = 1.0; 
    double dirThreshold = M_PI / 2;
    if (targetType == "direct") dirThreshold *= 1.2;


    // 计算目标点
    if (fabs(toPInPI(kickDir - theta_rb)) < dirThreshold) {
        log("targetType = direct");
        targetType = "direct";
        target_f.x = ballPos.x - dist * cos(kickDir);
        target_f.y = ballPos.y - dist * sin(kickDir);
    } else {
        targetType = "circle_back";
        double cbDirThreshold = 0.0; 
        cbDirThreshold -= 0.2 * circleBackDir; 
        circleBackDir = toPInPI(theta_br - kickDir) > cbDirThreshold ? 1.0 : -1.0;
        log(format("targetType = circle_back, circleBackDir = %.1f", circleBackDir));
        double tanTheta = theta_br + circleBackDir * acos(min(1.0, safeDist/max(ballRange, 1e-5))); 
        target_f.x = ballPos.x + safeDist * cos(tanTheta);
        target_f.y = ballPos.y + safeDist * sin(tanTheta);
    }
    target_r = brain->data->field2robot(target_f);
    brain->log->setTimeNow();
    brain->log->logBall("field/chase_target", Point({target_f.x, target_f.y, 0}), 0xFFFFFFFF, false, false);
            
    double targetDir = atan2(target_r.y, target_r.x);
    double distToObstacle = brain->distToObstacle(targetDir);
    if (avoidObstacle && distToObstacle < oaSafeDist) {
        log("avoid obstacle");
        auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
        const double speed = 0.5;
        vx = speed * cos(avoidDir);
        vy = speed * sin(avoidDir);
        vtheta = ballYaw;
    } else {
        vx = min(vxLimit, v*brain->data->ball.range);
        vy = 0;
        vtheta = targetDir;
        if (fabs(targetDir) < 0.1 && ballRange > 2.0) vtheta = 0.0;
        vx *= sigmoid((fabs(vtheta)), 1, 3); 
    }

    vx = cap(vx, vxLimit, -vxLimit);
    vy = cap(vy, vyLimit, -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    static double smoothVx = 0.0;
    static double smoothVy = 0.0;
    static double smoothVtheta = 0.0;
    smoothVx = smoothVx * 0 + vx * 1;
    smoothVy = smoothVy * 0.7 + vy * 0.3;
    smoothVtheta = smoothVtheta * 0.7 + vtheta * 0.3;

    // brain->client->setVelocity(smoothVx, smoothVy, smoothVtheta, false, false, false);
    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus SimpleChase::tick()
{
    double stopDist, stopAngle, vyLimit, vxLimit;
    getInput("stop_dist", stopDist);
    getInput("stop_angle", stopAngle);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);

    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vx = brain->data->ball.posToRobot.x;
    double vy = brain->data->ball.posToRobot.y;
    double vtheta = brain->data->ball.yawToRobot * 4.0; 

    double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3)); 
    vx *= linearFactor;
    vy *= linearFactor;

    vx = cap(vx, vxLimit, -1.0);    
    vy = cap(vy, vyLimit, -vyLimit); 

    if (brain->data->ball.range < stopDist)
    {
        vx = 0;
        vy = 0;
        // if (fabs(brain->data->ball.yawToRobot) < stopAngle) vtheta = 0; 
    }

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}


// NodeStatus GoToFreekickPosition::onStart() {
//     // brain->log->log("debug/freekick_position/onStart", rerun::TextLog(format("stage onStart")));
//     _isInFinalAdjust = false;
//     return NodeStatus::RUNNING;
// }

// NodeStatus GoToFreekickPosition::onRunning() {
//     auto log = [=](string msg) {
//         // brain->log->setTimeNow();
//         // brain->log->log("debug/GoToFreekickPosition", rerun::TextLog(msg));
//     };
//     log("running");


//     string side;
//     getInput("side", side);
//     if (side !="attack" && side != "defense") return NodeStatus::SUCCESS;
    
//     Pose2D targetPose;
//     auto fd = brain->config->fieldDimensions;
//     auto ballPos = brain->data->ball.posToField;
//     auto robotPose = brain->data->robotPoseToField;

//     if (side == "attack") {
//         double targetDir = brain->data->kickDir;
//        double dist;
//        getInput("attack_dist", dist);

//        targetPose.x = ballPos.x - dist * cos(targetDir);
//        targetPose.y = ballPos.y - dist * sin(targetDir);
//        targetPose.theta = targetDir;

//         if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2)
//         {
//             if (!brain->isPrimaryStriker()) {
//                 targetPose.y = 0;
//                 targetPose.x -= 1.5;
//                 if (targetPose.x < -fd.length / 2.0 + fd.goalAreaLength) targetPose.x = -fd.length / 2.0 + fd.goalAreaLength;
//                 auto buffer = 2.0;
//                 auto targetXPose = brain->config->fieldDimensions.length / 2 - buffer;
//                 if (targetPose.x > targetXPose) {
//                     targetPose.x = targetXPose;
//                     targetPose.theta = 0;
//                 }
//             }
//         }

//     } else if (side == "defense") {
//         double targetDir = atan2(ballPos.y, ballPos.x + fd.length / 2);
//         double dist;
//         getInput("defense_dist", dist);
//         targetPose.x = ballPos.x - dist * cos(targetDir);
//         targetPose.y = ballPos.y - dist * sin(targetDir);
//         targetPose.theta = targetDir;
//         if (ballPos.x < -fd.length / 2 + 1.0)  targetPose.x = -fd.length / 2 + 1.5;

//         if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2)
//         {
//             if (!brain->isPrimaryStriker()) {
//                 targetPose.y = targetPose.y > 0 ? targetPose.y - 1.0 : targetPose.y + 1.0;
//             }
//         }
//     }

//     double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
//     double deltaDir = toPInPI(targetPose.theta - robotPose.theta);


//     if ( 
//         dist < 0.2 
//         && fabs(deltaDir) < 0.1
//     ) {
//         brain->client->setVelocity(0, 0, 0);
//         return NodeStatus::SUCCESS;
//     }

//     if (!brain->get_parameter("obstacle_avoidance.enable_freekick_avoid").as_bool() || dist < 1.0 || _isInFinalAdjust) {
//         _isInFinalAdjust = true; 
//         auto targetPose_r = brain->data->field2robot(targetPose);

//         double vx = targetPose_r.x;
//         double vy = targetPose_r.y;
//         double vtheta = brain->data->ball.yawToRobot * 4.0; 

//         double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3)); 
//         vx *= linearFactor;
//         vy *= linearFactor;


//         Line path = {robotPose.x, robotPose.y, targetPose.x, targetPose.y};
//         if (
//             pointMinDistToLine(Point2D({ballPos.x, ballPos.y}), path) < 0.5
//             && brain->data->ball.range < 1.0
//         ) {
//             vx = min(0.0, vx);
//             vy = vy >= 0 ? vy + 0.1: vy - 0.1;
//         }

//         double vxLimit, vyLimit;
//         getInput("vx_limit", vxLimit);
//         getInput("vy_limit", vyLimit);
//         vx = cap(vx, vxLimit, -1.0);    
//         vy = cap(vy, vyLimit, -vyLimit);    
        

//         brain->client->setVelocity(vx, vy, vtheta, false, false, false);
//         return NodeStatus::RUNNING;
//     }

//     double longRangeThreshold = 1.0;
//     double turnThreshold = 0.4;
//     double vxLimit = 0.6;
//     double vyLimit = 0.5;
//     double vthetaLimit = 1.5;
//     bool avoidObstacle = true;
//     // brain->log->log("debug/freekick_position", rerun::TextLog(format("stage move: targetPose: (%.2f, %.2f, %.2f)", targetPose.x, targetPose.y, targetPose.theta)));
//     brain->client->moveToPoseOnField3(targetPose.x, targetPose.y, targetPose.theta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, 0.2, 0.2, 0.1, avoidObstacle);

//     return NodeStatus::RUNNING;
// }

// void GoToFreekickPosition::onHalted() {
//     // brain->log->log("debug/freekick_position/onHault", rerun::TextLog(format("stage OnHalted")));
// }

NodeStatus GoToFreekickPosition::onStart() {
    // brain->log->log("debug/freekick_position/onStart", rerun::TextLog(format("stage onStart")));
    _isInFinalAdjust = false;
    _wentToOrigin = false;
    return NodeStatus::RUNNING;
}

NodeStatus GoToFreekickPosition::onRunning() {
    auto log = [=](std::string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/GoToFreekickPosition", rerun::TextLog(msg));
    };
    log("running");

    std::string side;
    getInput("side", side);
    if (side != "attack" && side != "defense") return NodeStatus::SUCCESS;

    Pose2D targetPose;
    auto fd = brain->config->fieldDimensions;
    auto ballPos = brain->data->ball.posToField;
    auto robotPose = brain->data->robotPoseToField;

    // =========================
    // PRE-PHASE: go to (0,0,0) if requested
    // =========================
    bool startAtOrigin = false;
    (void)getInput("start_at_origin", startAtOrigin); // default false if not provided

    if (startAtOrigin && !_wentToOrigin) {
        Pose2D originPose{0.0, 0.0, 0.0};

        double dist0 = norm(originPose.x - robotPose.x, originPose.y - robotPose.y);
        double deltaDir0 = toPInPI(originPose.theta - robotPose.theta);

        if (dist0 < 0.2 && fabs(deltaDir0) < 0.1) {
            _wentToOrigin = true; // reached origin, continue to normal freekick logic
            brain->client->setVelocity(0, 0, 0, false, false, false);
        } else {
            // near/far split reusing your existing thresholds & planner
            if (!brain->get_parameter("obstacle_avoidance.enable_freekick_avoid").as_bool() || dist0 < 1.0 || _isInFinalAdjust) {
                _isInFinalAdjust = true;
                auto origin_r = brain->data->field2robot(originPose);

                double vx = origin_r.x;
                double vy = origin_r.y;
                // simple heading settle toward 0 (face field +X), modest gain
                double vtheta = toPInPI(-brain->data->robotPoseToField.theta) * 2.0;

                double vxLimit = 0.6, vyLimit = 0.5;
                (void)getInput("vx_limit", vxLimit);
                (void)getInput("vy_limit", vyLimit);
                vx = cap(vx, vxLimit, -1.0);
                vy = cap(vy, vyLimit, -vyLimit);

                brain->client->setVelocity(vx, vy, vtheta, false, false, false);
            } else {
                double longRangeThreshold = 1.0;
                double turnThreshold = 0.4;
                double vxLimit = 0.6;
                double vyLimit = 0.5;
                double vthetaLimit = 1.5;
                bool avoidObstacle = true;
                brain->client->moveToPoseOnField3(
                    originPose.x, originPose.y, originPose.theta,
                    longRangeThreshold, turnThreshold,
                    vxLimit, vyLimit, vthetaLimit,
                    0.2, 0.2, 0.1, avoidObstacle
                );
            }
            return NodeStatus::RUNNING; // keep moving to origin
        }
    }

    // =========================
    // NORMAL FREEKICK POSITIONING (unchanged)
    // =========================
    if (side == "attack") {
        double targetDir = brain->data->kickDir;
        double dist;
        getInput("attack_dist", dist);

        targetPose.x = ballPos.x - dist * cos(targetDir);
        targetPose.y = ballPos.y - dist * sin(targetDir);
        targetPose.theta = targetDir;

        // If the ball is on/near a boundary line, place robot OUTSIDE the field first,
        // so it kicks the ball inward (kick-in style) instead of from inside the field.
        const double eps = 0.15;                 // boundary proximity threshold (m)
        const double out_off = 0.25;            // how far to stand outside (m)
        const double axis_back = std::min(0.20, std::max(0.0, dist)); // small backoff along -kick axis
        const double sx = fd.length / 2.0;
        const double sy = fd.width / 2.0;

        bool near_left_goal  = std::fabs(ballPos.x + sx) < eps;
        bool near_right_goal = std::fabs(ballPos.x - sx) < eps;
        bool near_bottom_side = std::fabs(ballPos.y + sy) < eps;
        bool near_top_side    = std::fabs(ballPos.y - sy) < eps;

        bool outsideApplied = false;
        if (near_top_side || near_bottom_side) {
            // Sideline: stand outside along ±Y, then back off along -kick axis, clamp to remain outside
            double signy = (ballPos.y >= 0.0) ? 1.0 : -1.0;
            double y_out = signy * (sy + out_off);
            targetPose.x = ballPos.x - axis_back * std::cos(targetDir);
            targetPose.y = y_out       - axis_back * std::sin(targetDir);
            if (signy * targetPose.y < sy + out_off) targetPose.y = y_out; // ensure outside
            targetPose.theta = targetDir;
            outsideApplied = true;
        } else if (near_left_goal || near_right_goal) {
            // Goal line: stand outside along ±X, then back off along -kick axis, clamp to remain outside
            double signx = (ballPos.x >= 0.0) ? 1.0 : -1.0;
            double x_out = signx * (sx + out_off);
            targetPose.x = x_out       - axis_back * std::cos(targetDir);
            targetPose.y = ballPos.y   - axis_back * std::sin(targetDir);
            if (signx * targetPose.x < sx + out_off) targetPose.x = x_out; // ensure outside
            targetPose.theta = targetDir;
            outsideApplied = true;
        }

        // Force kick direction toward opponent goal center (positive X side)
        // regardless of previous kickDir, to avoid kicking toward our own goal on freekicks.
        {
            const double goal_x = +fd.length / 2.0; // opponent goal center X
            const double goal_y = 0.0;              // center Y
            brain->data->kickDir = atan2(goal_y - ballPos.y, goal_x - ballPos.x);
            targetPose.theta = brain->data->kickDir;
        }

        if (!outsideApplied && brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2) {
            if (!brain->isPrimaryStriker()) {
                targetPose.y = 0;
                targetPose.x -= 1.5;
                if (targetPose.x < -fd.length / 2.0 + fd.goalAreaLength)
                    targetPose.x = -fd.length / 2.0 + fd.goalAreaLength;
                auto buffer = 2.0;
                auto targetXPose = brain->config->fieldDimensions.length / 2 - buffer;
                if (targetPose.x > targetXPose) {
                    targetPose.x = targetXPose;
                    targetPose.theta = 0;
                }
            }
        }

    } else if (side == "defense") {
        double targetDir = atan2(ballPos.y, ballPos.x + fd.length / 2);
        double dist;
        getInput("defense_dist", dist);
        targetPose.x = ballPos.x - dist * cos(targetDir);
        targetPose.y = ballPos.y - dist * sin(targetDir);
        targetPose.theta = targetDir;
        if (ballPos.x < -fd.length / 2 + 1.0)  targetPose.x = -fd.length / 2 + 1.5;

        if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2) {
            if (!brain->isPrimaryStriker()) {
                targetPose.y = targetPose.y > 0 ? targetPose.y - 1.0 : targetPose.y + 1.0;
            }
        }
    }

    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    double deltaDir = toPInPI(targetPose.theta - robotPose.theta);

    if (dist < 0.2 && fabs(deltaDir) < 0.1) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    if (!brain->get_parameter("obstacle_avoidance.enable_freekick_avoid").as_bool() || dist < 1.0 || _isInFinalAdjust) {
        _isInFinalAdjust = true; 
        auto targetPose_r = brain->data->field2robot(targetPose);

        double vx = targetPose_r.x;
        double vy = targetPose_r.y;
        double vtheta = brain->data->ball.yawToRobot * 4.0; 

        double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3)); 
        vx *= linearFactor;
        vy *= linearFactor;

        Line path = {robotPose.x, robotPose.y, targetPose.x, targetPose.y};
        if (pointMinDistToLine(Point2D({ballPos.x, ballPos.y}), path) < 0.5 && brain->data->ball.range < 1.0) {
            vx = std::min(0.0, vx);
            vy = vy >= 0 ? vy + 0.1 : vy - 0.1;
        }

        double vxLimit, vyLimit;
        getInput("vx_limit", vxLimit);
        getInput("vy_limit", vyLimit);
        vx = cap(vx, vxLimit, -1.0);    
        vy = cap(vy, vyLimit, -vyLimit);    

        brain->client->setVelocity(vx, vy, vtheta, false, false, false);
        return NodeStatus::RUNNING;
    }

    double longRangeThreshold = 1.0;
    double turnThreshold = 0.4;
    double vxLimit = 0.6;
    double vyLimit = 0.5;
    double vthetaLimit = 1.5;
    bool avoidObstacle = true;
    brain->client->moveToPoseOnField3(
        targetPose.x, targetPose.y, targetPose.theta,
        longRangeThreshold, turnThreshold,
        vxLimit, vyLimit, vthetaLimit,
        0.2, 0.2, 0.1, avoidObstacle
    );

    return NodeStatus::RUNNING;
}


void GoToFreekickPosition::onHalted() {
    // brain->log->log("debug/freekick_position/onHault", rerun::TextLog(format("stage OnHalted")));
}


NodeStatus GoToGoalBlockingPosition::tick() {
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/GoToGoalBlockingPosition", rerun::TextLog(msg));
    };
    log("GoToGoalBlockingPosition ticked");

    // brain->log->setTimeNow();
    // brain->log->log("tree/GoToGoalBlockingPosition", rerun::TextLog("GoToGoalBlockingPosition tick"));
    
    double distTolerance = getInput<double>("dist_tolerance").value();
    double thetaTolerance = getInput<double>("theta_tolerance").value();
    double distToGoalline = getInput<double>("dist_to_goalline").value();

    auto fd = brain->config->fieldDimensions;
    auto ballPos = brain->data->ball.posToField;
    auto robotPose = brain->data->robotPoseToField;

    string curRole = brain->tree->getEntry<string>("player_role");

    Pose2D targetPose;
    targetPose.x = curRole == "striker" ? (std::max(- fd.length / 2.0 + distToGoalline, ballPos.x - 1.5))
            : (- fd.length / 2.0 + distToGoalline);
    if (ballPos.x + fd.length / 2.0 < distToGoalline) {
        targetPose.y = curRole == "striker" ? (ballPos.y > 0 ? fd.goalWidth / 2.0 : -fd.goalWidth / 2.0)
            : (ballPos.y > 0 ? fd.goalWidth / 4.0 : -fd.goalWidth / 4.0);
    } else {
        targetPose.y = ballPos.y * distToGoalline / (ballPos.x + fd.length / 2.0);
        targetPose.y = curRole == "striker" ? (cap(targetPose.y, fd.goalWidth / 2.0, -fd.goalWidth / 2.0))
            : (cap(targetPose.y, fd.penaltyAreaWidth/ 2.0, -fd.penaltyAreaWidth / 2.0));
    }

    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    if ( // 认为到达了目标位置
        dist < distTolerance
        && fabs(brain->data->ball.yawToRobot) < thetaTolerance
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    auto targetPose_r = brain->data->field2robot(targetPose);
    double vx = targetPose_r.x;
    double vy = targetPose_r.y;
    double vtheta = brain->data->ball.yawToRobot * 4.0; 


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    vx = cap(vx, vxLimit, -vxLimit);    
    vy = cap(vy, vyLimit, -vyLimit);    
    

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}

// NodeStatus Assist::tick()
// {
//     double distTolerance = getInput<double>("dist_tolerance").value();
//     double thetaTolerance = getInput<double>("theta_tolerance").value();
//     double distToGoalline = getInput<double>("dist_to_goalline").value();

//     auto fd        = brain->config->fieldDimensions;
//     auto ballPos   = brain->data->ball.posToField;
//     auto robotPose = brain->data->robotPoseToField;

//     int selfIdx     = brain->config->playerId - 1;
//     int attacker_id = -1, shadow_id = -1;

//     // read attacker/shadow chosen by StrikerDecide
//     try {
//         attacker_id = brain->tree->getEntry<int>("attacker_id");
//         shadow_id   = brain->tree->getEntry<int>("shadow_id");
//     } catch (...) {
//         attacker_id = -1; shadow_id = -1;
//     }

//     bool iAmShadow   = (selfIdx == shadow_id);
//     Pose2D targetPose{ robotPose.x, robotPose.y, robotPose.theta };

//     if (iAmShadow && attacker_id >= 0) {
//         // 1 m behind attacker along (attacker->ball), plus 1 m toward center line
//         auto atkSt  = brain->data->tmStatus[attacker_id];
//         Pose2D atk  = atkSt.robotPoseToField;

//         double dir_ab = std::atan2(ballPos.y - atk.y, ballPos.x - atk.x);

//         targetPose.x = atk.x - 1.0 * std::cos(dir_ab);
//         targetPose.y = atk.y - 1.0 * std::sin(dir_ab);

//         double centerStep = (atk.y > 0 ? -1.0 : +1.0);
//         if (std::fabs(atk.y) < 1.0) centerStep = (atk.y > 0 ? -std::fabs(atk.y) : +std::fabs(atk.y));
//         targetPose.y += centerStep;

//         targetPose.theta = std::atan2(ballPos.y - targetPose.y, ballPos.x - targetPose.x);

//         targetPose.x = cap(targetPose.x,  fd.length/2.0 - 0.5, -fd.length/2.0 + distToGoalline);
//         targetPose.y = cap(targetPose.y,  fd.width/2.0  - 0.1, -fd.width/2.0  + 0.1);
//     } else {
//         // legacy assist fallback (kept)
//         bool has2Assists = false;
//         bool isSecondary = false;

//         for (int i = 0; i < HL_MAX_NUM_PLAYERS; ++i) {
//             if (i == selfIdx) continue;
//             auto tm = brain->data->tmStatus[i];
//             if (!tm.isAlive) continue;
//             if (tm.isLead)   continue;
//             if (tm.role != "striker") continue;
//             has2Assists = true;
//             if (tm.robotPoseToField.x > robotPose.x) isSecondary = true;
//         }

//         targetPose.x = isSecondary ? ballPos.x - 4.0 : ballPos.x - 2.0;
//         targetPose.x = std::max(targetPose.x, - fd.length / 2.0 + distToGoalline);
//         targetPose.y = ballPos.y * (targetPose.x + fd.length / 2.0) / (ballPos.x + fd.length / 2.0);
//         if (has2Assists) targetPose.y += isSecondary ? -0.5 : 0.5;
//         targetPose.theta = std::atan2(ballPos.y - targetPose.y, ballPos.x - targetPose.x);
//     }

//     // Arrived?
//     double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
//     if (dist < distTolerance && std::fabs(brain->data->ball.yawToRobot) < thetaTolerance) {
//         brain->client->setVelocity(0, 0, 0);
//         return NodeStatus::SUCCESS;
//     }

//     // Move with your existing pattern
//     auto target_r = brain->data->field2robot(targetPose);
//     double targetDir = std::atan2(target_r.y, target_r.x);
//     double ballYaw = brain->data->ball.yawToRobot;

//     bool avoidObstacle;
//     brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle);
//     double oaSafeDist;
//     brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist);

//     double vx, vy, vtheta;
//     double distToObstacle = brain->distToObstacle(targetDir);

//     if (avoidObstacle && distToObstacle < oaSafeDist) {
//         auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
//         const double speed = 0.5;
//         vx = speed * std::cos(avoidDir);
//         vy = speed * std::sin(avoidDir);
//         vtheta = ballYaw;
//     } else {
//         vx = target_r.x;
//         vy = target_r.y;
//         vtheta = ballYaw * 4.0;
//     }

//     double vxLimit, vyLimit;
//     getInput("vx_limit", vxLimit);
//     getInput("vy_limit", vyLimit);
//     vx = cap(vx, vxLimit, -1.0);
//     vy = cap(vy, vyLimit, -vyLimit);

//     brain->client->setVelocity(vx, vy, vtheta, false, false, false);
//     return NodeStatus::SUCCESS;
// }

NodeStatus Assist::tick() {
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/Assist", rerun::TextLog(msg));
    };
    log("ticked");

    double distTolerance = getInput<double>("dist_tolerance").value();
    double thetaTolerance = getInput<double>("theta_tolerance").value();
    double distToGoalline = getInput<double>("dist_to_goalline").value();

    auto fd = brain->config->fieldDimensions; //dimension of the field
    auto ballPos = brain->data->ball.posToField; 
    auto robotPose = brain->data->robotPoseToField;
    string curRole = brain->tree->getEntry<string>("player_role"); //current role

    bool isSecondary = false; 
    bool has2Assists = false;
    int selfIdx = brain->config->playerId - 1;
    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == selfIdx) continue; 

        auto tmStatus = brain->data->tmStatus[i];
        if (!tmStatus.isAlive) continue; 
        if (tmStatus.isLead) continue; 
        if (tmStatus.role != "striker") continue; 

        has2Assists = true;
        log("2 assists found");
        if (tmStatus.robotPoseToField.x > robotPose.x) {
            log("i am secondary");
            isSecondary = true; 
        }
    }
    log(format("has2Assists: %d, isSecondary: %d", has2Assists, isSecondary));


    Pose2D targetPose;
    targetPose.x = isSecondary ? ballPos.x - 4.0 : ballPos.x - 2.0;
    targetPose.x = max(targetPose.x, - fd.length / 2.0 + distToGoalline); 
    targetPose.y = ballPos.y * (targetPose.x + fd.length / 2.0) / (ballPos.x + fd.length / 2.0); 
    if (has2Assists) { 
        targetPose.y += isSecondary ? - 0.5 : 0.5;
    }


    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    if ( 
        dist < distTolerance
        && fabs(brain->data->ball.yawToRobot) < thetaTolerance
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vx, vy, vtheta;
    auto targetPose_r = brain->data->field2robot(targetPose);
    double targetDir = atan2(targetPose_r.y, targetPose_r.x);
    double distToObstacle = brain->distToObstacle(targetDir);

    bool avoidObstacle;
    brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle);
    double oaSafeDist;
    brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist);

    if (avoidObstacle && distToObstacle < oaSafeDist) {
        log("avoid obstacle");
        auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
        const double speed = 0.5;
        vx = speed * cos(avoidDir);
        vy = speed * sin(avoidDir);
        vtheta = brain->data->ball.yawToRobot;
    } else {
        vx = targetPose_r.x;
        vy = targetPose_r.y;
        vtheta = brain->data->ball.yawToRobot * 4.0; 
    }


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    vx = cap(vx, vxLimit, -1.0);     
    vy = cap(vy, vyLimit, -vyLimit);     
    

    brain->client->setVelocity(vx, vy, vtheta, false, false, false);
    return NodeStatus::SUCCESS;
}


// NodeStatus Assist::tick() {
//     auto log = [=](string msg) {
//         brain->log->setTimeNow();
//         brain->log->log("debug/Assist", rerun::TextLog(msg));
//     };
//     log("ticked");

//     double distTolerance = getInput<double>("dist_tolerance").value();
//     double thetaTolerance = getInput<double>("theta_tolerance").value();
//     double distToGoalline = getInput<double>("dist_to_goalline").value();

//     auto fd = brain->config->fieldDimensions; //dimension of the field
//     auto ballPos = brain->data->ball.posToField; 
//     auto robotPose = brain->data->robotPoseToField;
//     string curRole = brain->tree->getEntry<string>("player_role"); //current role

//     bool isSecondary = false; 
//     bool has2Assists = false;
//     int selfIdx = brain->config->playerId - 1;
//     for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
//         if (i == selfIdx) continue; 

//         auto tmStatus = brain->data->tmStatus[i];
//         if (!tmStatus.isAlive) continue; 
//         if (tmStatus.isLead) continue; 
//         if (tmStatus.role != "striker") continue; 

//         has2Assists = true;
//         log("2 assists found");
//         if (tmStatus.robotPoseToField.x > robotPose.x) {
//             log("i am secondary");
//             isSecondary = true; 
//         }
//     }
//     log(format("has2Assists: %d, isSecondary: %d", has2Assists, isSecondary));


//     Pose2D targetPose;
//     targetPose.x = isSecondary ? ballPos.x - 4.0 : ballPos.x - 2.0;
//     targetPose.x = max(targetPose.x, - fd.length / 2.0 + distToGoalline); 
//     targetPose.y = ballPos.y * (targetPose.x + fd.length / 2.0) / (ballPos.x + fd.length / 2.0); 
//     if (has2Assists) { 
//         targetPose.y += isSecondary ? - 0.5 : 0.5;
//     }


//     double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
//     if ( 
//         dist < distTolerance
//         && fabs(brain->data->ball.yawToRobot) < thetaTolerance
//     ) {
//         brain->client->setVelocity(0, 0, 0);
//         return NodeStatus::SUCCESS;
//     }

//     double vx, vy, vtheta;
//     auto targetPose_r = brain->data->field2robot(targetPose);
//     double targetDir = atan2(targetPose_r.y, targetPose_r.x);
//     double distToObstacle = brain->distToObstacle(targetDir);

//     bool avoidObstacle;
//     brain->get_parameter("obstacle_avoidance.avoid_during_chase", avoidObstacle);
//     double oaSafeDist;
//     brain->get_parameter("obstacle_avoidance.chase_ao_safe_dist", oaSafeDist);

//     if (avoidObstacle && distToObstacle < oaSafeDist) {
//         log("avoid obstacle");
//         auto avoidDir = brain->calcAvoidDir(targetDir, oaSafeDist);
//         const double speed = 0.5;
//         vx = speed * cos(avoidDir);
//         vy = speed * sin(avoidDir);
//         vtheta = brain->data->ball.yawToRobot;
//     } else {
//         vx = targetPose_r.x;
//         vy = targetPose_r.y;
//         vtheta = brain->data->ball.yawToRobot * 4.0; 
//     }


//     double vxLimit, vyLimit;
//     getInput("vx_limit", vxLimit);
//     getInput("vy_limit", vyLimit);
//     vx = cap(vx, vxLimit, -1.0);     
//     vy = cap(vy, vyLimit, -vyLimit);     
    

//     brain->client->setVelocity(vx, vy, vtheta, false, false, false);
//     return NodeStatus::SUCCESS;
// }


NodeStatus Adjust::tick()
{
    auto log = [=](string msg) { 
        brain->log->setTimeNow();
        brain->log->log("debug/adjust5", rerun::TextLog(msg)); 
    };
    log("enter");

    if (!brain->tree->getEntry<bool>("ball_location_known"))
        return NodeStatus::SUCCESS;

    // -------- Inputs --------
    double turnThreshold, vxLimit, vyLimit, vthetaLimit, range, st_far, st_near, vtheta_factor, NEAR_THRESHOLD;
    getInput("near_threshold", NEAR_THRESHOLD);
    getInput("tangential_speed_far", st_far);
    getInput("tangential_speed_near", st_near);
    getInput("vtheta_factor", vtheta_factor);
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("range", range);

    double NO_TURN_THRESHOLD, TURN_FIRST_THRESHOLD;
    getInput("no_turn_threshold", NO_TURN_THRESHOLD);
    getInput("turn_first_threshold", TURN_FIRST_THRESHOLD);

    log(format("ballX: %.1f ballY: %.1f ballYaw: %.1f",
               brain->data->ball.posToRobot.x,
               brain->data->ball.posToRobot.y,
               brain->data->ball.yawToRobot));

    // -------- State --------
    double vx = 0, vy = 0, vtheta = 0;
    double kickDir  = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    double ballRange = brain->data->ball.range;
    double ballYaw   = brain->data->ball.yawToRobot;

    double R = ballRange; 
    double r = range;
    double sr = cap(R - r, 0.5, 0);  // inward-only radial bias
    log(format("R: %.2f, r: %.2f, sr: %.2f", R, r, sr));
    log(format("deltaDir = %.1f", deltaDir));

    // -------- CHANGED: Error-based tangential speed --------
    const double ST_MAX = st_far;
    const double ST_MIN = st_near;
    double align_err = std::min(1.0, std::fabs(deltaDir) * std::max(0.3, 1.0 / std::max(0.15, R)));
    double st = ST_MIN + (ST_MAX - ST_MIN) * align_err;
    if (std::fabs(deltaDir) * R < NEAR_THRESHOLD) {
        st = std::max(st, ST_MIN);
        log("use near speed (soft gate)");
    }

    // -------- Compose velocities in robot frame --------
    double theta_robot_f = brain->data->robotPoseToField.theta; 
    double thetat_r = dir_rb_f + M_PI / 2 * (deltaDir > 0 ? -1.0 : 1.0) - theta_robot_f; 
    double thetar_r = dir_rb_f - theta_robot_f; 

    vx = st * std::cos(thetat_r) + sr * std::cos(thetar_r); 
    vy = st * std::sin(thetat_r) + sr * std::sin(thetar_r); 

    // Rotation toward ball bearing
    vtheta  = ballYaw * vtheta_factor; 
    if (std::fabs(ballYaw) < NO_TURN_THRESHOLD) vtheta = 0.0;

    // -------- CHANGED: rarer rotate-first freeze --------
    bool need_rotate_first =
        (std::fabs(ballYaw) > TURN_FIRST_THRESHOLD) &&
        (std::fabs(deltaDir) < M_PI / 6) &&
        (ballRange > r + 0.30);

    if (need_rotate_first) {
        vx = 0;
        vy = 0;
    }

    // ============================================================
    // CHANGE: Derive the "front/behind" and "near/side" gates
    // ------------------------------------------------------------
    // x_rb is the projection of (robot->ball) on kick axis: ball_x - robot_x
    const double x_rb = R * std::cos(deltaDir);
    const bool   IS_FRONT = (x_rb >= 0.0);
    const bool   IS_BEHIND = !IS_FRONT;
    // "near" gate as in your note (keep current CRAB_EXTRA band)
    const double CRAB_EXTRA     = 0.20;
    const bool   IS_NEAR_RADIUS = (ballRange < range + CRAB_EXTRA);
    // "side >= 30deg" gate using TURN_FIRST_THRESHOLD (your 30°)
    const double CRAB_YAW_FAST  = TURN_FIRST_THRESHOLD;
    const bool   IS_SIDE_30 = (std::fabs(ballYaw) > CRAB_YAW_FAST);
    // Kick-axis angle in robot frame, for backing up along -X_kick
    const double theta_kick_r = kickDir - theta_robot_f;

    log(format("gates: x_rb=%.2f front=%d behind=%d near=%d side30=%d",
               x_rb, (int)IS_FRONT, (int)IS_BEHIND, (int)IS_NEAR_RADIUS, (int)IS_SIDE_30));
    // ============================================================

    // ============================================================
    // CHANGE: Priority 1 — "Behind → go back until x_rb >= 0.5"
    // ------------------------------------------------------------
    // If the ball is behind along kick axis, force a back-up along -X_kick
    // until we are at least 0.5 m behind the ball on that axis.
    if (x_rb < -0.5) {                    // keep backing until 0.5 m behind
    const double back_spd = std::max(0.35, st_near);
    vx = -back_spd * std::cos(theta_kick_r);
    vy = -back_spd * std::sin(theta_kick_r);
    log("mode: BACKUP (ball behind >=0.5)");
    }
    else {
    // ============================================================
    // CHANGE: Priority 2 — "In penalty area + aligned with goal + ball front → shoot straight"
    // ------------------------------------------------------------
    // When robot enters penalty area, is aligned with goal (within goal width D=2.6m),
    // and ball is in front, skip crabwalk and go straight to shoot.
    auto fd = brain->config->fieldDimensions;
    auto robotPose = brain->data->robotPoseToField;
    const double PENALTY_AREA_START_X = fd.length / 2.0 - fd.penaltyAreaLength;
    const double GOAL_WIDTH_HALF = fd.goalWidth / 2.0;  // D/2 = 2.6/2 = 1.3m
    const bool IN_PENALTY_AREA = (robotPose.x >= PENALTY_AREA_START_X && robotPose.x <= fd.length / 2.0);
    const bool ALIGNED_WITH_GOAL = (std::fabs(robotPose.y) <= GOAL_WIDTH_HALF);
    const bool BALL_IN_FRONT = (brain->data->ball.posToRobot.x > 0);
    
    bool skip_crabwalk_for_penalty = (IN_PENALTY_AREA && ALIGNED_WITH_GOAL && BALL_IN_FRONT);
    
    if (skip_crabwalk_for_penalty) {
        // Skip crabwalk, use NORMAL mode (already computed velocities above)
        log(format("mode: PENALTY_STRAIGHT (penalty=%d aligned=%d front=%d)", 
                   (int)IN_PENALTY_AREA, (int)ALIGNED_WITH_GOAL, (int)BALL_IN_FRONT));
    }
    // ============================================================
    // CHANGE: Priority 3 — Crabwalk only when Near AND Side AND Front (but NOT in penalty area)
    // ------------------------------------------------------------
    // This prevents over-crabwalking and stops side-shuffling that can nudge
    // the ball toward our own goal when we're not yet behind it.
    else if (!need_rotate_first && IS_FRONT && IS_NEAR_RADIUS && IS_SIDE_30)
    {
        const double VY_BASE        = std::max(0.35, st_near);
        const double VY_GAIN        = 1.2;
        const double VX_KEEP        = 0.25;

        vtheta = 0.0; // keep body fixed while shuffling
        double vy_cmd = (ballYaw > 0 ? 1.0 : -1.0) * (VY_BASE + VY_GAIN * std::fabs(ballYaw));
        vy = cap(vy_cmd, vyLimit, -vyLimit);

        double vx_bias = std::max(0.15, std::min(VX_KEEP, sr));
        vx = cap(vx_bias, vxLimit, 0.0);

        log("mode: CRABWALK (near + side + front)"); // CHANGE: log mode
    }
    else {
        // Otherwise we stay in NORMAL ADJUST (already computed above)
        log("mode: NORMAL"); // CHANGE: log mode
    }
    } // end of else block for Priority 1 (BACKUP)
    // ============================================================

    // -------- Caps --------
    vx     = cap(vx,     vxLimit,  -vxLimit);
    vy     = cap(vy,     vyLimit,  -vyLimit);
    vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

    // -------- OPTIONAL: Slew limits --------
    static double last_vx = 0, last_vy = 0, last_vtheta = 0;
    auto slew = [](double target, double prev, double max_step) {
        double dv = target - prev;
        if (dv >  max_step) return prev + max_step;
        if (dv < -max_step) return prev - max_step;
        return target;
    };
    const double MAX_DVX  = 0.20;  // m/s per tick
    const double MAX_DVY  = 0.30;  // m/s per tick
    const double MAX_DVTH = 0.60;  // rad/s per tick
    vx     = slew(vx,     last_vx,     MAX_DVX);
    vy     = slew(vy,     last_vy,     MAX_DVY);
    vtheta = slew(vtheta, last_vtheta, MAX_DVTH);
    last_vx = vx; last_vy = vy; last_vtheta = vtheta;

    // -------- Send --------
    log(format("vx: %.1f vy: %.1f vtheta: %.1f", vx, vy, vtheta));
    brain->client->setVelocity(vx, vy, vtheta);
    return NodeStatus::SUCCESS;
}

// NodeStatus Adjust::tick()
// {
//     auto log = [=](string msg) {
//         brain->log->setTimeNow();
//         brain->log->log("debug/adjust6", rerun::TextLog(msg));
//     };
//     log("enter");

//     if (!brain->tree->getEntry<bool>("ball_location_known"))
//         return NodeStatus::SUCCESS;

//     // Defaults first (safe if XML misses any input)
//     double vxLimit = 0.7, vyLimit = 0.35, vthetaLimit = 1.6;
//     double range = 0.30;                 // desired ring distance
//     double st_far = 0.6, st_near = 0.12; // tangential speed band
//     double vtheta_factor = 2.0;          // yaw gain
//     double NO_TURN_THRESHOLD = 0.06;     // ~3.4°
//     double TURN_FIRST_THRESHOLD = 0.8;   // ~34°
//     double NEAR_THRESHOLD = 0.10;        // soften |delta|*R small

//     getInput("vx_limit", vxLimit);
//     getInput("vy_limit", vyLimit);
//     getInput("vtheta_limit", vthetaLimit);
//     getInput("range", range);
//     getInput("tangential_speed_far", st_far);
//     getInput("tangential_speed_near", st_near);
//     getInput("vtheta_factor", vtheta_factor);
//     getInput("no_turn_threshold", NO_TURN_THRESHOLD);
//     getInput("turn_first_threshold", TURN_FIRST_THRESHOLD);
//     getInput("near_threshold", NEAR_THRESHOLD);

//     // State
//     const double kickDir   = brain->data->kickDir;                 // field
//     const double dir_rb_f  = brain->data->robotBallAngleToField;   // field
//     const double deltaDir  = toPInPI(kickDir - dir_rb_f);          // around-ball error
//     const double R         = brain->data->ball.range;              // robot->ball dist
//     const double r         = range;
//     const double ballYaw   = brain->data->ball.yawToRobot;         // robot frame
//     const double theta_robot_f = brain->data->robotPoseToField.theta;

//     // Inward-only radial bias (never push outward)
//     double sr = cap(R - r, 0.5, 0.0);
//     log(format("R: %.2f r: %.2f sr: %.2f dDeg: %.1f yawDeg: %.1f",
//                R, r, sr, std::fabs(deltaDir) * 180/M_PI, std::fabs(ballYaw) * 180/M_PI));

//     // Tangential speed based on error (bigger error/smaller R => higher)
//     const double ST_MAX = st_far, ST_MIN = st_near;
//     double align_err = std::min(1.0, std::fabs(deltaDir) * std::max(0.3, 1.0 / std::max(0.15, R)));
//     double st = ST_MIN + (ST_MAX - ST_MIN) * align_err;
//     if (std::fabs(deltaDir) * R < NEAR_THRESHOLD) {
//         st = std::min(st, ST_MIN); // avoid whirl close-in
//     }

//     // Base orbit + inward in ROBOT frame
//     const double thetat_r = dir_rb_f + M_PI/2.0 * (deltaDir > 0 ? -1.0 : 1.0) - theta_robot_f;
//     const double thetar_r = dir_rb_f - theta_robot_f;
//     double vx = st * std::cos(thetat_r) + sr * std::cos(thetar_r);
//     double vy = st * std::sin(thetat_r) + sr * std::sin(thetar_r);
//     double vtheta = (std::fabs(ballYaw) < NO_TURN_THRESHOLD) ? 0.0 : ballYaw * vtheta_factor;

//     // Rotate-first if body bearing is poor but around-ball is already okay
//     bool need_rotate_first =
//         (std::fabs(ballYaw) > TURN_FIRST_THRESHOLD) &&
//         (std::fabs(deltaDir) < M_PI / 6) &&
//         (R > r + 0.30);
//     if (need_rotate_first) {
//         vx = 0.0; vy = 0.0;
//         log("mode: ROTATE_FIRST");
//     } else {
//         // Kick-axis geometry
//         const double x_rb = R * std::cos(deltaDir);          // >0: robot behind ball
//         const double theta_kick_r = kickDir - theta_robot_f; // robot frame

//         // Priority 1 — BACKUP (get behind ball)
//         if (x_rb < -0.35) {
//             const double back_spd = std::max(0.35, ST_MIN);
//             vx = -back_spd * std::cos(theta_kick_r);
//             vy = -back_spd * std::sin(theta_kick_r);
//             log("mode: BACKUP");
//         }
//         // Priority 2 — STRICT CRAB (rare; only when it helps)
//         else {
//             const double FRONT_MARGIN = 0.20;
//             const double R_BAND = 0.12;
//             const double SIDE_MIN = std::max(TURN_FIRST_THRESHOLD, 0.6); // ~34°
//             const double MIN_RANGE_FOR_CRAB = 0.25;

//             const bool CLEAR_FRONT  = (x_rb > FRONT_MARGIN);
//             const bool TIGHT_RADIUS = (std::fabs(R - r) < R_BAND);
//             const bool SIDE_ENOUGH  = (std::fabs(ballYaw) > SIDE_MIN);
//             const bool RANGE_OK     = (R > MIN_RANGE_FOR_CRAB);

//             if (CLEAR_FRONT && TIGHT_RADIUS && SIDE_ENOUGH && RANGE_OK) {
//                 vtheta = 0.0; // lock heading while shuffling
//                 const double VY_BASE = std::max(0.35, ST_MIN);
//                 const double VY_GAIN = 1.0;
//                 double vy_cmd = (ballYaw > 0 ? 1.0 : -1.0) * (VY_BASE + VY_GAIN * std::fabs(ballYaw));
//                 vy = cap(vy_cmd, vyLimit, -vyLimit);

//                 double vx_bias = (R > r) ? std::max(0.12, std::min(0.20, sr)) : 0.0;
//                 vx = cap(vx_bias, vxLimit, 0.0);
//                 log("mode: CRAB");
//             }
//             // Priority 3 — DIRECT HIT (aligned + near ring)
//             else if (CLEAR_FRONT && (std::fabs(deltaDir) < 10.0 * M_PI/180.0) && (R < r + 0.18)) {
//                 vy = 0.0; vtheta = 0.0;
//                 vx = std::min(vxLimit, std::max(ST_MIN, 0.6));
//                 log("mode: DIRECT_HIT");
//             } else {
//                 log("mode: NORMAL"); // keep base orbit+inward
//             }
//         }
//     }

//     // Caps and slew
//     vx     = cap(vx,     vxLimit,  -vxLimit);
//     vy     = cap(vy,     vyLimit,  -vyLimit);
//     vtheta = cap(vtheta, vthetaLimit, -vthetaLimit);

//     static double last_vx = 0.0, last_vy = 0.0, last_vtheta = 0.0;
//     auto slew = [](double t, double p, double s){ double d=t-p; if(d>s) return p+s; if(d<-s) return p-s; return t; };
//     vx     = slew(vx,     last_vx,     0.20);
//     vy     = slew(vy,     last_vy,     0.30);
//     vtheta = slew(vtheta, last_vtheta, 0.60);
//     last_vx = vx; last_vy = vy; last_vtheta = vtheta;

//     log(format("cmd -> vx: %.2f vy: %.2f vtheta: %.2f", vx, vy, vtheta));
//     brain->client->setVelocity(vx, vy, vtheta);
//     return NodeStatus::SUCCESS;
// }

NodeStatus CalcKickDir::tick()
{
    // 读取和处理参数
    double crossThreshold;
    getInput("cross_threshold", crossThreshold);

    // string lastKickType = brain->data->kickType;
    // if (lastKickType == "cross") crossThreshold += 0.1;

    // auto gpAngles = brain->getGoalPostAngles(0.0);
    // auto thetal = gpAngles[0]; auto thetar = gpAngles[1];
    // auto bPos = brain->data->ball.posToField;
    // auto fd = brain->config->fieldDimensions;
    // auto color = 0xFFFFFFFF; // for log
    
    // if (thetal - thetar < crossThreshold && brain->data->ball.posToField.x > fd.circleRadius) {
    //     brain->data->kickType = "cross";
    //     color = 0xFF00FFFF;
    //     brain->data->kickDir = atan2(
    //         - bPos.y,
    //         fd.length/2 - fd.penaltyDist/2 - bPos.x
    //     );
    // }
    // else if (brain->isDefensing()) {
    //     brain->data->kickType = "block";
    //     color = 0xFFFF00FF;
    //     brain->data->kickDir = atan    double crossThreshold;
    // getInput("cross_threshold", crossThreshold);

    // string lastKickType = brain->data->kickType;
    // if (lastKickType == "cross") crossThreshold += 0.1;

    // auto gpAngles = brain->getGoalPostAngles(0.0);
    // auto thetal = gpAngles[0]; auto thetar = gpAngles[1];
    // auto bPos = brain->data->ball.posToField;
    // auto fd = brain->config->fieldDimensions;
    // auto color = 0xFFFFFFFF; // for log
    
    // if (thetal - thetar < crossThreshold && brain->data->ball.posToField.x > fd.circleRadius) {
    //     brain->data->kickType = "cross";
    //     color = 0xFF00FFFF;
    //     brain->data->kickDir = atan2(
    //         - bPos.y,
    //         fd.length/2 - fd.penaltyDist/2 - bPos.x
    //     );
    // }
    // else if (brain->isDefensing()) {
    //     brain->data->kickType = "block";
    //     color = 0xFFFF00FF;
    //     brain->data->kickDir = atan2(
    //         bPos.y,
    //         bPos.x + fd.length/2
    //     );

    // // } else { 
    // //     brain->data->kickType = "shoot";
    // //     color = 0x00FF00FF;
    // //     brain->data->kickDir = atan2(
    // //         - bPos.y,
    // //         fd.length/2 - bPos.x
    // //     );
    //     if (brain->data->ball.posToField.x > brain->config->fieldDimensions.length / 2) brain->data->kickDir = 0; 
    // }

    // brain->log->setTimeNow();
    // brain->log->log(
    //     "field/kick_dir",
    //     rerun::Arrows2D::from_vectors({{10 * cos(brain->data->kickDir), -10 * sin(brain->data->kickDir)}})
    //         .with_origins({{brain->data->ball.posToField.x, -brain->data->ball.posToField.y}})
    //         .with_colors({color})
    //         .with_radii(0.01)
    //         .with_draw_order(31)
    // );2(
    //         bPos.y,
    //         bPos.x + fd.length/2
    //     );

    // } else { 
    //     brain->data->kickType = "shoot";
    //     color = 0x00FF00FF;
    //     brain->data->kickDir = atan2(
    //         - bPos.y,
    //         fd.length/2 - bPos.x
    //     );
    //     if (brain->data->ball.posToField.x > brain->config->fieldDimensions.length / 2) brain->data->kickDir = 0; 
    // }

    // brain->log->setTimeNow();
    // brain->log->log(
    //     "field/kick_dir",
    //     rerun::Arrows2D::from_vectors({{10 * cos(brain->data->kickDir), -10 * sin(brain->data->kickDir)}})
    //         .with_origins({{brain->data->ball.posToField.x, -brain->data->ball.posToField.y}})
    //         .with_colors({color})
    //         .with_radii(0.01)
    //         .with_draw_order(31)
    // );

    return NodeStatus::SUCCESS;
}


// NodeStatus StrikerDecide::tick()
// {
//     double chaseRangeThreshold;
//     getInput("chase_threshold", chaseRangeThreshold);

//     std::string lastDecision, position;
//     getInput("decision_in", lastDecision);
//     getInput("position", position);

//     const double  kickDir   = brain->data->kickDir;
//     const double  dir_rb_f  = brain->data->robotBallAngleToField;
//     const auto    ball      = brain->data->ball;
//     const double  ballRange = ball.range;
//     const double  ballYaw   = ball.yawToRobot;

//     // original “good to kick” window
//     const double goalpostMargin = 0.3;
//     double variablemargins = 0.3;
//     bool angleGoodForKick = (dir_rb_f < kickDir + variablemargins && dir_rb_f > kickDir - variablemargins);
//     if (ball.posToRobot.x > 6 && std::fabs(ball.posToRobot.y) > 1.5) {
//         angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");
//     }

//     // Rank all strikers by cost (range + angle term), same pattern as original 2-way logic
//     struct SRank { int id; double cost; };
//     std::vector<SRank> ranks; ranks.reserve(HL_MAX_NUM_PLAYERS);

//     auto costOf = [&](double range, double thetaRb, double kdir) {
//         const double kcost = 1.5;
//         double thetaCost = std::abs(thetaRb - kdir);
//         return range + thetaCost * kcost;
//     };

//     // self
//     {
//         int self = brain->config->playerId - 1;
//         ranks.push_back({ self, costOf(ballRange, dir_rb_f, kickDir) });
//     }
//     // teammates
//     for (int i = 0; i < HL_MAX_NUM_PLAYERS; ++i) {
//         if (i == brain->config->playerId - 1) continue;
//         const auto st = brain->data->tmStatus[i];
//         const bool notPenalized = (brain->data->penalty[i] == PENALTY_NONE);
//         if (!notPenalized || !st.isAlive || st.role != "striker") continue;
//         double c = costOf(st.ballRange, st.thetaRb, st.kickDir);
//         ranks.push_back({ i, c });
//     }

//     if (!ranks.empty()) {
//         std::sort(ranks.begin(), ranks.end(),
//                   [](const SRank& a, const SRank& b){ return a.cost < b.cost; });
//     }

//     int attacker_id = -1, shadow_id = -1;
//     if (!ranks.empty()) {
//         attacker_id = ranks[0].id;
//         if (ranks.size() >= 2) shadow_id = ranks[1].id;
//     }
//     brain->tree->setEntry<int>("attacker_id", attacker_id);
//     brain->tree->setEntry<int>("shadow_id",   shadow_id);

//     const int selfIdx = brain->config->playerId - 1;
//     const bool iAmAttacker = (selfIdx == attacker_id);
//     const bool iAmShadow   = (selfIdx == shadow_id);
//     const bool iAmSupport  = (!iAmAttacker && !iAmShadow);

//     // original obstacle-avoid during kick
//     bool avoidPushing;
//     double kickAoSafeDist;
//     brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
//     brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
//     bool avoidKick = avoidPushing
//         && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
//         && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;

//     // original “reached kick dir” helper
//     static rclcpp::Time timeLastTick = rclcpp::Time(0,0,RCL_ROS_TIME);
//     static double lastDeltaDir = 0.0;
//     double deltaDir = toPInPI(kickDir - dir_rb_f);
//     auto now = brain->get_clock()->now();
//     auto dt  = brain->msecsSince(timeLastTick);
//     bool reachedKickDir =
//         (deltaDir * lastDeltaDir <= 0 && std::fabs(deltaDir) < M_PI/6 && dt < 100) ||
//         std::fabs(deltaDir) < 0.1;
//     timeLastTick = now;
//     lastDeltaDir = deltaDir;

//     const bool iKnowBallPos      = brain->tree->getEntry<bool>("ball_location_known");
//     const bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");

//     std::string newDecision;

//     if (!(iKnowBallPos || tmBallPosReliable)) {
//         newDecision = "find";
//     }
//     else if (iAmSupport) {
//         newDecision = "retreat";       // 3rd = backstop
//     }
//     else if (iAmShadow) {
//         newDecision = "assist";        // 2nd = shadow (Assist handles geometry)
//     }
//     else { // Attacker
//         if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0)) {
//             newDecision = "chase";
//         }
//         else if (((angleGoodForKick && !brain->data->isFreekickKickingOff) || reachedKickDir)
//                  && brain->data->ballDetected
//                  && std::fabs(ballYaw) < M_PI/2.
//                  && !avoidKick) {
//             newDecision = (brain->data->kickType == "cross") ? "cross" : "kick";
//             brain->data->isFreekickKickingOff = false;
//         }
//         else {
//             newDecision = "adjust";
//         }
//     }

//     setOutput("decision_out", newDecision);
//     return NodeStatus::SUCCESS;
// }

NodeStatus StrikerDecide::tick() {
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/striker_decide", rerun::TextLog(msg));
    };

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);
    getInput("position", position);

    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField; 
    auto ball = brain->data->ball;
    double ballRange = ball.range;
    double ballYaw = ball.yawToRobot;
    double ballX = ball.posToRobot.x;
    double ballY = ball.posToRobot.y;
    
    const double goalpostMargin = 0.3 ;
    double variablemargins = 0.4;//- 0.2 * ballX/7; 
    //bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");
    bool angleGoodForKick = (dir_rb_f < kickDir + variablemargins && dir_rb_f > kickDir - variablemargins );
    auto myIdx = brain->config->playerId - 1;
    if (ballX > 6 && abs(ballY) > 1.5) {bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");}
    // Initialize from my own status (needed for lost_cost / defender_id to be valid)
    // this section determines if I am a defender or striker using cost comparison
    int defender_id = myIdx;
    bool defender = false; 
    double thetaCost = abs(dir_rb_f - kickDir);
    double kcost = 1.5;

    for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
        if (i == myIdx) {
            continue; // skip myself
        }

        auto status = brain->data->tmStatus[i];

        if (status.isAlive && status.role == "striker") {
             prtDebug("If statement line 1826");

            // compare cost
            double statusThetaCost = abs(status.thetaRb - status.kickDir);
            if (status.ballRange + statusThetaCost * kcost <  ballRange + thetaCost * kcost) {
                defender = true;
                cout << "defender: " << defender;
                prtDebug("is defender is put true");
                break;
            }
            else{
                cout << "defender: " << defender;
                defender = false;
                prtDebug("is defender is put false");

            }
        } 
    }
    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    bool avoidKick = avoidPushing 
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;

    log(format("ballRange: %.2f, ballYaw: %.2f, ballX:%.2f, ballY: %.2f kickDir: %.2f, dir_rb_f: %.2f, angleGoodForKick: %d",
        ballRange, ballYaw, ballX, ballY, kickDir, dir_rb_f, angleGoodForKick));

    
    double deltaDir = toPInPI(kickDir - dir_rb_f);
    auto now = brain->get_clock()->now();
    auto dt = brain->msecsSince(timeLastTick);
    bool reachedKickDir = 
        deltaDir * lastDeltaDir <= 0 
        && fabs(deltaDir) < M_PI / 6
        && dt < 100;
    reachedKickDir = reachedKickDir || fabs(deltaDir) < 0.1;
    timeLastTick = now;
    lastDeltaDir = deltaDir;
    bool isKickoff = brain->tree->getEntry<bool>("gc_is_kickoff_side");
    bool isReady = brain->tree->getEntry<bool>("is_ready");
    
    
    string newDecision;
    auto color = 0xFFFFFFFF; 
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
        color = 0xFFFFFFFF;
    } else if (defender && ball.posToField.x > 0) {
        newDecision = "retreat";
        color = 0x00FFFFFF;
    } else if (isKickoff && isReady){
        newDecision = "shoot";
        prtDebug("Kick Leg Executing");
        brain->tree->setEntry<bool>("is_ready", false);
    }
    
    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x0000FFFF;
    } else if (
        (
            (angleGoodForKick && !brain->data->isFreekickKickingOff) 
            || reachedKickDir
        )
        && brain->data->ballDetected
        && fabs(brain->data->ball.yawToRobot) < M_PI / 2.
        && !avoidKick
        //&& ball.range < 1.5
    ) {
        if (brain->data->kickType == "cross") newDecision = "cross";
        else newDecision = "kick";      
        color = 0x00FF00FF;
        brain->data->isFreekickKickingOff = false; 
    }
    else
    {
        newDecision = "adjust";
        color = 0xFFFF00FF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen(
        "tree/Decide",
        format(
            "Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleGoodForKick: %d lead: %d", 
            newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleGoodForKick, brain->data->tmImLead
        ),
        color
    );
    return NodeStatus::SUCCESS;
}

// NodeStatus StrikerDecide::tick() {
//     auto log = [=](string msg) {
//         brain->log->setTimeNow();
//         brain->log->log("debug/striker_decide", rerun::TextLog(msg));
//     };

//     double chaseRangeThreshold;
//     getInput("chase_threshold", chaseRangeThreshold);
//     string lastDecision, position;
//     getInput("decision_in", lastDecision);
//     getInput("position", position);

//     double kickDir = brain->data->kickDir;
//     double dir_rb_f = brain->data->robotBallAngleToField; 
//     auto ball = brain->data->ball;
//     double ballRange = ball.range;
//     double ballYaw = ball.yawToRobot;
//     double ballX = ball.posToRobot.x;
//     double ballY = ball.posToRobot.y;
    
//     const double goalpostMargin = 0.3 ;
//     double variablemargins = 0.4;//- 0.2 * ballX/7; 
//     //bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");
//     bool angleGoodForKick = (dir_rb_f < kickDir + variablemargins && dir_rb_f > kickDir - variablemargins );
//     auto myIdx = brain->config->playerId - 1;
//     //if (ballX > 6 && abs(ballY) > 1.5) {bool angleGoodForKick = brain->isAngleGood(goalpostMargin, "kick");}
//     // Initialize from my own status (needed for lost_cost / defender_id to be valid)
//     // this section determines if I am a defender or striker using cost comparison
//     int defender_id = myIdx;
//     bool defender = false; 
//     double thetaCost = abs(dir_rb_f - kickDir);
//     double kcost = 1.5;

//     for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
//         if (i == myIdx) continue; // skip myself
//         auto status = brain->data->tmStatus[i];
//         if (status.isAlive && status.role == "striker") {
//             // compare cost
//             double statusThetaCost = abs(status.thetaRb - status.kickDir);
//             if (status.ballRange + statusThetaCost * kcost <  ballRange + thetaCost * kcost) {
//                 defender = true;
//                 cout << "defender: " << defender;
//                 prtDebug("is defender is put true");
//                 break;
//             }
//             else{
//                 cout << "defender: " << defender;
//                 defender = false;
//                 prtDebug("is defender is put false");

//             }
//         } 
//     }
//     bool avoidPushing;
//     double kickAoSafeDist;
//     brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
//     brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
//     bool avoidKick = avoidPushing 
//         && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
//         && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist;

//     log(format("ballRange: %.2f, ballYaw: %.2f, ballX:%.2f, ballY: %.2f kickDir: %.2f, dir_rb_f: %.2f, angleGoodForKick: %d",
//         ballRange, ballYaw, ballX, ballY, kickDir, dir_rb_f, angleGoodForKick));

    
//     double deltaDir = toPInPI(kickDir - dir_rb_f);
//     auto now = brain->get_clock()->now();
//     auto dt = brain->msecsSince(timeLastTick);
//     bool reachedKickDir = 
//         deltaDir * lastDeltaDir <= 0 
//         && fabs(deltaDir) < M_PI / 6
//         && dt < 100;
//     reachedKickDir = reachedKickDir || fabs(deltaDir) < 0.1;
//     timeLastTick = now;
//     lastDeltaDir = deltaDir;
    
//     string newDecision;
//     auto color = 0xFFFFFFFF; 
//     bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
//     bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
//     if (!(iKnowBallPos || tmBallPosReliable))
//     {
//         newDecision = "find";
//         color = 0xFFFFFFFF;
//     } else if (defender && ball.posToField.x > 0) {
//         newDecision = "retreat";
//         color = 0x00FFFFFF;
//     } else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
//     {
//         newDecision = "chase";
//         color = 0x0000FFFF;
//     } else if (
//         (
//             (angleGoodForKick && !brain->data->isFreekickKickingOff) 
//             || reachedKickDir
//         )
//         && brain->data->ballDetected
//         && fabs(brain->data->ball.yawToRobot) < M_PI / 2.
//         && !avoidKick
//         //&& ball.range < 1.5
//     ) {
//         if (brain->data->kickType == "cross") newDecision = "cross";
//         else newDecision = "kick";      
//         color = 0x00FF00FF;
//         brain->data->isFreekickKickingOff = false; 
//     }
//     else
//     {
//         newDecision = "adjust";
//         color = 0xFFFF00FF;
//     }

//     setOutput("decision_out", newDecision);
//     brain->log->logToScreen(
//         "tree/Decide",
//         format(
//             "Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleGoodForKick: %d lead: %d", 
//             newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleGoodForKick, brain->data->tmImLead
//         ),
//         color
//     );
//     return NodeStatus::SUCCESS;
// }

NodeStatus GoalieDecide::tick()
{

    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);

    double kickDir = atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2);
    double dir_rb_f = brain->data->robotBallAngleToField;
    auto goalPostAngles = brain->getGoalPostAngles(0.3);
    double theta_l = goalPostAngles[0]; 
    double theta_r = goalPostAngles[1]; 
    bool angleIsGood = (dir_rb_f > -M_PI / 2 && dir_rb_f < M_PI / 2);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;

    string newDecision;
    auto color = 0xFFFFFFFF; 
    bool iKnowBallPos = brain->tree->getEntry<bool>("ball_location_known");
    bool tmBallPosReliable = brain->tree->getEntry<bool>("tm_ball_pos_reliable");
    if (!(iKnowBallPos || tmBallPosReliable))
    {
        newDecision = "find";
        color = 0x0000FFFF;
    }
    else if (brain->data->ball.posToField.x > 0 - static_cast<double>(lastDecision == "retreat"))
    {
        newDecision = "retreat";
        color = 0xFF00FFFF;
    } else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x00FF00FF;
    }
    else if (angleIsGood)
    {
        newDecision = "kick";
        color = 0xFF0000FF;
    }
    else
    {
        newDecision = "adjust";
        color = 0x00FFFFFF;
    }

    setOutput("decision_out", newDecision);
    brain->log->logToScreen("tree/Decide",
                            format("Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleIsGood: %d", newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleIsGood),
                            color);
    return NodeStatus::SUCCESS;
}

tuple<double, double, double> Kick::_calcSpeed() {
    double vx, vy, msecKick;


    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    int minMSecKick;
    getInput("min_msec_kick", minMSecKick);
    double vxFactor = brain->config->vxFactor;   
    double yawOffset = brain->config->yawOffset; 


    double adjustedYaw = brain->data->ball.yawToRobot + yawOffset;
    double tx = cos(adjustedYaw) * brain->data->ball.range; 
    double ty = sin(adjustedYaw) * brain->data->ball.range;

    if (fabs(ty) < 0.01 && fabs(adjustedYaw) < 0.01)
    { 
        vx = vxLimit;
        vy = 0.0;
    }
    else
    { 
        vy = ty > 0 ? vyLimit : -vyLimit;
        vx = vy / ty * tx * vxFactor;
        if (fabs(vx) > vxLimit)
        {
            vy *= vxLimit / vx;
            vx = vxLimit;
        }
    }


    double speed = norm(vx, vy);
    msecKick = speed > 1e-5 ? minMSecKick + static_cast<int>(brain->data->ball.range / speed * 1000) : minMSecKick;
    
    return make_tuple(vx, vy, msecKick);
}

NodeStatus Kick::onStart()
{
    _minRange = brain->data->ball.range;
    _speed = 2.5;
    _startTime = brain->get_clock()->now();


    bool avoidPushing;
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    string role = brain->tree->getEntry<string>("player_role");
    if (
        avoidPushing
        && (role != "goal_keeper")
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // 发布运动指令
    double angle = brain->data->ball.yawToRobot;
    brain->client->crabWalk(angle, _speed);
    return NodeStatus::RUNNING;
}

NodeStatus Kick::onRunning()
{
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/Kick", rerun::TextLog(msg));
    };


    bool enableAbort;
    brain->get_parameter("strategy.abort_kick_when_ball_moved", enableAbort);
    auto ballRange = brain->data->ball.range;
    const double MOVE_RANGE_THRESHOLD = 0.3;
    const double BALL_LOST_THRESHOLD = 1000;  
    if (
        enableAbort 
        && (
            (brain->data->ballDetected && ballRange - _minRange > MOVE_RANGE_THRESHOLD) 
            || brain->msecsSince(brain->data->ball.timePoint) > BALL_LOST_THRESHOLD 
        )
    ) {
        log("ball moved, abort kick");
        return NodeStatus::SUCCESS;
    }


    if (ballRange < _minRange) _minRange = ballRange;    

    
    bool avoidPushing;
    brain->get_parameter("obstacle_avoidance.avoid_during_kick", avoidPushing);
    double kickAoSafeDist;
    brain->get_parameter("obstacle_avoidance.kick_ao_safe_dist", kickAoSafeDist);
    if (
        avoidPushing
        && brain->data->robotPoseToField.x < brain->config->fieldDimensions.length / 2 - brain->config->fieldDimensions.goalAreaLength
        && brain->distToObstacle(brain->data->ball.yawToRobot) < kickAoSafeDist
    ) {
        brain->client->setVelocity(-0.1, 0, 0);
        return NodeStatus::SUCCESS;
    }


    double msecs = getInput<double>("min_msec_kick").value();
    double speed = getInput<double>("speed_limit").value();
    msecs = msecs + brain->data->ball.range / speed * 1000;
    if (brain->msecsSince(_startTime) > msecs) { 
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }


    if (brain->data->ballDetected) { 
        double angle = brain->data->ball.yawToRobot;
        double speed = getInput<double>("speed_limit").value();
        _speed += 0.1; 
        speed = min(speed, _speed);
        brain->client->crabWalk(angle, speed);
    }

    return NodeStatus::RUNNING;
}

void Kick::onHalted()
{
    _startTime -= rclcpp::Duration(100, 0);
}

NodeStatus StandStill::onStart()
{

    _startTime = brain->get_clock()->now();


    brain->client->setVelocity(0, 0, 0);
    return NodeStatus::RUNNING;
}

NodeStatus StandStill::onRunning()
{
    double msecs;
    getInput("msecs", msecs);
    if (brain->msecsSince(_startTime) < msecs) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::RUNNING;
    }


    return NodeStatus::SUCCESS;
}

void StandStill::onHalted()
{
    double msecs;
    getInput("msecs", msecs);
    _startTime -= rclcpp::Duration(- 2 * msecs, 0);
}


NodeStatus RobotFindBall::onStart()
{
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/RobotFindBall", rerun::TextLog(msg));
    };
    log("RobotFindBall onStart");

    if (brain->data->ballDetected)
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }
    _turnDir = brain->data->ball.yawToRobot > 0 ? 1.0 : -1.0;

    return NodeStatus::RUNNING;
}

NodeStatus RobotFindBall::onRunning()
{
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/RobotFindBall", rerun::TextLog(msg));
    };
    log("RobotFindBall onRunning");

    if (brain->data->ballDetected)
    {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    double vyawLimit;
    getInput("vyaw_limit", vyawLimit);

    double vx = 0;
    double vy = 0;
    double vtheta = 0;
    if (brain->data->ball.range < 0.3)
    { 
      // vx = cap(-brain->data->ball.posToRobot.x, 0.2, -0.2);
      // vy = cap(-brain->data->ball.posToRobot.y, 0.2, -0.2);
    }
    // vtheta = _turnDir > 0 ? vyawLimit : -vyawLimit;
    brain->client->setVelocity(0, 0, vyawLimit * _turnDir);
    return NodeStatus::RUNNING;
}

void RobotFindBall::onHalted()
{
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/RobotFindBall", rerun::TextLog(msg));
    };
    log("RobotFindBall onHalted");
    _turnDir = 1.0;
}

NodeStatus CamFastScan::onStart()
{
    _cmdIndex = 0;
    _timeLastCmd = brain->get_clock()->now();
    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    return NodeStatus::RUNNING;
}

NodeStatus CamFastScan::onRunning()
{
    double interval = getInput<double>("msecs_interval").value();
    if (brain->msecsSince(_timeLastCmd) < interval) return NodeStatus::RUNNING;

    // else 
    if (_cmdIndex >= 6) return NodeStatus::SUCCESS;

    // else
    _cmdIndex++;
    _timeLastCmd = brain->get_clock()->now();
    brain->client->moveHead(_cmdSequence[_cmdIndex][0], _cmdSequence[_cmdIndex][1]);
    return NodeStatus::RUNNING;
}

NodeStatus TurnOnSpot::onStart()
{
    _timeStart = brain->get_clock()->now();
    _lastAngle = brain->data->robotPoseToOdom.theta;
    _cumAngle = 0.0;

    bool towardsBall = false;
    _angle = getInput<double>("rad").value();
    getInput("towards_ball", towardsBall);
    if (towardsBall) {
        double ballPixX = (brain->data->ball.boundingBox.xmin + brain->data->ball.boundingBox.xmax) / 2;
        _angle = fabs(_angle) * (ballPixX < brain->config->camPixX / 2 ? 1 : -1);
    }

    brain->client->setVelocity(0, 0, _angle, false, false, true);
    return NodeStatus::RUNNING;
}

NodeStatus TurnOnSpot::onRunning()
{
    double curAngle = brain->data->robotPoseToOdom.theta;
    double deltaAngle = toPInPI(curAngle - _lastAngle);
    _lastAngle = curAngle;
    _cumAngle += deltaAngle;
    double turnTime = brain->msecsSince(_timeStart);
    // brain->log->log("debug/turn_on_spot", rerun::TextLog(format(
    //     "angle: %.2f, cumAngle: %.2f, deltaAngle: %.2f, time: %.2f",
    //     _angle, _cumAngle, deltaAngle, turnTime
    // )));
    if (
        fabs(_cumAngle) - fabs(_angle) > -0.1
        || turnTime > _msecLimit
    ) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    // else 
    brain->client->setVelocity(0, 0, (_angle - _cumAngle)*2);
    return NodeStatus::RUNNING;
}

NodeStatus MoveToPoseOnField::tick()
{
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/Move", rerun::TextLog(msg));
    };
    log("Move ticked");

    double tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance;
    getInput("x", tx);
    getInput("y", ty);
    getInput("theta", ttheta);
    getInput("long_range_threshold", longRangeThreshold);
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("x_tolerance", xTolerance);
    getInput("y_tolerance", yTolerance);
    getInput("theta_tolerance", thetaTolerance);
    bool avoidObstacle;
    getInput("avoid_obstacle", avoidObstacle);

    brain->client->moveToPoseOnField2(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance, avoidObstacle);
    return NodeStatus::SUCCESS;
}


NodeStatus GoToReadyPosition::tick()
{
    double distTolerance, thetaTolerance;
    getInput("dist_tolerance", distTolerance);
    getInput("theta_tolerance", thetaTolerance);

    std::string role = brain->tree->getEntry<std::string>("player_role");
    bool isKickoff    = brain->tree->getEntry<bool>("gc_is_kickoff_side");
    auto fd           = brain->config->fieldDimensions;
    brain->tree->setEntry<bool>("is_ready", true);
    prtDebug("is Ready True");

    double tx = 0.0, ty = 0.0, ttheta = 0.0;
    double longRangeThreshold = 1.0;
    double turnThreshold      = 0.4;

    double vxLimit, vyLimit;
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    if (brain->distToBorder() > -1.0) {
        vxLimit = 0.5;
        vyLimit = 0.3;
    }
    double vthetaLimit = 1.3;
    bool avoidObstacle = true;

    if (role == "striker")
    {
        // X: on circle if we kick off; just outside if we don't (circle line counts as inside).
        const double R = fd.circleRadius;                 // 0.75 kid, 1.5 adult
        const double marginOpp = isKickoff ? 0.0 : 0.20;
        const double txKick    = -std::max(R, 1.5);       // same as original
        const double txNoKick  = -(R + marginOpp);

        tx = isKickoff ? txKick : txNoKick;


        // Y: three lanes by alive, non-penalized striker ID order (no new helper).
        std::vector<int> aliveStrikers;
        aliveStrikers.reserve(HL_MAX_NUM_PLAYERS);
        int selfIdx = brain->config->playerId - 1;
        for (int i = 0; i < HL_MAX_NUM_PLAYERS; ++i) {
            const auto st = brain->data->tmStatus[i];
            const bool notPenalized = (brain->data->penalty[i] == PENALTY_NONE);
            if (notPenalized && st.isAlive && st.role == "striker") {
                aliveStrikers.push_back(i);
            }
        }
        std::sort(aliveStrikers.begin(), aliveStrikers.end());

        if (aliveStrikers.size() >= 3) {
            static const double LANES[3] = { +1.5, 0.0, -1.5 };
            int myLane = 1; // center default
            auto it = std::find(aliveStrikers.begin(), aliveStrikers.end(), selfIdx);
            if (it != aliveStrikers.end()) {
                int rank = static_cast<int>(std::distance(aliveStrikers.begin(), it)); // 0..N-1
                if (rank < 0) rank = 0;
                if (rank > 2) rank = 2;
                myLane = rank;
            }
            ty = LANES[myLane];
        } else if (aliveStrikers.size() == 2) {
            // original 2-striker fallback
            ty = brain->isPrimaryStriker() ? 1.5 : -1.5;
        } else {
            ty = 0.0;
        }

        ttheta = brain->data->ball.yawToRobot;
    }
    else if (role == "goal_keeper")
    {
        tx = -fd.length / 2.0 + fd.goalAreaLength;
        ty = 0.0;
        ttheta = 0.0;
    }
    else
    {
        tx = 0.0; ty = 0.0; ttheta = 0.0;
    }

    brain->client->moveToPoseOnField2(
        -1.8, 0, ttheta,
        longRangeThreshold, turnThreshold,
        vxLimit, vyLimit, vthetaLimit,
        distTolerance / 3, distTolerance / 1.5, 0.1,
        avoidObstacle
    );
    return NodeStatus::SUCCESS;
}

// NodeStatus GoToReadyPosition::tick()
// {
//     auto log = [=](string msg) {
//         // brain->log->setTimeNow();
//         // brain->log->log("debug/GoToReadyPosition", rerun::TextLog(msg));
//     };
//     log("GoToReadyPosition ticked");

//     double distTolerance, thetaTolerance;
//     getInput("dist_tolerance", distTolerance);
//     getInput("theta_tolerance", thetaTolerance);
//     string role = brain->tree->getEntry<string>("player_role");
//     bool isKickoff = brain->tree->getEntry<bool>("gc_is_kickoff_side");
//     auto fd = brain->config->fieldDimensions;


//     double tx = 0, ty = 0, ttheta = 0; 
//     double longRangeThreshold = 1.0;
//     double turnThreshold = 0.4;
//     double vxLimit, vyLimit;
//     getInput("vx_limit", vxLimit);
//     getInput("vy_limit", vyLimit);
//     if (brain->distToBorder() > - 1.0) { 
//         vxLimit = 0.5;
//         vyLimit = 0.3;
//     }
//     double vthetaLimit = 1.3;
//     bool avoidObstacle = true;

//     if (role == "striker" && isKickoff) {
//         tx = - max(fd.circleRadius, 1.5);
//         ty = 0;
//         if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2)
//         {
//             if (brain->isPrimaryStriker()) {
//                 ty = 1.5;
//             } else {
//                 ty = -1.5;
//             }
//         }
//         ttheta = 0;
//     } else if (role == "striker" && !isKickoff) {
//         tx = - fd.circleRadius * 1.0;
//         ty = 0;
//         if (brain->config->numOfPlayers == 3 && brain->data->liveCount >= 2)
//         {
//             if (brain->isPrimaryStriker()) {
//                 ty = 1.5;
//             } else {
//                 ty = -1.5;
//             }
//         }
//         ttheta = 0;
//     } else if (role == "goal_keeper") {
//         tx = -fd.length / 2.0 + fd.goalAreaLength;
//         ty = 0;
//         ttheta = 0;
//     }

//     brain->client->moveToPoseOnField2(tx, ty, ttheta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, distTolerance / 1.5, distTolerance / 1.5, thetaTolerance, avoidObstacle);
//     return NodeStatus::SUCCESS;
// }

NodeStatus GoBackInField::tick()
{
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/GoBackInField", rerun::TextLog(msg));
    };
    log("GoBackInField ticked");

    double valve;
    getInput("valve", valve);
    double vx = 0; 
    double vy = 0; 
    double dir = 0;
    auto fd = brain->config->fieldDimensions;
    if (brain->data->robotPoseToField.x > fd.length / 2.0 - valve) dir = - M_PI;
    else if (brain->data->robotPoseToField.x < - fd.length / 2.0 + valve) dir = 0;
    else if (brain->data->robotPoseToField.y > fd.width / 2.0 + valve) dir = - M_PI / 2.0;
    else if (brain->data->robotPoseToField.y < - fd.width / 2.0 - valve) dir = M_PI / 2.0;
    else { 
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    
    double dir_r = toPInPI(dir - brain->data->robotPoseToField.theta);
    vx = 0.4 * cos(dir_r);
    vy = 0.4 * sin(dir_r);
    brain->client->setVelocity(vx, vy, 0, false, false, false);
    return NodeStatus::SUCCESS;
}

NodeStatus WaveHand::tick()
{
    string action;
    getInput("action", action);
    if (action == "start")
        brain->client->waveHand(true);
    else
        brain->client->waveHand(false);
    return NodeStatus::SUCCESS;
}

NodeStatus MoveHead::tick()
{
    double pitch, yaw;
    getInput("pitch", pitch);
    getInput("yaw", yaw);
    brain->client->moveHead(pitch, yaw);
    return NodeStatus::SUCCESS;
}

NodeStatus CheckAndStandUp::tick()
{
    if (brain->tree->getEntry<bool>("gc_is_under_penalty") || brain->data->currentRobotModeIndex == 1) {
        brain->data->recoveryPerformedRetryCount = 0;
        brain->data->recoveryPerformed = false;
        brain->log->log("recovery", rerun::TextLog("reset recovery"));
        return NodeStatus::SUCCESS;
    }
    brain->log->log("recovery", rerun::TextLog(format("Recovery retry count: %d, recoveryPerformed: %d recoveryState: %d currentRobotModeIndex: %d", brain->data->recoveryPerformedRetryCount, brain->data->recoveryPerformed, brain->data->recoveryState, brain->data->currentRobotModeIndex)));

    if (!brain->data->recoveryPerformed &&
        brain->data->recoveryState == RobotRecoveryState::HAS_FALLEN &&
        // brain->data->isRecoveryAvailable && 
        brain->data->currentRobotModeIndex == 3 && 
        brain->data->recoveryPerformedRetryCount < brain->get_parameter("recovery.retry_max_count").get_value<int>()) {
        brain->client->standUp();
        brain->data->recoveryPerformed = true;
        brain->speak("Trying to stand up");
        brain->log->log("recovery", rerun::TextLog(format("Recovery retry count: %d", brain->data->recoveryPerformedRetryCount)));
        return NodeStatus::SUCCESS;
    }

    if (brain->data->recoveryPerformed && brain->data->currentRobotModeIndex == 12) {
        brain->data->recoveryPerformedRetryCount +=1;
        brain->data->recoveryPerformed = false;
        brain->log->log("recovery", rerun::TextLog(format("Add retry count: %d", brain->data->recoveryPerformedRetryCount)));
    }


    if (brain->data->recoveryState == RobotRecoveryState::IS_READY &&
        brain->data->currentRobotModeIndex == 8) { 
        brain->data->recoveryPerformedRetryCount = 0;
        brain->data->recoveryPerformed = false;
        brain->log->log("recovery", rerun::TextLog("Reset recovery, recoveryState: " + to_string(static_cast<int>(brain->data->recoveryState))));
    }

    return NodeStatus::SUCCESS;
}


NodeStatus CalibrateOdom::tick()
{
    double x, y, theta;
    getInput("x", x);
    getInput("y", y);
    getInput("theta", theta);

    brain->calibrateOdom(x, y, theta);
    return NodeStatus::SUCCESS;
}

NodeStatus PrintMsg::tick()
{
    Expected<std::string> msg = getInput<std::string>("msg");
    if (!msg)
    {
        throw RuntimeError("missing required input [msg]: ", msg.error());
    }
    std::cout << "[MSG] " << msg.value() << std::endl;
    return NodeStatus::SUCCESS;
}

NodeStatus PlaySound::tick()
{
    string sound;
    getInput("sound", sound);
    bool allowRepeat;
    getInput("allow_repeat", allowRepeat);
    brain->playSound(sound, allowRepeat);
    return NodeStatus::SUCCESS;
}

NodeStatus Speak::tick()
{
    const string lastText;
    string text;
    getInput("text", text);
    if (text == lastText) return NodeStatus::SUCCESS;

    brain->speak(text, false);
    return NodeStatus::SUCCESS;
}

NodeStatus newGoalieDecide::tick()
{
 
    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);
 
    double kickDir = atan2(brain->data->ball.posToField.y, brain->data->ball.posToField.x + brain->config->fieldDimensions.length / 2);
    double dir_rb_f = brain->data->robotBallAngleToField;
    auto goalPostAngles = brain->getGoalPostAngles(0.3);
    double theta_l = goalPostAngles[0];
    double theta_r = goalPostAngles[1];
    bool angleIsGood = (dir_rb_f > -M_PI / 2 && dir_rb_f < M_PI / 2);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
 
    string newDecision;
    auto color = 0xFFFFFFFF; // for log
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        newDecision = "find";
        color = 0x0000FFFF;
    }
    else if (brain->data->ball.posToField.x > -3 - static_cast<double>(lastDecision == "retreat"))
    {
        newDecision = "retreat";
        color = 0xFF00FFFF;
    }
    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x00FF00FF;
    }
    else if (angleIsGood && brain->data->ball.posToField.x > -2.5)
    {
        newDecision = "kick";
        color = 0x00FFFFFF;
    }    else if (angleIsGood)
    {
        newDecision = "chase";
        color = 0xFF0000FF;
    }
    else
    {
        newDecision = "adjust";
        color = 0x00FFFFFF;
    }
 
    setOutput("decision_out", newDecision);
    brain->log->logToScreen("tree/Decide",
                            format("Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleIsGood: %d", newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleIsGood),
                            color);
    return NodeStatus::SUCCESS;
}

// NodeStatus Y_Keeper::tick()
// {
//     double longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance;
//     getInput("long_range_threshold", longRangeThreshold);
//     getInput("turn_threshold", turnThreshold);
//     getInput("vx_limit", vxLimit);
//     getInput("vx_limit", vxLimit);
//     getInput("vy_limit", vyLimit);
//     getInput("vtheta_limit", vthetaLimit);
//     getInput("x_tolerance", xTolerance);
//     getInput("y_tolerance", yTolerance);
//     getInput("theta_tolerance", thetaTolerance);
 
//     double y_pose;
//     double theta;
//     bool ballLocationKnown = brain->tree->getEntry<bool>("ball_location_known");
//     // if(ballLocationKnown){
//     y_pose = brain->data->ball.posToField.y;
//     theta = brain->data->robotBallAngleToField;
//     // }
//     // else{
//     // for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
//     //         auto status = data->tmStatus[i];
//     //         int tmID = status.playerID + 1;
//     //         if (tmID = config->playerId)
//     //         continue;
//     //         else if(status.isAlive && status.ballLocationKnown)
//     //         {
//     //             brain->data->ball.posToField = status.ballPosToField;
//     //             y_pose = brain->data->ball.posToField.y;
//     //             theta = brain->data->robotBallAngleToField;
//     //             break;
//     //         }
            
//     //     }
//     // }
//     longRangeThreshold = 100;
//     turnThreshold = 3.14;
     
//     //brain->client->moveToPoseOnField(-6.7, y_pose/3, theta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance);

//     auto fd = brain->config->fieldDimensions;
//     double keeperX = -fd.length/2.0 + 0.5; // Stay 0.5m in front of goal line
//     double keeperY = std::clamp(y_pose/1.5, -fd.goalWidth/2.0, fd.goalWidth/2.0); // More aggressive Y movement but constrained
//     brain->client->moveToPoseOnField(keeperX, keeperY, theta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance);
//     return NodeStatus::SUCCESS;
// }


NodeStatus Y_Keeper::tick()
{
    double longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance;
    getInput("long_range_threshold", longRangeThreshold);
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("x_tolerance", xTolerance);
    getInput("y_tolerance", yTolerance);
    getInput("theta_tolerance", thetaTolerance);
 
    double y_pose;
    double theta;
    bool ballLocationKnown = brain->tree->getEntry<bool>("ball_location_known");
    if(ballLocationKnown){
    y_pose = brain->data->ball.posToField.y;
    theta = brain->data->robotBallAngleToField;
     }
     else{
     for (int i = 0; i < HL_MAX_NUM_PLAYERS; i++) {
            auto status = brain->data->tmStatus[i];
             int tmID = brain->config->playerId + 1;
             if (tmID = brain->config->playerId)
             continue;
             else if(status.isAlive && status.ballLocationKnown)
             {
                 brain->data->ball.posToField = status.ballPosToField;
                 y_pose = brain->data->ball.posToField.y;
                 theta = brain->data->robotBallAngleToField;
                 break;
             }
            
         }
     }
    longRangeThreshold = 100;
    turnThreshold = 3.14;
 
    brain->client->moveToPoseOnField(-5.5, y_pose/2, theta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance);
    return NodeStatus::SUCCESS;
}

NodeStatus GoToFreekickPositionDefender::onStart() {
    // brain->log->log("debug/freekick_position/onStart", rerun::TextLog(format("stage onStart")));
    _isInFinalAdjust = false;
    return NodeStatus::RUNNING;
}

NodeStatus GoToFreekickPositionDefender::onRunning() {
    auto log = [=](string msg) {
        // brain->log->setTimeNow();
        // brain->log->log("debug/GoToFreekickPosition", rerun::TextLog(msg));
    };
    log("running");

    string side;
    getInput("side", side);
    if (side !="attack" && side != "defense") return NodeStatus::SUCCESS;
    
    Pose2D targetPose;
    auto fd = brain->config->fieldDimensions;
    auto ballPos = brain->data->ball.posToField;
    auto robotPose = brain->data->robotPoseToField;

    if (side == "attack") {
        // Dynamic support distance: closer if ball already in opponent half, else stay further back.
        double targetDir = brain->data->kickDir;
        double baseDist;
        getInput("attack_dist", baseDist);
        double supportDist = (ballPos.x > 0 ? baseDist * 0.6 : max(baseDist * 0.9, baseDist * 0.8));
        targetPose.x = ballPos.x - supportDist * cos(targetDir);
        targetPose.y = ballPos.y - supportDist * sin(targetDir);
        targetPose.y += (ballPos.y >= 0 ? 0.25 : -0.25); // small lateral lane
        targetPose.x = min(targetPose.x, fd.length / 2.0 - 0.5);
        targetPose.x = max(targetPose.x, -fd.length / 2.0 + fd.goalAreaLength);
        targetPose.theta = targetDir;
    } else if (side == "defense") {
        // Position between our goal and ball:
        // If ball on opponent half (ball.x > 0): halfway (0.5); else quarter (0.25).
        double goalX = -fd.length / 2.0;
        double goalY = 0.0;
        double dx = ballPos.x - goalX;
        double dy = ballPos.y - goalY;
        double factor = (ballPos.x > 0 ? 0.5 : 0.25);
        double px = goalX + dx * factor;
        double py = goalY + dy * factor;

        // Clamp: don't sit inside goal / outside penalty width
        px = max(px, goalX + 0.8); // safety from goal line
        double lateralLimit = fd.penaltyAreaWidth / 2.0 - 0.1;
        py = cap(py, lateralLimit, -lateralLimit);

        targetPose.x = px;
        targetPose.y = py;
        targetPose.theta = atan2(ballPos.y - py, ballPos.x - px);
    }

    double dist = norm(targetPose.x - robotPose.x, targetPose.y - robotPose.y);
    double deltaDir = toPInPI(targetPose.theta - robotPose.theta);

    if (dist < 0.2 && fabs(deltaDir) < 0.1) {
        brain->client->setVelocity(0, 0, 0);
        return NodeStatus::SUCCESS;
    }

    if (!brain->get_parameter("obstacle_avoidance.enable_freekick_avoid").as_bool() || dist < 1.0 || _isInFinalAdjust) {
        _isInFinalAdjust = true; 
        auto targetPose_r = brain->data->field2robot(targetPose);

        double vx = targetPose_r.x;
        double vy = targetPose_r.y;
        double vtheta = brain->data->ball.yawToRobot * 4.0; 

        double linearFactor = 1 / (1 + exp(3 * (brain->data->ball.range * fabs(brain->data->ball.yawToRobot)) - 3)); 
        vx *= linearFactor;
        vy *= linearFactor;

        Line path = {robotPose.x, robotPose.y, targetPose.x, targetPose.y};
        if (pointMinDistToLine(Point2D({ballPos.x, ballPos.y}), path) < 0.5 && brain->data->ball.range < 1.0) {
            vx = min(0.0, vx);
            vy = vy >= 0 ? vy + 0.1: vy - 0.1;
        }

        double vxLimit, vyLimit;
        getInput("vx_limit", vxLimit);
        getInput("vy_limit", vyLimit);
        vx = cap(vx, vxLimit, -1.0);    
        vy = cap(vy, vyLimit, -vyLimit);    

        brain->client->setVelocity(vx, vy, vtheta, false, false, false);
        return NodeStatus::RUNNING;
    }

    double longRangeThreshold = 1.0;
    double turnThreshold = 0.4;
    double vxLimit = 0.6;
    double vyLimit = 0.5;
    double vthetaLimit = 1.5;
    bool avoidObstacle = true;
    brain->client->moveToPoseOnField3(
        targetPose.x, targetPose.y, targetPose.theta,
        longRangeThreshold, turnThreshold,
        vxLimit, vyLimit, vthetaLimit,
        0.2, 0.2, 0.1, avoidObstacle
    );

    return NodeStatus::RUNNING;
}

void GoToFreekickPositionDefender::onHalted() {
    // brain->log->log("debug/freekick_position/onHault", rerun::TextLog(format("stage OnHalted")));
}


NodeStatus DefenderDecide::tick()
{
 
    double chaseRangeThreshold;
    getInput("chase_threshold", chaseRangeThreshold);
    string lastDecision, position;
    getInput("decision_in", lastDecision);

    double kickDir = brain->data->kickDir;
    double dir_rb_f = brain->data->robotBallAngleToField;
    auto goalPostAngles = brain->getGoalPostAngles(0.3);
    double theta_l = goalPostAngles[0];
    double theta_r = goalPostAngles[1];
    bool angleIsGood = (dir_rb_f > -M_PI / 2 && dir_rb_f < M_PI / 2);
    double ballRange = brain->data->ball.range;
    double ballYaw = brain->data->ball.yawToRobot;
 
    string newDecision;
    auto color = 0xFFFFFFFF; // for log
    if (!brain->tree->getEntry<bool>("ball_location_known"))
    {
        newDecision = "find";
        color = 0x0000FFFF;
    }
    else if (brain->data->ball.posToField.x > 3 - static_cast<double>(lastDecision == "retreat"))
    {
        newDecision = "retreat";
        color = 0xFF00FFFF;
    }
    else if (ballRange > chaseRangeThreshold * (lastDecision == "chase" ? 0.9 : 1.0))
    {
        newDecision = "chase";
        color = 0x00FF00FF;
    }
    else if (angleIsGood )
    {
        newDecision = "kick";
        color = 0x00FFFFFF;
    }   
    else
    {
        newDecision = "adjust";
        color = 0x00FFFFFF;
    }
 
    setOutput("decision_out", newDecision);
    brain->log->logToScreen("tree/Decide",
                            format("Decision: %s ballrange: %.2f ballyaw: %.2f kickDir: %.2f rbDir: %.2f angleIsGood: %d", newDecision.c_str(), ballRange, ballYaw, kickDir, dir_rb_f, angleIsGood),
                            color);
    return NodeStatus::SUCCESS;
}

NodeStatus DefenceChase::tick()
{
    double longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance;
    getInput("long_range_threshold", longRangeThreshold);
    getInput("turn_threshold", turnThreshold);
    getInput("vx_limit", vxLimit);
    getInput("vx_limit", vxLimit);
    getInput("vy_limit", vyLimit);
    getInput("vtheta_limit", vthetaLimit);
    getInput("x_tolerance", xTolerance);
    getInput("y_tolerance", yTolerance);
    getInput("theta_tolerance", thetaTolerance);
 
    double y_pose = brain->data->ball.posToField.y;
    double theta = brain->data->robotBallAngleToField;
    longRangeThreshold = 100;
    turnThreshold = 3.14;
 
    brain->client->moveToPoseOnField(-2, y_pose, theta, longRangeThreshold, turnThreshold, vxLimit, vyLimit, vthetaLimit, xTolerance, yTolerance, thetaTolerance);
    return NodeStatus::SUCCESS;
}

namespace { // ================== Python-logic scorer (headerless) ==================

struct Vec2 { double x{0}, y{0}; };
struct Opp { Vec2 p; Vec2 v; };

struct Params {
    // defaults mirror your Python dict
    double goal_reward_gain    = 10.0; // og was 10
    // double goal_miss_tolerance = 0.6; og
    double goal_miss_tolerance = 0.6;
    double forward_bias        = 0.6;
    double back_kick_penalty   = 9.0; // og was 8
    double out_penalty         = 8.0;
    double near_post_repulse   = 0.18;
    double player_radius       = 1.20;
    double lane_clearance      = 0.25;
    double error_tol_margin    = 0.22;
    double block_sigma_lat     = 0.22;
    double block_decay_L       = 2.0;
    double k_dist              = 0.65;
    double k_lane              = 0.32;
    double jitter_deg          = 7.0;
    int    jitter_samples      = 5;
    double yaw_rate_max        = 3.0;
    double vmax                = 1.7;
    double tau_settle          = 0.12;
    double v_ball              = 8.0;
    double lambda_reach        = 0.6;
    double lambda_aim          = 0.6;
    double angle_sticky_prev   = 0.0;
    double opp_react_delay     = 0.25;
    double horizon_cap         = 2.5;
    double w_flight_time       = 1.0;
    double w_open_space        = 2.6;
    double open_space_D        = 3.2;
    double w_flank             = 3.8;
    double min_kick_clearance  = 0.30;
    double max_kick_dist       = 3.0;
    bool   use_smoothed_peak   = true;
    double smooth_sigma_deg    = 8.0;
    int    smooth_win_deg      = 15;   // unused (we only implement gaussian smoothing here)
    double peak_margin         = 0.5;
};

struct Ctx {
    // field
    // double field_len{9.0}, field_wid{6.0}, goal_half{0.6}; og
    double field_len{9.0}, field_wid{6.0}, goal_half{0.6};
    // world
    Vec2 ball{};
    Vec2 robot{};
    double robot_th{0.0};
    std::vector<Opp> opps;
    // params
    Params p{};
    // sweep
    int a_min_deg{-120}, a_max_deg{120}, a_step_deg{1};
};

static inline double wrapPI(double a) {
    while (a >  M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
}
static inline double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }

static std::optional<std::pair<double,double>> rayEndForX(double x0, double y0, double th, double target_x) {
    const double c = std::cos(th), s = std::sin(th);
    if (c <= 1e-6) return std::nullopt;
    const double t = (target_x - x0) / c;
    if (t < 0.0) return std::nullopt;
    return std::make_pair(y0 + t*s, t);
}

static std::optional<double> earliestFieldExitT(double x0, double y0, double th,
                                                double maxDist, double fldLen, double fldWid)
{
    const double c = std::cos(th), s = std::sin(th);
    std::vector<double> ts;
    const double hx = fldLen/2.0, hy = fldWid/2.0;
    if (std::abs(c) > 1e-9) {
        const double tR = (+hx - x0) / c;
        const double tL = (-hx - x0) / c;
        if (tR > 0.0) ts.push_back(tR);
        if (tL > 0.0) ts.push_back(tL);
    }
    if (std::abs(s) > 1e-9) {
        const double tT = (+hy - y0) / s;
        const double tB = (-hy - y0) / s;
        if (tT > 0.0) ts.push_back(tT);
        if (tB > 0.0) ts.push_back(tB);
    }
    if (ts.empty()) return std::nullopt;
    const double tmin = *std::min_element(ts.begin(), ts.end());
    if (tmin > maxDist) return std::nullopt;
    return tmin;
}

static inline double effectiveLane(const Params& p) {
    return p.lane_clearance + p.player_radius + p.error_tol_margin;
}

struct BlockCost { double cost{0}; std::optional<double> minClear; };

static BlockCost opponentBlockCost(const Ctx& c, double th,
                                   std::optional<double> D_cap = std::nullopt,
                                   bool collect_min_clear = false)
{
    const auto& p = c.p;
    const double ct = std::cos(th), st = std::sin(th);
    double cost = 0.0;
    double min_clear = std::numeric_limits<double>::infinity();

    for (const auto& o : c.opps) {
        const double dx = o.p.x - c.ball.x;
        const double dy = o.p.y - c.ball.y;
        const double proj = dx*ct + dy*st;
        if (proj <= 0.0) continue;

        double t_ball = p.opp_react_delay + proj / std::max(1e-6, p.v_ball);
        t_ball = clamp(t_ball, 0.0, p.horizon_cap);

        const double oxp = o.p.x + o.v.x * t_ball;
        const double oyp = o.p.y + o.v.y * t_ball;

        const double dxp = oxp - c.ball.x;
        const double dyp = oyp - c.ball.y;
        const double projp = dxp*ct + dyp*st;
        if (projp <= 0.0) continue;
        if (D_cap.has_value() && projp > D_cap.value()) continue;

        const double lat = std::abs(-st*dxp + ct*dyp);
        if (collect_min_clear) min_clear = std::min(min_clear, lat);

        const double dist_gain = 1.0 + p.k_dist * (projp / 3.0);
        const double lane      = effectiveLane(p) + p.k_lane * projp;

        const double w_lat   = std::exp(-0.5*std::pow(lat / p.block_sigma_lat, 2.0));
        const double w_lane  = std::exp(-0.5*std::pow(lat / std::max(1e-6, lane), 2.0));
        const double w_ahead = std::exp(-projp / p.block_decay_L);

        cost += dist_gain * w_lat * w_lane * w_ahead;
    }

    BlockCost r; r.cost = cost;
    if (collect_min_clear && std::isfinite(min_clear)) r.minClear = min_clear;
    return r;
}

struct GoalTerms { double reward{0}; std::optional<double> y_at_goal; std::optional<double> t_hit; double flight_pen{0}; };

static GoalTerms goalTerms(const Ctx& c, double th) {
    GoalTerms gt;
    const double goalX = c.field_len/2.0;

    auto hit = rayEndForX(c.ball.x, c.ball.y, th, goalX);
    if (!hit) return gt;

    const double y_at_goal = hit->first;
    const double t_hit     = hit->second;
    gt.y_at_goal = y_at_goal;
    gt.t_hit     = t_hit;

    const double dist_to_goal_x = std::max(0.0, goalX - c.ball.x);
    const double shoot_gate = 1.0 / (1.0 + std::exp(1.2*(dist_to_goal_x - 4.5)));

    double reward = 0.0;
    if (-c.goal_half <= y_at_goal && y_at_goal <= c.goal_half) {
        reward = shoot_gate * c.p.goal_reward_gain * (1.0 + 0.2*std::exp(-t_hit/6.0));
    } else {
        const double dist_to_gap = std::max(0.0, std::abs(y_at_goal) - c.goal_half);
        if (dist_to_gap < c.p.goal_miss_tolerance) {
            reward = shoot_gate * c.p.goal_reward_gain * (1.0 - dist_to_gap/c.p.goal_miss_tolerance) * 0.35;
        }
    }

    const double flight_pen = c.p.w_flight_time * (t_hit / std::max(1e-6, c.p.v_ball));
    gt.reward = reward;
    gt.flight_pen = flight_pen;
    return gt;
}

static double lineOutPenalty(const Ctx& c, double th, std::optional<double> t_goal) {
    auto t_exit = earliestFieldExitT(c.ball.x, c.ball.y, th, c.p.max_kick_dist, c.field_len, c.field_wid);
    if (!t_exit) return 0.0;
    if (!t_goal) {
        return c.p.out_penalty * std::exp(-t_exit.value()/3.0);
    }
    if (t_exit.value() < t_goal.value() - 1e-6) {
        return c.p.out_penalty * std::exp(-t_exit.value()/3.0);
    }
    return 0.0;
}

static double postSafetyPenalty(const Ctx& c, std::optional<double> y_at_goal) {
    if (!y_at_goal) return 0.0;
    const double d = std::min(std::abs(y_at_goal.value() - c.goal_half),
                              std::abs(y_at_goal.value() + c.goal_half));
    if (d < c.p.near_post_repulse) {
        return (c.p.near_post_repulse - d)/c.p.near_post_repulse * 2.0;
    }
    return 0.0;
}

static double reachabilityCost(const Ctx& c, double th) {
    const double r_approach = 0.65;
    const double px = c.ball.x - r_approach*std::cos(th);
    const double py = c.ball.y - r_approach*std::sin(th);
    const double dx = px - c.robot.x;
    const double dy = py - c.robot.y;

    const double theta_to = std::atan2(dy, dx);
    const double t_turn  = std::abs(wrapPI(theta_to - c.robot_th)) / std::max(1e-6, c.p.yaw_rate_max);
    const double t_drive = std::hypot(dx, dy) / std::max(1e-6, c.p.vmax);
    const double t_align = std::abs(wrapPI(th - theta_to)) / std::max(1e-6, c.p.yaw_rate_max);
    const double t_reach = t_turn + t_drive + 0.5*t_align;
    const double t_aim   = std::abs(wrapPI(th - c.robot_th)) / std::max(1e-6, c.p.yaw_rate_max) + c.p.tau_settle;

    return c.p.lambda_reach*t_reach + c.p.lambda_aim*t_aim;
}

static double openSpaceReward(const Ctx& c, double th) {
    auto bc = opponentBlockCost(c, th, c.p.open_space_D, true);
    double reward = c.p.w_open_space * (1.0 / (1.0 + bc.cost));
    if (bc.minClear.has_value()) {
        reward += 0.5 * c.p.w_open_space * std::tanh((bc.minClear.value() - effectiveLane(c.p)) / 0.2);
    }
    return reward;
}

static double flankReward(const Ctx& c, double th) {
    const double L = 2.8;
    const double Yoff = 1.2;

    auto lane_cost_to = [&](double tx, double ty){
        const double t = std::atan2(ty - c.ball.y, tx - c.ball.x);
        return opponentBlockCost(c, t, L, false).cost;
    };
    const double cL = lane_cost_to(c.ball.x + L, c.ball.y + Yoff);
    const double cR = lane_cost_to(c.ball.x + L, c.ball.y - Yoff);
    const double sideiness = std::pow(std::sin(std::abs(th)), 2.0);
    const double base = 1.0 / (1.0 + std::min(cL, cR));
    return c.p.w_flank * sideiness * base;
}

static double rawScore(const Ctx& c, double th) {
    const double fwd = std::cos(th);
    const double forward = c.p.forward_bias * std::max(0.0, fwd);
    const double back    = c.p.back_kick_penalty * std::max(0.0, -fwd);

    const auto gt   = goalTerms(c, th);
    const double blk = opponentBlockCost(c, th).cost;
    const double outp= lineOutPenalty(c, th, gt.t_hit);
    const double post= postSafetyPenalty(c, gt.y_at_goal);
    const double reach = reachabilityCost(c, th);
    const double sticky= 0.6 * std::abs(wrapPI(th - c.p.angle_sticky_prev));
    const double openR = openSpaceReward(c, th);
    const double flankR= flankReward(c, th);

    double tight_pen = 0.0;
    // We’d need min_clear from openSpace; acceptable approximation: re-evaluate w/ collect true and cap=D
    auto bc_for_tight = opponentBlockCost(c, th, std::nullopt, true);
    if (bc_for_tight.minClear.has_value() && bc_for_tight.minClear.value() < c.p.min_kick_clearance) {
        tight_pen = 6.0 * (c.p.min_kick_clearance - bc_for_tight.minClear.value()) / c.p.min_kick_clearance;
    }

    return forward + gt.reward + openR + flankR
         - (blk + outp + post + back + reach + sticky + gt.flight_pen + tight_pen);
}

static double robustScore(const Ctx& c, double th) {
    const double sd = c.p.jitter_deg * M_PI / 180.0;
    const int n = std::max(1, c.p.jitter_samples);
    // symmetric integer offsets: i - (n/2)
    std::vector<int> ks(n);
    for (int i=0;i<n;i++) ks[i] = i - (n/2);

    std::vector<double> wts(n);
    for (int i=0;i<n;i++) wts[i] = std::exp(-0.5 * double(ks[i]*ks[i]));
    const double sw = std::accumulate(wts.begin(), wts.end(), 0.0);
    for (auto& w : wts) w /= std::max(1e-9, sw);

    double S = 0.0;
    for (int i=0;i<n;i++) S += wts[i] * rawScore(c, th + ks[i]*sd);
    return S;
}

static std::vector<double> gaussianSmooth(const std::vector<double>& x, double sigma_deg, int step_deg) {
    if (sigma_deg <= 0.0) return x;
    const double sigma_samp = std::max(1e-6, sigma_deg / std::max(1, step_deg));
    const int half = std::max(1, int(4.0 * sigma_samp));
    const int ksz = 2*half + 1;

    std::vector<double> w(ksz);
    for (int i=0;i<ksz;++i) {
        const double t = (i - half) / sigma_samp;
        w[i] = std::exp(-0.5 * t*t);
    }
    const double sw = std::accumulate(w.begin(), w.end(), 0.0);
    for (auto& a : w) a /= std::max(1e-9, sw);

    int pad = half;
    std::vector<double> xp; xp.reserve(x.size()+2*pad);
    xp.insert(xp.end(), pad, x.front());
    xp.insert(xp.end(), x.begin(), x.end());
    xp.insert(xp.end(), pad, x.back());

    std::vector<double> out; out.reserve(x.size());
    for (size_t i=0;i<x.size();++i) {
        double s=0.0;
        for (int j=0;j<ksz;++j) s += xp[i+j] * w[j];
        out.push_back(s);
    }
    return out;
}

static size_t chooseBestIndex(const std::vector<int>& angles_deg,
                              const std::vector<double>& scores,
                              const Params& p,
                              int step_deg)
{
    if (!p.use_smoothed_peak) {
        return size_t(std::distance(scores.begin(), std::max_element(scores.begin(), scores.end())));
    }

    auto scores_s = gaussianSmooth(scores, p.smooth_sigma_deg, step_deg);
    const double smax = *std::max_element(scores_s.begin(), scores_s.end());
    const double margin = p.peak_margin;

    std::vector<size_t> cand;
    for (size_t i=0;i<scores_s.size();++i)
        if (scores_s[i] >= smax - margin) cand.push_back(i);

    if (cand.size() == 1) return cand[0];

    // Among candidates, choose the one with max RAW score (like your Python)
    size_t idx_raw = cand.front();
    double best = scores[idx_raw];
    for (size_t k : cand) if (scores[k] > best) { best = scores[k]; idx_raw = k; }
    return idx_raw;
}

static double computeKickDirection(const Ctx& baseCtx) {
    Ctx c = baseCtx; // copy (we may write angle_sticky_prev inside)
    c.p.angle_sticky_prev = baseCtx.p.angle_sticky_prev;

    // angle grid
    std::vector<int> a_deg;
    for (int a=c.a_min_deg; a<=c.a_max_deg; a+=std::max(1, c.a_step_deg)) a_deg.push_back(a);
    const size_t N = a_deg.size();

    std::vector<double> scores; scores.reserve(N);
    for (size_t i=0;i<N;++i) {
        const double th = a_deg[i] * M_PI / 180.0;
        scores.push_back(robustScore(c, th));
    }

    const size_t best_idx = chooseBestIndex(a_deg, scores, c.p, std::max(1, c.a_step_deg));
    return a_deg[best_idx] * M_PI / 180.0; // radians
}

} // namespace (scorer)

// ================== BehaviorTree node ==================

NodeStatus CalcKickDirPF::tick() {
    // light throttle so direction updates frequently
    if (lastExecutionTime.nanoseconds() != 0) {
        if (brain->msecsSince(lastExecutionTime) < 100.0) {
            return NodeStatus::SUCCESS;
        }
    }
    lastExecutionTime = brain->get_clock()->now();

    // ------- 1) Build scorer context from brain -------
    Ctx ctx;
    ctx.field_len = brain->config->fieldDimensions.length;
    ctx.field_wid = brain->config->fieldDimensions.width;

    // optional: goal half-height from port
    double goal_half_port;
    if (getInput("goal_half", goal_half_port)) ctx.goal_half = goal_half_port;

    // ball
    const auto& b = brain->data->ball.posToField;
    ctx.ball = { b.x, b.y };

    // robot (self)
    const int selfIdx = std::max(0, brain->config->playerId - 1);
    const auto& selfPose =  brain->data->robotPoseToField;
    ctx.robot    = { selfPose.x, selfPose.y };
    ctx.robot_th =  selfPose.theta;

    // opponents / obstacles (if your list includes self/teammates, consider filtering there)
    ctx.opps.clear();
    const auto obs = brain->data->getRobots(); // must expose .posToField.{x,y}
    ctx.opps.reserve(obs.size());
    for (const auto& op : obs) {
        Opp o; o.p = { op.posToField.x, op.posToField.y }; o.v = { 0.0, 0.0 }; // fill velocities if you have them

        ctx.opps.push_back(o);
    }

    // angle sweep (same as Python)
    ctx.a_min_deg  = -120;
    ctx.a_max_deg  = +120;
    ctx.a_step_deg = 1;

    // ------- 2) Params: defaults + BT ports -------
    // (Only reading ports we exposed in brain.h; the legacy PF ports are ignored.)
    double d;
    int    n;
    bool   bflag;

    if (getInput("ball_speed", d))             ctx.p.v_ball = d;
    if (getInput("open_space_D", d))           ctx.p.open_space_D = d;
    if (getInput("w_open_space", d))           ctx.p.w_open_space = d;
    if (getInput("w_flank", d))                ctx.p.w_flank = d;
    if (getInput("goal_reward_gain", d))       ctx.p.goal_reward_gain = d;
    if (getInput("out_penalty", d))            ctx.p.out_penalty = d;
    if (getInput("near_post_repulse", d))      ctx.p.near_post_repulse = d;
    if (getInput("min_kick_clearance", d))     ctx.p.min_kick_clearance = d;
    if (getInput("jitter_deg", d))             ctx.p.jitter_deg = d;
    if (getInput("jitter_samples", n))         ctx.p.jitter_samples = std::max(1, n);
    if (getInput("use_smoothed_peak", bflag))  ctx.p.use_smoothed_peak = bflag;
    if (getInput("smooth_sigma_deg", d))       ctx.p.smooth_sigma_deg = d;
    if (getInput("peak_margin", d))            ctx.p.peak_margin = d;

    ctx.p.angle_sticky_prev = prev_angle_rad_;

    // ------- 3) Compute best direction -------
    const double dir = computeKickDirection(ctx);
    prev_angle_rad_ = dir; // update stickiness memory

    // ------- 4) Publish -------
    brain->data->kickDir = dir;
    setOutput("kick_dir", dir);

    return NodeStatus::SUCCESS;
} 