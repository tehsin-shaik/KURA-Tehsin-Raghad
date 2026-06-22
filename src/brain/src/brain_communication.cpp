#include <cmath>
#include "brain.h"
#include "brain_communication.h"

// Compile-time check: rule \UDPPacketSize = 512 bytes.
// If you ever extend TeamCommunicationMsg, this assert prevents the GC from
// silently zeroing your score (see HSL-Rules-main/rules/game.tex:613).
static_assert(sizeof(TeamCommunicationMsg) <= 512,
              "TeamCommunicationMsg must be <= 512 bytes (HSL rule \\UDPPacketSize)");

BrainCommunication::BrainCommunication(Brain *argBrain) : brain(argBrain)
{
}

BrainCommunication::~BrainCommunication()
{
    clearupGameControllerUnicast();
    clearupDiscoveryBroadcast();
    clearupDiscoveryReceiver();
    clearupCommunicationUnicast();
    clearupCommunicationReceiver();
}


void BrainCommunication::initCommunication()
{
    initGameControllerUnicast();
    if (brain->config->enableCom)
    {
        cout << RED_CODE << "Team communication enabled (UDP broadcast on port 10000+teamId)." << RESET_CODE << endl;
        // Per HSL rule robot_players.tex:331-332, team comms MUST be UDP broadcast
        // on port 10000+teamNumber. Unicast between robots is prohibited.
        _discovery_udp_port = 20000 + brain->config->teamId; // unused (kept for log compat)
        _unicast_udp_port   = 10000 + brain->config->teamId;

        // Discovery broadcast on :20025 is no longer needed because we no
        // longer build a teammate IP table — broadcast doesn't need one.
        // The discovery init/clearup methods are still invoked in case any
        // diagnostic display reads from them; they just don't transmit.
        initCommunicationUnicast();
        initCommunicationReceiver();
    }
    else
    {
        cout << RED_CODE << "Team communication disabled." << RESET_CODE << endl;
    }
}


void BrainCommunication::initGameControllerUnicast()
{
    try
    {
        _gc_send_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_gc_send_socket < 0)
        {
            cout << RED_CODE << format("gc socket failed: %s", strerror(errno))
                 << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }
        string gamecontrol_ip = brain->get_parameter("game_control_ip").as_string();
        cout << GREEN_CODE << format("GameControl IP: %s", gamecontrol_ip.c_str())
             << RESET_CODE << endl;
        _gcsaddr.sin_family = AF_INET;
        _gcsaddr.sin_addr.s_addr = inet_addr(gamecontrol_ip.c_str());
        _gcsaddr.sin_port = htons(GAMECONTROLLER_RETURN_PORT);

        _unicast_gamecontrol_flag = true;
        _gamecontrol_unicast_thread = std::thread([this](){ this->unicastToGameController(); });
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

void BrainCommunication::clearupGameControllerUnicast()
{
    _unicast_gamecontrol_flag = false;
    if (_gc_send_socket >= 0)
    {
        close(_gc_send_socket);
        _gc_send_socket = -1;
        cout << RED_CODE << format("GameControl send socket has been closed.")
             << RESET_CODE << endl;
    }
    if (_gamecontrol_unicast_thread.joinable())
    {
        _gamecontrol_unicast_thread.join();
    }
}

// Discovery init/clearup are now no-ops (kept as empty methods so destructor
// and external references continue to compile). Once we are confident no other
// code reads _discovery_msg_id / discoveryMsgId, these can be deleted entirely.
void BrainCommunication::initDiscoveryBroadcast() {}
void BrainCommunication::clearupDiscoveryBroadcast()
{
    _broadcast_discovery_flag = false;
    if (_discovery_send_socket >= 0)
    {
        close(_discovery_send_socket);
        _discovery_send_socket = -1;
    }
    if (_discovery_broadcast_thread.joinable())
    {
        _discovery_broadcast_thread.join();
    }
}
void BrainCommunication::initDiscoveryReceiver() {}
void BrainCommunication::clearupDiscoveryReceiver()
{
    _receive_discovery_flag = false;
    if (_discovery_recv_socket >= 0)
    {
        close(_discovery_recv_socket);
        _discovery_recv_socket = -1;
    }
    if (_discovery_recv_thread.joinable())
    {
        _discovery_recv_thread.join();
    }
}
void BrainCommunication::broadcastDiscovery() {}
void BrainCommunication::spinDiscoveryReceiver() {}
void BrainCommunication::cleanupExpiredTeammates() {}

void BrainCommunication::unicastToGameController()
{
    auto safe_f = [](double v) -> float {
        return std::isfinite(v) ? static_cast<float>(v) : 0.0f;
    };

    while (_unicast_gamecontrol_flag)
    {
        // v4 status packet (32 bytes). The constructor pre-fills header / version,
        // and pre-fills fallen=255 (intentionally invalid) — we MUST overwrite
        // every field every iteration, otherwise the GC validator drops the packet.
        gc_return_data.teamNum   = brain->config->teamId;
        gc_return_data.playerNum = brain->config->playerId;
        gc_return_data.fallen    = brain->data->isFallen ? 1 : 0;

        // Pose: BrainData stores metres + radians; GC v4 wants millimetres + radians.
        // NaN guard: localization may be NaN before convergence, and the GC validator
        // (game_controller_msgs/src/status_message.rs) rejects any NaN.
        gc_return_data.pose[0] = safe_f(brain->data->robotPoseToField.x * 1000.0);
        gc_return_data.pose[1] = safe_f(brain->data->robotPoseToField.y * 1000.0);
        gc_return_data.pose[2] = safe_f(brain->data->robotPoseToField.theta);

        if (brain->data->ballDetected) {
            const double age = (brain->get_clock()->now() - brain->data->ball.timePoint).seconds();
            gc_return_data.ballAge = safe_f(age);
            gc_return_data.ball[0] = safe_f(brain->data->ball.posToRobot.x * 1000.0);
            gc_return_data.ball[1] = safe_f(brain->data->ball.posToRobot.y * 1000.0);
        } else {
            gc_return_data.ballAge = -1.0f;
            gc_return_data.ball[0] = 0.0f;
            gc_return_data.ball[1] = 0.0f;
        }

        int ret = sendto(_gc_send_socket, &gc_return_data, sizeof(gc_return_data),
                         0, (sockaddr *)&_gcsaddr, sizeof(_gcsaddr));
        if (ret < 0)
        {
            cout << RED_CODE << format("gc sendto failed: %s", strerror(errno))
                 << RESET_CODE << endl;
        }
        this_thread::sleep_for(chrono::milliseconds(BROADCAST_GAME_CONTROL_INTERVAL_MS));
    }
}

void BrainCommunication::initCommunicationUnicast()
{
    try
    {
        _unicast_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_unicast_socket < 0) {
            cout << RED_CODE << format("team-msg socket failed: %s", strerror(errno))
                 << RESET_CODE << endl;
            throw std::runtime_error("Failed to create team-msg socket");
        }

        // SO_BROADCAST is required because the rule mandates UDP broadcast (not unicast).
        int broadcast = 1;
        if (setsockopt(_unicast_socket, SOL_SOCKET, SO_BROADCAST,
                       &broadcast, sizeof(broadcast)) < 0)
        {
            cout << RED_CODE << format("SO_BROADCAST failed: %s", strerror(errno))
                 << RESET_CODE << endl;
            throw std::runtime_error("Failed to enable SO_BROADCAST");
        }

        _unicast_saddr.sin_family = AF_INET;
        _unicast_saddr.sin_addr.s_addr = INADDR_BROADCAST; // 255.255.255.255
        _unicast_saddr.sin_port = htons(_unicast_udp_port); // 10000 + teamId

        _last_team_msg_time = brain->get_clock()->now();
        _unicast_communication_flag = true;
        _unicast_thread = std::thread([this](){ this->unicastCommunication(); });
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        brain->log->log("error/communication", rerun::TextLog(format("Failed to initialize team-msg broadcast: %s", e.what())));
    }
}

void BrainCommunication::unicastCommunication()
{
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/sendMsg", rerun::TextLog(msg));
    };
    while (_unicast_communication_flag)
    {
        // Per HSL rule robot_players.tex:346, only Ready/Set/Playing count toward
        // the budget. Sending in INITIAL/FINISHED/TIMEOUT wastes nothing budget-wise
        // (free messages), but we still skip to avoid noise on the GC's UI.
        const string &state = brain->data->rawGameState;
        const bool sendable = (state == "READY" || state == "SET" || state == "PLAY");
        if (!sendable) {
            this_thread::sleep_for(chrono::milliseconds(UNICAST_INTERVAL_MS));
            continue;
        }

        // Rate-limit: 0.5 Hz. Rule cap is 12000/game (\UDPPacketLimit), so even
        // at 1 Hz over 20 min you'd send 1200 messages = 10% of budget. 0.5 Hz
        // leaves headroom for added time without recomputation.
        const auto now = brain->get_clock()->now();
        if ((now - _last_team_msg_time).seconds() * 1000.0 < TEAM_MSG_MIN_INTERVAL_MS) {
            this_thread::sleep_for(chrono::milliseconds(UNICAST_INTERVAL_MS));
            continue;
        }
        _last_team_msg_time = now;

        TeamCommunicationMsg msg;
        msg.validation = VALIDATION_COMMUNICATION;
        msg.communicationId = _team_communication_msg_id++;
        msg.teamId = brain->config->teamId;
        msg.playerId = brain->config->playerId;
        msg.playerRole = brain->tree->getEntry<string>("player_role") == "striker" ? 1 : 2;
        msg.isAlive = brain->data->tmImAlive;
        msg.isLead = brain->data->tmImLead;
        msg.ballDetected = brain->data->ballDetected;
        msg.ballLocationKnown = brain->tree->getEntry<bool>("ball_location_known");
        msg.ballConfidence = brain->data->ball.confidence;
        msg.ballRange = brain->data->ball.range;
        msg.cost = brain->data->tmMyCost;
        msg.ballPosToField = brain->data->ball.posToField;
        msg.robotPoseToField = brain->data->robotPoseToField;
        msg.kickDir = brain->data->kickDir;
        msg.thetaRb = brain->data->robotBallAngleToField;
        msg.passActive = brain->tree->getEntry<bool>("pass_active");
        msg.passReceiverId = brain->tree->getEntry<int>("pass_receiver_id");
        msg.passTargetToField = Point{
            brain->tree->getEntry<double>("pass_target_x"),
            brain->tree->getEntry<double>("pass_target_y"),
            0.0
        };
        msg.passSpeedLimit = brain->tree->getEntry<double>("pass_speed_limit");
        msg.passSeq = brain->tree->getEntry<int>("pass_seq");
        msg.cmdId = brain->data->tmMyCmdId;
        msg.cmd = brain->data->tmMyCmd;
        log(format("ImAlive: %d, ImLead: %d, myCost: %.1f, pass: %d->%d (%.1f, %.1f), myCmdId: %d, myCmd: %d (broadcast)",
                   msg.isAlive, msg.isLead, msg.cost, msg.passActive, msg.passReceiverId,
                   msg.passTargetToField.x, msg.passTargetToField.y, msg.cmdId, msg.cmd));

        brain->data->tmIP = "BROADCAST";
        brain->data->sendId = msg.communicationId;
        brain->data->sendTime = brain->get_clock()->now();

        int ret = sendto(_unicast_socket, &msg, sizeof(msg), 0,
                         (sockaddr *)&_unicast_saddr, sizeof(_unicast_saddr));
        if (ret < 0) {
            cout << RED_CODE << format("team-msg broadcast sendto failed: %s", strerror(errno))
                 << RESET_CODE << endl;
        }
    }
}

void BrainCommunication::clearupCommunicationUnicast()
{
    _unicast_communication_flag = false;
    if (_unicast_socket >= 0) {
        close(_unicast_socket);
        _unicast_socket = -1;
        cout << RED_CODE << format("Team-msg broadcast socket has been closed.")
             << RESET_CODE << endl;
    }
    if (_unicast_thread.joinable()) {
        _unicast_thread.join();
    }
}

void BrainCommunication::initCommunicationReceiver()
{
    try
    {
        _communication_recv_socket = socket(AF_INET, SOCK_DGRAM, 0);
        if (_communication_recv_socket < 0) {
            cout << RED_CODE << format("socket failed: %s", strerror(errno))
                 << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        int reuse = 1;
        if (setsockopt(_communication_recv_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
        {
            cout << RED_CODE << format("Failed to set SO_REUSEADDR: %s", strerror(errno))
                 << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        // _unicast_udp_port is now 10000 + teamId (the rule-mandated team-msg port).
        addr.sin_port = htons(_unicast_udp_port);

        if (bind(_communication_recv_socket, (sockaddr *)&addr, sizeof(addr)) < 0) {
            cout << RED_CODE << format("bind failed: %s (port=%d)", strerror(errno), _unicast_udp_port)
                 << RESET_CODE << endl;
            throw std::runtime_error(strerror(errno));
        }

        cout << GREEN_CODE << format("Listening for team broadcasts on port %d", _unicast_udp_port)
             << RESET_CODE << endl;

        _receive_communication_flag = true;
        _communication_recv_thread = std::thread([this](){ this->spinCommunicationReceiver(); });
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        brain->log->log("error/communication", rerun::TextLog(format("Failed to initialize communication receiver: %s", e.what())));
    }
}

void BrainCommunication::spinCommunicationReceiver()
{
    auto log = [=](string msg) {
        brain->log->setTimeNow();
        brain->log->log("debug/receiveMsg", rerun::TextLog(msg));
    };
    sockaddr_in addr{};
    socklen_t addr_len = sizeof(addr);

    TeamCommunicationMsg msg;

    while (_receive_communication_flag) {

        ssize_t len = recvfrom(_communication_recv_socket, &msg, sizeof(msg), 0, (sockaddr *)&addr, &addr_len);

        if (len < 0) {
            cout << RED_CODE << format("receiving UDP message failed: %s", strerror(errno))
                 << RESET_CODE << endl;
            continue;
        }

        if (len != sizeof(TeamCommunicationMsg)) {
            cout << YELLOW_CODE << format("received TeamCommunicationMsg packet with wrong size: %ld, expected: %ld", len, sizeof(TeamCommunicationMsg))
                 << RESET_CODE << endl;
            continue;
        }

        if (msg.validation != VALIDATION_COMMUNICATION) {
            cout << RED_CODE << format("received TeamCommunicationMsg packet with invalid validation: %d", msg.validation)
                 << RESET_CODE << endl;
            continue;
        }

        if (msg.teamId != brain->config->teamId) {
            cout << YELLOW_CODE << format("Received message from team %d, expected team %d", msg.teamId, brain->config->teamId)
                 << RESET_CODE << endl;
            continue;
        }

        // Self-broadcast filter: kernel delivers our own broadcasts back to us.
        if (msg.playerId == brain->config->playerId) {
            brain->data->sendId = msg.communicationId;
            brain->data->sendTime = brain->get_clock()->now();
            continue;
        }

        auto tmIdx = msg.playerId - 1;

        if (tmIdx < 0 || tmIdx >= HL_MAX_NUM_PLAYERS) {
            cout << YELLOW_CODE << format("Received message with invalid playerId: %d", msg.playerId) << RESET_CODE << endl;
            continue;
        }

        if (brain->data->penalty[tmIdx] == PENALTY_SUBSTITUTE) {
            cout << YELLOW_CODE << format("Communication playerId %d is substitute", msg.playerId) << RESET_CODE << endl;
            continue;
        }

        log(format("TMID: %.d, alive: %d, lead: %d, cost: %.1f, CmdId: %d, Cmd: %d", msg.playerId, msg.isAlive, msg.isLead, msg.cost, msg.cmdId, msg.cmd));

        TMStatus &tmStatus = brain->data->tmStatus[tmIdx];
        tmStatus.role = msg.playerRole == 1 ? "striker" : "goal_keeper";
        tmStatus.isAlive = msg.isAlive;
        tmStatus.ballDetected = msg.ballDetected;
        tmStatus.ballLocationKnown = msg.ballLocationKnown;
        tmStatus.ballConfidence = msg.ballConfidence;
        tmStatus.ballRange = msg.ballRange;
        tmStatus.cost = msg.cost;
        tmStatus.isLead = msg.isLead;
        tmStatus.ballPosToField = msg.ballPosToField;
        tmStatus.robotPoseToField = msg.robotPoseToField;
        tmStatus.kickDir = msg.kickDir;
        tmStatus.thetaRb = msg.thetaRb;
        tmStatus.passActive = msg.passActive;
        tmStatus.passReceiverId = msg.passReceiverId;
        tmStatus.passTargetToField = msg.passTargetToField;
        tmStatus.passSpeedLimit = msg.passSpeedLimit;
        tmStatus.passSeq = msg.passSeq;
        tmStatus.timeLastCom = brain->get_clock()->now();
        tmStatus.cmd = msg.cmd;
        tmStatus.cmdId = msg.cmdId;

        if (msg.cmdId > brain->data->tmCmdId) {
            brain->data->tmCmdId = msg.cmdId;
            brain->data->tmReceivedCmd = msg.cmd;
            brain->data->tmLastCmdChangeTime = brain->get_clock()->now();
            log(format("Received new command from teammate %d: %d", msg.playerId, msg.cmd));
        }
    }
}

void BrainCommunication::clearupCommunicationReceiver()
{
    _receive_communication_flag = false;
    if (_communication_recv_socket >= 0) {
        close(_communication_recv_socket);
        _communication_recv_socket = -1;
        cout << RED_CODE << format("Communication receive socket has been closed.")
             << RESET_CODE << endl;
    }
    if (_communication_recv_thread.joinable()) {
        _communication_recv_thread.join();
    }
}
