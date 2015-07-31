#include "myriolinkbot/myriolinkbot.hpp"

#include <iostream>
#include <chrono>

std::string myRio::timeout_exception::mMsg;

class JointStateHeartbeat
{
public:
    JointStateHeartbeat(myRio::Linkbot* linkbot) : mLinkbot(linkbot),
        mHeartbeatEnable(true)
    { 
        auto heartbeat = std::thread(
        [this] 
        {
            std::unique_lock<std::mutex> heartbeat_lock(mHeartbeatLock);
            bool waitrc = false;
            while(!waitrc) {
                waitrc = mHeartbeatCond.wait_for(
                    heartbeat_lock,
                    std::chrono::milliseconds(1000),
                    [this] 
                    { 
                        if(!mHeartbeatEnable) {
                            return true;
                        }
                        int timestamp;
                        barobo::JointState::Type jointStates[3];
                        mLinkbot->getJointStates(timestamp, 
                                       jointStates[0],
                                       jointStates[1],
                                       jointStates[2]);
                        mLinkbot->_setJointStates(std::vector<barobo::JointState::Type>(jointStates,
                            jointStates+3));
                        return !mHeartbeatEnable; 
                    }
                );
            }
        });
        std::swap(mThread, heartbeat);
    }

    ~JointStateHeartbeat() 
    {
        mHeartbeatEnable = false;
        mHeartbeatCond.notify_all();
        if(mThread.joinable()) {
            mThread.join();
        }
    }

private:
    myRio::Linkbot* mLinkbot;
    std::thread mThread;
    std::mutex mHeartbeatLock;
    std::condition_variable mHeartbeatCond;
    bool mHeartbeatEnable;
};

enum class RobotMoveStatus { STOPPED, MOVING, ERROR };

RobotMoveStatus isRobotMoving(int mask, 
                              std::vector<barobo::JointState::Type> states)
{
    auto status = RobotMoveStatus::STOPPED;
    int jointmask = 1;
    for (auto& s : states) {
        if(!(jointmask & mask)) {
            jointmask <<=1 ;
            continue;
        }
        switch (s){
            case barobo::JointState::COAST:
            case barobo::JointState::HOLD:
                break;
            case barobo::JointState::MOVING:
                status = RobotMoveStatus::MOVING;
                return status;
            case barobo::JointState::FAILURE:
                return RobotMoveStatus::ERROR;
            default:
                break;
        }
        jointmask <<=1 ;
    }
}

void _jointEventCb(int jointNo, barobo::JointState::Type event, int timestamp,
    void* userdata)
{
    myRio::Linkbot *l = static_cast<myRio::Linkbot*>(userdata);
    l->jointEventCb(jointNo, event);
}

myRio::Linkbot::Linkbot(const std::string& serialId) : barobo::Linkbot(serialId)
{
    for(int i = 0; i < 3; i++) {
        mJointStates[i] = barobo::JointState::COAST;
    }
    setJointEventCallback(_jointEventCb, this);
    getFormFactor(mFormFactor);
    switch(mFormFactor) {
        case barobo::FormFactor::I:
            mMotorMask = 0x05;
            break;
        case barobo::FormFactor::L:
            mMotorMask = 0x03;
            break;
        case barobo::FormFactor::T:
            mMotorMask = 0x07;
            break;
    }
}

myRio::Linkbot::~Linkbot()
{
}

void myRio::Linkbot::jointEventCb(int jointNo, barobo::JointState::Type event)
{
    std::unique_lock<std::mutex> lock(mJointStateLock);
    mJointStates[jointNo] = event;
    mJointStateCond.notify_all();
}

void myRio::Linkbot::drive(int mask, double j1, double j2, double j3)
{
    barobo::Linkbot::drive(mask, j1, j2, j3);
    setJointStatesMoving(mask);
}

void myRio::Linkbot::driveTo(int mask, double j1, double j2, double j3)
{
    barobo::Linkbot::driveTo(mask, j1, j2, j3);
    setJointStatesMoving(mask);
}

void myRio::Linkbot::move(int mask, double j1, double j2, double j3)
{
    barobo::Linkbot::move(mask, j1, j2, j3);
    setJointStatesMoving(mask);
}

void myRio::Linkbot::moveContinuous(int mask, double j1, double j2, double j3)
{
    barobo::Linkbot::moveContinuous(mask, j1, j2, j3);
    setJointStatesMoving(mask);
}

void myRio::Linkbot::moveTo(int mask, double j1, double j2, double j3)
{
    barobo::Linkbot::moveTo(mask, j1, j2, j3);
    setJointStatesMoving(mask);
}

void myRio::Linkbot::moveWait(int mask, double timeout)
{
    JointStateHeartbeat heartbeat(this);

    std::unique_lock<std::mutex> lock(mJointStateLock);
    int timestamp;
    getJointStates(timestamp, mJointStates[0], mJointStates[1],
        mJointStates[2]);
    auto state = isRobotMoving(
                    mask&mMotorMask,
                    std::vector<barobo::JointState::Type>(
                        mJointStates, mJointStates+3
                    )
                 );
    switch(state) {
        case RobotMoveStatus::STOPPED:
            return;
        case RobotMoveStatus::ERROR:
            throw motor_exception();
        default:
            break;
    }
    bool timeout_flag = false;
    auto starttime = std::chrono::system_clock::now();
    if(timeout != 0.0) {
    while(
        mJointStateCond.wait_for(
            lock,
            std::chrono::milliseconds(int(timeout*1000)),
            [this, &timeout_flag, mask, starttime, timeout] 
            {
                if(isRobotMoving(mask&mMotorMask, 
                                 std::vector<barobo::JointState::Type>(mJointStates,
                                     mJointStates+3 )
                                ) == RobotMoveStatus::MOVING
                )
                {
                    std::chrono::duration<double> elapsed_seconds = 
                        std::chrono::system_clock::now() - starttime;
                    if(elapsed_seconds.count() > timeout) {
                        throw timeout_exception("Motor has not finished moving.");
                    } else {
                        return false;
                    }
                } else {
                    return true;
                }
            }
        )
    );
    } else {
        mJointStateCond.wait(
            lock,
            [this, &timeout_flag, mask, starttime, timeout] 
            {
                if(isRobotMoving(mask&mMotorMask, 
                                 std::vector<barobo::JointState::Type>(mJointStates,
                                     mJointStates+3 )
                                ) == RobotMoveStatus::MOVING
                )
                {
                    return false;
                } else {
                    return true;
                }
            }
        );
    }
}

bool myRio::Linkbot::isMoving(int mask)
{
    static std::chrono::time_point<std::chrono::system_clock> lastChecked;
    auto now = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedTime = now-lastChecked;
    if(elapsedTime.count() > 2.0) {
        refreshJointStates();
        lastChecked = now;
    }
    bool moving = false;
    for(int i = 0; i < 3; i++) {
        if(!((1<<i)&mask&mMotorMask)) continue;
        if(mJointStates[i] == barobo::JointState::MOVING) {
            moving = true;
            break;
        }
    }
    return moving;
}

void myRio::Linkbot::_setJointStates(std::vector<barobo::JointState::Type> states)
{
    std::unique_lock<std::mutex> lock(mJointStateLock);
    for(int i = 0; i < 3; i++) {
        mJointStates[i] = states[i];
    }
    mJointStateCond.notify_all();
}

void myRio::Linkbot::refreshJointStates()
{
    std::unique_lock<std::mutex> lock(mJointStateLock);
    int timestamp;
    getJointStates(timestamp, mJointStates[0], mJointStates[1],
        mJointStates[2]);
    mJointStateCond.notify_all();
}

void myRio::Linkbot::setJointStatesMoving(int mask)
{
    for(int i = 0; i < 3; i++) {
        if( (1<<i) & mask) {
            mJointStates[i] = barobo::JointState::MOVING;
        }
    }
}

