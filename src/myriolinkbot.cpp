#include "myriolinkbot/myriolinkbot.hpp"

#include <chrono>

class JointStateHeartbeat
{
public:
    JointStateHeartbeat(myRio::Linkbot* linkbot) : mLinkbot(linkbot) 
    { 
        auto heartbeat = std::thread(
        [this] 
        {
            std::unique_lock<std::mutex> heartbeat_lock(mHeartbeatLock);
            while(
                mHeartbeatCond.wait_for(
                    heartbeat_lock,
                    std::chrono::milliseconds(1000),
                    [this] 
                    { 
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
                )
            );
        });
        std::swap(mThread, heartbeat);
    }

    ~JointStateHeartbeat() 
    {
        std::unique_lock<std::mutex> heartbeat_lock(mHeartbeatLock);
        mHeartbeatEnable = false;
        mHeartbeatCond.notify_all();
        mThread.join();
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
            case barobo::JointState::STOP:
            case barobo::JointState::HOLD:
                break;
            case barobo::JointState::MOVING:
                status = RobotMoveStatus::MOVING;
                return status;
            case barobo::JointState::ERROR:
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
        mJointStates[i] = barobo::JointState::STOP;
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

void myRio::Linkbot::moveWait(int mask, double timeout)
{
    JointStateHeartbeat heartbeat(this);

    std::unique_lock<std::mutex> lock(mJointStateLock);
    int timestamp;
    getJointStates(timestamp, mJointStates[0], mJointStates[1],
        mJointStates[2]);
    auto state = isRobotMoving(
                    mask,
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
    while(
        mJointStateCond.wait_for(
            lock,
            std::chrono::milliseconds(int(timeout*1000)),
            [this, &timeout_flag, mask, starttime, timeout] 
            {
                if(isRobotMoving(mask, 
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
}

void myRio::Linkbot::_setJointStates(std::vector<barobo::JointState::Type> states)
{
    std::unique_lock<std::mutex> lock(mJointStateLock);
    for(int i = 0; i < 3; i++) {
        mJointStates[i] = states[i];
    }
    mJointStateCond.notify_all();
}

