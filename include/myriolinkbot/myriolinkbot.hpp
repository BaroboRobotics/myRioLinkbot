#ifndef MYRIOLINKBOT_HPP_
#define MYRIOLINKBOT_HPP_

#include "baromesh/linkbot.hpp"

#include <exception>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <thread>

namespace myRio {

class motor_exception : public std::exception
{
    public:
    virtual const char* what() const throw()
    {
        return "A motor encountered an error during a motion." ;
    }
};

class timeout_exception : public std::exception
{
public:
    timeout_exception(std::string msg) {
        mMsg = "Timeout occured: " + msg;
    }

    virtual const char* what() const throw()
    {
        return mMsg.c_str();
    }

    private:
    static std::string mMsg;
};

class Linkbot : public barobo::Linkbot {
public:
    Linkbot(const std::string& serialId);
    ~Linkbot();

    void jointEventCb(int jointNo, barobo::JointState::Type event);

    void moveWait(int mask = 0x07, double timeout = 0.0);

    void _setJointStates(std::vector<barobo::JointState::Type> states);

    int motorMask() {return mMotorMask;}

private:
    std::mutex mJointStateLock;
    std::condition_variable mJointStateCond;
    barobo::JointState::Type mJointStates[3];
    barobo::FormFactor::Type mFormFactor;
    int mMotorMask;
};

}

#endif
