#include "myriolinkbot/myriolinkbot.h"
#include "myriolinkbot/myriolinkbot.hpp"

namespace myRioC {
    struct Linkbot {
        Linkbot(const char* serialId) : impl(serialId) { }
        myRio::Linkbot impl;
    };
}

myRioC::Linkbot* myRioLinkbotNew(const char* serialId)
{
    myRioC::Linkbot* l;
    try {
        l = new myRioC::Linkbot(serialId);
    } catch (...) {
        return nullptr;
    }
    return l;
}

void myRioLinkbotDelete(myRioC::Linkbot* linkbot)
{
    delete linkbot;
}

int myRioLinkbotIsMoving(myRioC::Linkbot* linkbot, int mask, int* moving)
{
    try {
        *moving = linkbot->impl.isMoving(mask);
    } catch (...) {
        return -1;
    }
    return 0;
}

int myRioLinkbotMoveWait(myRioC::Linkbot* linkbot, int mask, double timeout)
{
    try {
        linkbot->impl.moveWait(mask, timeout);
    } catch (...) {
        return -1;
    }
    return 0;
}
