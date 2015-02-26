#ifndef MYRIOLINKBOT_H_
#define MYRIOLINKBOT_H_

#ifdef __cplusplus
extern "C" {
#endif

namespace myRioC {
typedef struct Linkbot Linkbot;
}

myRioC::Linkbot* myRioLinkbotNew(const char* serialId);
void myRioLinkbotDelete(myRioC::Linkbot* linkbot);

int myRioLinkbotIsMoving(myRioC::Linkbot* linkbot, int* moving);
int myRioLinkbotMoveWait(myRioC::Linkbot* linkbot, int mask, double timeout);

#ifdef __cplusplus
}
#endif

#endif
