#include <iostream>
#include "baromesh/linkbot.h"
#include "myriolinkbot/myriolinkbot.h"

int main()
{
    auto l = myRioLinkbotNew("LOCL");
    linkbotMove((baromesh::Linkbot*)l, 0x07, 90, 90, 90);
    myRioLinkbotMoveWait(l, 0x03, 0.0);
    linkbotMove((baromesh::Linkbot*)l, 0x07, -90, -90, -90);
    int moving;
    while(1) {
        myRioLinkbotIsMoving(l, 0x07, &moving);
        std::cout << moving << std::endl;
        if(!moving) break;
    }
    myRioLinkbotMoveWait(l, 0x03, 0.0);
}
