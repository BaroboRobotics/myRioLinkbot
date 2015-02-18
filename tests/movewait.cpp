#include "myriolinkbot/myriolinkbot.hpp"

int main()
{
    myRio::Linkbot l("LOCL");
    l.move(0x07, 90, 90, 90);
    l.moveWait();
    l.move(0x07, -90, -90, -90);
    l.moveWait();
    l.move(0x07, 5, 5, 5);
    l.moveWait();
    l.move(0x07, -5, -5, -5);
    l.moveWait();

    l.move(0x07, 360, 360, 360);
    /* Next line should throw exception */
    l.moveWait(0x07, 1);
}
