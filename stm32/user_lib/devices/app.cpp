#include "task.h"

extern "C" void app_ready(void)
{
    ;
}

extern "C" void app_loop(void)
{
    task::loop();
}
