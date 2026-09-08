#include "app.h"

bool app::ready()
{
    return true;
}

void app::loop()
{
}

extern "C" int app_ready(void)
{
    return app::ready() ? 1 : 0;
}

extern "C" void app_loop(void)
{
    app::loop();
}
