#include "GameApp.hpp"

int main(int argc, char** argv)
{
    GameApp app;
    app.initApp();
    app.getRoot()->startRendering();
    app.closeApp();
    return 0;
}
