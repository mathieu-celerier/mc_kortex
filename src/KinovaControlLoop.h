#include "KinovaRobot.h"

namespace mc_kortex
{

class KinovaControlLoop
{
private:
    
public:
    KinovaControlLoop();
    ~KinovaControlLoop();
};

using KinovaControlLoopPtr = std::unique_ptr<KinovaControlLoop>;

} // namespace mc_kortex