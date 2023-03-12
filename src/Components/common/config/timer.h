#include <chrono>

class Timer {
public:
    Timer() = delete;
    
    /*
    @brief get current time stamp
    */ 
    static double Now() {
        auto t = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        );
        return static_cast<double>(t.count()) / 1000.0;
    }

    // @brief get milliseconds
    static std::chrono::milliseconds MilliSeconds(unsigned int t) {
        return std::chrono::milliseconds(t);
    }
};