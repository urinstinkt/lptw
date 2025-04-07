

#ifndef LINEPLANNING_UTIL_H
#define LINEPLANNING_UTIL_H

#include <chrono>
#include <string>

template <class ClockT = std::chrono::steady_clock>
class Timer
{
    using timep_t = typename ClockT::time_point;
    timep_t _start = ClockT::now();

public:

    template <class T = std::chrono::milliseconds>
    void get_and_restart() {
        auto _end = ClockT::now();
        auto dur = std::chrono::duration_cast<T>(_end - _start);
        _start = _end;
        return dur;
    }

    template <class T = std::chrono::milliseconds>
    auto get() const {
        auto _end = ClockT::now();
        return std::chrono::duration_cast<T>(_end - _start);
    }

    auto get_string() const {
        auto dur = get<std::chrono::duration<double>>().count();
        if (dur >= 1) {
            std::string str = std::to_string(dur);
            str += "s";
            return str;
        } else {
            std::string str = std::to_string(dur*1000);
            str += "ms";
            return str;
        }
    }
};

#endif //LINEPLANNING_UTIL_H
