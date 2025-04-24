// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#ifndef __TIMER_HPP__
#define __TIMER_HPP__

#include <string>
#include <time.h>
#include <cstdlib>
#include <chrono>
#include <map>
#include <thread>
#include <sstream>

namespace tools { // tools

static std::chrono::time_point<std::chrono::system_clock> nowTime() {
    return std::chrono::system_clock::now();
}

static inline std::string getCurrentDate() {
    char tmpBuf[256];
    const time_t t = time(nullptr);
    const struct tm *local = localtime(&t);
    strftime(tmpBuf, 80, "%Y%m%d", local);
    return std::string(tmpBuf);
}

static inline std::string getCurrentTime() {
    char tmpBuf[256];
    const time_t t = time(nullptr);
    const struct tm *local = localtime(&t);
    strftime(tmpBuf, 80, "%H:%M:%S", local);
    return std::string(tmpBuf);
}

class TicToc
{
public:
    TicToc() {
        start = nowTime();
    }

    double toc() {
        std::chrono::duration<double> time_diff = nowTime() - start;
        return time_diff.count() * 1000;
    }

private:
    std::chrono::time_point<std::chrono::system_clock> start;
};

class Timer 
{
public:
    typedef std::map<std::string, std::chrono::time_point<std::chrono::system_clock>> MapTimePoint;
    typedef std::map<std::string, std::chrono::time_point<std::chrono::system_clock>>::iterator MapTimePointIt;

    Timer() {
        (find_time_point(std::string(" ")))->second = nowTime();
    }

    ~Timer() = default;

    MapTimePointIt find_time_point(const std::string &str) {
        auto it = map_time_point.find(str);
        if (it == map_time_point.end()) {
            map_time_point.insert(std::make_pair(str, nowTime()));
            return map_time_point.find(str);
        }
        else {
            return it;
        }
    }

    std::string get_thread_id() {
        if (with_thread_id) {
            std::stringstream ss;
            ss << std::this_thread::get_id();
            return ss.str();
        }
        else {
            return std::to_string(0);
        }
    }

    void tic(std::string str = std::string(" ")) {
        find_time_point(str.append(get_thread_id()))->second = nowTime();
    }

    double toc(std::string str = std::string(" "), bool retick = true) {
        auto it = find_time_point(str.append(get_thread_id()));
        std::chrono::duration<double> time_diff = nowTime() - it->second;
        if (retick) {
            it->second = nowTime();
        }
        return time_diff.count() * 1000;
    }

private:
    MapTimePoint map_time_point;
    bool with_thread_id = true;
};

}   // namespace tools

#endif
