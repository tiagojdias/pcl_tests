#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>

namespace prodrone
{

class ElapseTimer
{
    std::chrono::system_clock::time_point start;

public:
    ElapseTimer():
    start(std::chrono::high_resolution_clock::now())
    {}

    double elapsed()
    {
        auto finish = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = finish - start;
        return elapsed.count();
    }
};


} /// namespace prodrone

#endif /// TIMER_HPP 
