#ifndef TIMER_HPP
#define TIMER_HPP

#include <chrono>
#include <iostream>

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

    void printElapsed()
    {
        std::cout << "Time to execute: " << this->elapsed() << " seconds." << std::endl;
    }
};


} /// namespace prodrone

#endif /// TIMER_HPP 
