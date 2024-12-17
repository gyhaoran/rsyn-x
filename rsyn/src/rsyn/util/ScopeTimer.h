#ifndef B3021A5F_D620_458E_BE9E_3B17161847B9
#define B3021A5F_D620_458E_BE9E_3B17161847B9

#include "symbol.h"
#include <string>
#include <chrono>
#include <iostream>

namespace Rsyn
{

struct ScopeTimer 
{
    ScopeTimer(const std::string& name) : name_(name), start_(std::chrono::high_resolution_clock::now()) 
    {}

    ~ScopeTimer() 
    {
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> duration = end - start_;
        std::cout << name_ << " took " << duration.count() << " ms" << std::endl;
    }

private:
    std::string name_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};

} // namespace Rsyn

#define MEASURE_TIME() ::Rsyn::ScopeTimer UNIQUE_NAME(timer_){__func__}

#endif /* B3021A5F_D620_458E_BE9E_3B17161847B9 */
