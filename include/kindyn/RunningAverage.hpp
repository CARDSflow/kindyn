#include <vector>
#include <iostream>
#include <cmath>
using namespace std;

class RunningAverage
{
private:
    double val;
    size_t count;

public:
    RunningAverage()
    {
        this->Reset();
    }

    double Update(double valIn)
    {
        double scaling = 1. / (double)(count + 1);
        val = valIn * scaling + val * (1. - scaling);
        count++;
        return val;
    }

    double Get()
    {
        return val;
    }

    size_t Count()
    {
        return count;
    }

    void Reset()
    {
        val = 0.;
        count = 0;
    }
};