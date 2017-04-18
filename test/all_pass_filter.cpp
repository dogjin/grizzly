#include <iostream>

#include "doctest.h"

#include "../all_pass_filter.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("test AllPassFilter.hpp")
{
    SUBCASE("output")
    {
        AllPassFilter<float> allPass(7);

        // std::cout << "0: " << allPass.process(1, 7, 0.167772) << std::endl;

        // for (auto i = 1; i < 25; ++i)
        //     std::cout << i << ": " << allPass.process(0, 7, 0.167772) << std::endl;
    }

    SUBCASE("delaytime 0")
    {
        AllPassFilter<float> allPass(0);

        CHECK(allPass.getMaximalDelayTime() == 0);
    }

    SUBCASE("all 0's")
    {
        AllPassFilter<float> allPass(10);
        auto bufferSize = 100;
        float buffer[bufferSize];

        // fill
        for (auto i = 0; i < bufferSize; i++)
            buffer[i] = allPass.writeAndRead(0, 10, 1);

        // check
        for (auto i = 0; i < bufferSize; i++)
            CHECK(buffer[i] == 0);
    }
}