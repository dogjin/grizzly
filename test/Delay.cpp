#include <vector>

#include "doctest.h"

#include "../delay.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("Delay")
{
    SUBCASE("Delay()")
    {
        Delay<int> delay(4);
        
        REQUIRE(delay.getMaximalDelayTime() == 4);
        
        for (int i = 0; i < 5; i++)
            CHECK(delay.read(i, math::linearInterpolation) == 0);
    }
    
    SUBCASE("write()")
    {
        Delay<int> delay(2);
        
        for (auto& value : { 0, 1, 2 })
            delay.write(value);
        
        for (int i = 0; i < 3; ++i)
            CHECK(delay.read(2 - i, math::linearInterpolation) == i);
    }
    
    SUBCASE("read()")
    {
        Delay<float> delay(1);
        
        for (auto& x: {0, 1})
            delay.write(x);
        
        CHECK(delay.read(0, math::linearInterpolation) == 1);
        CHECK(delay.read(1, math::linearInterpolation) == 0);
        CHECK(delay.read(0.2, math::linearInterpolation) == doctest::Approx(0.8));
        CHECK(delay.read(0.8, math::linearInterpolation) == doctest::Approx(0.2));
        
        REQUIRE_NOTHROW(delay.read(1.2, math::linearInterpolation));
        CHECK(delay.read(1.2, math::linearInterpolation) == doctest::Approx(0));
        
        REQUIRE_NOTHROW(delay.read(-0.2, math::linearInterpolation));
        CHECK(delay.read(-0.2, math::linearInterpolation) == doctest::Approx(1));
    }
    
    SUBCASE("resize()")
    {
        Delay<int> delay(1);
        
        delay.write(1);
        delay.write(2);
        
        delay.setMaximalDelayTime(2);
        REQUIRE(delay.getMaximalDelayTime() == 2);
        
        CHECK(delay.read(0, math::linearInterpolation) == 2);
        CHECK(delay.read(1, math::linearInterpolation) == 1);
        CHECK(delay.read(2, math::linearInterpolation) == 0);
    }
}
