#include <iostream>
#include <vector>

#include "catch.hpp"
#include "../down_sample.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("DownSample")
{
    DownSample<float> down(4, 64);
    
    REQUIRE(down.getFilterSize() == 64);
    REQUIRE(down.getFactor() == 4);
    
    SECTION("process()")
    {
        const vector<float> in = { 1, 1, 1, 1 };
        
        CHECK(down(in.begin()) == Approx(0).epsilon(0.1));
        down(in.begin());
        down(in.begin());
        CHECK(down(in.begin()) == Approx(1).epsilon(0.0002));
    }
    
    SECTION("setFactor()")
    {
        down.setFactor(8);
        REQUIRE(down.getFactor() == 8);
        REQUIRE(down.getFilterSize() == 64);
    }
    
    SECTION("setBetaFactor()")
    {
        REQUIRE_NOTHROW(down.setBetaFactor(8));
    }
}
