#include <vector>

#include "catch.hpp"
#include "../up_sample.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("UpSample")
{
    UpSample<float> up(4, 64);
    
    REQUIRE(up.getFilterSize() == 64);
    REQUIRE(up.getFactor() == 4);
    
    SECTION("process()")
    {
        auto y = up(1);
        REQUIRE(y.size() == 4);
        CHECK(y.front() == Approx(0).epsilon(0.00005));
        
        for (auto i = 0; i < 15; ++i)
            y = up(1);
        CHECK(y.back() == Approx(1));
    }
    
    SECTION("setFactor()")
    {
        up.setFactor(8);
        REQUIRE(up.getFactor() == 8);
        REQUIRE(up.getFilterSize() == 64);
    }
    
    SECTION("setBetaFactor()")
    {
        REQUIRE_NOTHROW(up.setBetaFactor(8));
    }
}
