#include "catch.hpp"

#include "../dynamic.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("Dynamic")
{
    SECTION("Make-up gain")
    {
        CHECK(compressorMakeUpGain(-24, 2.f).value == Approx(6));
        CHECK_THROWS_AS(compressorMakeUpGain(-24, 0.f), std::invalid_argument);
    }
    
    SECTION("compress")
    {
        CHECK(compressDownFactor(0, -6, 2, 0).value== Approx(-3));
        CHECK(compressDownFactor(-6, 0, 2, 0).value == 0);
        CHECK(compressDownFactor(0, -6, 0.5, 0).value == Approx(6));
        CHECK_THROWS_AS(compressDownFactor(0, -6, 0, 0), std::invalid_argument);
    }
    
    SECTION("expand")
    {
        CHECK(expandDownFactor(0, -6, 2, 0).value == 0);
        CHECK(expandDownFactor(-6, 0, 2, 0).value == Approx(-3));
        CHECK(expandDownFactor(-6, 0, 0.5, 0).value == Approx(6));
        CHECK_THROWS_AS(expandDownFactor(-6, 0, 0, 0), std::invalid_argument);
    }
}
