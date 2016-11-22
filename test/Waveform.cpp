#include "doctest.h"

#include "../Waveform.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("Waveform")
{
    SUBCASE("sine")
    {
        CHECK(generateSine<float>(-0.25) == doctest::Approx(-1));
        CHECK(generateSine<float>(0) == doctest::Approx(0));
        CHECK(generateSine<float>(0.125) == doctest::Approx(0.7071067812));
        CHECK(generateSine<float>(0.25) == doctest::Approx(1));
        CHECK(generateSine<float>(0.375) == doctest::Approx(0.7071067812));
        CHECK(generateSine<float>(0.5) == doctest::Approx(0));
        CHECK(generateSine<float>(0.625) == doctest::Approx(-0.7071067812));
        CHECK(generateSine<float>(0.75) == doctest::Approx(-1));
        CHECK(generateSine<float>(0.875) == doctest::Approx(-0.7071067812));
        CHECK(generateSine<float>(1) == doctest::Approx(0));
        CHECK(generateSine<float>(1.25) == doctest::Approx(1));
    }

    SUBCASE("unipolar sine")
    {
        CHECK(generateUnipolarSine<float>(-0.25) == doctest::Approx(0));
        CHECK(generateUnipolarSine<float>(0) == doctest::Approx(0.5));
        CHECK(generateUnipolarSine<float>(0.125) == doctest::Approx(0.85355339059328));
        CHECK(generateUnipolarSine<float>(0.25) == doctest::Approx(1));
        CHECK(generateUnipolarSine<float>(0.375) == doctest::Approx(0.85355339059328));
        CHECK(generateUnipolarSine<float>(0.5) == doctest::Approx(0.5));
        CHECK(generateUnipolarSine<float>(0.625) == doctest::Approx(0.14644660940672));
        CHECK(generateUnipolarSine<float>(0.75) == doctest::Approx(0));
        CHECK(generateUnipolarSine<float>(0.875) == doctest::Approx(0.14644660940672));
        CHECK(generateUnipolarSine<float>(1) == doctest::Approx(0.5));
        CHECK(generateUnipolarSine<float>(1.25) == doctest::Approx(1));
    }
    
    SUBCASE("saw")
    {
        CHECK(generateSaw<float>(-0.25) == doctest::Approx(-0.5));
        CHECK(generateSaw<float>(0) == doctest::Approx(0));
        CHECK(generateSaw<float>(0.125) == doctest::Approx(0.25));
        CHECK(generateSaw<float>(0.25) == doctest::Approx(0.5));
        CHECK(generateSaw<float>(0.375) == doctest::Approx(0.75));
        CHECK(generateSaw<float>(0.5) == doctest::Approx(-1));
        CHECK(generateSaw<float>(0.625) == doctest::Approx(-0.75));
        CHECK(generateSaw<float>(0.75) == doctest::Approx(-0.5));
        CHECK(generateSaw<float>(0.875) == doctest::Approx(-0.25));
        CHECK(generateSaw<float>(1) == doctest::Approx(0));
        CHECK(generateSaw<float>(1.25) == doctest::Approx(0.5));
    }
    
    SUBCASE("square")
    {
        SUBCASE("bipolar")
        {
            SUBCASE("pulse width 50%")
            {
                CHECK(generateSquare<float>(-0.25, 0.5) == -1);
                CHECK(generateSquare<float>(0, 0.5) == 1);
                CHECK(generateSquare<float>(0.25, 0.5) == 1);
                CHECK(generateSquare<float>(0.5, 0.5) == -1);
                CHECK(generateSquare<float>(0.75, 0.5) == -1);
                CHECK(generateSquare<float>(1, 0.5) == 1);
                CHECK(generateSquare<float>(1.25, 0.5) == 1);
            }
            
            SUBCASE("pulse width 25%")
            {
                CHECK(generateSquare<float>(-0.25, 0.25) == -1);
                CHECK(generateSquare<float>(0, 0.25) == 1);
                CHECK(generateSquare<float>(0.25, 0.25) == -1);
                CHECK(generateSquare<float>(0.5, 0.25) == -1);
                CHECK(generateSquare<float>(0.75, 0.25) == -1);
                CHECK(generateSquare<float>(1, 0.25) == 1);
                CHECK(generateSquare<float>(1.25, 0.25) == -1);
            }
            
            SUBCASE("pulse width 0%")
            {
                CHECK(generateSquare<float>(-0.25, 0) == -1);
                CHECK(generateSquare<float>(0, 0) == -1);
                CHECK(generateSquare<float>(0.25, 0) == -1);
                CHECK(generateSquare<float>(0.5, 0) == -1);
                CHECK(generateSquare<float>(0.75, 0) == -1);
                CHECK(generateSquare<float>(1, 0) == -1);
                CHECK(generateSquare<float>(1.25, 0) == -1);
            }
            
            SUBCASE("pulse width 100%")
            {
                CHECK(generateSquare<float>(-0.25, 1) == 1);
                CHECK(generateSquare<float>(0, 1) == 1);
                CHECK(generateSquare<float>(0.25, 1) == 1);
                CHECK(generateSquare<float>(0.5, 1) == 1);
                CHECK(generateSquare<float>(0.75, 1) == 1);
                CHECK(generateSquare<float>(1, 1) == 1);
                CHECK(generateSquare<float>(1.25, 1) == 1);
            }
        }
        
        SUBCASE("unipolar")
        {
            SUBCASE("pulse width 50%")
            {
                CHECK(generateUnipolarSquare<float>(-0.25, 0.5) == 0);
                CHECK(generateUnipolarSquare<float>(0, 0.5) == 1);
                CHECK(generateUnipolarSquare<float>(0.25, 0.5) == 1);
                CHECK(generateUnipolarSquare<float>(0.5, 0.5) == 0);
                CHECK(generateUnipolarSquare<float>(0.75, 0.5) == 0);
                CHECK(generateUnipolarSquare<float>(1, 0.5) == 1);
                CHECK(generateUnipolarSquare<float>(1.25, 0.5) == 1);
            }
            
            SUBCASE("pulse width 25%")
            {
                CHECK(generateUnipolarSquare<float>(-0.25, 0.25) == 0);
                CHECK(generateUnipolarSquare<float>(0, 0.25) == 1);
                CHECK(generateUnipolarSquare<float>(0.25, 0.25) == 0);
                CHECK(generateUnipolarSquare<float>(0.5, 0.25) == 0);
                CHECK(generateUnipolarSquare<float>(0.75, 0.25) == 0);
                CHECK(generateUnipolarSquare<float>(1, 0.25) == 1);
                CHECK(generateUnipolarSquare<float>(1.25, 0.25) == 0);
            }
            
            SUBCASE("pulse width 0%")
            {
                CHECK(generateUnipolarSquare<float>(-0.25, 0) == 0);
                CHECK(generateUnipolarSquare<float>(0, 0) == 0);
                CHECK(generateUnipolarSquare<float>(0.25, 0) == 0);
                CHECK(generateUnipolarSquare<float>(0.5, 0) == 0);
                CHECK(generateUnipolarSquare<float>(0.75, 0) == 0);
                CHECK(generateUnipolarSquare<float>(1, 0) == 0);
                CHECK(generateUnipolarSquare<float>(1.25, 0) == 0);
            }
            
            SUBCASE("pulse width 100%")
            {
                CHECK(generateUnipolarSquare<float>(-0.25, 1) == 1);
                CHECK(generateUnipolarSquare<float>(0, 1) == 1);
                CHECK(generateUnipolarSquare<float>(0.25, 1) == 1);
                CHECK(generateUnipolarSquare<float>(0.5, 1) == 1);
                CHECK(generateUnipolarSquare<float>(0.75, 1) == 1);
                CHECK(generateUnipolarSquare<float>(1, 1) == 1);
                CHECK(generateUnipolarSquare<float>(1.25, 1) == 1);
            }
        }
    }
}
