#include "catch.hpp"

#include "../waveform.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("Waveform")
{
//    SECTION("sine")
//    {
//        SECTION("bipolar")
//        {
//            CHECK(generateBipolarSine<float>(-0.25) == Approx(-1));
//            CHECK(generateBipolarSine<float>(0) == Approx(0));
//            CHECK(generateBipolarSine<float>(0.125) == Approx(0.7071067812));
//            CHECK(generateBipolarSine<float>(0.25) == Approx(1));
//            CHECK(generateBipolarSine<float>(0.375) == Approx(0.7071067812));
//            CHECK(generateBipolarSine<float>(0.5) == Approx(0));
//            CHECK(generateBipolarSine<float>(0.625) == Approx(-0.7071067812));
//            CHECK(generateBipolarSine<float>(0.75) == Approx(-1));
//            CHECK(generateBipolarSine<float>(0.875) == Approx(-0.7071067812));
//            CHECK(generateBipolarSine<float>(1) == Approx(0));
//            CHECK(generateBipolarSine<float>(1.25) == Approx(1));
//        }
//
//        SECTION("unipolar")
//        {
//            CHECK(generateUnipolarSine<float>(-0.25) == Approx(0.5));
//            CHECK(generateUnipolarSine<float>(0) == Approx(0));
//            CHECK(generateUnipolarSine<float>(0.125) == Approx(0.14644660940672));
//            CHECK(generateUnipolarSine<float>(0.25) == Approx(0.5));
//            CHECK(generateUnipolarSine<float>(0.375) == Approx(0.85355339059328));
//            CHECK(generateUnipolarSine<float>(0.5) == Approx(1));
//            CHECK(generateUnipolarSine<float>(0.625) == Approx(0.85355339059328));
//            CHECK(generateUnipolarSine<float>(0.75) == Approx(0.5));
//            CHECK(generateUnipolarSine<float>(0.875) == Approx(00.14644660940672));
//            CHECK(generateUnipolarSine<float>(1) == Approx(0));
//            CHECK(generateUnipolarSine<float>(1.25) == Approx(0.5));
//        }
//    }
//    
//    SECTION("saw")
//    {
//        SECTION("bipolar")
//        {
//            CHECK(generateBipolarSaw<float>(-0.25) == Approx(-0.5));
//            CHECK(generateBipolarSaw<float>(0) == Approx(0));
//            CHECK(generateBipolarSaw<float>(0.125) == Approx(0.25));
//            CHECK(generateBipolarSaw<float>(0.25) == Approx(0.5));
//            CHECK(generateBipolarSaw<float>(0.375) == Approx(0.75));
//            CHECK(generateBipolarSaw<float>(0.5) == Approx(-1));
//            CHECK(generateBipolarSaw<float>(0.625) == Approx(-0.75));
//            CHECK(generateBipolarSaw<float>(0.75) == Approx(-0.5));
//            CHECK(generateBipolarSaw<float>(0.875) == Approx(-0.25));
//            CHECK(generateBipolarSaw<float>(1) == Approx(0));
//            CHECK(generateBipolarSaw<float>(1.25) == Approx(0.5));
//        }
//
//        SECTION("unipolar")
//        {
//            CHECK(generateUnipolarSaw<float>(-0.25) == Approx(0.75));
//            CHECK(generateUnipolarSaw<float>(0) == Approx(0));
//            CHECK(generateUnipolarSaw<float>(0.125) == Approx(0.125));
//            CHECK(generateUnipolarSaw<float>(0.25) == Approx(0.25));
//            CHECK(generateUnipolarSaw<float>(0.375) == Approx(0.375));
//            CHECK(generateUnipolarSaw<float>(0.5) == Approx(0.5));
//            CHECK(generateUnipolarSaw<float>(0.625) == Approx(0.625));
//            CHECK(generateUnipolarSaw<float>(0.75) == Approx(0.75));
//            CHECK(generateUnipolarSaw<float>(0.875) == Approx(0.875));
//            CHECK(generateUnipolarSaw<float>(1) == Approx(0));
//            CHECK(generateUnipolarSaw<float>(1.25) == Approx(0.25));
//        }
//    }
//
//    SECTION("triangle")
//    {
//        SECTION("bipolar")
//        {
//            CHECK(generateBipolarTriangle<float>(-0.25) == Approx(-1));
//            CHECK(generateBipolarTriangle<float>(0) == Approx(0));
//            CHECK(generateBipolarTriangle<float>(0.125) == Approx(0.5));
//            CHECK(generateBipolarTriangle<float>(0.25) == Approx(1));
//            CHECK(generateBipolarTriangle<float>(0.375) == Approx(0.5));
//            CHECK(generateBipolarTriangle<float>(0.5) == Approx(0));
//            CHECK(generateBipolarTriangle<float>(0.625) == Approx(-0.5));
//            CHECK(generateBipolarTriangle<float>(0.75) == Approx(-1));
//            CHECK(generateBipolarTriangle<float>(0.875) == Approx(-0.5));
//            CHECK(generateBipolarTriangle<float>(1) == Approx(0));
//            CHECK(generateBipolarTriangle<float>(1.25) == Approx(1));
//        }
//
//        SECTION("unipolar")
//        {
//            CHECK(generateUnipolarTriangle<float>(-0.25) == Approx(0.5));
//            CHECK(generateUnipolarTriangle<float>(0) == Approx(0));
//            CHECK(generateUnipolarTriangle<float>(0.125) == Approx(0.25));
//            CHECK(generateUnipolarTriangle<float>(0.25) == Approx(0.5));
//            CHECK(generateUnipolarTriangle<float>(0.375) == Approx(0.75));
//            CHECK(generateUnipolarTriangle<float>(0.5) == Approx(1));
//            CHECK(generateUnipolarTriangle<float>(0.625) == Approx(0.75));
//            CHECK(generateUnipolarTriangle<float>(0.75) == Approx(0.5));
//            CHECK(generateUnipolarTriangle<float>(0.875) == Approx(0.25));
//            CHECK(generateUnipolarTriangle<float>(1) == Approx(0));
//            CHECK(generateUnipolarTriangle<float>(1.25) == Approx(0.5));
//        }
//    }
//    
//    SECTION("square")
//    {
//        SECTION("bipolar")
//        {
//            SECTION("pulse width 50%")
//            {
//                CHECK(generateSquare<float>(-0.25, 0.5, -1, 1) == -1);
//                CHECK(generateSquare<float>(0, 0.5, -1, 1) == 1);
//                CHECK(generateSquare<float>(0.25, 0.5, -1, 1) == 1);
//                CHECK(generateSquare<float>(0.5, 0.5, -1, 1) == -1);
//                CHECK(generateSquare<float>(0.75, 0.5, -1, 1) == -1);
//                CHECK(generateSquare<float>(1, 0.5, -1, 1) == 1);
//                CHECK(generateSquare<float>(1.25, 0.5, -1, 1) == 1);
//            }
//            
//            SECTION("pulse width 25%")
//            {
//                CHECK(generateSquare<float>(-0.25, 0.25, -1, 1) == -1);
//                CHECK(generateSquare<float>(0, 0.25, -1, 1) == 1);
//                CHECK(generateSquare<float>(0.25, 0.25, -1, 1) == -1);
//                CHECK(generateSquare<float>(0.5, 0.25, -1, 1) == -1);
//                CHECK(generateSquare<float>(0.75, 0.25, -1, 1) == -1);
//                CHECK(generateSquare<float>(1, 0.25, -1, 1) == 1);
//                CHECK(generateSquare<float>(1.25, 0.25, -1, 1) == -1);
//            }
//            
//            SECTION("pulse width 0%")
//            {
//                CHECK(generateSquare<float>(-0.25, 0, -1, 1) == -1);
//                CHECK(generateSquare<float>(0, 0, -1, 1) == -1);
//                CHECK(generateSquare<float>(0.25, 0, -1, 1) == -1);
//                CHECK(generateSquare<float>(0.5, 0, -1, 1) == -1);
//                CHECK(generateSquare<float>(0.75, 0, -1, 1) == -1);
//                CHECK(generateSquare<float>(1, 0, -1, 1) == -1);
//                CHECK(generateSquare<float>(1.25, 0, -1, 1) == -1);
//            }
//            
//            SECTION("pulse width 100%")
//            {
//                CHECK(generateSquare<float>(-0.25, 1, -1, 1) == 1);
//                CHECK(generateSquare<float>(0, 1, -1, 1) == 1);
//                CHECK(generateSquare<float>(0.25, 1, -1, 1) == 1);
//                CHECK(generateSquare<float>(0.5, 1, -1, 1) == 1);
//                CHECK(generateSquare<float>(0.75, 1, -1, 1) == 1);
//                CHECK(generateSquare<float>(1, 1, -1, 1) == 1);
//                CHECK(generateSquare<float>(1.25, 1, -1, 1) == 1);
//            }
//
//            SECTION("pulse width -25%")
//            {
//                CHECK(generateSquare<float>(-0.25, -0.25, -1, 1) == -1);
//                CHECK(generateSquare<float>(0, -0.25, -1, 1) == -1);
//                CHECK(generateSquare<float>(0.25, -0.25, -1, 1) == -1);
//                CHECK(generateSquare<float>(0.5, -0.25, -1, 1) == -1);
//                CHECK(generateSquare<float>(0.75, -0.25, -1, 1) == -1);
//                CHECK(generateSquare<float>(1, -0.25, -1, 1) == -1);
//                CHECK(generateSquare<float>(1.25, -0.25, -1, 1) == -1);
//            }
//            
//            SECTION("pulse width 125%")
//            {
//                CHECK(generateSquare<float>(-0.25, 1.25, -1, 1) == 1);
//                CHECK(generateSquare<float>(0, 1.25, -1, 1) == 1);
//                CHECK(generateSquare<float>(0.25, 1.25, -1, 1) == 1);
//                CHECK(generateSquare<float>(0.5, 1.25, -1, 1) == 1);
//                CHECK(generateSquare<float>(0.75, 1.25, -1, 1) == 1);
//                CHECK(generateSquare<float>(1, 1.25, -1, 1) == 1);
//                CHECK(generateSquare<float>(1.25, 1.25, -1, 1) == 1);
//            }
//        }
//        
//        SECTION("unipolar")
//        {
//            SECTION("pulse width 50%")
//            {
//                CHECK(generateSquare<float>(-0.25, 0.5, 0, 1) == 0);
//                CHECK(generateSquare<float>(0, 0.5, 0, 1) == 1);
//                CHECK(generateSquare<float>(0.25, 0.5, 0, 1) == 1);
//                CHECK(generateSquare<float>(0.5, 0.5, 0, 1) == 0);
//                CHECK(generateSquare<float>(0.75, 0.5, 0, 1) == 0);
//                CHECK(generateSquare<float>(1, 0.5, 0, 1) == 1);
//                CHECK(generateSquare<float>(1.25, 0.5, 0, 1) == 1);
//            }
//            
//            SECTION("pulse width 25%")
//            {
//                CHECK(generateSquare<float>(-0.25, 0.25, 0, 1) == 0);
//                CHECK(generateSquare<float>(0, 0.25, 0, 1) == 1);
//                CHECK(generateSquare<float>(0.25, 0.25, 0, 1) == 0);
//                CHECK(generateSquare<float>(0.5, 0.25, 0, 1) == 0);
//                CHECK(generateSquare<float>(0.75, 0.25, 0, 1) == 0);
//                CHECK(generateSquare<float>(1, 0.25, 0, 1) == 1);
//                CHECK(generateSquare<float>(1.25, 0.25, 0, 1) == 0);
//            }
//            
//            SECTION("pulse width 0%")
//            {
//                CHECK(generateSquare<float>(-0.25, 0, 0, 1) == 0);
//                CHECK(generateSquare<float>(0, 0, 0, 1) == 0);
//                CHECK(generateSquare<float>(0.25, 0, 0, 1) == 0);
//                CHECK(generateSquare<float>(0.5, 0, 0, 1) == 0);
//                CHECK(generateSquare<float>(0.75, 0, 0, 1) == 0);
//                CHECK(generateSquare<float>(1, 0, 0, 1) == 0);
//                CHECK(generateSquare<float>(1.25, 0, 0, 1) == 0);
//            }
//            
//            SECTION("pulse width 100%")
//            {
//                CHECK(generateSquare<float>(-0.25, 1, 0, 1) == 1);
//                CHECK(generateSquare<float>(0, 1, 0, 1) == 1);
//                CHECK(generateSquare<float>(0.25, 1, 0, 1) == 1);
//                CHECK(generateSquare<float>(0.5, 1, 0, 1) == 1);
//                CHECK(generateSquare<float>(0.75, 1, 0, 1) == 1);
//                CHECK(generateSquare<float>(1, 1, 0, 1) == 1);
//                CHECK(generateSquare<float>(1.25, 1, 0, 1) == 1);
//            }
//
//            SECTION("pulse width -25%")
//            {
//                CHECK(generateSquare<float>(-0.25, -0.25, 0, 1) == 0);
//                CHECK(generateSquare<float>(0, -0.25, 0, 1) == 0);
//                CHECK(generateSquare<float>(0.25, -0.25, 0, 1) == 0);
//                CHECK(generateSquare<float>(0.5, -0.25, 0, 1) == 0);
//                CHECK(generateSquare<float>(0.75, -0.25, 0, 1) == 0);
//                CHECK(generateSquare<float>(1, -0.25, 0, 1) == 0);
//                CHECK(generateSquare<float>(1.25, -0.25, 0, 1) == 0);
//            }
//            
//            SECTION("pulse width 125%")
//            {
//                CHECK(generateSquare<float>(-0.25, 1.25, 0, 1) == 1);
//                CHECK(generateSquare<float>(0, 1.25, 0, 1) == 1);
//                CHECK(generateSquare<float>(0.25, 1.25, 0, 1) == 1);
//                CHECK(generateSquare<float>(0.5, 1.25, 0, 1) == 1);
//                CHECK(generateSquare<float>(0.75, 1.25, 0, 1) == 1);
//                CHECK(generateSquare<float>(1, 1.25, 0, 1) == 1);
//                CHECK(generateSquare<float>(1.25, 1.25, 0, 1) == 1);
//            }
//        }
//    }
}
