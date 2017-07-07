#include "catch.hpp"

#include "../biquad.hpp"

using namespace dsp;
using namespace std;

#include <iostream>

TEST_CASE("Biquad")
{
    SECTION("BiquadDirectForm1")
    {
        BiquadDirectForm1<float> filter;

        filter.coefficients.a0 = 0.1;
        filter.coefficients.a1 = 0.2;
        filter.coefficients.a2 = 0.3;
        filter.coefficients.b1 = 0.4;
        filter.coefficients.b2 = 0.5;
        
        filter.write(1);
        CHECK(filter.read() == Approx(0.1));
        
        filter.write(0);
        CHECK(filter.read() == Approx(0.16));
        
        filter.write(0);
        CHECK(filter.read() == Approx(0.186));
        
        filter.write(0);
        CHECK(filter.read() == Approx(-0.1544));
        
        filter.write(0);
        CHECK(filter.read() == Approx(-0.03124));
        
        filter.write(0);
        CHECK(filter.read() == Approx(0.0897));
    }
    
    SECTION("BiquadTransposedDirectForm2")
    {
        BiquadTransposedDirectForm2<float> filter;
        
        filter.coefficients.a0 = 0.1;
        filter.coefficients.a1 = 0.2;
        filter.coefficients.a2 = 0.3;
        filter.coefficients.b1 = 0.4;
        filter.coefficients.b2 = 0.5;
        
        filter.write(1);
        CHECK(filter.read() == Approx(0.1));
        
        filter.write(0);
        CHECK(filter.read() == Approx(0.16));
        
        filter.write(0);
        CHECK(filter.read() == Approx(0.186));
        
        filter.write(0);
        CHECK(filter.read() == Approx(-0.1544));
        
        filter.write(0);
        CHECK(filter.read() == Approx(-0.03124));
        
        filter.write(0);
        CHECK(filter.read() == Approx(0.0897));
    }

    SECTION("Coefficient setup")
    {
        BiquadCoefficients<float> coefficients;
        
        SECTION("throughPass()")
        {
            throughPass(coefficients);
            
            CHECK(coefficients.a0 == Approx(1));
            CHECK(coefficients.a1 == Approx(0));
            CHECK(coefficients.a2 == Approx(0));
            CHECK(coefficients.b1 == Approx(0));
            CHECK(coefficients.b2 == Approx(0));
        }
        
        SECTION("lowPass()")
        {
            lowPass(coefficients, 44100, unit::hertz<float>(10000), 0.707);
            
            CHECK(coefficients.a0 == Approx(0.25136438));
            CHECK(coefficients.a1 == Approx(0.50272876));
            CHECK(coefficients.a2 == Approx(0.25136438));
            CHECK(coefficients.b1 == Approx(-0.171230644));
            CHECK(coefficients.b2 == Approx(0.176688224));
        }
        
        SECTION("highPass()")
        {
            highPass(coefficients, 44100, 10000, 0.707);
            
            CHECK(coefficients.a0 == Approx(0.336979717));
            CHECK(coefficients.a1 == Approx(-0.673959434));
            CHECK(coefficients.a2 == Approx(0.336979717));
            CHECK(coefficients.b1 == Approx(-0.171230644));
            CHECK(coefficients.b2 == Approx(0.176688224));
        }
        
        SECTION("bandPassConstantSkirt()")
        {
            bandPassConstantSkirt(coefficients, 44100, 10000, 0.707);
            
            CHECK(coefficients.a0 == Approx(0.291040719));
            CHECK(coefficients.a1 == Approx(0));
            CHECK(coefficients.a2 == Approx(-0.291040719));
            CHECK(coefficients.b1 == Approx(-0.171230644));
            CHECK(coefficients.b2 == Approx(0.176688224));
        }
        
        SECTION("bandPassConstantPeak()")
        {
            bandPassConstantPeak(coefficients, 44100, 10000, 0.707);
            
            CHECK(coefficients.a0 == Approx(0.411655873));
            CHECK(coefficients.a1 == Approx(0));
            CHECK(coefficients.a2 == Approx(-0.411655873));
            CHECK(coefficients.b1 == Approx(-0.171230644));
            CHECK(coefficients.b2 == Approx(0.176688224));
        }
        
        SECTION("peakConstantSkirt()")
        {
            peakConstantSkirt(coefficients, 44100, 10000, 0.707, 6);
            
            CHECK(coefficients.a0 == Approx(1.32968616));
            CHECK(coefficients.a1 == Approx(-0.194630221));
            CHECK(coefficients.a2 == Approx(0.00780280912));
            CHECK(coefficients.b1 == Approx(-0.194630221));
            CHECK(coefficients.b2 == Approx(0.337488979));
        }
        
        SECTION("peakConstantQ()")
        {
            peakConstantQ(coefficients, 44100, 10000, 0.707, -6);
            
            CHECK(coefficients.a0 == Approx(0.854829788));
            CHECK(coefficients.a1 == Approx(-0.146373048));
            CHECK(coefficients.a2 == Approx(0.151038364));
            CHECK(coefficients.b1 == Approx(-0.146373048));
            CHECK(coefficients.b2 == Approx(0.0058681583));
        }
        
        SECTION("lowShelf()")
        {
            lowShelf(coefficients, 44100, 10000, 0.707, 6);
            
            CHECK(coefficients.a0 == Approx(1.37153018));
            CHECK(coefficients.a1 == Approx(0.0419876873));
            CHECK(coefficients.a2 == Approx(0.235444129));
            CHECK(coefficients.b1 == Approx(-0.36927399));
            CHECK(coefficients.b2 == Approx(0.195712686));
        }
        
        SECTION("highShelf()")
        {
            highShelf(coefficients, 44100, 10000, 0.707, 6);
            
            CHECK(coefficients.a0 == Approx(1.45477104));
            CHECK(coefficients.a1 == Approx(-0.537209094));
            CHECK(coefficients.a2 == Approx(0.284717143));
            CHECK(coefficients.b1 == Approx(0.0306137521));
            CHECK(coefficients.b2 == Approx(0.171665296));
        }
        
        SECTION("notch()")
        {
            notch(coefficients, 44100, 10000, 0.707);
            
            CHECK(coefficients.a0 == Approx(0.588344097));
            CHECK(coefficients.a1 == Approx(-0.171230644));
            CHECK(coefficients.a2 == Approx(0.588344097));
            CHECK(coefficients.b1 == Approx(-0.171230644));
            CHECK(coefficients.b2 == Approx(0.176688224));
        }
        
        SECTION("allPass()")
        {
            allPass(coefficients, 44100, 10000, 0.707);
            
            CHECK(coefficients.a0 == Approx(0.176688224));
            CHECK(coefficients.a1 == Approx(-0.171230644));
            CHECK(coefficients.a2 == Approx(1));
            CHECK(coefficients.b1 == Approx(-0.171230644));
            CHECK(coefficients.b2 == Approx(0.176688224));
        }
    }
}
