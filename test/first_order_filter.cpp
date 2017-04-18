#include "catch.hpp"

#include "../first_order_filter.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("FirstOrderFilter")
{
    SECTION("process")
    {
        FirstOrderFilter<float> filter;
        
        filter.coefficients.a0 = 0.5;
        filter.coefficients.a1 = 0.5;
        filter.coefficients.b1 = 0.5;
        
        filter.write(1);
        CHECK(filter.read() == Approx(0.5));
        
        filter.write(0);
        CHECK(filter.read() == Approx(0.75));
        
        filter.write(0);
        CHECK(filter.read() == Approx(0.375));
        
        filter.write(0);
        CHECK(filter.read() == Approx(0.1875));
        
        filter.write(0);
        CHECK(filter.read() == Approx(0.09375));
        
        filter.write(0);
        CHECK(filter.read() == Approx(0.046875));
    }
    
    SECTION("Coefficients setup")
    {
        FirstOrderCoefficients<float> coefficients;
        
        SECTION("throughPass()")
        {
            throughPass(coefficients);
            
            CHECK(coefficients.a0 == Approx(1));
            CHECK(coefficients.a1 == Approx(0));
            CHECK(coefficients.b1 == Approx(0));
        }
        
        SECTION("lowPassOnePole")
        {
            SECTION("with cutoff")
            {
                lowPassOnePole(coefficients, 44100, unit::hertz<float>(10000));
                
                CHECK(coefficients.a0 == Approx(0.759433464558704));
                CHECK(coefficients.a1 == Approx(0));
                CHECK(coefficients.b1 == Approx(0.240566535441296));
            }
            
            SECTION("with time and constant")
            {
                lowPassOnePole(coefficients, 44100, 1, 5);
                
                CHECK(coefficients.a0 == Approx(0.0001133723));
                CHECK(coefficients.a1 == Approx(0));
                CHECK(coefficients.b1 == Approx(0.9998866277));
            }
        }
        
        SECTION("lowPassOnePoleZero")
        {
            lowPassOnePoleZero(coefficients, 44100, unit::hertz<float>(10000));
            
            CHECK(coefficients.a0 == Approx(0.379716732279352));
            CHECK(coefficients.a1 == Approx(0.379716732279352));
            CHECK(coefficients.b1 == Approx(0.240566535441296));
        }
        
        SECTION("highPassOnePoleZero")
        {
            highPassOnePoleZero(coefficients, 44100, 10000);
            
            CHECK(coefficients.a0 == Approx(0.620283267720648));
            CHECK(coefficients.a1 == Approx(-0.620283267720648));
            CHECK(coefficients.b1 == Approx(0.240566535441296));
        }
    }
}
