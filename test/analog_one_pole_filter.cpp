#include "catch.hpp"

#include "../analog_one_pole_filter.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("Analog one-pole filter")
{
    AnalogOnePoleFilter<float> filter;
    filter.setCutOff(1000, 10000);
    
    SECTION("process")
    {
        filter.write(1);
        CHECK(filter.readLowPass() == Approx(0.24524f));
        CHECK(filter.readHighPass() == Approx(0.75476f));
        
        filter.write(0);
        CHECK(filter.readLowPass() == Approx(0.37019f));
        CHECK(filter.readHighPass() == Approx(-0.37019f));
        
        filter.write(0);
        CHECK(filter.readLowPass() == Approx(0.18862f));
        CHECK(filter.readHighPass() == Approx(-0.18862f));
        
        filter.write(0);
        CHECK(filter.readLowPass() == Approx(0.09611f));
        CHECK(filter.readHighPass() == Approx(-0.09611f));
        
        filter.write(0);
        CHECK(filter.readLowPass() == Approx(0.04897f));
        CHECK(filter.readHighPass() == Approx(-0.04897f));
        
        filter.write(0);
        CHECK(filter.readLowPass() == Approx(0.02495f));
        CHECK(filter.readHighPass() == Approx(-0.02495f));
    }
    
    SECTION("reset")
    {
        filter.reset();
        
        CHECK(filter.getIntegratorState() == Approx(0));
        CHECK(filter.readLowPass() == Approx(0));
        CHECK(filter.readHighPass() == Approx(0));
    }
    
    SECTION("set state")
    {
        filter.setState(2);
        
        CHECK(filter.getIntegratorState() == Approx(2));
        CHECK(filter.readLowPass() == Approx(2));
        CHECK(filter.readHighPass() == Approx(0));
    }
}
