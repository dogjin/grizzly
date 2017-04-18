#include <algorithm>
#include <vector>

#include "catch.hpp"

#include "../comb_filter.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("CombFilter")
{
    SECTION("FeedBackCombFilter")
    {
        SECTION("constructor")
        {
            SECTION("is initialized to 0")
            {
                FeedBackCombFilter<int> filter (4);
                
                REQUIRE(filter.getMaximalDelayTime() == 4);
                
                for (int i = 0; i < 4; i++)
                    CHECK(filter.writeAndRead(0., 0., 0.) == 0);
            }
        }
        
        SECTION("process()")
        {
            SECTION("Impulse")
            {
                FeedBackCombFilter<double> filter (2);
                
                CHECK(filter.writeAndRead(1.0, 1.0, 0.5) == Approx(1.));
                CHECK(filter.writeAndRead(0.0, 1.0, 0.5) == Approx(0.5));
                CHECK(filter.writeAndRead(0.0, 1.0, 0.5) == Approx(0.25));
                CHECK(filter.writeAndRead(0.0, 1.0, 0.5) == Approx(0.125));
                CHECK(filter.writeAndRead(0.0, 1.0, 0.5) == Approx(0.0625));
            }
            
            SECTION("postDelay")
            {
                FeedBackCombFilter<double> filter (2);
                filter.postDelay = [&](const double& x)
                {
                    static double xHistory = 0.;
                    auto sum = x + xHistory;
                    xHistory = x;
                    return sum / 2.;
                };
                
                CHECK(filter.writeAndRead(1.0, 1.0, 0.5) == Approx(1.0));
                CHECK(filter.writeAndRead(0.0, 1.0, 0.5) == Approx(0.25));
                CHECK(filter.writeAndRead(0.0, 1.0, 0.5) == Approx(0.3125));
            }
        }
    }
    SECTION("FeedForwardCombFilter")
    {
        SECTION("constructor")
        {
            SECTION("is initialized to 0")
            {
                FeedForwardCombFilter<int> filter (4);
                
                REQUIRE (filter.getMaximalDelayTime() == 4);
                
                for (int i = 0; i < 4; i++)
                    CHECK(filter.writeAndRead(0., 0., 0.) == 0);
            }
        }
        
        SECTION("process()")
        {
            SECTION("Impulse")
            {
                FeedForwardCombFilter<double> filter (2);
                
                CHECK(filter.writeAndRead(1., 1., 0.5) == Approx(1.));
                CHECK(filter.writeAndRead(0., 1., 0.5) == Approx(0.5));
                CHECK(filter.writeAndRead(0., 1., 0.5) == Approx(0.));
            }
            
            SECTION("postDelay")
            {
                FeedForwardCombFilter<double> filter (2);
                filter.postDelay = [&](const double& x)
                {
                    static double xHistory = 0.;
                    auto sum = x + xHistory;
                    xHistory = x;
                    return sum / 2.;
                };
                
                CHECK(filter.writeAndRead(1., 1., 0.5) == Approx(1.0));
                CHECK(filter.writeAndRead(0., 1., 0.5) == Approx(0.25));
                CHECK(filter.writeAndRead(0., 1., 0.5) == Approx(0.25));
                CHECK(filter.writeAndRead(0., 1., 0.5) == Approx(0.));
            }
        }
    }
}
