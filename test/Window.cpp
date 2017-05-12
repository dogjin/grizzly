#include <algorithm>
#include <vector>

#include "catch.hpp"

#include "../window.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("Window")
{
    SECTION("rectangular")
    {
        SECTION("default")
        {
            auto window = createRectangularWindow<float>(4);
            for(auto& value : window)
                CHECK(value == Approx(1));
        }
        
        SECTION("with argument")
        {
            auto window = createRectangularWindow<float>(4, 2.3);
            
            for(auto& value : window)
                CHECK(value == Approx(2.3));
        }
    }
    
    SECTION("triangle")
    {
        SECTION("symmetric")
        {
            auto window = createSymmetricTriangleWindow<float>(10);
            
            CHECK(window[4] == window[5]);
        }
        
        SECTION("periodic")
        {
            auto window = createTriangleWindow<float>(10);
            float peakValue = *max_element(window.begin(), window.end());
            
            CHECK(peakValue == Approx(1));
        }
    }
    
    SECTION("Hanning")
    {
        SECTION("symmetric")
        {
            auto window = createSymmetricHannWindow<float>(10);
            
            CHECK(window[4] == window[5]);
        }
        
        SECTION("periodic")
        {
            auto window = createHannWindow<float>(10);
            float peakValue = *max_element(window.begin(), window.end());
            
            CHECK(peakValue == Approx(1));
        }
    }
    
    SECTION("Hamming")
    {
        SECTION("symmetric")
        {
            auto window = createSymmetricHammingWindow<float>(10);
            
            CHECK(window[4] == window[5]);
        }
        
        SECTION("periodic")
        {
            auto window = createHammingWindow<float>(10);
            float peakValue = *max_element(window.begin(), window.end());
            
            CHECK(peakValue == Approx(1));
        }
    }
    
    SECTION("Blackman")
    {
        SECTION("symmetric")
        {
            auto window = createSymmetricBlackmanWindow<float>(10);
            
            CHECK(window[4] == window[5]);
        }
        
        SECTION("periodic")
        {
            auto window = createBlackmanWindow<float>(10);
            float peakValue = *max_element(window.begin(), window.end());
            
            CHECK(peakValue == Approx(1));
        }
    }
    
    SECTION("sinc")
    {
        SECTION("symmetric")
        {
            auto window = createSincWindow<float>(10, M_PI, 4.5);
            CHECK(window[4] == window[5]);
            
            window = createSincWindow<float>(10, 0.1, 4.5);
            CHECK(window[4] == window[5]);
        }
        
        SECTION("periodic")
        {
            auto window = createSincWindow<float>(10, M_PI, 5);
            float peakValue = *max_element(window.begin(), window.end());
            
            CHECK(peakValue == Approx(1));
            
            window = createSincWindow<float>(10, 0.1, 5);
            peakValue = *max_element(window.begin(), window.end());
            
            CHECK(peakValue == Approx(1));
        }
    }
    
    SECTION("Kaiser")
    {
        SECTION("symmetric")
        {
            auto window = createSymmetricKaiserWindow<float>(10, 1);
            CHECK(window[4] == window[5]);
        }
        
        SECTION("periodic")
        {
            auto window = createKaiserWindow<float>(10, 1);
            CHECK(window[5] == Approx(1));
        }
    }
}
