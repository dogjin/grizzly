#include <complex>
#include <vector>

#include "doctest.h"

#include "../spectrum.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("Spectrum")
{
    Spectrum<float> spectrum = vector<complex<float>>{{3, 4}, {-3, 4}, {3, -4}, {-3, -4}};
    
    SUBCASE("real")
    {
        auto real = spectrum.real();
        CHECK(real[0] == doctest::Approx(3));
        CHECK(real[1] == doctest::Approx(-3));
        CHECK(real[2] == doctest::Approx(3));
        CHECK(real[3] == doctest::Approx(-3));
    }
    
    SUBCASE("imaginary")
    {
        auto imaginary = spectrum.imaginary();
        CHECK(imaginary[0] == doctest::Approx(4));
        CHECK(imaginary[1] == doctest::Approx(4));
        CHECK(imaginary[2] == doctest::Approx(-4));
        CHECK(imaginary[3] == doctest::Approx(-4));
    }
    
    SUBCASE("magnitudes")
    {
        auto magnitudes = spectrum.magnitudes();
        for (auto& value: magnitudes)
            CHECK(value == doctest::Approx(5));
    }
    
    SUBCASE("phases")
    {
        auto phases = spectrum.phases();
        CHECK(phases[0].value == doctest::Approx(0.9273));
        CHECK(phases[1].value == doctest::Approx(2.2143));
        CHECK(phases[2].value == doctest::Approx(-0.9273));
        CHECK(phases[3].value == doctest::Approx(-2.2143));
    }
    
    SUBCASE("unwrapped phases")
    {
        Spectrum<float> spectrum = vector<complex<float>>{ {-1, 0}, {-4, -5} };
        auto unwrapped = spectrum.unwrappedPhases();
        
        CHECK(unwrapped[1].value == doctest::Approx(4.0377));
    }
    
    SUBCASE("replace magnitudes")
    {
        Spectrum<float> a = vector<complex<float>>{{0, 0}, {0, 0}};
        Spectrum<float> b = vector<complex<float>>{{3, 4}, {3, 4}};
        
        a.replaceMagnitudes(b.magnitudes());
        auto real = a.real();
        for (auto& value: real)
            CHECK(value == doctest::Approx(5));
        
        auto imaginary = a.imaginary();
        for (auto& value: imaginary)
            CHECK(value == doctest::Approx(0));
    }
    
    SUBCASE("replace phases")
    {
        Spectrum<float> a = vector<complex<float>>{{0, 0}, {0, 0}};
        Spectrum<float> b = vector<complex<float>>{{3, 4}, {3, 4}};
        
        a.replacePhases(b.phases());
        auto real = a.real();
        for (auto& value: real)
            CHECK(value == doctest::Approx(0));
        
        auto imaginary = a.imaginary();
        for (auto& value: imaginary)
            CHECK(value == doctest::Approx(0));
    }
    
    SUBCASE("replace real data")
    {
        spectrum.replaceRealData(std::vector<float>{0, 0, 0, 0});
        
        for (auto& value: spectrum)
            CHECK(value.real() == 0);
    }
    
    SUBCASE("replace imaginary data")
    {
        spectrum.replaceImaginaryData(std::vector<float>{0, 0, 0, 0});
        
        for (auto& value: spectrum)
            CHECK(value.imag() == 0);
    }
}