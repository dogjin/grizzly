#include <complex>
#include <vector>

#include "catch.hpp"

#include "../spectrum.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("Spectrum")
{
    Spectrum<float> spectrum = vector<complex<float>>{{3, 4}, {-3, 4}, {3, -4}, {-3, -4}};
    
    SECTION("real")
    {
        auto real = spectrum.real();
        CHECK(real[0] == Approx(3));
        CHECK(real[1] == Approx(-3));
        CHECK(real[2] == Approx(3));
        CHECK(real[3] == Approx(-3));
    }
    
    SECTION("imaginary")
    {
        auto imaginary = spectrum.imaginary();
        CHECK(imaginary[0] == Approx(4));
        CHECK(imaginary[1] == Approx(4));
        CHECK(imaginary[2] == Approx(-4));
        CHECK(imaginary[3] == Approx(-4));
    }
    
    SECTION("magnitudes")
    {
        auto magnitudes = spectrum.magnitudes();
        for (auto& value: magnitudes)
            CHECK(value == Approx(5));
    }
    
    SECTION("phases")
    {
        auto phases = spectrum.phases();
        CHECK(phases[0].value == Approx(0.9273));
        CHECK(phases[1].value == Approx(2.2143));
        CHECK(phases[2].value == Approx(-0.9273));
        CHECK(phases[3].value == Approx(-2.2143));
    }
    
    SECTION("unwrapped phases")
    {
        Spectrum<float> spectrum = vector<complex<float>>{ {-1, 0}, {-4, -5} };
        auto unwrapped = spectrum.unwrappedPhases();
        
        CHECK(unwrapped[0].value == Approx(3.14159));
        CHECK(unwrapped[1].value == Approx(4.0377));
    }
    
    SECTION("replace magnitudes")
    {
        Spectrum<float> a = vector<complex<float>>{{0, 0}, {0, 0}};
        Spectrum<float> b = vector<complex<float>>{{3, 4}, {3, 4}};
        
        const auto bmags = b.magnitudes();
        a.replaceMagnitudes(bmags.begin(), bmags.end());
        auto real = a.real();
        for (auto& value: real)
            CHECK(value == Approx(5));
        
        auto imaginary = a.imaginary();
        for (auto& value: imaginary)
            CHECK(value == Approx(0));
    }
    
    SECTION("replace phases")
    {
        Spectrum<float> a = vector<complex<float>>{{0, 0}, {0, 0}};
        Spectrum<float> b = vector<complex<float>>{{3, 4}, {3, 4}};
        
        const auto bph = b.phases();
        a.replacePhases(bph.begin(), bph.end());
        auto real = a.real();
        for (auto& value: real)
            CHECK(value == Approx(0));
        
        auto imaginary = a.imaginary();
        for (auto& value: imaginary)
            CHECK(value == Approx(0));
    }
    
    SECTION("replace real data")
    {
        std::vector<float> d{0, 0, 0, 0};
        spectrum.replaceReal(d.begin(), d.end());
        
        for (auto& value: spectrum)
            CHECK(value.real() == 0);
    }
    
    SECTION("replace imaginary data")
    {
        std::vector<float> d{0, 0, 0, 0};
        spectrum.replaceImaginary(d.begin(), d.end());
        
        for (auto& value: spectrum)
            CHECK(value.imag() == 0);
    }
}
