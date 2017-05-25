#include <complex>
#include <vector>

#include "catch.hpp"

#include "../complex.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("Complex")
{
    std::vector<std::complex<float>> complexVector = vector<complex<float>>{{3, 4}, {-3, 4}, {3, -4}, {-3, -4}};
    
    SECTION("real")
    {
        const auto r = real(complexVector.begin(), complexVector.end());
        CHECK(r[0] == Approx(3));
        CHECK(r[1] == Approx(-3));
        CHECK(r[2] == Approx(3));
        CHECK(r[3] == Approx(-3));
    }
    
    SECTION("imaginary")
    {
        const auto i = imaginary(complexVector.begin(), complexVector.end());
        CHECK(i[0] == Approx(4));
        CHECK(i[1] == Approx(4));
        CHECK(i[2] == Approx(-4));
        CHECK(i[3] == Approx(-4));
    }
    
    SECTION("magnitudes")
    {
        for (auto& value: magnitudes(complexVector.begin(), complexVector.end()))
            CHECK(value == Approx(5));
    }
    
    SECTION("phases")
    {
        auto p = phases(complexVector.begin(), complexVector.end());
        CHECK(p[0].value == Approx(0.9273));
        CHECK(p[1].value == Approx(2.2143));
        CHECK(p[2].value == Approx(-0.9273));
        CHECK(p[3].value == Approx(-2.2143));
    }
    
    SECTION("unwrapped phases")
    {
        std::vector<std::complex<float>> complexVector = vector<complex<float>>{ {-1, 0}, {-4, -5} };
        auto unwrapped = unwrappedPhases(complexVector.begin(), complexVector.end());
        
        CHECK(unwrapped[0].value == Approx(3.14159));
        CHECK(unwrapped[1].value == Approx(4.0377));
    }
    
    SECTION("replace magnitudes")
    {
        std::vector<std::complex<float>> a = vector<complex<float>>{{0, 0}, {0, 0}};
        std::vector<std::complex<float>> b = vector<complex<float>>{{3, 4}, {3, 4}};
        
        const auto bmags = magnitudes(b.begin(), b.end());
        replaceMagnitudes(bmags.begin(), bmags.end(), a.begin());
        for (auto& value: real(a.begin(), a.end()))
            CHECK(value == Approx(5));
        
        for (auto& value: imaginary(a.begin(), a.end()))
            CHECK(value == Approx(0));
    }
    
    SECTION("replace phases")
    {
        std::vector<std::complex<float>> a = vector<complex<float>>{{0, 0}, {0, 0}};
        std::vector<std::complex<float>> b = vector<complex<float>>{{3, 4}, {3, 4}};
        
        const auto bph = phases(b.begin(), b.end());
        replacePhases(bph.begin(), bph.end(), a.begin());
        for (auto& value: real(a.begin(), a.end()))
            CHECK(value == Approx(0));
        
        for (auto& value: imaginary(a.begin(), a.end()))
            CHECK(value == Approx(0));
    }
    
    SECTION("replace real data")
    {
        std::vector<float> d{0, 0, 0, 0};
        replaceReal(d.begin(), d.end(), complexVector.begin());
        
        for (auto& value: complexVector)
            CHECK(value.real() == 0);
    }
    
    SECTION("replace imaginary data")
    {
        std::vector<float> d{0, 0, 0, 0};
        replaceImaginary(d.begin(), d.end(), complexVector.begin());
        
        for (auto& value: complexVector)
            CHECK(value.imag() == 0);
    }
}
