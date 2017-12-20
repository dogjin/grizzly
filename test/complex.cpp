#include <complex>
#include <vector>

#include "catch.hpp"

#include "../complex.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("Complex")
{
    std::vector<std::complex<float>> complexVector = vector<complex<float>>{{3, 4}, {-3, 4}, {3, -4}, {-3, -4}};
    
    SECTION("getReals")
    {
        const auto r = getReals(complexVector.begin(), complexVector.end());
        CHECK(r[0] == Approx(3));
        CHECK(r[1] == Approx(-3));
        CHECK(r[2] == Approx(3));
        CHECK(r[3] == Approx(-3));
    }
    
    SECTION("getImaginaries")
    {
        const auto i = getImaginaries(complexVector.begin(), complexVector.end());
        CHECK(i[0] == Approx(4));
        CHECK(i[1] == Approx(4));
        CHECK(i[2] == Approx(-4));
        CHECK(i[3] == Approx(-4));
    }
    
    SECTION("computeMagnitudes")
    {
        for (auto& value: computeMagnitudes(complexVector.begin(), complexVector.end()))
            CHECK(value == Approx(5));
    }
    
    SECTION("computePhases")
    {
        auto p = computePhases(complexVector.begin(), complexVector.end());
        CHECK(p[0] == Approx(0.9273));
        CHECK(p[1] == Approx(2.2143));
        CHECK(p[2] == Approx(-0.9273));
        CHECK(p[3] == Approx(-2.2143));
    }
    
    SECTION("computeUnwrappedPhases")
    {
        std::vector<std::complex<float>> complexVector = vector<complex<float>>{ {-1, 0}, {-4, -5} };
        auto unwrapped = computeUnwrappedPhases(complexVector.begin(), complexVector.end());
        
        CHECK(unwrapped[0] == Approx(3.14159));
        CHECK(unwrapped[1] == Approx(4.0377));
    }
    
    SECTION("replaceMagnitudes")
    {
        std::vector<std::complex<float>> a = vector<complex<float>>{{0, 0}, {0, 0}};
        std::vector<std::complex<float>> b = vector<complex<float>>{{3, 4}, {3, 4}};
        
        const auto magnitudes = computeMagnitudes(b.begin(), b.end());
        replaceMagnitudes(magnitudes.begin(), magnitudes.end(), a.begin());
        for (auto& value: getReals(a.begin(), a.end()))
            CHECK(value == Approx(5));
        
        for (auto& value: getImaginaries(a.begin(), a.end()))
            CHECK(value == Approx(0));
    }
    
    SECTION("replacePhases")
    {
        std::vector<std::complex<float>> a = vector<complex<float>>{{0, 0}, {0, 0}};
        std::vector<std::complex<float>> b = vector<complex<float>>{{3, 4}, {3, 4}};
        
        const auto phases = computePhases(b.begin(), b.end());
        replacePhases(phases.begin(), phases.end(), a.begin());
        for (auto& value: getReals(a.begin(), a.end()))
            CHECK(value == Approx(0));
        
        for (auto& value: getImaginaries(a.begin(), a.end()))
            CHECK(value == Approx(0));
    }
    
    SECTION("replaceReals")
    {
        std::vector<float> d{0, 0, 0, 0};
        replaceReals(d.begin(), d.end(), complexVector.begin());
        
        for (auto& value: complexVector)
            CHECK(value.real() == 0);
    }
    
    SECTION("replaceImaginaries")
    {
        std::vector<float> d{0, 0, 0, 0};
        replaceImaginaries(d.begin(), d.end(), complexVector.begin());
        
        for (auto& value: complexVector)
            CHECK(value.imag() == 0);
    }
}
