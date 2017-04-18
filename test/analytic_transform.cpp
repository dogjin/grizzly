#include <vector>

#include "catch.hpp"
#include "../analytic_transform.hpp"

using namespace dsp;

TEST_CASE("AnalyticTransform")
{
    std::vector<float> in = { 0, 0.70710678118655, 1, 0.70710678118655, 0, -0.70710678118655, -1, -0.70710678118655 };
    std::vector<std::complex<float>> out(8);
    analyticTransform(in.begin(), in.end(), out.begin());
    
    REQUIRE(std::abs(out[0]) == Approx(1));
    CHECK(out[0].real() == Approx(0));
    CHECK(out[0].imag() == Approx(-1));
    
    REQUIRE(std::abs(out[1]) == Approx(1));
    CHECK(out[1].real() == Approx(0.70710678118655));
    CHECK(out[1].imag() == Approx(-0.70710678118655));
    
    REQUIRE(std::abs(out[2]) == Approx(1));
    CHECK(out[2].real() == Approx(1));
    CHECK(out[2].imag() == Approx(0));
    
    REQUIRE(std::abs(out[3]) == Approx(1));
    CHECK(out[3].real() == Approx(0.70710678118655));
    CHECK(out[3].imag() == Approx(0.70710678118655));
    
    REQUIRE(std::abs(out[4]) == Approx(1));
    CHECK(out[4].real() == Approx(0));
    CHECK(out[4].imag() == Approx(1));
    
    REQUIRE(std::abs(out[5]) == Approx(1));
    CHECK(out[5].real() == Approx(-0.70710678118655));
    CHECK(out[5].imag() == Approx(0.70710678118655));
    
    REQUIRE(std::abs(out[6]) == Approx(1));
    CHECK(out[6].real() == Approx(-1));
    CHECK(out[6].imag() == Approx(0));
    
    REQUIRE(std::abs(out[7]) == Approx(1));
    CHECK(out[7].real() == Approx(-0.70710678118655));
    CHECK(out[7].imag() == Approx(-0.70710678118655));
}
