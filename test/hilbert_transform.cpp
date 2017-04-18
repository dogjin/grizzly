#include <vector>

#include "catch.hpp"
#include "../hilbert_transform.hpp"

using namespace dsp;

TEST_CASE("HilbertTransform")
{
	SECTION("Forward")
	{
        std::vector<float> in = { 0, 0.70710678118655, 1, 0.70710678118655, 0, -0.70710678118655, -1, -0.70710678118655 };
        std::vector<float> out(8);
        hilbertTransform(in.begin(), in.end(), out.begin(), HilbertTransformDirection::FORWARD);
        
        CHECK(out[0] == Approx(-1));
        CHECK(out[1] == Approx(-0.70710678118655));
        CHECK(out[2] == Approx(0));
        CHECK(out[3] == Approx(0.70710678118655));
        CHECK(out[4] == Approx(1));
        CHECK(out[5] == Approx(0.70710678118655));
        CHECK(out[6] == Approx(0));
        CHECK(out[7] == Approx(-0.70710678118655));
	}
    
    SECTION("Inverse")
    {
        std::vector<float> in = { -1, -0.70710678118655, 0, 0.70710678118655, 1, 0.70710678118655, 0, -0.70710678118655 };
        std::vector<float> out(8);
        hilbertTransform(in.begin(), in.end(), out.begin(), HilbertTransformDirection::INVERSE);
        
        CHECK(out[0] == Approx(0));
        CHECK(out[1] == Approx(0.70710678118655));
        CHECK(out[2] == Approx(1));
        CHECK(out[3] == Approx(0.70710678118655));
        CHECK(out[4] == Approx(0));
        CHECK(out[5] == Approx(-0.70710678118655));
        CHECK(out[6] == Approx(-1));
        CHECK(out[7] == Approx(-0.70710678118655));
    }
}
