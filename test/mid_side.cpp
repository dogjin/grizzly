#include "catch.hpp"

#include "../mid_side.hpp"

using namespace dsp;

TEST_CASE("MidSide")
{
	SECTION("stereo")
	{
		SECTION("comparison")
		{
			Stereo<float> s1(1, 2);
			Stereo<float> s2(1, 2);
			REQUIRE(s1 == s2);

			s2.left = 3;
			REQUIRE(s1 != s2);
		}

		SECTION("stereo2midSide")
		{
			SECTION("left == right")
			{
			    CHECK(stereo2midSide<float>(0, 0) == MidSide<float>(0, 0));
			    CHECK(stereo2midSide<float>(1, 1) == MidSide<float>(1, 0));
			    CHECK(stereo2midSide<float>(-1, -1) == MidSide<float>(-1, 0));
			}

			SECTION("left != right")
			{
				CHECK(stereo2midSide<float>(0, 1) == MidSide<float>(0.5, -0.5));
				CHECK(stereo2midSide<float>(0, -1) == MidSide<float>(-0.5, 0.5));
				CHECK(stereo2midSide<float>(1, 0) == MidSide<float>(0.5, 0.5));
				CHECK(stereo2midSide<float>(-1, 0) == MidSide<float>(-0.5, -0.5));
			}
		}
	}

	SECTION("mid-side")
	{
		SECTION("comparison")
		{
			MidSide<float> midSide1(1, 2);
			MidSide<float> midSide2(1, 2);
			REQUIRE(midSide1 == midSide2);

			midSide2.mid = 3;
			REQUIRE(midSide1 != midSide2);
		}

		SECTION("midSide2stereo")
		{
			SECTION("side == 0")
			{
				CHECK(midSide2stereo<float>(0, 0) == Stereo<float>(0, 0));
				CHECK(midSide2stereo<float>(1, 0) == Stereo<float>(1, 1));
				CHECK(midSide2stereo<float>(-1, 0) == Stereo<float>(-1, -1));
			}

			SECTION("side != 0")
			{
				CHECK(midSide2stereo<float>(0.5, -0.5) == Stereo<float>(0, 1));
				CHECK(midSide2stereo<float>(-0.5, 0.5) == Stereo<float>(0, -1));
				CHECK(midSide2stereo<float>(0.5, 0.5) == Stereo<float>(1, 0));
				CHECK(midSide2stereo<float>(-0.5, -0.5) == Stereo<float>(-1, 0));
			}
		}
	}
}
