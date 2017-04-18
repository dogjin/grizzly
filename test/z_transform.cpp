#include <vector>

#include "catch.hpp"
#include "../z_transform.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("zTransform()")
{
    vector<float> sine = { 0, 0.70710678118655, 1, 0.70710678118655, 0, -0.70710678118655, -1, -0.70710678118655 };
    auto transfer = zTransform(sine.begin(), sine.end());
    
    CHECK(abs(transfer(0)) == Approx(0));
    CHECK(abs(transfer(0.78539816339745)) == Approx(4));
    CHECK(abs(transfer(1.5707963267949)) == Approx(0));
    CHECK(abs(transfer(2.35619449019234)) == Approx(0));
    CHECK(abs(transfer(3.141592653589793)) == Approx(0));
}
