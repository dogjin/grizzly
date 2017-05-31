#include <vector>

#include "catch.hpp"
#include "../impulse_response.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("createImpulseResponse()")
{
    auto filter = [](const auto& x)
    {
        static float prev = 0;
        const auto y = x * 0.5 + prev * 0.5;
        prev = x;
        return y;
    };
    
    vector<float> response(8);
    createImpulseResponse(response.begin(), response.end(), filter);
    
    CHECK(response[0] == Approx(0.5));
    CHECK(response[1] == Approx(0.5));
    CHECK(response[2] == Approx(0));
    CHECK(response[3] == Approx(0));
    CHECK(response[4] == Approx(0));
    CHECK(response[5] == Approx(0));
    CHECK(response[6] == Approx(0));
    CHECK(response[7] == Approx(0));
}
