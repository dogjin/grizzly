#include <random>
#include <vector>

#include "catch.hpp"
#include "../even_odd.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("Even/odd")
{
    std::random_device device;
    std::default_random_engine engine(device());
    std::uniform_real_distribution<> distribution;
    
    srand(time(0));
    vector<float> data(10);
    for (auto& x : data)
        x = distribution(engine);
    
    const auto even = decomposeEven(data.begin(), data.end());
    
    SECTION("even")
    {
        for (auto i = 0; i < data.size(); ++i)
            REQUIRE(even[i] == Approx(even[data.size() - 1 - i]));
    }
    
    const auto odd = decomposeOdd(data.begin(), data.end());
    
    SECTION("odd")
    {
        for (auto i = 0; i < data.size(); ++i)
            REQUIRE(odd[i] == Approx(-odd[data.size() - 1 - i]));
    }
    
    for (auto i = 0; i < data.size(); ++i)
        REQUIRE(data[i] == Approx(even[i] + odd[i]));
}
