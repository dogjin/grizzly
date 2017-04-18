#include <vector>

#include "catch.hpp"

#include "../spectral_centroid.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("Spectral Centroid Bin")
{
    SECTION("inequal values")
    {
        vector<int> x{1, 1, 1, 1, 2};
        auto centroid = spectralCentroid(x.begin(), x.end());
        
        CHECK(centroid == Approx(2.33333));
    }
    
    SECTION("value at middle bin")
    {
        vector<int> x{0, 0, 1, 0, 0};
        auto centroid = spectralCentroid(x.begin(), x.end());
        
        CHECK(centroid == Approx(2));
    }
    
    SECTION("value at last bin")
    {
        vector<int> x{0, 0, 0, 0, 1};
        auto centroid = spectralCentroid(x.begin(), x.end());
        
        CHECK(centroid == Approx(4));
    }
    
    
}
