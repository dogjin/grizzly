#include <cmath>
#include <complex>
#include <vector>

#include "catch.hpp"

#include "../cepstrum.hpp"
#include "../fast_fourier_transform.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("Cepstrum")
{
    SECTION("Complex")
    {
        std::vector<std::complex<float>> sine(512);
        for (auto i = 0; i < sine.size(); ++i)
            sine[i] = std::sin((float)i / sine.size());
        
        FastFourierTransform fft(sine.size());
        const std::vector<std::complex<float>> cepstrum = cepstrumComplex(fft, sine.begin());
    }
}
