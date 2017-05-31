#include <vector>

#include "catch.hpp"

#include "../ooura/fast_fourier_transform_ooura.hpp"

using namespace dsp;
using namespace std;

TEST_CASE("FastFourierTransformOoura")
{
    FastFourierTransformOoura fft(8);
    
    SECTION("Float")
    {
        vector<float> inputReal = {0, 0.707106781186548, 1, 0.707106781186548, 0, -0.707106781186548, -1, -0.707106781186548};
        
        SECTION("Real")
        {
            vector<float> outputReal = {0, 0, 0, 0, 0};
            vector<float> outputImaginary = {0, -4, 0, 0, 0};
            
            SECTION("Forward")
            {
                vector<float> real(5, 0);
                vector<float> imaginary(5, 0);
                
                fft.forward(inputReal.data(), real.data(), imaginary.data());
                
                for (auto i = 0; i < 5; ++i)
                {
                    CHECK(real[i] == Approx(outputReal[i]));
                    CHECK(imaginary[i] == Approx(outputImaginary[i]));
                }
            }
            
            SECTION("Inverse")
            {
                vector<float> output(8, 0);
                
                fft.inverse(outputReal.data(), outputImaginary.data(), output.data());
                
                for (auto i = 0; i < 8; ++i)
                    CHECK(output[i] == Approx(inputReal[i]));
            }
        }
        
        SECTION("Complex")
        {
            vector<float> inputImaginary = {0, 0, 0, 0, 0, 0, 0, 0};
            vector<float> outputReal = {0, 0, 0, 0, 0, 0, 0, 0};
            vector<float> outputImaginary = {0, -4, 0, 0, 0, 0, 0, 4};
            
            SECTION("Forward")
            {
                vector<float> real(8);
                vector<float> imaginary(8);
                
                fft.forwardComplex(inputReal.data(), inputImaginary.data(), real.data(), imaginary.data());
                
                for (auto i = 0; i < 8; ++i)
                {
                    CHECK(real[i] == Approx(outputReal[i]));
                    CHECK(imaginary[i] == Approx(outputImaginary[i]));
                }
            }
            
            SECTION("Inverse")
            {
                vector<float> real(8);
                vector<float> imaginary(8);
                
                fft.inverseComplex(outputReal.data(), outputImaginary.data(), real.data(), imaginary.data());
                
                for (auto i = 0; i < 8; ++i)
                {
                    CHECK(real[i] == Approx(inputReal[i]));
                    CHECK(imaginary[i] == Approx(inputImaginary[i]));
                }
            }
        }
    }
    
    SECTION("Double")
    {
        vector<double> inputReal = {0, 0.707106781186548, 1, 0.707106781186548, 0, -0.707106781186548, -1, -0.707106781186548};
        
        SECTION("Real")
        {
            vector<double> outputReal = {0, 0, 0, 0, 0};
            vector<double> outputImaginary = {0, -4, 0, 0, 0};
            
            SECTION("Forward")
            {
                vector<double> real(5, 0);
                vector<double> imaginary(5, 0);
                
                fft.forward(inputReal.data(), real.data(), imaginary.data());
                
                for (auto i = 0; i < 5; ++i)
                {
                    CHECK(real[i] == Approx(outputReal[i]));
                    CHECK(imaginary[i] == Approx(outputImaginary[i]));
                }
            }
            
            SECTION("Inverse")
            {
                vector<double> output(8, 0);
                
                fft.inverse(outputReal.data(), outputImaginary.data(), output.data());
                
                for (auto i = 0; i < 8; ++i)
                    CHECK(output[i] == Approx(inputReal[i]));
            }
        }
        
        SECTION("Complex")
        {
            vector<double> inputImaginary = {0, 0, 0, 0, 0, 0, 0, 0};
            vector<double> outputReal = {0, 0, 0, 0, 0, 0, 0, 0};
            vector<double> outputImaginary = {0, -4, 0, 0, 0, 0, 0, 4};
            
            SECTION("Forward")
            {
                vector<double> real(8);
                vector<double> imaginary(8);
                
                fft.forwardComplex(inputReal.data(), inputImaginary.data(), real.data(), imaginary.data());
                
                for (auto i = 0; i < 8; ++i)
                {
                    CHECK(real[i] == Approx(outputReal[i]));
                    CHECK(imaginary[i] == Approx(outputImaginary[i]));
                }
            }
            
            SECTION("Inverse")
            {
                vector<double> real(8);
                vector<double> imaginary(8);
                
                fft.inverseComplex(outputReal.data(), outputImaginary.data(), real.data(), imaginary.data());
                
                for (auto i = 0; i < 8; ++i)
                {
                    CHECK(real[i] == Approx(inputReal[i]));
                    CHECK(imaginary[i] == Approx(inputImaginary[i]));
                }
            }
        }
    }
}
