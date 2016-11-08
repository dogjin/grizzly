//
//  FastFourierTransformAccelerate.cpp
//  bear-dsp
//
//  Created by Stijn on 09/06/2016.
//  Copyright © 2016 Dsperados. All rights reserved.
//

#include <stdexcept>

#include "../FastFourierTransform.hpp"
#include "../Parallel.hpp"

using namespace gsl;
using namespace std;

namespace bear::dsp
{
    FastFourierTransformAccelerate::FastFourierTransformAccelerate(size_t size) :
        FastFourierTransformBase(size),
        evenFloat(size / 2),
        oddFloat(size / 2),
        evenDouble(size / 2),
        oddDouble(size / 2)
    {
        floatSetup.forward = vDSP_DFT_zrop_CreateSetup(nullptr, size, vDSP_DFT_FORWARD);
        floatSetup.inverse = vDSP_DFT_zrop_CreateSetup(nullptr, size, vDSP_DFT_INVERSE);
        
        floatComplexSetup.forward = vDSP_DFT_zop_CreateSetup(nullptr, size, vDSP_DFT_FORWARD);
        floatComplexSetup.inverse = vDSP_DFT_zop_CreateSetup(nullptr, size, vDSP_DFT_INVERSE);
        
        doubleSetup.forward = vDSP_DFT_zrop_CreateSetupD(nullptr, size, vDSP_DFT_FORWARD);
        doubleSetup.inverse = vDSP_DFT_zrop_CreateSetupD(nullptr, size, vDSP_DFT_INVERSE);
        
        doubleComplexSetup.forward = vDSP_DFT_zop_CreateSetupD(nullptr, size, vDSP_DFT_FORWARD);
        doubleComplexSetup.inverse = vDSP_DFT_zop_CreateSetupD(nullptr, size, vDSP_DFT_INVERSE);
        
        if (!floatSetup.forward || !floatSetup.inverse || !floatComplexSetup.forward || !floatComplexSetup.inverse ||
            !doubleSetup.forward || !doubleSetup.inverse || !doubleComplexSetup.forward || !doubleComplexSetup.inverse)
        {
            throw runtime_error("Could not setup vDSP FFT");
        }
    }
    
    void FastFourierTransformAccelerate::doForward(const vector<float>& input, vector<float>& real, vector<float>& imaginary)
    {
        // Split the input signal to even and odd arrays
        deinterleave(input, vector<float>&(evenFloat), vector<float>&(oddFloat));

        // Do the transform
        vDSP_DFT_Execute(floatSetup.forward, evenFloat.data(), oddFloat.data(), real.data(), imaginary.data());

        // In the forward direction, the scale is 2 (for some reason), so scale back by a half
        // Probably because both the negative and positive frequencies get summed, or something. The complex-to-complex
        // DFT in Accelerate (see function below) doesn't need scaling).
        const float factor = 0.5f;
        multiply(const vector<float>&(real), factor, real);
        multiply(const vector<float>&(imaginary), factor, imaginary);

        // Because the Im[0] and Im[Nyquist] are always 0, vDSP stores the Re[Nyquist]
        // in Im[0], so that it can return one complex number less. Imho, this only makes
        // things more complicated and hardly seems worth the hassle, but there you have it.
        // Therefore, we copy Im[0] to Re[Nyquist]...
        real[size / 2] = imaginary[0];

        // ... And set Im[0] and Im[Nyquist] to 0, which they always are
        imaginary[0] = 0.0f;
        imaginary[size / 2] = 0.0f;
    }
    
    void FastFourierTransformAccelerate::doForward(const vector<double>& input, vector<double>& real, vector<double>& imaginary)
    {
        // Split the input signal to even and odd arrays
        deinterleave(input, vector<double>&(evenDouble), vector<double>&(oddDouble));
        
        // Do the transform
        vDSP_DFT_ExecuteD(doubleSetup.forward, evenDouble.data(), oddDouble.data(), real.data(), imaginary.data());
        
        // In the forward direction, the scale is 2 (for some reason), so scale back by a half
        // Probably because both the negative and positive frequencies get summed, or something. The complex-to-complex
        // DFT in Accelerate (see function below) doesn't need scaling).
        const double factor = 0.5f;
        multiply(const vector<double>&(real), factor, real);
        multiply(const vector<double>&(imaginary), factor, imaginary);
        
        // Because the Im[0] and Im[Nyquist] are always 0, vDSP stores the Re[Nyquist]
        // in Im[0], so that it can return one complex number less. Imho, this only makes
        // things more complicated and hardly seems worth the hassle, but there you have it.
        // Therefore, we copy Im[0] to Re[Nyquist]...
        real[size / 2] = imaginary[0];
        
        // ... And set Im[0] and Im[Nyquist] to 0, which they always are
        imaginary[0] = 0.0f;
        imaginary[size / 2] = 0.0f;
    }
    
    void FastFourierTransformAccelerate::doInverse(const vector<float>& real, const vector<float>& imaginary, vector<float>& output)
    {
        // Copy the input reals and imaginaries, so that we can change the format around to
        // the way vDSP accepts it
        vector<float> real_(real.begin(), real.end());
        vector<float> imaginary_(imaginary.begin(), imaginary.end());

        // Re[Nyquist] is supposed to be stored in Im[0] for vDSP
        imaginary_[0] = real[size / 2];

        // Do the transform
        vDSP_DFT_Execute(floatSetup.inverse, real.data(), imaginary_.data(), real_.data(), imaginary_.data());

        // Combine the even and odd output signals into one interleaved output signal
        interleave(const vector<float>&(real_), const vector<float>&(imaginary_), output);

        // For inverse DFT, the scaling is Size, so scale back by multiplying with its reciprocal
        const float factor = 1.0f / output.size();
        multiply(const vector<float>&(output), factor, output);
    }
    
    void FastFourierTransformAccelerate::doInverse(const vector<double>& real, const vector<double>& imaginary, vector<double>& output)
    {
        // Copy the input reals and imaginaries, so that we can change the format around to
        // the way vDSP accepts it
        vector<double> real_(real.begin(), real.end());
        vector<double> imaginary_(imaginary.begin(), imaginary.end());
        
        // Re[Nyquist] is supposed to be stored in Im[0] for vDSP
        imaginary_[0] = real[size / 2];
        
        // Do the transform
        vDSP_DFT_ExecuteD(doubleSetup.inverse, real.data(), imaginary_.data(), real_.data(), imaginary_.data());
        
        // Combine the even and odd output signals into one interleaved output signal
        interleave(const vector<double>&(real_), const vector<double>&(imaginary_), output);
        
        // For inverse DFT, the scaling is Size, so scale back by multiplying with its reciprocal
        const double factor = 1.0 / output.size();
        multiply(const vector<double>&(output), factor, output);
    }
    
    void FastFourierTransformAccelerate::doForwardComplex(const vector<float>& inReal, const vector<float>& inImaginary, vector<float>& outReal, vector<float>& outImaginary)
    {
        // Do the transform
        vDSP_DFT_Execute(floatComplexSetup.forward, inReal.data(), inImaginary.data(), outReal.data(), outImaginary.data());
    }
    
    void FastFourierTransformAccelerate::doForwardComplex(const vector<double>& inReal, const vector<double>& inImaginary, vector<double>& outReal, vector<double>& outImaginary)
    {
        // Do the transform
        vDSP_DFT_ExecuteD(doubleComplexSetup.forward, inReal.data(), inImaginary.data(), outReal.data(), outImaginary.data());
    }
    
    void FastFourierTransformAccelerate::doInverseComplex(const vector<float>& inReal, const vector<float>& inImaginary, vector<float>& outReal, vector<float>& outImaginary)
    {
        // Do the transform
        vDSP_DFT_Execute(floatComplexSetup.inverse, inReal.data(), inImaginary.data(), outReal.data(), outImaginary.data());
        
        // For inverse DFT, the scaling is Size, so scale back by multiplying with its reciprocal
        const float factor = 1.0f / outReal.size();
        multiply(const vector<float>&(outReal), factor, outReal);
        multiply(const vector<float>&(outImaginary), factor, outImaginary);
    }
    
    void FastFourierTransformAccelerate::doInverseComplex(const vector<double>& inReal, const vector<double>& inImaginary, vector<double>& outReal, vector<double>& outImaginary)
    {
        // Do the transform
        vDSP_DFT_ExecuteD(doubleComplexSetup.inverse, inReal.data(), inImaginary.data(), outReal.data(), outImaginary.data());
        
        // For inverse DFT, the scaling is Size, so scale back by multiplying with its reciprocal
        const double factor = 1.0 / outReal.size();
        multiply(const vector<double>&(outReal), factor, outReal);
        multiply(const vector<double>&(outImaginary), factor, outImaginary);
    }
}
