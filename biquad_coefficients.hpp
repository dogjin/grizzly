/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/moditone/grizzly for more information.
 
 Copyright (C) 2016-2018 Moditone <info@moditone.com>
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>
 
 --------------------------------------------------------------------
 
 If you would like to use Grizzly for commercial or closed-source
 purposes, please contact us for a commercial license.
 
 */

#ifndef GRIZZLY_BIQUAD_COEFFICIENTS_HPP
#define GRIZZLY_BIQUAD_COEFFICIENTS_HPP

#include <cmath>
#include <complex>

#include <moditone/math/constants.hpp>


namespace dsp
{
    //! Biquad Coefficients
    /*! Credits to Robert Bristow-Johnson for providing the cooking formulas (see "Audio-EQ-cookbook"). 
        Notice we use "a" for feed-forward and "b" for feed-back. */
    template <class T>
    struct BiquadCoefficients
    {
        //! The a0 feed-forward coefficient (gain)
        T a0 = 0;
        
        //! The a1 feed-forward coefficient
        T a1 = 0;
        
        //! The a2 feed-forward coefficient
        T a2 = 0;
        
        //! The b1 feed-back coefficient
        T b1 = 0;
        
        //! The b2 feed-back coefficient
        T b2 = 0;
        
        bool isStable()
        {
            // a = 1, b = b1, c = b2
            // solve for ax^2 + bx + c
            std::complex<T> discriminant(b1 * b1 - 4 * b2, 0);
            auto sqrtDiscriminant = std::sqrt(discriminant);
            
            auto x1 = (-b1 + sqrtDiscriminant) / 2.f;
            auto x2 = (-b1 - sqrtDiscriminant) / 2.f;
            
            if (std::abs(x1) < 1 && std::abs(x2) < 1)
                return true;
            else
                return false;
        }
    };
    
    //! Set biquad to through pass
    template <typename T>
    void throughPass(BiquadCoefficients<T>& coefficients)
    {
        coefficients.a0 = 1;
        coefficients.a1 = 0;
        coefficients.a2 = 0;
        coefficients.b1 = 0;
        coefficients.b2 = 0;
    }
    
    //! Set biquad to no pass
    template <typename T>
    void noPass(BiquadCoefficients<T>& coefficients)
    {
        coefficients.a0 = 0;
        coefficients.a1 = 0;
        coefficients.a2 = 0;
        coefficients.b1 = 0;
        coefficients.b2 = 0;
    }
    
    //! Compute the radial frequency given a cut-off and sample-rate
    template <class T>
    constexpr T computeRadialFrequency(T cutOff_Hz, T sampleRate_Hz)
    {
        return math::TWO_PI<T> * (cutOff_Hz / sampleRate_Hz);
    }
    
    //! Set biquad to low pass filtering using a cut-off frequency
    template <class T>
    void lowPass(BiquadCoefficients<T>& coefficients, T sampleRate_Hz, T cutOff_Hz, T q)
    {
        const auto w = computeRadialFrequency(cutOff_Hz, sampleRate_Hz);
        const auto sinw = std::sin(w);
        const auto cosw = std::cos(w);
        const auto alpha = sinw / (2 * q);
        
        const auto b0 = 1 / (1 + alpha);
        
        coefficients.a0 = ((1 - cosw) / 2) * b0;
        coefficients.a1 = (1 - cosw) * b0;
        coefficients.a2 = ((1 - cosw) / 2) * b0;
        
        coefficients.b1 = (-2 * cosw) * b0;
        coefficients.b2 = (1 - alpha) * b0;
    }
    
    //! Set biquad to low pass filtering using a time and time constant factor
    template <class T>
    void lowPass(BiquadCoefficients<T>& coefficients, T sampleRate_Hz, T time_s, T q, T timeConstantFactor)
    {
        // correct time for two-pole timed filtering
        const auto t = time_s * math::SQRT_HALF<T>;
        
        const auto w = timeConstantFactor / (t * sampleRate_Hz);
        const auto sinw = std::sin(w);
        const auto cosw = std::cos(w);
        const auto alpha = sinw / (2 * q);
        
        const auto b0 = 1 / (1 + alpha);
        
        coefficients.a0 = ((1 - cosw) / 2) * b0;
        coefficients.a1 = (1 - cosw) * b0;
        coefficients.a2 = ((1 - cosw) / 2) * b0;
        
        coefficients.b1 = (-2 * cosw) * b0;
        coefficients.b2 = (1 - alpha) * b0;
    }
    
    //! Set biquad to high pass filtering
    template <class T>
    void highPass(BiquadCoefficients<T>& coefficients, T sampleRate_Hz, T cutOff_Hz, T q)
    {
        const auto w = computeRadialFrequency(cutOff_Hz, sampleRate_Hz);
        const auto sinw = std::sin(w);
        const auto cosw = std::cos(w);
        const auto alpha = sinw / (2 * q);
        
        const auto b0 = 1 / (1 + alpha);
        
        coefficients.a0 = ((1 + cosw) / 2) * b0;
        coefficients.a1 = (-(1 + cosw)) * b0;
        coefficients.a2 = ((1 + cosw) / 2) * b0;
        
        coefficients.b1 = (-2 * cosw) * b0;
        coefficients.b2 = (1 - alpha) * b0;
    }
    
    //! Set biquad to band pass filtering with a constant skirt gain
    template <class T>
    void bandPassConstantSkirt(BiquadCoefficients<T>& coefficients, T sampleRate_Hz, T cutOff_Hz, T q)
    {
        const auto w = computeRadialFrequency(cutOff_Hz, sampleRate_Hz);
        const auto sinw = std::sin(w);
        const auto cosw = std::cos(w);
        const auto alpha = sinw / (2 * q);
        
        const auto b0 = 1 / (1 + alpha);
        
        coefficients.a0 = (q * alpha) * b0;
        coefficients.a1 = 0;
        coefficients.a2 = (-q * alpha) * b0;
        
        coefficients.b1 = (-2 * cosw) * b0;
        coefficients.b2 = (1 - alpha) * b0;
    }
    
    //! Set biquad to band pass filtering with a constant peak gain
    template <class T>
    void bandPassConstantPeak(BiquadCoefficients<T>& coefficients, T sampleRate_Hz, T cutOff_Hz, T q)
    {
        const auto w = computeRadialFrequency(cutOff_Hz, sampleRate_Hz);
        const auto sinw = std::sin(w);
        const auto cosw = std::cos(w);
        const auto alpha = sinw / (2 * q);
        
        const auto b0 = 1 / (1 + alpha);
        
        coefficients.a0 = alpha * b0;
        coefficients.a1 = 0;
        coefficients.a2 = -alpha * b0;
        
        coefficients.b1 = (-2 * cosw) * b0;
        coefficients.b2 = (1 - alpha) * b0;
    }
    
    //! Set biquad to peak filtering with a constant peak gain
    template <class T>
    void peak(BiquadCoefficients<T>& coefficients, T sampleRate_Hz, T cutOff_Hz, T q, T gain_dB)
    {
        const auto w = computeRadialFrequency(cutOff_Hz, sampleRate_Hz);
        const auto sinw = std::sin(w);
        const auto cosw = std::cos(w);
        const auto alpha = sinw / (2 * q);
        const auto A = std::pow(10.0, gain_dB / 40.0);
        
        const auto b0 = 1 / (1 + alpha / A);
        
        coefficients.a0 = (1 + alpha * A) * b0;
        coefficients.a1 = (-2 * cosw) * b0;
        coefficients.a2 = (1 - alpha * A) * b0;
        
        coefficients.b1 = (-2 * cosw) * b0;
        coefficients.b2 = (1 - alpha / A) * b0;
    }
    
    //! Set biquad to peak filtering with a constant Q, coeffs taken van Will Pirkle (TODO check if this is ok)
    template <class T>
    void peakConstantQ(BiquadCoefficients<T>& coefficients, T sampleRate_Hz, T cutOff_Hz, T q, T gain_dB)
    {
        const auto W = std::tan(math::PI<T> * cutOff_Hz / sampleRate_Hz);
        const auto W2 = W * W;
        
        const auto A = std::pow(10.0, gain_dB / 20.0);
        
        const auto boost = 1 + (W / q) + W2;
        const auto boostReciprocal = 1 / boost;
        
        const auto cutReciprocal = 1 / (1 + (W / (A * q)) + W2);
        
        const auto b = 2 * (W2 - 1);
        const auto d = 1 - (W / q) + W2;
        
        if (gain_dB > 0)
        {
            const T AWq = (A * W) / q;
            const T a = 1 + AWq + W2;
            const T c = 1 - AWq + W2;
            
            coefficients.a0 = a * boostReciprocal;
            coefficients.a1 = b * boostReciprocal;
            coefficients.a2 = c * boostReciprocal;
            
            coefficients.b1 = b * boostReciprocal;
            coefficients.b2 = d * boostReciprocal;
        }
        else
        {
            const T e = 1 - (1 / (A * q)) * W + W2;
            
            coefficients.a0 = boost * cutReciprocal;
            coefficients.a1 = b * cutReciprocal;
            coefficients.a2 = d * cutReciprocal;
            
            coefficients.b1 = b * cutReciprocal;
            coefficients.b2 = e * cutReciprocal;
        }
    }
    
    //! Set biquad to low shelf filtering
    template <class T>
    void lowShelf(BiquadCoefficients<T>& coefficients, T sampleRate_Hz, T cutOff_Hz, T q, T gain_dB)
    {
        const auto w = computeRadialFrequency(cutOff_Hz, sampleRate_Hz);
        const auto sinw = std::sin(w);
        const auto cosw = std::cos(w);
        const auto A = std::pow(10.0, gain_dB / 40.0);
        
        const auto beta = std::sqrt(A)/q;
        const auto b0 = 1 / ((A + 1) + (A - 1) * cosw + beta * sinw);
        
        coefficients.a0 = (A * ((A + 1) - (A - 1) * cosw + beta * sinw)) * b0;
        coefficients.a1 = (2 * A * ((A-1) - (A + 1) * cosw)) * b0;
        coefficients.a2 = (A * ((A + 1) - (A - 1) * cosw - beta * sinw)) * b0;
        
        coefficients.b1 = (-2 * ((A - 1) + (A + 1) * cosw)) * b0;
        coefficients.b2 = ((A + 1) + (A - 1) * cosw - beta * sinw) * b0;
    }
    
    //! Set biquad to high shelf filtering
    template <class T>
    void highShelf(BiquadCoefficients<T>& coefficients, T sampleRate_Hz, T cutOff_Hz, T q, T gain_dB)
    {
        const auto w = computeRadialFrequency(cutOff_Hz, sampleRate_Hz);
        const auto sinw = std::sin(w);
        const auto cosw = std::cos(w);
        const auto A = std::pow(10.0, gain_dB / 40.0);
        
        const auto beta = std::sqrt(A)/q;
        const auto b0 = 1 / ((A + 1) - (A - 1) * cosw + beta * sinw);
        
        coefficients.a0 = (A * ((A + 1) + (A-1) * cosw + beta * sinw)) * b0;
        coefficients.a1 = (-2 * A* ((A - 1) + (A + 1) * cosw)) * b0;
        coefficients.a2 = (A * ((A + 1) + (A - 1) * cosw - beta * sinw)) * b0;
        
        coefficients.b1 = (2 *((A - 1) - (A + 1) * cosw)) * b0;
        coefficients.b2 = ((A + 1) - (A - 1) * cosw - beta * sinw) * b0;
    }
    
    //! Set biquad to notch filtering
    template <class T>
    void notch(BiquadCoefficients<T>& coefficients, T sampleRate_Hz, T cutOff_Hz, T q)
    {
        const auto w = computeRadialFrequency(cutOff_Hz, sampleRate_Hz);
        const auto sinw = std::sin(w);
        const auto cosw = std::cos(w);
        const auto alpha = sinw / (2 * q);
        
        const auto b0 = 1 / (1 + alpha);
        
        coefficients.a0 = 1 * b0;
        coefficients.a1 = (-2 * cosw) * b0;
        coefficients.a2 = 1 * b0;
        coefficients.b1 = (-2 * cosw) * b0;
        coefficients.b2 = (1 - alpha) * b0;
    }
    
    //! Set biquad to all pass filtering
    template <class T>
    void allPass(BiquadCoefficients<T>& coefficients, T sampleRate_Hz, T cutOff_Hz, T q)
    {
        const auto w = computeRadialFrequency(cutOff_Hz, sampleRate_Hz);
        const auto sinw = sin(w);
        const auto cosw = cos(w);
        const auto alpha = sinw / (2 * q);
        
        const auto b0 = 1 / (1 + alpha);
        
        coefficients.a0 = (1 - alpha) * b0;
        coefficients.a1 = (-2 * cosw) * b0;
        coefficients.a2 = (1 + alpha) * b0;
        coefficients.b1 = (-2 * cosw) * b0;
        coefficients.b2 = (1 - alpha) * b0;
    }
    
}

#endif
