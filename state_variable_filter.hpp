/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/dsperados/grizzly for more information.
 
 Copyright (C) 2017 Dsperados <info@dsperados.com>
 
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

#ifndef GRIZZLY_STATE_VARIABLE_FILTER_HPP
#define GRIZZLY_STATE_VARIABLE_FILTER_HPP

#include <cmath>
#include <dsperados/math/constants.hpp>
#include <experimental/optional>
#include <functional>
#include <stdexcept>
#include <unit/amplitude.hpp>
#include <unit/hertz.hpp>

namespace dsp
{
    //! Topology preserving 2-pole state variable filter with resolved zero feedback delay
    /*! See "Designing software synthesizer plug-ins in c++" by Will Pirkle.
        See "The Art Of VA Filter Design" by Vadim Zavalishin. */
    template <class T>
    class StateVariableFilter
    {
    public:
        //! Construct the filter with a cut-off and sample rate
        StateVariableFilter(unit::hertz<float> cutOff, unit::hertz<float> sampleRate) :
            cutOff(cutOff),
            sampleRate(sampleRate)
        {
            setCoefficients(cutOff, q, sampleRate);
        }
        
        //! Set the cut-off frequency
        void setCutOff(unit::hertz<float> cutOff)
        {
            this->cutOff = cutOff;
            setCoefficients(cutOff, q, sampleRate);
        }
        
        //! Set the q factor for a resonance peak (> sqrt(0.5)) at the cut-off frequency
        void setQ(float q)
        {
            this->q = q;
            dampingFactor = 1.0 / (2.0 * q);
            setCoefficients(cutOff, q, sampleRate);
        }
        
        //! Set the sample rate
        void setSampleRate(unit::hertz<float> sampleRate)
        {
            this->sampleRate = sampleRate;
            setCoefficients(cutOff, q, sampleRate);
        }
        
        //! Set coefficients given a cut-off, q factor and sample rate
        void setCoefficients(unit::hertz<float> cutOff, float q, unit::hertz<float> sampleRate)
        {
            // check cut-off
            if (cutOff.value <= 0 || cutOff.value > sampleRate.value / 2 - 10)
                throw std::invalid_argument("cut-off <= 0 or > nyquist - 10");
            
            // check q
            if (q < 0.01)
                throw std::invalid_argument("q < 0.01");
            
            // check sample rate
            if (sampleRate.value <= 0)
                throw std::invalid_argument("sample rate <= 0");
            
            dampingFactor = 1.0 / (2.0 * q);
            integratorGainFactor = std::tan(math::PI<float> * (cutOff.value / sampleRate.value));
            cutOffGain = 1.0 / (1.0 + 2.0 * dampingFactor * integratorGainFactor + integratorGainFactor * integratorGainFactor);
        }
        
        //! Set the filter state directly
        void setState(const T& state)
        {
            integratorState1 = integratorState2 = state;
        }
        
        // Write a sample to the filter
        void write(const T& x)
        {
            this->x = x;
            
            highPass = (x - 2.0 * dampingFactor * integratorState1 - integratorGainFactor * integratorState1 - integratorState2) * cutOffGain;
            
            bandPass = integratorGainFactor * highPass + integratorState1;
            
            // Optional non-linear processing
            if (nonLinear)
                bandPass = nonLinear(bandPass);
            
            lowPass = integratorGainFactor * bandPass + integratorState2;
            
            integratorState1 = integratorGainFactor * highPass + bandPass;
            integratorState2 = integratorGainFactor * bandPass + lowPass;
        }
        
        // Read low-pass output
        T readLowPass() const
        {
            return lowPass;
        }
        
        //! Write and read low-pass output
        T writeAndReadLowPass(const T& x)
        {
            write(x);
            return readLowPass();
        }
        
        // Read band-pass output
        T readBandPass() const
        {
            return bandPass;
        }
        
        //! Write and read band-pass output
        T writeAndReadBandPass(const T& x)
        {
            write(x);
            return readBandPass();
        }
        
        // Read high-pass output
        T readHighPass() const
        {
            return highPass;
        }
        
        //! Write and read high-pass output
        T writeAndReadHighPass(const T& x)
        {
            write(x);
            return readHighPass();
        }
        
        // Read unit-gain output
        T readUnitGainBandPass() const
        {
            return 2 * dampingFactor * bandPass;
        }
        
        //! Write and read unit-gain output
        T writeAndReadUnitGainBandPass(const T& x)
        {
            write(x);
            return readUnitGainBandPass();
        }
        
        // Read band-shelving (bell) output
        T readBandShelf() const
        {
            return x + 2 * gain.value * dampingFactor * bandPass;
        }
        
        //! Write and read band-shelving (bell) output
        T writeAndReadBandShelf(const T& x)
        {
            write(x);
            return readBandShelf();
        }
        
        // Read notch output
        T readNotch() const
        {
            return x - 2 * dampingFactor * bandPass;
        }
        
        //! Write and read notch output
        T writeAndReadNotch(const T& x)
        {
            write(x);
            return readNotch();
        }
        
        // Read all-pass output
        T readAllPass() const
        {
            return x - 4 * dampingFactor * bandPass;
        }
        
        //! Write and read all-pass output
        T writeAndReadAllPass(const T& x)
        {
            write(x);
            return readAllPass();
        }
        
        // Read peaking output
        T readPeak() const
        {
            return lowPass - highPass;
        }
        
        //! Write and read peaking output
        T writeAndReadPeak(const T& x)
        {
            write(x);
            return readPeak();
        }
        
        //! Set the gain (for band-shelf type)
        void setGain(unit::decibel<float> gain)
        {
            // Set gain as amplitude value but subtract 1 (gain = 0 for pass-band)
            this->gain = unit::amplitude<float>(gain).value - 1;
        }
        
    public:
        //! Function for non-linear processing
        std::function<T(const T&)> nonLinear;
        
    private:
        //! The cut-off
        unit::hertz<float> cutOff = sampleRate.value * 0.25;
        
        //! The sample rate
        unit::hertz<float> sampleRate = 44100;
        
        //! The q factor for resonance
        float q = math::SQRT_HALF<float>;
        
        //! Damping factor, related to q
        T dampingFactor = 1.0 / (2.0 * q);
        
        //! Input of last writing call
        T x = 0;
        
        //! High-pass output state
        T highPass = 0;
        
        //! Band-pass output state
        T bandPass = 0;
        
        //! Low-pass output state
        T lowPass = 0;
        
        //! Integrator gain factor
        T integratorGainFactor = 0;
        
        //! Filter gain factor with resolved zero delay feedback
        T cutOffGain = 0;
        
        //! Integrator state 1
        T integratorState1 = 0;
        
        //! Integrator state 2
        T integratorState2 = 0;
        
        //! Gain (for band-shelf type)
        unit::amplitude<float> gain = 0;
    };
}

#endif
