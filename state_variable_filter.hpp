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
#include <moditone/math/constants.hpp>
#include <functional>
#include <stdexcept>
#include <moditone/unit/amplitude.hpp>
#include <moditone/unit/hertz.hpp>
#include <moditone/unit/time.hpp>

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
        StateVariableFilter(unit::hertz<double> sampleRate) :
            sampleRate(sampleRate)
        {}
        
        // Write a sample to the filter
        void write(const T& x)
        {
            this->x = x;
            
            highPass = (x - 2.0 * damping * state1 - gain * state1 - state2) * resolvedGain;
            
            bandPass = gain * highPass + state1;
            
            // Optional non-linear processing
            if (nonLinear)
                bandPass = nonLinear(bandPass);
            
            lowPass = gain * bandPass + state2;
            
            state1 = gain * highPass + bandPass;
            state2 = gain * bandPass + lowPass;
        }
        
        //! Set the sample rate
        void setSampleRate(unit::hertz<double> sampleRate)
        {
            if (this->sampleRate == sampleRate)
                return;
            
            this->sampleRate = sampleRate;
            
            gain = std::tan(math::PI<double> * (cutOff.value / sampleRate.value));
            computeResolvedGain(gain, damping);
        }
        
        //! Set the cut-off frequency
        void setCutOff(unit::hertz<double> cutOff)
        {
            if (this->cutOff == cutOff)
                return;
            
            this->cutOff = cutOff;
            
            gain = std::tan(math::PI<double> * (cutOff.value / sampleRate.value));
            
            computeResolvedGain(gain, damping);
        }
        
        //! Set the cut-off frequency and q
        void setCutOffAndQ(unit::hertz<double> cutOff, double q)
        {
            if (this->cutOff == cutOff && this->q == q)
                return;
            
            this->cutOff = cutOff;
            this->q = q;
            
            gain = std::tan(math::PI<double> * (cutOff.value / sampleRate.value));
            damping = 1.0 / (2.0 * q);
            
            computeResolvedGain(gain, damping);
        }
        
        //! Set the time
        /*! @param time The time is takes to reach the input value (dependant on timeConstantFactor).
             @param sampleRate The sampleRate.
             @param timeConstantFactor The factor that influences the actual time, a factor of ~5 results in a accurate time response. */
        void setTime(unit::second<double> time, double timeConstantFactor)
        {
            if (this->time == time && this->timeConstantFactor == timeConstantFactor)
                return;
            
            this->time = time;
            this->timeConstantFactor = timeConstantFactor;
            
            const double t = time.value * math::SQRT_HALF<double>;
            gain = std::tan(timeConstantFactor / (t * sampleRate.value * 2));
            
            computeResolvedGain(gain, damping);
        }
        
        void setTimeAndQ(unit::second<double> time, double timeConstantFactor, double q)
        {
            if (this->time == time && this->timeConstantFactor == timeConstantFactor && this->q == q)
                return;
            
            this->time = time;
            this->timeConstantFactor = timeConstantFactor;
            this->q = q;
            
            const double t = time.value * math::SQRT_HALF<double>;
            gain = std::tan(timeConstantFactor / (t * sampleRate.value * 2));
            
            damping = 1.0 / (2.0 * q);
            
            computeResolvedGain(gain, damping);
        }
        
        //! Set the q factor for a resonance peak (> sqrt(0.5)) at the cut-off frequency
        void setQ(double q)
        {
            if (this->q == q)
                return;
            
            this->q = q;
            
            damping = 1.0 / (2.0 * q);
            
            computeResolvedGain(gain, damping);
        }
        
        void copyCoefficients(const StateVariableFilter& rhs)
        {
            //TODO should we also copy the input x for types like notch
            sampleRate = rhs.sampleRate;
            cutOff = rhs.cutOff;
            time = rhs.time;
            timeConstantFactor = rhs.timeConstantFactor;
            q = rhs.q;
            gain = rhs.gain;
            resolvedGain = rhs.resolvedGain;
            damping = rhs.damping;
        }
        
        //! Set the filter state directly
        /*! The 2nd state is always reaching for the input value, while the first one is reaching towards zero. */
        void setState(T state1, T state2)
        {
            this->state1 = state1;
            this->state2 = state2;
        }
        
        //! Set the gain (for band-shelf type)
        void setBandShelfGain(unit::decibel<double> gain)
        {
            // Set gain as amplitude value but subtract 1 (gain = 0 for pass-band)
            this->bandShelfGain = unit::amplitude<double>(gain).value - 1;
        }
        
        T getState1() const noexcept
        {
            return state1;
        }
        
        T getState2() const noexcept
        {
            return state2;
        }
        
        double getGain() const noexcept
        {
            return gain;
        }
        
        double getResolvedGain() const noexcept
        {
            return resolvedGain;
        }
        
        /////////////////////////////////////////////
        // Write & read methods for sample processing
        /////////////////////////////////////////////
        
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
            return 2 * damping * bandPass;
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
            return x + 2 * bandShelfGain.value * damping * bandPass;
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
            return x - 2 * damping * bandPass;
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
            return x - 4 * damping * bandPass;
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
        
        /////////////////////////////////////////////
        // Methods for block processing
        /////////////////////////////////////////////
        
        void processLowPass(const T* x, const T* cutOff, const T* q, T* y, size_t size)
        {
            for (auto i = 0; i < size; i++)
            {
                if (this->cutOff.value != cutOff[i] && this->q != q[i])
                {
                    setCutOffAndQ(cutOff[i], q[i]);
                }
                else if (this->cutOff.value != cutOff[i])
                {
                    setCutOff(cutOff[i]);
                }
                else if (this->q != q[i])
                {
                    setQ(q[i]);
                }
                
                y[i] = writeAndReadLowPass(x[i]);
            }
        }
        
        void processBandPass(const T* x, const T* cutOff, const T* q, T* y, size_t size)
        {
            for (auto i = 0; i < size; i++)
            {
                if (this->cutOff != cutOff[i] && this->q != q[i])
                {
                    setCutOffAndQ(cutOff[i], q[i]);
                }
                else if (this->cutOff != cutOff[i])
                {
                    setCutoff(cutOff[i]);
                }
                else if (this->q != q[i])
                {
                    setQ(q[i]);
                }
                
                y[i] = writeAndReadBandPass(x[i]);
            }
        }
        
        void processHighPass(const T* x, const T* cutOff, const T* q, T* y, size_t size)
        {
            for (auto i = 0; i < size; i++)
            {
                if (this->cutOff != cutOff[i] && this->q != q[i])
                {
                    setCutOffAndQ(cutOff[i], q[i]);
                }
                else if (this->cutOff != cutOff[i])
                {
                    setCutoff(cutOff[i]);
                }
                else if (this->q != q[i])
                {
                    setQ(q[i]);
                }
                
                y[i] = writeAndReadHighPass(x[i]);
            }
        }
        
    private:
        //! compute the gain factor with resolved zero delay feedack
        void computeResolvedGain(float gain, float damping)
        {
            resolvedGain = 1.0 / (1.0 + 2.0 * damping * gain + gain * gain);
        }
        
    public:
        //! Function for non-linear processing
        std::function<T(const T&)> nonLinear;
        
    private:
        //! The sample rate
        unit::hertz<double> sampleRate = 0;
        
        /*! Cut-off frequency of the filter
         *  The cut-off determines the gain factor which is used in the system.
         *  We can set this to a default value of 0.
         */
        unit::hertz<double> cutOff = 0;
        
        /*! Filter Time
         *  The time it takes to reach for the input.
         *  The time determines the gain factor which is used in the system.
         *  We can set this to a default value of 0.
         */
        unit::second<double> time = 0;
        
        /*! Time constant factor for the filter time
         *  Determines the gain factor which is used in the system.
         *  Influences the actual time to reach for the input (~5 to reach in 100%
         *  of the given time.
         *  We can set this to a default value of 0.
         */
        double timeConstantFactor = 0;
        
        /*! Q factor for a resonace peak
         *  This factor determines the damping which is used in the system.
         *  We can therefore set this to a default value of 0.
         */
        double q = 0;
        
        //! Damping factor, related to q
        double damping = 1;
        
        //! The gain factor
        double gain = 0;
        
        //! The gain factor with resolved zero delay feedback
        double resolvedGain = 0;
        
        //! Input of last writing call
        T x = 0;
        
        //! High-pass output state
        T highPass = 0;
        
        //! Band-pass output state
        T bandPass = 0;
        
        //! Low-pass output state
        T lowPass = 0;
    
        //! The state of the first integrator
        T state1 = 0;
        
        //! The state of the second integrator
        T state2 = 0;
        
        //! Gain (for band-shelf type)
        unit::amplitude<double> bandShelfGain = 0;
    };
}

#endif
