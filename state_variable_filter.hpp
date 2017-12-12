/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/moditone/grizzly for more information.
 
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

#include "decibel_conversion.hpp"
#include "integrator.hpp"
#include "topology_preserving_filter.hpp"


namespace dsp
{
    //! Topology preserving 2-pole state variable filter with resolved zero feedback delay
    /*! See "Designing software synthesizer plug-ins in c++" by Will Pirkle.
        See "The Art Of VA Filter Design" by Vadim Zavalishin. */
    template <class T>
    class StateVariableFilter :
    public TopologyPreservingFilter<T>
    {
    public:
        //! Construct the filter with a cut-off and sample rate
        StateVariableFilter(double sampleRate_Hz) :
        TopologyPreservingFilter<T>(sampleRate_Hz)
        {
            this->resonance = 0.5;
        }
        
        // Write a sample to the filter
        void write(T x) final
        {
            this->x = x;
            
            const auto g = integrator1.g; // in this case the g = gain;
            const auto s1 = integrator1.state;
            const auto s2 = integrator2.state;
            
            highPass = (x - 2.0 * damping * s1 - g * s1 - s2) * this->gainFactor;
            
            bandPass = integrator1(highPass);
            
            // Optional non-linear processing
            if (this->nonLinear)
                bandPass = this->nonLinear(bandPass);
                
            lowPass = integrator2(bandPass);
        }
        
        //! Set the time
        /*! @param time The time is takes to reach the input value (dependant on timeConstantFactor).
             @param sampleRate The sampleRate.
             @param timeConstantFactor The factor that influences the actual time, a factor of ~5 results in a accurate time response. */
        void setTime(double time_s, double timeConstantFactor)
        {
            if (this->time_s == time_s && this->timeConstantFactor == timeConstantFactor)
                return;

            this->time_s = time_s;
            this->timeConstantFactor = timeConstantFactor;

            setCoefficients(this->sampleRate_Hz, time_s, timeConstantFactor, this->resonance);
        }

        void setTimeAndResonance(double time_s, double timeConstantFactor, double resonance)
        {
            if (this->time_s == time_s && this->timeConstantFactor == timeConstantFactor && this->resonance == resonance)
                return;

            this->time_s = time_s;
            this->timeConstantFactor = timeConstantFactor;
            this->resonance = resonance;

            setCoefficients(this->sampleRate_Hz, time_s, timeConstantFactor, resonance);
        }
        
        //! Set the filter state directly
        /*! The 2nd state is always reaching for the input value, while the first one is reaching towards zero. */
        void setState(T state1, T state2)
        {
            integrator1.state = state1;
            integrator2.state = state2;
        }

        //! Set the gain (for band-shelf type)
        void setBandShelfGain(double gain_dB)
        {
            // Set gain as amplitude value but subtract 1 (gain = 0 for pass-band)
            bandShelfGain_lin = decibelToAmplitude(gain_dB) - 1;
        }
        
        void copyCoefficients(const StateVariableFilter& rhs)
        {
            x = rhs.x;
            this->sampleRate_Hz = rhs.sampleRate_Hz;
            this->cutOff_Hz = rhs.cutOff_Hz;
            this->resonance = rhs.resonance;
            this->gainFactor = rhs.gainFactor;
            
            damping = rhs.damping;
            integrator1.g = rhs.integrator1.g;
            integrator2.g = rhs.integrator2.g;
            time_s = rhs.time_s;
            timeConstantFactor = rhs.timeConstantFactor;
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
            return x + 2 * bandShelfGain_lin * damping * bandPass;
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

        void processLowPass(const T* x, const T* cutOff, const T* resonance, T* y, size_t size)
        {
            for (auto i = 0; i < size; i++)
            {
                this->setCutOffAndResonance(cutOff[i], resonance[i]);
                y[i] = writeAndReadLowPass(x[i]);
            }
        }

        void processBandPass(const T* x, const T* cutOff, const T* resonance, T* y, size_t size)
        {
            for (auto i = 0; i < size; i++)
            {
                this->setCutOffAndResonance(cutOff[i], resonance[i]);
                y[i] = writeAndReadBandPass(x[i]);
            }
        }

        void processHighPass(const T* x, const T* cutOff, const T* resonance, T* y, size_t size)
        {
            for (auto i = 0; i < size; i++)
            {
                this->setCutOffAndResonance(cutOff[i], resonance[i]);
                y[i] = writeAndReadHighPass(x[i]);
            }
        }

    private:
        void setCoefficients(double sampleRate_Hz, double cutOff_Hz, double resonance) final
        {
            const double g = std::tan(math::PI<double> * cutOff_Hz / sampleRate_Hz);
            
            integrator1.g = g;
            integrator2.g = g;
            
            damping = 1.0 / (2.0 * resonance);
            this->gainFactor = 1.0 / (1.0 + 2.0 * damping * g + g * g);
        }
        
        void setCoefficients(double sampleRate_Hz, double time_s, double timeConstantFactor, double resonance)
        {
            const double t = time_s * math::SQRT_HALF<double>;
            
            this->cutOff_Hz = timeConstantFactor / (t * math::TWO_PI<double>);
            
            setCoefficients(sampleRate_Hz, this->cutOff_Hz, resonance);
        }
        
    private:
        TrapezoidalIntegrator<T> integrator1;
        TrapezoidalIntegrator<T> integrator2;
        
        /*! Filter Time
         *  The time it takes to reach for the input.
         *  The time determines the gain factor which is used in the system.
         *  We can set this to a default value of 0.
         */
        double time_s = 0;
        
        /*! Time constant factor for the filter time
         *  Determines the gain factor which is used in the system.
         *  Influences the actual time to reach for the input (~5 to reach in 100%
         *  of the given time.
         *  We can set this to a default value of 0.
         */
        double timeConstantFactor = 0;

        //! Damping factor, related to q
        double damping = 1;
        
        //! Input of last writing call
        T x = 0;
        
        //! High-pass output state
        T highPass = 0;
        
        //! Band-pass output state
        T bandPass = 0;
        
        //! Low-pass output state
        T lowPass = 0;
        
        //! Gain (for band-shelf type)
        double bandShelfGain_lin = 0;
    };
}

#endif
