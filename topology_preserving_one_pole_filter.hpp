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

#pragma once

#include <cmath>
#include <moditone/math/constants.hpp>
#include <functional>

#include "integrator.hpp"
#include "solver.hpp"


namespace dsp
{    
    /*! @brief Topology preserving one pole filter with resolved zero feedback delay
     *
     *  @discussion One pole filter with resolved feedback delay. Write to the filter and
     *  read the low- or high-pass output. Besides setting the cut-off, time time can be set.
     *  This results in a low-pass response depening on the time and a time constant factor.
     *  A function for non-linear processing can be supplied which modifies the state.
     *
     * @see "The Art Of VA Filter Design" by Vadim Zavalishin
     */
    template <class T>
    class TopologyPreservingOnePoleFilter
    {
    public:
        //! Write a sample to the filter
        /* Using the direct form II integrator
         * Alternative: y = (g * x + state) / (g + 1) then
         * update state by doing 2 * lowPassOutput - state
         * with g begin tan(math::PI<T> * cutOff.value / sampleRate.value) */
        void write(T x)
        {
            lowPassOutput = integrator(x - integrator.state);
            
            // non-linear low-pass
            if (isNonLinear)
                this->lowPassOutput = solveImplicit(function, derivative, this->lowPassOutput, 0.00001, 20);
            
            // write high-pass state
            this->highPassOutput = x - this->lowPassOutput;
        }
        
        //! Read the low-pass output
        T readLowPass() const
        {
            return lowPassOutput;
        }
        
        //! Read the high-pass output
        T readHighPass() const
        {
            return highPassOutput;
        }
        
        //! Write and read low-pass output (in that order)
        T writeAndReadLowPass(const T& x)
        {
            write(x);
            return readLowPass();
        }
        
        //! Write and read high-pass output (in that order)
        T writeAndReadHighPass(const T& x)
        {
            write(x);
            return readHighPass();
        }
        
        //! Set cut-off
        void setCoefficients(double cutOff_Hz, double sampleRate_Hz)
        {
            warpedCutOff_ = std::tan(math::PI<T> * cutOff_Hz / sampleRate_Hz);
            integrator.gain = warpedCutOff_ / (1.0 + warpedCutOff_);
        }
        
        //! Set time with a default time-constant-factor
        /*! @param timeConstantFactor Affects the actual time. A factor of 1 means a step response where the output reaches to ~63% in the given time. A factor of 5 reaches to ~99%. */
        void setCoefficients(double time_s, double sampleRate_Hz, double timeConstantFactor)
        {
            warpedCutOff_ = std::tan(timeConstantFactor / (time_s * sampleRate_Hz * 2.0));
            integrator.gain = warpedCutOff_ / (1.0 + warpedCutOff_);
        }
        
        void setState(T state)
        {
            integrator.state = state;
        }
        
        //! Reset the filter to zero
        void reset()
        {
            integrator.state = 0;
            lowPassOutput = T(0);
            highPassOutput = T(0);
        }
        
        void copyCoefficients(const TopologyPreservingOnePoleFilter& rhs)
        {
            warpedCutOff_ = rhs.warpedCutOff_;
            integrator = rhs.integrator;
            
            function = rhs.function;
            derivative = rhs.derivative;
        }
        
        double warpedCutOff() const
        {
            return warpedCutOff_;
        }
        
        double gain() const
        {
            return integrator.gain;
        }
        
        double state() const
        {
            return integrator.state;
        }
        
    public:
        bool isNonLinear = false;
        
    private: // TODO voor nu even private moet werken voor andere dist functies
        T x = 0;
        
        //! Low-pass output state
        T lowPassOutput = 0;
        
        //! Low-pass output state
        T highPassOutput = 0;
        
        double warpedCutOff_ = 0;
        
        TrapezoidalIntegrator<T> integrator;

        std::function<float(float)> function = [&](const auto& estimate){ return warpedCutOff_ * (std::tanh(x) - std::tanh(estimate)) + integrator.state; };
        std::function<float(float)> derivative = [&](const auto& estimate){ return -warpedCutOff_ * (1 - std::tanh(estimate) * std::tanh(estimate)) - 1; };
    };
}

