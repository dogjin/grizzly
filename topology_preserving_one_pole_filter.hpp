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
#include <stdexcept>

namespace dsp
{
    //    inline float tanhSolver(float input, float estimate, float g, float state, float error)
    //    {
    //        for (auto i = 0; i < 50; i++)
    //        {
    //            // calculate y based on the estimate
    //            float y = g * (input - std::tanh(estimate)) + state;
    //
    //            // get the resiude
    //            float residue = y - estimate;
    //
    //            // if the residue is smaller than the error, the estimate is close enough
    //            if (std::abs(residue) <= error)
    //                return estimate;
    //
    //            // if not, compute a new estimate
    //            float derivative = -g * (1.f - std::tanh(estimate) * std::tanh(estimate)) - 1.f;
    //            estimate = estimate - residue/derivative;
    //        }
    //
    //        return estimate;
    //    }
    
    inline float implicitFunctionSolver(const std::function<float(float)> function,
                                        const std::function<float(float)> derivative,
                                        float yEstimate, const float error, const size_t maxIterations)
    {
        for (auto i = 0; i < maxIterations; i++)
        {
            // compute the y based on the function that also uses the estimate
            float y = function(yEstimate);
            
            // get the resiude
            float residue = y - yEstimate;
            
            // We want y and the y estimate to converge,
            // if the residue is smaller than the error, the estimate is close enough
            if (std::abs(residue) <= error)
                return yEstimate;
            
            // if not, compute a new estimate
            /* this will update the outputEstimate outside the function
             * and is used in the next iteration in the function and derivative
             */
            yEstimate = yEstimate - residue / derivative(yEstimate);
        }
        
        return yEstimate;
    }
    
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
        virtual void write(T x) = 0;
        
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
            g = std::tan(math::PI<T> * cutOff_Hz / sampleRate_Hz);
            gain = g / (1.0 + g);
        }
        
        //! Set time with a default time-constant-factor
        /*! @param timeConstantFactor Affects the actual time. A factor of 1 means a step response where the output reaches to ~63% in the given time. A factor of 5 reaches to ~99%. */
        void setCoefficients(double time_s, double sampleRate_Hz, double timeConstantFactor)
        {
            g = std::tan(timeConstantFactor / (time_s * sampleRate_Hz * 2.0));
            gain = g / (1.0 + g);
        }
        
        //! Reset the filter to zero
        void reset()
        {
            state = 0;
            lowPassOutput = T(0);
            highPassOutput = T(0);
        }
        
    public:
        //! Low-pass output state
        T lowPassOutput = 0;
        
        //! Low-pass output state
        T highPassOutput = 0;
        
        double g = 0;
        
        //! Filter gain factor with resolved zero delay feedback
        double gain = 0;
        
        //! Integrator state
        double state = 0;
    };
    
    template <class T>
    class TopologyPreservingOnePoleFilterLinear : public TopologyPreservingOnePoleFilter<T>
    {
    public:
        void write(T x)
        {
            const auto v = (x - this->state) * this->gain;
            
            // write low-pass state
            this->lowPassOutput = v + this->state;
            
            // write high-pass state
            this->highPassOutput = x - this->lowPassOutput;
            
            // update the new state
            this->state = this->lowPassOutput + v;
            
            /* Alternative
             * g = std::tan(math::PI<T> * cutOff.value / sampleRate.value)
             * lowPassOutput = (g * x + state) / (g + 1); //
             * highPassOutput = x - lowPassOutput;
             * state = 2 * lowPassOutput - state;
             */
        }
    };
    
    template <class T>
    class TopologyPreservingOnePoleFilterNonLinear : public TopologyPreservingOnePoleFilter<T>
    {
    public:
        void write(T x)
        {
            this->x = x;
            
            const auto v = (x - this->state) * this->gain;
            
            // linear low-pass
            this->lowPassOutput = v + this->state;
            
            // non-linear low-pass
            this->lowPassOutput = implicitFunctionSolver(function, derivative, this->lowPassOutput, 0.00001, 20);
            
            // write high-pass state
            this->highPassOutput = x - this->lowPassOutput;
            
            // update the new state
            this->state = this->lowPassOutput + v;
        }
        
    private:
        T x = 0;
        
        std::function<float(float)> function = [&](const auto& estimate){ return this->g * (std::tanh(x) - std::tanh(estimate)) + this->state; };
        std::function<float(float)> derivative = [&](const auto& estimate){ return -this->g * (1 - std::tanh(estimate) * std::tanh(estimate)) - 1; };
    };
}

