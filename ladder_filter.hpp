/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/dsperados/grizzly for more information.
 
 Copyright (C) 2016-2017 Dsperados <info@dsperados.com>
 
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

#ifndef GRIZZLY_ANALOG_LADDER_FILTER_HPP
#define GRIZZLY_ANALOG_LADDER_FILTER_HPP

#include <cmath>
#include <dsperados/math/utility.hpp>
#include <functional>
#include <unit/hertz.hpp>

#include "analog_one_pole_filter.hpp"

namespace dsp
{
    //! Topology preserving ladder filter with resolved zero delay feedback
    /*! See "Designing software synthesizer plug-ins in c++" by Will Pirkle.
        See "The Art Of VA Filter Design" by Vadim Zavalishin. */
    template <class T>
    class AnalogLadderFilter
    {
    public:
        //! Oberheim Xpander modes
        enum class XpanderMode
        {
            SECONDORDERLOWPASS,
            SECONDORDERHIGHPASS,
            SECONDORDERBANDPASS,
            FOURTHORDERBANDPASS,
            FOURTHORDERHIGHPASS
        };

    public:
        //! Set coefficients
        void setCoefficients(unit::hertz<T> cutOff, const T& feedbackFactor, unit::hertz<T> sampleRate)
        {
            this->feedbackFactor = feedbackFactor;
            auto integratorGainFactor = std::tan(math::PI<T> * cutOff.value / sampleRate.value);
            auto gainFactorOnePole = integratorGainFactor / (1.0 + integratorGainFactor);
            
            stage1.setCutOffGain(gainFactorOnePole);
            stage2.setCutOffGain(gainFactorOnePole);
            stage3.setCutOffGain(gainFactorOnePole);
            stage4.setCutOffGain(gainFactorOnePole);
            
            feedbackFactorLP1 = gainFactorOnePole * gainFactorOnePole * gainFactorOnePole / (1.0 + integratorGainFactor);
            feedbackFactorLP2 = gainFactorOnePole * gainFactorOnePole / (1.0 + integratorGainFactor);
            feedbackFactorLP3 = gainFactorOnePole / (1.0 + integratorGainFactor);
            feedbackFactorLP4 = 1.0 / (1.0 + integratorGainFactor);
            
            gainFactor = 1.0 / (1.0 + feedbackFactor * (gainFactorOnePole * gainFactorOnePole * gainFactorOnePole * gainFactorOnePole));
        }
        
        //! Write a sample to the filter
        void write(const T& x)
        {
            auto feedbackSum = feedbackFactorLP1 * stage1.getIntegratorState() +
            feedbackFactorLP2 * stage2.getIntegratorState() +
            feedbackFactorLP3 * stage3.getIntegratorState() +
            feedbackFactorLP4 * stage4.getIntegratorState();
            
            ladderInput = passBandGain ? (x * (1.0 + feedbackFactor) - feedbackFactor * feedbackSum) * gainFactor : (x - feedbackFactor * feedbackSum) * gainFactor;
            
            // Optional non-linear processing
            if (nonLinear)
                ladderInput = nonLinear(ladderInput);
            
            stage1Output = stage1.writeAndReadLowPass(ladderInput);
            stage2Output = stage2.writeAndReadLowPass(stage1Output);
            stage3Output = stage3.writeAndReadLowPass(stage2Output);
            stage4Output = stage4.writeAndReadLowPass(stage3Output);
        }
        
        //! Read the low-pass output
        T readLowPass()
        {
            return stage4Output;
        }
        
        //! Write and read the low-pass output
        T writeAndReadLowPass(const T& x)
        {
            write(x);
            return readLowPass();
        }
        
        //! Read the band-pass output
        T readBandPass()
        {
            return 4 * stage2Output + -8 * stage3Output + 4 * stage4Output;
        }
        
        //! Write and read the band-pass output
        T writeAndReadBandPass(const T& x)
        {
            write(x);
            return readBandPass();
        }
        
        //! Read the high-pass output
        T readHighPass()
        {
            return ladderInput + -4 * stage1Output + 6 * stage2Output + -4 * stage3Output + stage4Output;
        }
        
        //! Write and read the high-pass output
        T writeAndReadHighPass(const T& x)
        {
            write(x);
            return readHighPass();
        }
        
        //! Read the second-order low-pass output
        T readSecondOrderLowPass()
        {
            return stage2Output;
        }
        
        //! Write and read the second-order low-pass output
        T writeAndReadSecondOrderLowPass(const T& x)
        {
            write(x);
            return readSecondOrderLowPass();
        }
        
        //! Read the second-order band-pass output
        T readSecondOrderBandPass()
        {
            return 2 * stage1Output + -2 * stage2Output;
        }
        
        //! Write and read the second-order band-pass output
        T writeAndReadSecondOrderBandPass(const T& x)
        {
            write(x);
            return readSecondOrderBandPass();
        }
        
        //! Read the second-order high-pass output
        T readSecondOrderHighPass()
        {
            return ladderInput -2 * stage1Output + stage2Output;
        }
        
        //! Write and read the second-order high-pass output
        T writeAndReadSecondOrderHighPass(const T& x)
        {
            write(x);
            return readSecondOrderHighPass();
        }
        
        //! Set a function for non-linear processing (or nullptr for linear)
        void setNonLinear(std::function<T(T)> nonLinear)
        {
            this->nonLinear = nonLinear;
            stage1.nonLinear = nonLinear;
            stage2.nonLinear = nonLinear;
            stage3.nonLinear = nonLinear;
            stage4.nonLinear = nonLinear;
        }
        
    public:
        //! Pass-band gain compensation
        /*! Compensate for the loss of gain when the feedback factor increases, used in ARP filter models */
        bool passBandGain = false;
        
    private:
        //! Low-pass output state
        T lowPassOutput = 0;
        
        //! The input state before the first stage of the ladder
        T ladderInput = 0;
        
        //! The output state of the first stage
        T stage1Output = 0;
        
        //! The output state of the second stage
        T stage2Output = 0;
        
        //! The output state of the third stage
        T stage3Output = 0;
        
        //! The output state of the fourth stage, the 4th order low-pass
        T stage4Output = 0;
        
        //! Low-pass filter stage 1
        AnalogOnePoleFilter<T> stage1;
        
        //! Low-pass filter stage 2
        AnalogOnePoleFilter<T> stage2;
        
        //! Low-pass filter stage 3
        AnalogOnePoleFilter<T> stage3;
        
        //! Low-pass filter stage 4
        AnalogOnePoleFilter<T> stage4;
        
        //! Feedback factor
        T feedbackFactor = 1;
        
        //! Feedback factor for first low-pass filter stage
        T feedbackFactorLP1 = 0;
        
        //! Feedback factor for second low-pass filter stage
        T feedbackFactorLP2 = 0;
        
        //! Feedback factor for third low-pass filter stage
        T feedbackFactorLP3 = 0;
        
        //! Feedback factor for fourth low-pass filter stage
        T feedbackFactorLP4 = 0;
        
        //! Filter gain factor with resolved zero delay feedback
        T gainFactor = 0;
        
        //! Function for non-linear processing
        std::function<T(T)> nonLinear;
    };
}

#endif /* AnalogLadderFilter_hpp */
