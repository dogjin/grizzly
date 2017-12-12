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

#include "topology_preserving_filter.hpp"
#include "topology_preserving_one_pole_filter.hpp"


namespace dsp
{
    //! Topology preserving 4-pole diode ladder filter with resolved zero delay feedback.
    /*! The ladder filter contains four stages resulting in a slope of 24 dB per octave.
     See "Designing software synthesizer plug-ins in c++" by Will Pirkle.
     See "The Art Of VA Filter Design" by Vadim Zavalishin. */
    template <class T>
    class DiodeLadderFilter :
    public TopologyPreservingFilter<T>
    {
    public:
        DiodeLadderFilter(float sampleRate_Hz) :
        TopologyPreservingFilter<T>(sampleRate_Hz)
        {
        }
        
        //! Write a sample to the filter
        void write(T x) final
        {
            const double S4 = stage4.filter.state * stage4.beta;
            const double addBefore4 = S4 * stage4.epsilon;
            
            const double S3 = (stage3.filter.state + S4 * stage3.delta) * stage3.beta;
            const double addBefore3 = S3 * stage3.epsilon + S4;
            
            const double S2 = (stage2.filter.state + S3 * stage2.delta) * stage2.beta;
            const double addBefore2 = S2 * stage2.epsilon + S3;
            
            const double S1 = (stage1.filter.state + S2 * stage1.delta) * stage1.beta;
            const double addBefore1 = S1 * stage1.epsilon + S2;
            
            const double feedbackSum =
            stage1.feedbackFactor * S1 +
            stage2.feedbackFactor * S2 +
            stage3.feedbackFactor * S3 +
            stage4.feedbackFactor * S4;
            
            ladderInput = (x - this->resonance * feedbackSum) * this->gainFactor;
            
            // Optional non-linear processing
            if (this->nonLinear)
                ladderInput = this->nonLinear(ladderInput);
            
            const T x1 = ladderInput * stage1.gamma + addBefore1;
            stage1(x1 * stage1.a0);
            
            const T x2 = stage1.output * stage2.gamma + addBefore2;
            stage2(x2 * stage2.a0);
            
            const T x3 = stage2.output * stage3.gamma + addBefore3;
            stage3(x3 * stage3.a0);
            
            const T x4 = stage3.output * stage4.gamma + addBefore4;
            stage4(x4 * stage4.a0);
        }
        
        //! Read the low-pass output
        T readLowPass() const
        {
            return stage4.output;
        }
        
        //! Write and read the low-pass output
        T writeAndReadLowPass(const T& x)
        {
            write(x);
            return readLowPass();
        }
        
        //! Read the band-pass output
        T readBandPass() const
        {
            return 4 * stage2.output + -8 * stage3.output + 4 * stage4.output;
        }
        
        //! Write and read the band-pass output
        T writeAndReadBandPass(const T& x)
        {
            write(x);
            return readBandPass();
        }
        
        //! Read the high-pass output
        T readHighPass() const
        {
            return ladderInput + -4 * stage1.output + 6 * stage2.output + -4 * stage3.output + stage4.output;
        }
        
        //! Write and read the high-pass output
        T writeAndReadHighPass(const T& x)
        {
            write(x);
            return readHighPass();
        }
        
        //! Read the second-order low-pass output
        T readSecondOrderLowPass() const
        {
            return stage2.output;
        }
        
        //! Write and read the second-order low-pass output
        T writeAndReadSecondOrderLowPass(const T& x)
        {
            write(x);
            return readSecondOrderLowPass();
        }
        
        //! Read the second-order band-pass output
        T readSecondOrderBandPass() const
        {
            return 2 * stage1.output + -2 * stage2.output;
        }
        
        //! Write and read the second-order band-pass output
        T writeAndReadSecondOrderBandPass(const T& x)
        {
            write(x);
            return readSecondOrderBandPass();
        }
        
        //! Read the second-order high-pass output
        T readSecondOrderHighPass() const
        {
            return ladderInput -2 * stage1.output + stage2.output;
        }
        
        //! Write and read the second-order high-pass output
        T writeAndReadSecondOrderHighPass(const T& x)
        {
            write(x);
            return readSecondOrderHighPass();
        }
        
    private:
        //! The filter stage
        /*! Each stage contains an one-pole filter with a slope of 6 dB per octave. */
        struct Stage
        {
        public:
            //! Calculate the stage output given an input sample
            void operator()(const T& input)
            {
                output = filter.writeAndReadLowPass(input);
            }
            
        public:
            //! The one-pole filter
            TopologyPreservingOnePoleFilter<float> filter;
            
            //! The output of the filter
            T output = 0;
            
            double gamma = 0;
            double a0 = 0;
            double epsilon = 0;
            double beta = 0;
            double delta = 0;
            double G = 0;
            
            //! The feedback factor for the resonance peak
            double feedbackFactor = 0;
        };
        
    private:
        void setCoefficients(double sampleRate_Hz, double cutOff_Hz, double resonance) final
        {
            stage1.filter.setCoefficients(cutOff_Hz, sampleRate_Hz);
            
            const auto g = this->stage1.filter.g;
            const auto gHalf = g * 0.5;
                        
            stage2.filter.copyCoefficients(stage1.filter);
            stage3.filter.copyCoefficients(stage1.filter);
            stage4.filter.copyCoefficients(stage1.filter);
            
            // global G's
            const double G4Denom = (1 + g);
            stage4.G = gHalf / G4Denom;
            
            const double G3Denom = (1 + g - gHalf * stage4.G);
            stage3.G = gHalf / G3Denom;
            
            const double G2Denom = (1 + g - gHalf * stage3.G);
            stage2.G = gHalf / G2Denom;
            
            const double G1Denom = (1 + g - g * stage2.G);
            stage1.G = g / G1Denom;
            
            //// set stage factors
            // a0
            stage1.a0 = 1;
            stage2.a0 = 0.5;
            stage3.a0 = 0.5;
            stage4.a0 = 0.5;
            
            // gamma
            stage1.gamma = 1 + stage1.G * stage2.G;
            stage2.gamma = 1 + stage2.G * stage3.G;
            stage3.gamma = 1 + stage3.G * stage4.G;
            stage4.gamma = 1;
            
            // epsilon
            stage1.epsilon = stage2.G;
            stage2.epsilon = stage3.G;
            stage3.epsilon = stage4.G;
            //stage4.epsilon = 0;
            
            // beta
            stage1.beta = 1 / G1Denom;
            stage2.beta = 1 / G2Denom;
            stage3.beta = 1 / G3Denom;
            stage4.beta = 1 / G4Denom;
            
            // delta
            stage1.delta = g;
            stage2.delta = gHalf;
            stage3.delta = gHalf;
            //stage4.delta = 0;
            
            // feedback-factor
            stage1.feedbackFactor = stage4.G * stage3.G * stage2.G;
            stage2.feedbackFactor = stage4.G * stage3.G;
            stage3.feedbackFactor = stage4.G;
            stage4.feedbackFactor = 1;
            
            this->gainFactor = 1.0 / (1.0 + this->resonance * stage1.feedbackFactor * stage1.G);
        }
        
    private:
        //! Filter stage 1
        Stage stage1;
        
        //! Filter stage 2
        Stage stage2;
        
        //! Filter stage 3
        Stage stage3;
        
        //! Filter stage 4
        Stage stage4;
        
        //! The input state before the first stage of the ladder
        T ladderInput = 0;        
    };
}

