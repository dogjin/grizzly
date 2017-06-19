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

#ifndef GRIZZLY_ENVELOPE_FOLLOWER_HPP
#define GRIZZLY_ENVELOPE_FOLLOWER_HPP

#include <cmath>
#include <dsperados/math/utility.hpp>
#include <stdexcept>

#include "biquad.hpp"
#include "biquad_coefficients.hpp"
#include "first_order_filter.hpp"
#include "first_order_filter_coefficients.hpp"

namespace dsp
{
    //! AttackReleaseEnvelopeFollower
    /*! Abstract base class for envelope followers that use an attack and release time */
    template <class T, class CoeffType = T>
    class AttackReleaseEnvelopeFollower
    {
    public:
        //! Construct the Follower
        AttackReleaseEnvelopeFollower(unit::hertz<float> sampleRate, float timeConstantFactor = 5)
        {
            setSampleRate(sampleRate);
            setTimeConstantFactor(timeConstantFactor);
        }
        
        //! Virtual destructor
        virtual ~AttackReleaseEnvelopeFollower() = default;
        
    public:
        //! Set the sample rate
        void setSampleRate(unit::hertz<float> sampleRate)
        {
            this->sampleRate = sampleRate;
            lowPassOnePole(attackCoefficients, sampleRate, attackTime, timeConstantFactor);
        }
        
        //! Set the time-constant factor
        void setTimeConstantFactor(float factor)
        {
            timeConstantFactor = factor;
            lowPassOnePole(attackCoefficients, sampleRate, attackTime, timeConstantFactor);
        }
        
        //! Set the attack time
        virtual void setAttackTime(unit::second<float> attackTime)
        {
            this->attackTime = attackTime;
            
            if (attackTime.value > 0)
                lowPassOnePole(attackCoefficients, sampleRate, attackTime, timeConstantFactor);
            else
                throughPass(attackCoefficients);
        }
        
        //! Set the release time
        virtual void setReleaseTime(unit::second<float> releaseTime)
        {
            this->releaseTime = releaseTime;
            
            if (releaseTime.value > 0)
                lowPassOnePole(releaseCoefficients, sampleRate, releaseTime, timeConstantFactor);
            else
                throughPass(releaseCoefficients);
        }
        
        //! Write the new input to the follower
        virtual void write(const T& x) = 0;
        
        //! Read the last computed value
        virtual T read() const = 0;
        
        //! Write a new sample and read the output (in that order)
        T writeAndRead(const T& x)
        {
            write(x);
            return read();
        }
        
        //! Set the envelope state
        virtual void setState(const T& y) = 0;
        
        //! Set the envelope state to zero
        void reset()
        {
            setState(0);
        }
        
    protected:
        //! The attack coefficients
        FirstOrderCoefficients<CoeffType> attackCoefficients;
        
        //! The release coefficients
        FirstOrderCoefficients<CoeffType> releaseCoefficients;
        
        //! The sample rate
        unit::hertz<float> sampleRate = 44100;
        
        //! Time constant factor
        float timeConstantFactor = 5;
        
        //! The attack time
        unit::second<float> attackTime = 1;
        
        //! The release time
        unit::second<float> releaseTime = 1;
        
        //! The most recently computed value
        T y = 0;
    };
    
    //! Envelope detector based on an analog circuit with two resistors, a capacitor and a diode
    /*! The capacitor continuously discharges via the release resistor. Due to this design, the peak will not
     reach its maximal value. choosing a bigger release time makes this less noticeably. See "Investigation
     in Dynamic Range Compression" by Massberg. */
    template <class T, class CoeffType = T>
    class EnvelopeFollowerRCR : public AttackReleaseEnvelopeFollower<T, CoeffType>
    {
    public:
        //! Construct the Follower
        using AttackReleaseEnvelopeFollower<T, CoeffType>::AttackReleaseEnvelopeFollower;
        
        //! Write the new input to the follower
        void write(const T& x) final override
        {
            this->y = static_cast<T>(-this->releaseCoefficients.b1 * this->y + this->attackCoefficients.a0 * std::max<T>(x - this->y, 0));
        }
        
        //! Read the last computed value
        T read() const final override
        {
            return this->y;
        }
        
        //! Set the envelope state
        void setState(const T& y) final override
        {
            this->y = y;
        }
    };
    
    
    //! Envelope detector based on an analog circuit with two resistor, two capacitors and a diode
    /*! The first set of resistor/capacitor is decoupled from the second set. In contrast to the EnvelopeDetectorRCR,
     the envelope will reach the maximal value regardless of different release settings. See "Investigation in Dynamic
     Range Compression" by Massberg. */
    template <class T, class CoeffType = T>
    class EnvelopeFollowerDecoupled : public AttackReleaseEnvelopeFollower<T, CoeffType>
    {
    public:
        //! Construct the Follower
        using AttackReleaseEnvelopeFollower<T, CoeffType>::AttackReleaseEnvelopeFollower;
        
        //! Write the new input to the follower
        void write(const T& x) final override
        {
            yRelease = std::max<T>(x, yRelease - this->releaseCoefficients.a0 * yRelease);
            this->y += this->attackCoefficients.a0 * (yRelease - this->y);
        }
        
        //! Read the last computed value
        T read() const final override
        {
            return this->y;
        }
        
        //! Set the envelope state
        void setState(const T& y) final override
        {
            this->y = y;
        }
        
    private:
        //! Intermediate release state
        T yRelease = 0;
    };
    
    //! Envelope detector using a one-pole low-pass filter
    /*! Notice the input is not rectified */
    template <class T, class CoeffType = T>
    class EnvelopeFollowerOnePole : public AttackReleaseEnvelopeFollower<T, CoeffType>
    {
    public:
        //! Construct the Follower
        EnvelopeFollowerOnePole(unit::hertz<float> sampleRate, bool releaseToZero, float timeConstantFactor = 5) :
            AttackReleaseEnvelopeFollower<T, CoeffType>(sampleRate, timeConstantFactor),
            releaseToZero(releaseToZero)
        {
        }
        
        //! Write the new input to the follower
        void write(const T& x) final override
        {
            if (x > lowPassFilter.read())
            {
                lowPassFilter.coefficients = this->attackCoefficients;
                lowPassFilter.write(x);
            }
            else
            {
                lowPassFilter.coefficients = this->releaseCoefficients;
                lowPassFilter.write(releaseToZero ? 0 : x);
            }
        }
        
        //! Read the last computed value
        T read() const final override
        {
            return lowPassFilter.read();
        }
        
        //! Set the envelope state
        void setState(const T& y) final override
        {
            lowPassFilter.setState(y);
        }
        
    public:
        //! Boolian for release mode
        /*! When true, the follower reaches for zero in its release state, otherwise the follower reaches to the input */
        bool releaseToZero = false;
        
    private:
        //! First order filter
        FirstOrderFilter<T, CoeffType> lowPassFilter;
    };
    
    //! Envelope detector using a biquad low-pass filter
    /*! Notice the input is not rectified */
    template <class T, class CoeffType = T>
    class EnvelopeFollowerBiquad : public AttackReleaseEnvelopeFollower<T, CoeffType>
    {
    public:
        //! Construct the Follower
        EnvelopeFollowerBiquad(unit::hertz<float> sampleRate, bool releaseToZero, float timeConstantFactor = 5) :
            AttackReleaseEnvelopeFollower<T, CoeffType>(sampleRate, timeConstantFactor),
            releaseToZero(releaseToZero)
        {
        }
        
        //! Write the new input to the follower
        void write(const T& x) final override
        {
            if (x > lowPassFilter.read())
            {
                lowPassFilter.coefficients = this->attackCoefficients;
                lowPassFilter.write(x);
            }
            else
            {
                lowPassFilter.coefficients = this->releaseCoefficients;
                lowPassFilter.write(releaseToZero ? 0 : x);
            }
        }
        
        //! Read the last computed value
        T read() const final override
        {
            return lowPassFilter.read();
        }
        
        //! Set the envelope state
        void setState(const T& y) final override
        {
            lowPassFilter.setState(y);
        }
        
        //! Set the attack time
        void setAttackTime(unit::second<float> attackTime) override
        {
            this->attackTime = attackTime;
            
            if (attackTime.value > 0)
                lowPass(attackCoefficients, this->sampleRate, attackTime, 0.5);
            else
                throughPass(attackCoefficients);
        }
        
        //! Set the release time
        void setReleaseTime(unit::second<float> releaseTime) override
        {
            this->releaseTime = releaseTime;
            
            if (releaseTime.value > 0)
                lowPass(releaseCoefficients, this->sampleRate, releaseTime, 0.5);
            else
                throughPass(releaseCoefficients);
        }
        
    public:
        //! Boolian for release mode
        /*! When true, the follower reaches for zero in its release state, otherwise the follower reaches to the input */
        bool releaseToZero = false;
        
    private:
        //! First order filter
        BiquadDirectFormI<T, CoeffType> lowPassFilter;
        
        BiquadCoefficients<CoeffType> attackCoefficients;
        BiquadCoefficients<CoeffType> releaseCoefficients;
    };
}

#endif /* GRIZZLY_ENVELOPEDETECTOR_HPP */
