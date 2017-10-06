//
//  analog_envelope.hpp
//  Grizzly
//
//  Created by Milan van der Meer on 06/10/2017.
//

# pragma once

#include <cmath>
#include <functional>

#include <moditone/unit/time.hpp>
#include <moditone/grizzly/first_order_filter.hpp>
#include <moditone/math/clamp.hpp>

namespace dsp
{
    //! Analog style ADSR envelope generator
    /*! Envelope generator based on a charging and discharging a capacitor.
     The curve determines the skew factor (slope) of the attack (see setAttackSkew for more info).
     By tweaking the ADSR parameters, a AR, ASR or ADSD envelope is possible. */
    template <class T, class CoeffType = float>
    class AnalogEnvelope
    {
    public:
        //! The states in which the envelope can be at any given moment
        enum class State
        {
            IDLE,
            ATTACK,
            DECAY,
            RELEASE
        };
        
    public:
        //! Construct the envelope
        AnalogEnvelope(float sampleRate, unit::second<float> attackTime, unit::second<float> decayTime, T sustain, unit::second<float> releaseTime, float attackShape = 0.77) :
        sampleRate(sampleRate)
        {
            // Initialize the constant factors per stage
            decayStage.timeConstantFactor = 4.95;
            releaseStage.timeConstantFactor = 4.95;
            
            this->sustain = sustain;
            attackStage.time = attackTime;
            setAttackShape(attackShape); // sets attack coefficients and adjusts sustain
            
            setDecayTime(decayTime);
            setReleaseTime(releaseTime);
        }
        
        //! Set the shape, from flat to steap, of the attack
        /*! see Stage::timeConstantReciprocal for more information */
        void setAttackShape(float maximumCharge)
        {
            maximumCharge = math::clamp<float>(maximumCharge, 0.1f, 0.99f);
            this->maximumCharge = maximumCharge;
            normalizeFactor = 1.0 / maximumCharge;
            
            // solve time constant factor for maximum charge = 1 - e^-timeConstantFactor
            long double temp = 1.l - maximumCharge;
            attackStage.timeConstantFactor = -log(temp);
            setAttackTime(attackStage.time);
            setSustain(sustain);
        }
        
        //! Set the attack time
        void setAttackTime(unit::second<float> time)
        {
            attackStage.set(time, sampleRate);
            
            if (state == State::ATTACK)
                updateFilterCoefficients();
        }
        
        //! Set the decay time
        void setDecayTime(unit::second<float> time)
        {
            decayStage.set(time, sampleRate);
            
            if (state == State::DECAY)
                updateFilterCoefficients();
        }
        
        
        //! Set the release time
        void setReleaseTime(unit::second<float> time)
        {
            releaseStage.set(time, sampleRate);
            
            if (state == State::RELEASE)
                updateFilterCoefficients();
        }
        
        //! Set the sustain level
        void setSustain(float sustain)
        {
            this->sustain = math::clamp(sustain, 0.0f, 1.0f);
            //
            //            // multiply with maximum charge to correct for normalizing
            //            adjustedSustain = this->sustain * maximumCharge;
        }
        
        //! Start the envelop by setting the mode to attack
        void start()
        {
            state = State::ATTACK;
            updateFilterCoefficients();
        }
        
        //! End the envelope by setting the mode to release
        void release()
        {
            if (state != State::IDLE)
                state = State::RELEASE;
            
            updateFilterCoefficients();
        }
        
        //! Sets the envelope to 0 and goes to idle state
        void reset()
        {
            state = State::IDLE;
            lowPassFilter.reset();
        }
        
        //! Returns the output of the envelope
        /*! A call to start() sets the envelope to the attack stage
         After the release() call, the envelope will shut down when the output is significantly low and returns 0. */
        T process()
        {
            T y = 0;
            
            switch (state)
            {
                case State::IDLE:
                {
                    break;
                }
                    
                case State::ATTACK:
                {
                    if (attackStage.time.value <= 0)
                    {
                        y = 1;
                        lowPassFilter.setState(y);
                        state = State::DECAY;
                        updateFilterCoefficients();
                        break;
                    }
                    
                    auto previousY = lowPassFilter.read();
                    y = lowPassFilter.writeAndRead(gateOn);
                    if (y > maximumCharge)
                    {
                        state = State::DECAY;
                        updateFilterCoefficients();
                        
                        // Figure out where the attack exactly intersects the maximalCharge value
                        // Set the previousY value as x = 0 (y-axis intersept)
                        // Solve x for attack curve 1 - e^(-x * TCF / attackTime * sampleRate) + previousY = maximalCharge
                        // Value x should be between 0 and 1
                        double tc = attackStage.time.value * sampleRate;
                        double tmp = (y - previousY - 1) * -1;
                        tmp = log(tmp) * tc;
                        double x = tmp / -attackStage.timeConstantFactor;
                        
                        // at the maximalCharge intersept, the true y value = 1
                        // The x can be plugged in the decay curve e^(-x * TCF / attackTime * sampleRate)
                        // We should now have the exact y value
                        y = std::exp( (-x * decayStage.timeConstantFactor) / (decayStage.time.value * sampleRate));
                        lowPassFilter.setState(y);
                        break;
                    }
                    
                    y *= normalizeFactor;
                    break;
                }
                    
                case State::DECAY:
                {
                    y = lowPassFilter.writeAndRead(sustain);// * normalizeFactor;
                    break;
                }
                    
                case State::RELEASE:
                {
                    y = lowPassFilter.writeAndRead(gateOff);// * normalizeFactor;
                    if (y < 0)
                    {
                        reset();
                        if (end)
                            end();
                        
                        y = 0;
                    }
                    break;
                }
            }
            
            return y;
        }
        
        //! Returns the output of the envelope
        /*! A call to start() sets the envelope to the attack stage
         After the release() call, the envelope will shut down when the output is significantly low and returns 0. */
        T operator()() { return process(); }
        
        // Return the current state
        State getState() const { return state; }
        
    public:
        //! Called when the envelope reaches zero. Useful for updating some code elsewhere (e.g. free a voice)
        std::function<void()> end = [](){};
        
    private:
        //! Make sure we're using the correct coefficients
        void updateFilterCoefficients()
        {
            switch (state)
            {
                case State::IDLE: break;
                case State::ATTACK: lowPassFilter.coefficients = attackStage.coefficients; break;
                case State::DECAY: lowPassFilter.coefficients = decayStage.coefficients; break;
                case State::RELEASE: lowPassFilter.coefficients = releaseStage.coefficients; break;
            }
        }
        
    private:
        //! The envelope consists goes through three stages, each needing the same set of variables
        class Stage
        {
        public:
            void set(unit::second<float> time, unit::hertz<float> sampleRate)
            {
                this->time = time;
                lowPassOnePole(coefficients, sampleRate, time, timeConstantFactor);
                
                if (!coefficients.isStable())
                    throughPass(coefficients);
            }
            
        public:
            //! The filter coefficients set to function as the state
            FirstOrderCoefficients<CoeffType> coefficients;
            
            unit::second<float> time = 0;
            
            //! The shape of the filter curve is determined by the maximum charge of a 'capacitor' (0.1 - 0.99)
            /*! By default, the maximum charge is 77% and approximates a CEM3310 chip.
             The output (y) of charging is given by: y(n) = 1 - e^(-n/RC), where time constant RC is set to 1 and time n is in samples.
             At n = 1, y = 0.63. We need 5 samples to a full charge of 0.99. If we replace n with -5, we can go from 0 to 0.99 in one tick (1 - e^-5)
             and use this value to set a maximum charge. In this case 5 is the time constant factor to make this step in one sample.
             We can then solve time constant factors for different charges between 0% and 100% with maximum charge = 1 - e^-factor.
             If we then normalize the envelope, the maximum charge results in changing the filter shape. */
            double timeConstantFactor = 0;
        };
        
    private:
        //! The output of the envelope is generated using a simple low-pass filter
        FirstOrderFilter<T, CoeffType> lowPassFilter;
        
        //! The envelope consists of three different stages, each with their own coefficients
        Stage attackStage;
        Stage decayStage;
        Stage releaseStage;
        
        //! The Envelope State
        State state = State::IDLE;
        
        //! The sample rate at which the envelope runs
        float sampleRate;
        
        //! Maximum charge for the capacitor
        float maximumCharge = 0.77;
        
        //! The normalise factor
        float normalizeFactor = 1;
        
        //! The sustain level
        float sustain = 0;
        
        //! A gate signal of 1
        const float gateOn = 1;
        
        //! A gate signal of 0
        const float gateOff = -0.0001;
    };
}
