//
//  sallen_key_filter.hpp
//  Grizzly
//
//  Created by Milan van der Meer on 27/11/2017.
//

#pragma once

#include <cassert>
#include <functional>

#include "topology_preserving_filter.hpp"
#include "topology_preserving_one_pole_filter.hpp"

namespace dsp
{
    template <typename T>
    class SallenKeyFilter :
    public TopologyPreservingFilter<T>
    {
    public:
        SallenKeyFilter(double sampleRate_Hz) :
        TopologyPreservingFilter<T>(sampleRate_Hz)
        {
            this->resonance = 0.00000001;
        }
        
        T writeAndRead(T x)
        {
            this->write(x);
            return read();
        }
        
        T read() const
        {
            return this->y;
        }
        
        void setState(T state1, T state2, T state3)
        {
            onePole1.setState(state1);
            onePole2.setState(state2);
            onePole3.setState(state3);
        }
        
        void copyCoefficients(const SallenKeyFilter& rhs)
        {
            this->copyBaseCoefficients(&rhs);
                        
            onePole1.copyCoefficients(rhs.onePole1);
            onePole2.copyCoefficients(rhs.onePole2);
            onePole3.copyCoefficients(rhs.onePole3);
            feedbackFactorPole2 = rhs.feedbackFactorPole2;
            feedbackFactorPole3 = rhs.feedbackFactorPole3;
            resonanceReciprocal = rhs.resonanceReciprocal;
        }
        
    protected:
        T y = 0;
        
        TopologyPreservingOnePoleFilter<T> onePole1;
        TopologyPreservingOnePoleFilter<T> onePole2;
        TopologyPreservingOnePoleFilter<T> onePole3;
        
        double feedbackFactorPole2 = 0;
        double feedbackFactorPole3 = 0;
        
        double resonanceReciprocal = 0;
        
    private:
        void setCoefficients(double sampleRate_Hz, double cutOff_Hz, double resonance) final
        {
            assert(resonance > 0);
            
            onePole1.setCoefficients(cutOff_Hz, sampleRate_Hz);
            onePole2.copyCoefficients(onePole1);
            onePole3.copyCoefficients(onePole1);
            
            const auto g = onePole1.warpedCutOff();
            const auto gain = onePole1.gain();
            
            resonanceReciprocal = 1.0 / resonance;
            
            this->gainFactor = 1.0 / (1.0 - resonance * gain + resonance * gain * gain);
            
            setFeedBackFactors(g, gain, resonance);
        }
        
        virtual void setFeedBackFactors(double g, double gain, double resonance) = 0;
    };
    
    
    template <typename T>
    class SallenKeyLowPass :
    public SallenKeyFilter<T>
    {
    public:
        SallenKeyLowPass(double sampleRate_Hz) :
        SallenKeyFilter<T>(sampleRate_Hz)
        {
        }
        
        void write(T x) final
        {
            T lowPass2Input = this->gainFactor * (this->onePole1.writeAndReadLowPass(x) + this->onePole2.state() * this->feedbackFactorPole2 + this->onePole3.state() * this->feedbackFactorPole3);
            
            if (this->nonLinear)
                lowPass2Input = this->nonLinear(lowPass2Input);
            
            this->y = this->resonance * this->onePole2.writeAndReadLowPass(lowPass2Input);
            
            this->onePole3.write(this->y);
            
            this->y *= this->resonanceReciprocal;
        }
        
    private:
        void setFeedBackFactors(double g, double gain, double resonance) final
        {
            this->feedbackFactorPole2 = (resonance - resonance * gain) / (1.0 + g);
            this->feedbackFactorPole3 = -1.0 / (1.0 + g);
        }
    };
    
    
    template <typename T>
    class SallenKeyHighPass :
    public SallenKeyFilter<T>
    {
    public:
        SallenKeyHighPass(double sampleRate_Hz) :
        SallenKeyFilter<T>(sampleRate_Hz)
        {
        }
        
        void write(T x) final
        {
            this->y = (this->onePole1.writeAndReadHighPass(x) + this->onePole2.state() * this->feedbackFactorPole2 + this->onePole3.state() * this->feedbackFactorPole3) * this->gainFactor * this->resonance;
            
            if (this->nonLinear)
                this->y = this->nonLinear(this->y);
            
            this->onePole3.write(this->onePole2.writeAndReadHighPass(this->y));
            
            this->y *= this->resonanceReciprocal;
        }
        
    private:
        void setFeedBackFactors(double g, double gain, double resonance) final
        {
            this->feedbackFactorPole2 = -gain / (1.0 + g);
            this->feedbackFactorPole3 = 1.0 / (1.0 + g);
        }
    };
}
