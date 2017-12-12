//
//  topology_preserving_filter.hpp
//  Grizzly
//
//  Created by Milan van der Meer on 04/12/2017.
//

#pragma once

#include <cmath>
#include <moditone/math/constants.hpp>
#include <functional>


namespace dsp
{
    template <typename T>
    class TopologyPreservingFilter
    {
    public:
        TopologyPreservingFilter(double sampleRate_Hz) :
        sampleRate_Hz(sampleRate_Hz)
        {
        }
        
        virtual void write(T x) = 0;
        
        void setSampleRate(double sampleRate_Hz)
        {
            if (this->sampleRate_Hz == sampleRate_Hz)
                return;
            
            this->sampleRate_Hz = sampleRate_Hz;
            
            setCoefficients(sampleRate_Hz, cutOff_Hz, resonance);
        }
        
        void setCutOff(double cutOff_Hz)
        {
            if (this->cutOff_Hz == cutOff_Hz)
                return;
            
            this->cutOff_Hz = cutOff_Hz;
            
            setCoefficients(sampleRate_Hz, cutOff_Hz, resonance);
        }
        
        //! Set the feedback factor for a resonance peak
        virtual void setResonance(float resonance)
        {
            if (this->resonance == resonance)
                return;
            
            this->resonance = resonance;
            
            setCoefficients(sampleRate_Hz, cutOff_Hz, resonance);
        }
        
        void setCutOffAndResonance(double cutOff_Hz, double resonance)
        {
            if (this->cutOff_Hz == cutOff_Hz &&
                this->resonance == resonance)
                return;
            
            this->cutOff_Hz = cutOff_Hz;
            this->resonance = resonance;
            
            setCoefficients(sampleRate_Hz, cutOff_Hz, resonance);
        }
        
    public:
        //! Function for non-linear processing
        std::function<T(const T&)> nonLinear;
        
    protected:
        virtual void setCoefficients(double sampleRate_Hz, double cutOff_Hz, double resonance) = 0;
        
    protected:
        double sampleRate_Hz = 0;
        
        double cutOff_Hz = 0;
        
        double resonance = 0;
    };
}
