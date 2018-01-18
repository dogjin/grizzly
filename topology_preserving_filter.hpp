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
    /*! @brief Abstract base class for a topology preserving filter
     *
     *  @discussion An abstract base class to construct a
     *  topology preserved filter type. When deriving from this
     *  class, the write() and setCoefficients() calls need to
     *  be overriden.
     */
    template <typename T>
    class TopologyPreservingFilter
    {
    public:
        //! Construct a filter given a sample-rate
        TopologyPreservingFilter(double sampleRate_Hz) :
        sampleRate_Hz(sampleRate_Hz)
        {
        }
        
        virtual ~TopologyPreservingFilter() = default;
        
        //! Write a sample to the filter
        virtual void write(T x) = 0;
        
        //! Set the sample-rate
        void setSampleRate(double sampleRate_Hz)
        {
            if (this->sampleRate_Hz == sampleRate_Hz)
                return;
            
            this->sampleRate_Hz = sampleRate_Hz;
            
            setCoefficients(sampleRate_Hz, cutOff_Hz, resonance);
        }
        
        //! Set the cut-off
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
        
        //! Set the cut-off and resonance
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
        //! Set the coefficients given a sample-rate cut-off and resonance
        virtual void setCoefficients(double sampleRate_Hz, double cutOff_Hz, double resonance) = 0;
        
        //! Take over the coefficients from another filter
        void copyBaseCoefficients(const TopologyPreservingFilter<T>* rhs)
        {
            sampleRate_Hz = rhs->sampleRate_Hz;
            cutOff_Hz = rhs->cutOff_Hz;
            resonance = rhs->resonance;
            gainFactor = rhs->gainFactor;
        }
        
    protected:
        //! The sample-rate
        double sampleRate_Hz = 0;
        
        //! The cut-off
        double cutOff_Hz = 0;
        
        //! The resonance factor
        double resonance = 0;
        
        //! The gain factor for delay free loops
        double gainFactor = 0;
    };
}
