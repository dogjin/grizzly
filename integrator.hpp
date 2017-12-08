//
//  integrator.hpp
//  Grizzly
//
//  Created by Milan van der Meer on 05/12/2017.
//

#pragma once


namespace dsp
{
    /*! @brief Integrator
     *
     *  @discussion
     *
     */
    template <class T>
    class ForwardEulerIntegrator
    {
    public:
        //! Integrate the input
        T process(T x)
        {
            // the output is the previous state
            const auto y = state;
            
            // update the state
            state += (g * x);
            
            // return the output
            return y;
        }
        
        //! Integrate the input
        T operator()(const T& x)
        {
            return process(x);
        }
        
    public:
        //! The previous value, state
        T state = 0;
        
        //! The gain value
        double g = 1;
    };
    
    template <class T>
    class ReverseEulerIntegrator
    {
    public:
        //! Integrate the input
        T process(T x)
        {
            // update the state
            state += (g * x);
            
            // return state as output (previous output + input)
            return state;
        }
        
        //! Integrate the input
        T operator()(const T& x)
        {
            return process(x);
        }
        
    public:
        //! The previous value, state
        T state = 0;
        
        //! The gain value
        double g = 1;
    };
    
    //! in comment zo zetten dat dit transposed direct form 2 is!!
    template <class T>
    class TrapezoidalIntegrator
    {
    public:
        //! Integrate the input
        T process(T x)
        {
            // multiply the input with the gain coefficient
            x *= g;
            
            // form the output by adding the state to the input
            const auto y = x + state;
            
            // update the stae
            state = x + y;
            
            return y;
        }
        
        //! Integrate the input
        T operator()(const T& x)
        {
            return process(x);
        }
        
    public:
        //! The previous value, state
        T state = 0;
        
        //! The gain value
        double g = 0.5;
    };
}