//
//  integrator.hpp
//  Grizzly
//
//  Created by Milan van der Meer on 05/12/2017.
//

#pragma once


namespace dsp
{
    /*! @brief Forward Euler integrator
     *
     *  @discussion Forward Euler integrator. In
     *  contrary to the reverse version, the output
     *  does not include the current input.
     *
     *  @see ReverseEulerIntegrator TrapezoidalIntegrator
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
            state += (gain * x);
            
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
        
        //! The gain factor
        double gain = 1;
    };
    
    /*! @brief Reverse Euler integrator
     *
     *  @discussion Reverse Euler integrator. In
     *  contrary to the forward version, the output
     *  also included the current input.
     *
     *  @see ForwardEulerIntegrator TrapezoidalIntegrator
     */
    template <class T>
    class ReverseEulerIntegrator
    {
    public:
        //! Integrate the input
        T process(T x)
        {
            // update the state
            state += (gain * x);
            
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
        
        //! The gain factor
        double gain = 1;
    };
    
    /*! @brief Trapezoidal integrator
     *
     *  @discussion Trapezoidal integrator in a
     *  transposed direct form II. The gain element
     *  is placed at the input and has a default value
     *  of 0.5. This integrator is useful for builing
     *  filters.
     *
     *  @see ForwardEulerIntegrator ReverseEulerIntegrator
     *  TopologyPreservingOnePoleFilter
     */
    template <class T>
    class TrapezoidalIntegrator
    {
    public:
        //! Integrate the input
        T process(T x)
        {
            // multiply the input with the gain coefficient
            gainedInput = x * gain;
            
            // form the output by adding the state to the input
            y = gainedInput + state;
            
            // update the state
            state = gainedInput + y;
            
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
        double gain = 0.5;
        
    private:
        // The output
        T y = 0;
        
        // The input multiplied with the gain factor
        T gainedInput = 0;
    };
}
