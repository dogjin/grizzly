//
//  DelayedFilter.hpp
//  octoSandbox1
//
//  Created by Vincent Pruijn on 01/11/16.
//  Copyright © 2016 Dsperados. All rights reserved.
//

#ifndef GRIZZLY_DELAYED_FILTER_HPP
#define GRIZZLY_DELAYED_FILTER_HPP


#include <cstddef>
#include <functional>
#include <numeric>
#include <vector>

#include "Delay.hpp"
#include "Filter.hpp"


namespace bear::dsp
{
    //! Delayed Filter
    template <class T>
    class DelayedFilter : public dsp::Filter<T>
    {
        
    public:
        //! Create a CombFilter
        DelayedFilter (const std::size_t maxDelay, const double delayTime) :
        maxDelay(maxDelay),
        delay(maxDelay),
        delayTime(delayTime)
        {
            
        }
        
        //! Virtual destructor for polymorhic purposes
        virtual ~DelayedFilter () = default;
        
        //! Resize the delay buffer
        void resize (const std::size_t maxDelay)
        {
            this->maxDelay = maxDelay;
            delay.resize(maxDelay);
        }
        
        //! Get the  time of the delay
        const std::size_t getDelayTime () const
        {
            return delayTime;
        }
        
        //! Get the size of the delay buffer
        const std::size_t getMaxDelaySize () const
        {
            return delay.getMaxDelay();
        }
        
        double delayTime;
        
    protected:
        //! Access the delay
        delayRead(T delayTime)
        {
            return delay.read(delayTime);
        }
        
        //! Access the delay
        delayWrite(T sample)
        {
            return delay.write(sample);
        }
        
    private:
        //! The single delay buffer that will be used for all stages
        dsp::Delay<T> delay;
    };
}


#endif /* GRIZZLY_DELAYED_FILTER_HPP */
