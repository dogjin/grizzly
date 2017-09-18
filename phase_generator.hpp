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

#ifndef GRIZZLY_PHASE_GENERATOR_HPP
#define GRIZZLY_PHASE_GENERATOR_HPP

#include <functional>

#include <moditone/math/wrap.hpp>
#include <moditone/unit/hertz.hpp>

namespace dsp
{
    //! Generates a waveform using an incrementable phase
    template <typename T>
    class PhaseGenerator
    {
    public:
        //! Virtual destructor
        virtual ~PhaseGenerator() = default;
        
        virtual void increment()
        {
            phase = math::wrap<long double>(phase += increment_, 0, 1);
            y = convertPhaseToY();
        }
        
        void setIncrement(long double increment)
        {
            this->increment_ = increment;
        }
        
        void setIncrement(unit::hertz<float> frequency, unit::hertz<float> sampleRate)
        {
            setIncrement(frequency.value / sampleRate.value);
        }
        
        //! Read the most recently computed output
        T read() const
        {
            return y;
        }
        
        //! Change the phase manually
        void setPhase(long double phase)
        {
            this->phase = math::wrap<long double>(phase, 0, 1);
            y = convertPhaseToY();
        }
        
        //! Return the current phase between 0 and 1
        long double getPhase() const
        {
            return phase;
        }
        
        long double getIncrement() const
        {
            return increment_;
        }
        
        void setPhaseOffset(long double offset)
        {
            phaseOffset = offset;
        }
        
        long double getPhaseOffset() const
        {
            return phaseOffset;
        }
        
    protected:
        //! The current phase of the saw (ranged from 0 to 1)
        long double phase = 0;
        
        long double increment_ = 0;
        
        long double phaseOffset = 0;
        
        //! The to be returned value from read
        T y = 0;
        
    private:
        //! Recompute the most recently computed value
        virtual T convertPhaseToY() = 0;
    };
    
    
    
    template <typename T>
    class BlepSlave : public PhaseGenerator<T>
    {
    public:
        //! Virtual destructor
        virtual ~BlepSlave() = default;
        
    public:
        T adjustValue = 0;
        
        virtual void afterReset(long double masterPhase, long double masterIncrement) = 0;
        virtual void beforeReset(long double masterPhase, long double masterIncrement) = 0;
    };
    
    
    
    template <typename T>
    class BlepMaster : public PhaseGenerator<T>
    {
    public:
        //! Virtual destructor
        virtual ~BlepMaster() = default;
        
        //! Increment the generator
        void increment()
        {
            // increment the master
            this->phase = math::wrap<long double>(this->phase += this->increment_, 0, 1);
            
            this->y = convertPhaseToY();
            
            // if there're no slaves, return
            if (slavess.empty())
                return;
            
            // increment the slaves
            for (auto& slave : slavess)
            {
                // Check if we are before or after a reset
                if (this->phase < this->increment_)
                    slave->afterReset(this->phase, this->increment_);
                else if (this->phase > 1.0l - this->increment_)
                    slave->beforeReset(this->phase, this->increment_);
                
                slave->increment();
            }
        }
        
    public:
        std::vector<std::unique_ptr<BlepSlave<T>>> slavess;
        
    private:
        //! Recompute the most recently computed value
        virtual T convertPhaseToY() = 0;
    };
}

#endif /* GRIZZLY_PHASE_GENERATOR_HPP */
