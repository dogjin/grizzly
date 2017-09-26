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

#include <iostream>
using namespace std;


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
            cout << id << "  " << index << endl;
            unwrappedPhase = phase + increment_;
            
            phase = math::wrap<long double>(unwrappedPhase, 0, 1);
            
            for (auto& slave : slaves)
                slave->increment();
            
            // if true, then slaves must sync
            if (master && master->unwrappedPhase >= 1.0l)
            {
                resetForSync(master);
            }
            
            adjustForSync();
            y = convertPhaseToY();
            
            index++;
        }
        
        void setIncrement(long double increment)
        {
            this->increment_ = increment;
        }
        
        void setIncrement(unit::hertz<float> frequency, unit::hertz<float> sampleRate)
        {
            setIncrement(frequency.value / sampleRate.value);
        }
        
        long double getIncrement() const
        {
            return increment_;
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
        
        void setPhaseOffset(long double offset)
        {
            phaseOffset = offset;
        }
        
        long double getPhaseOffset() const
        {
            return phaseOffset;
        }
        
        template <typename Slave, typename... Args>
        Slave& createSlave(Args&&... args)
        {
            return dynamic_cast<Slave&>(addSlave(std::make_unique<Slave>(std::forward<Args&&>(args)...)));
        }
        
        PhaseGenerator& addSlave(std::unique_ptr<PhaseGenerator> slave)
        {
            if (slave->master != nullptr)
                throw std::runtime_error("slave already has a master!");
            
            slave->master = this;
            slaves.emplace_back(std::move(slave));
            return *slaves.back();
        }
        
        const PhaseGenerator* getMaster() const { return master; }
        
        bool hasMaster() const { return master == nullptr ? false : true; }
        
        //        long double getUnwrappedPhase() const { return unwrappedPhase; }
        
    protected:
        //! The current phase of the saw (ranged from 0 to 1)
        long double phase = 0;
        
        long double unwrappedPhase = phase;
        
        long double increment_ = 0;
        
        long double phaseOffset = 0;
        
        //! The to be returned value from read
        T y = 0;
        
    private:
        virtual void adjustForSync() { }
        
        //! Recompute the most recently computed value
        virtual T convertPhaseToY() = 0;
        
        void resetForSync(PhaseGenerator* master)
        {
            const auto ratio = increment_ / master->getIncrement();
            setPhase(master->getPhase() * ratio);
            
            for (auto& slave : slaves)
                slave->resetForSync(master);
        }
        
    private:
        PhaseGenerator* master = nullptr;
        
        std::vector<std::unique_ptr<PhaseGenerator>> slaves;
        
    public:
        std::string id;
        int index = 0;
    };
}

#endif /* GRIZZLY_PHASE_GENERATOR_HPP */

