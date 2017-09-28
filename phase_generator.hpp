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

#pragma once

#include <memory>
#include <moditone/math/wrap.hpp>
#include <moditone/unit/hertz.hpp>

#include "poly_blep.hpp"

namespace dsp
{
    //! Generates a waveform using an incrementable phase
    template <typename T>
    class Phasor
    {
    public:
        //! Virtual destructor
        virtual ~Phasor() = default;
        
        virtual void increment()
        {
            incrementUnwrappedPhases();
            
            computeNewPhases();
            
            convertPhasesToYs();
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
        void setPhase(long double phase, bool recomputeY)
        {
            this->phase = math::wrap<long double>(phase, 0, 1);
            
            if (recomputeY)
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
        
        Phasor& addSlave(std::unique_ptr<Phasor> slave)
        {
            if (slave->master != nullptr)
                throw std::runtime_error("slave already has a master!");
            
            slave->master = this;
            slaves.emplace_back(std::move(slave));
            return *slaves.back();
        }
        
        const Phasor* getMaster() const { return master; }
        
        bool hasMaster() const { return master == nullptr ? false : true; }
        
    protected:
        //! The current phase of the saw (ranged from 0 to 1)
        long double phase = 0;
        
        long double increment_ = 0;
        
        long double phaseOffset = 0;
        
        //! The to be returned value from read
        T y = 0;
        
        Phasor* master = nullptr;
        
    private:
        //! Recompute the most recently computed value
        virtual T convertPhaseToY() = 0;
        
        
        void incrementUnwrappedPhases()
        {
            unwrappedPhase = phase + increment_;
            
            for (auto& slave : slaves)
                slave->incrementUnwrappedPhases();
        }
        
        void computeNewPhases()
        {
            phase = math::wrap<long double>(unwrappedPhase, 0, 1);
            
            if (unwrappedPhase >= 1.0l)
                resetSlaves(this);
            else
                for (auto& slave : slaves)
                    slave->computeNewPhases();
        }
        
        void convertPhasesToYs()
        {
            y = convertPhaseToY();
            
            for (auto& slave : slaves)
                slave->convertPhasesToYs();
        }
        
        void resetSlaves(Phasor* m)
        {
            for (auto& slave : slaves)
            {
                const auto ratio = slave->getIncrement() / m->getIncrement();
                slave->setPhase(m->getPhase() * ratio, false);
                slave->resetSlaves(m);
            }
        }
        
    private:
        long double unwrappedPhase = phase;
        
        std::vector<std::unique_ptr<Phasor>> slaves;
    };
    
    template <typename T>
    class PhasorBlep : public Phasor<T>
    {
    protected:
        bool adjustForSync(const Phasor<T>& master)
        {
            if (master.hasMaster() && adjustForSync(*master.getMaster()))
                return true;
            
            const auto masterPhase = master.getPhase();
            const auto masterIncrement = master.getIncrement();
            
            if (masterPhase > 1.0l - masterIncrement)
            {
                syncAdjust = std::make_unique<T>(beforeReset(masterPhase, masterIncrement));
                return true;
            }
            else if (masterPhase < masterIncrement)
            {
                syncAdjust = std::make_unique<T>(afterReset(masterPhase, masterIncrement));
                return true;
            }
            
            return false;
        }
        
        T beforeReset(long double masterPhase, long double masterIncrement)
        {
            const long double ratio = this->increment_ / masterIncrement;
            
            // hoever verschilt de phase tot precies 1 waar reset echt plaats vindt
            const long double phaseDiffMasterToEnd = 1 - masterPhase;
            
            // gebruik dit verschil om bij de slave op te tellen
            // vermenigvuldig met de ratio want slave gaat sneller
            // dit is de phase waar de slave exact zou resetten
            const long double phaseEndOfSlave = this->phase + (phaseDiffMasterToEnd * ratio);
            
            // Hoeveel phase moet de slave nog tot hij bij bovenstaand einde is
            const long double phaseDifffSlaveToEnd = phaseEndOfSlave - this->phase;
            
            // bereken de 'on-geblepte' eind positie van de golf
            const auto slaveYAtEnd = computeAliasedYBeforeReset(this->phase, this->phaseOffset);
            
            // bereken de 'on-geblepte' begin positie van de golf
            // we incrementen de phase door increment erbij op te tellen.
            // Maaaar, we moeten doen alsof de phaseEndOfSlave het eindpunt was en dus hiermee wrappen (aftrekken)
            const auto slaveYatBegin = computeAliasedYAfterReset(this->phase + this->increment_ - phaseEndOfSlave, this->phaseOffset);
            //            const auto slaveYatBegin = computeAliasedYAfterReset(0, this->phaseOffset); // minder accuraat maar werkt wel, je moet iets verder zijn dan phase 0
            
            // Bereken de scaling relatief tot de master
            // Je deelt omdat je normaliter van -1 tot 1 gaat
            blepScale = (slaveYAtEnd - slaveYatBegin) / 2;
            
            // Doe een echte blep step, alsof van -1 naar 1...
            const long double x = insertPolyBlepBeforeReset(1.l - phaseDifffSlaveToEnd, this->increment_);
            
            // ...end scale dat met de zojuist berekende scale
            return x * blepScale;
        }
        
        T afterReset(long double masterPhase, long double masterIncrement)
        {
            auto x = insertPolyBlepAfterReset(this->phase, this->increment_);
            
            return x * blepScale;
        }
        
        virtual T computeAliasedYBeforeReset(long double phase, long double phaseOffset) = 0;
        virtual T computeAliasedYAfterReset(long double phase, long double phaseOffset) = 0;
        
    protected:
        std::unique_ptr<T> syncAdjust;
        
        long double blepScale = 0;
    };
    
}
