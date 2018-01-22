//
//  generator.hpp
//  Sandbox
//
//  Created by Stijn Frishert on 20/10/2017.
//  Copyright © 2017 Stijn Frishert. All rights reserved.
//

#pragma once

#include <cassert>
#include <functional>
#include <memory>

#include "phasor.hpp"
#include "poly_blep.hpp"

namespace dsp
{
    class GeneratorBase
    {
    public:
        GeneratorBase() = default;
        GeneratorBase(Phasor& phasor);
        
        virtual ~GeneratorBase();
                
        void attachToPhasor(Phasor& phasor);
        
        void detachFromPhasor();

        double getPhase() const noexcept
        {
            if (phaseDistortion)
                return phaseDistortion(phasor ? phasor->getPhase() : 0);
            else
                return phasor ? phasor->getPhase() : 0;
        }
                    
        double getIncrement() const noexcept { return phasor ? phasor->getIncrement() : 0; }
        
        void setPhaseOffset(double offset, bool recompute);
        double getPhaseOffset() const noexcept { return phaseOffset; }
        double getUnwrappedPhase() const noexcept { return phasor ? phasor->getUnwrappedPhase() : 0; }
        
        const Phasor* getMaster() const noexcept { return phasor ? phasor->getMaster() : nullptr; }
        bool hasMaster() const noexcept { return phasor ? phasor->hasMaster() : false; }
        
        void recompute();
        
    public:
        std::function<double(double)> phaseDistortion;
        
    private:
        virtual void recomputeRequested() = 0;
        
    private:
        Phasor* phasor = nullptr;
        double phaseOffset = 0;
    };
    
    template <typename T>
    class Generator :
        public GeneratorBase
    {
    public:
        using GeneratorBase::GeneratorBase;
        virtual ~Generator() = default;
        
        const T& read() const { return y; }
        
        void recomputeRequested() final { y = convert(); }
        
    private:
        virtual T convert() = 0;
        
    private:
        T y;
    };
    
    template <typename T, typename Derivative>
    class BandLimitedGenerator :
        public Generator<T>
    {
    public:
        using Generator<T>::Generator;
        
        T convert() final
        {
            const auto phase = this->getPhase();
            const auto phaseOffset = this->getPhaseOffset();
            
            // Compute the y without any anti aliasing
            auto y = static_cast<const Derivative&>(*this).computeAliasedY(phase, phaseOffset);
            
            // There's a hard sync going on
            syncAdjusted = false;
            auto master = this->getMaster();
            if (master != nullptr && this->adjustForSync(*master) && syncAdjusted)
            {
                y -= this->syncAdjust;
                return y;
            }
            else
            {
                // If there's a syncAdjust value, it shoud never perform a 'normal' blep
                assert(syncAdjusted == false);
                
                static_cast<Derivative&>(*this).applyRegularBandLimiting(phase, phaseOffset, this->getIncrement(), y);
                return y;
            }
        }
        
    protected:
        T syncAdjust;
        bool syncAdjusted = false;
        
        double blepScale = 0;
        
    private:
        bool adjustForSync(const Phasor& master)
        {
            if (master.hasMaster() && adjustForSync(*master.getMaster()))
                return true;
            
            const auto masterPhase = master.getPhase();
            const auto masterIncrement = master.getIncrement();
            
            if (masterPhase > 1.0l - masterIncrement)
            {
                syncAdjust = beforeReset(masterPhase, masterIncrement);
                syncAdjusted = true;
                return true;
            }
            else if (masterPhase < masterIncrement)
            {
                syncAdjust = afterReset(masterPhase, masterIncrement);
                syncAdjusted = true;
                return true;
            }
            
            return false;
        }
        
        T beforeReset(double masterPhase, double masterIncrement) noexcept
        {
            const auto phase = this->getPhase();
            const auto phaseOffset = this->getPhaseOffset();
            const auto increment = this->getIncrement();
            
            const double ratio = increment / masterIncrement;
            
            // hoever verschilt de phase tot precies 1 waar reset echt plaats vindt
            const double phaseDiffMasterToEnd = 1 - masterPhase;
            
            // gebruik dit verschil om bij de slave op te tellen
            // vermenigvuldig met de ratio want slave gaat sneller
            // dit is de phase waar de slave exact zou resetten
            const double phaseEndOfSlave = phase + (phaseDiffMasterToEnd * ratio);
            
            // Hoeveel phase moet de slave nog tot hij bij bovenstaand einde is
            const double phaseDifffSlaveToEnd = phaseEndOfSlave - phase;
            
            // bereken de 'on-geblepte' eind positie van de golf
            const auto slaveYAtEnd = static_cast<const Derivative&>(*this).computeAliasedY(phase, phaseOffset);
            
            // bereken de 'on-geblepte' begin positie van de golf
            // we incrementen de phase door increment erbij op te tellen.
            // Maaaar, we moeten doen alsof de phaseEndOfSlave het eindpunt was en dus hiermee wrappen (aftrekken)
            const auto slaveYatBegin = static_cast<const Derivative&>(*this).computeAliasedY(phase + increment - phaseEndOfSlave, phaseOffset);
//            const auto slaveYatBegin = static_cast<Derivative&>(*this).computeAliasedYAfterReset(0, this->phaseOffset); // minder accuraat maar werkt wel, je moet iets verder zijn dan phase 0
            
            // Bereken de scaling relatief tot de master
            // Je deelt omdat je normaliter van -1 tot 1 gaat
            blepScale = (slaveYAtEnd - slaveYatBegin) / 2;
            
            // Doe een echte blep step, alsof van -1 naar 1...
            const double x = insertPolyBlepBeforeReset(1 - phaseDifffSlaveToEnd, increment);
            
            // ...end scale dat met de zojuist berekende scale
            return x * blepScale;
        }
        
        T afterReset(double masterPhase, double masterIncrement) const noexcept
        {
            auto x = insertPolyBlepAfterReset(this->getPhase(), this->getIncrement());
            
            return x * blepScale;
        }
    };
}
