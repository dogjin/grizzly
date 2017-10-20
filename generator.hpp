//
//  generator.hpp
//  Sandbox
//
//  Created by Stijn Frishert on 20/10/2017.
//  Copyright © 2017 Stijn Frishert. All rights reserved.
//

#pragma once

#include <memory>

#include "phasor.hpp"
#include "poly_blep.hpp"

namespace dsp
{
    class GeneratorBase
    {
    public:
        GeneratorBase(Phasor& phasor);
        virtual ~GeneratorBase();
        
        long double getPhase() const;
        long double getIncrement() const;
        
        void setPhaseOffset(long double offset);
        long double getPhaseOffset() const;
        
        const Phasor* getMaster() const;
        bool hasMaster() const;
        
        virtual void recompute() = 0;
        
    private:
        Phasor& phasor;
        long double phaseOffset = 0;
    };
    
    template <typename T>
    class Generator :
        public GeneratorBase
    {
    public:
        using GeneratorBase::GeneratorBase;
        virtual ~Generator() = default;
        
        const T& read() const { return y; }
        
        void recompute() final { y = convert(); }
        
    private:
        virtual T convert() = 0;
        
    private:
        T y;
    };
    
    template <typename T>
    class BandLimitedGenerator :
        public Generator<T>
    {
    public:
        using Generator<T>::Generator;
        
        T convert() final
        {
            // Compute the y without any anti aliasing
            auto y = computeAliasedY();
            
            // There's a hard sync going on
            this->syncAdjust.reset();
            if (this->hasMaster() && this->adjustForSync(*this->getMaster()) && this->syncAdjust != nullptr)
            {
                y -= *this->syncAdjust;
                return y;
            }
            
            // If there's a syncAdjust value, it shoud never perform a 'normal' blep
            assert(this->syncAdjust == nullptr);
            
            applyRegularBandLimiting(y);
            
            return y;
        }
        
    protected:
        std::unique_ptr<T> syncAdjust;
        
        long double blepScale = 0;
        
    private:
        virtual T computeAliasedY() = 0;
        virtual void applyRegularBandLimiting(T& y) = 0;
        virtual T computeAliasedYBeforeReset(long double phase, long double phaseOffset) = 0;
        virtual T computeAliasedYAfterReset(long double phase, long double phaseOffset) = 0;
        
        bool adjustForSync(const Phasor& master)
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
            const auto phase = this->getPhase();
            const auto phaseOffset = this->getPhaseOffset();
            const auto increment = this->getIncrement();
            
            const long double ratio = increment / masterIncrement;
            
            // hoever verschilt de phase tot precies 1 waar reset echt plaats vindt
            const long double phaseDiffMasterToEnd = 1 - masterPhase;
            
            // gebruik dit verschil om bij de slave op te tellen
            // vermenigvuldig met de ratio want slave gaat sneller
            // dit is de phase waar de slave exact zou resetten
            const long double phaseEndOfSlave = phase + (phaseDiffMasterToEnd * ratio);
            
            // Hoeveel phase moet de slave nog tot hij bij bovenstaand einde is
            const long double phaseDifffSlaveToEnd = phaseEndOfSlave - phase;
            
            // bereken de 'on-geblepte' eind positie van de golf
            const auto slaveYAtEnd = computeAliasedYBeforeReset(phase, phaseOffset);
            
            // bereken de 'on-geblepte' begin positie van de golf
            // we incrementen de phase door increment erbij op te tellen.
            // Maaaar, we moeten doen alsof de phaseEndOfSlave het eindpunt was en dus hiermee wrappen (aftrekken)
            const auto slaveYatBegin = computeAliasedYAfterReset(phase + increment - phaseEndOfSlave, phaseOffset);
//            const auto slaveYatBegin = computeAliasedYAfterReset(0, this->phaseOffset); // minder accuraat maar werkt wel, je moet iets verder zijn dan phase 0
            
            // Bereken de scaling relatief tot de master
            // Je deelt omdat je normaliter van -1 tot 1 gaat
            blepScale = (slaveYAtEnd - slaveYatBegin) / 2;
            
            // Doe een echte blep step, alsof van -1 naar 1...
            const long double x = insertPolyBlepBeforeReset(1.l - phaseDifffSlaveToEnd, increment);
            
            // ...end scale dat met de zojuist berekende scale
            return x * blepScale;
        }
        
        T afterReset(long double masterPhase, long double masterIncrement)
        {
            auto x = insertPolyBlepAfterReset(this->getPhase(), this->getIncrement());
            
            return x * blepScale;
        }
    };
}
