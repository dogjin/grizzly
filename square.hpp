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

#ifndef GRIZZLY_SQUARE_HPP
#define GRIZZLY_SQUARE_HPP

#include <cassert>
#include <cmath>
#include <memory>
#include <moditone/math/wrap.hpp>

#include "phase_generator.hpp"
#include "poly_blep.hpp"

namespace dsp
{
    //! Generate a square wave given a normalized phase
    template <typename T, typename Phase, typename PulseWidth>
    constexpr T generateSquare(Phase phase, Phase phaseOffset, PulseWidth pulseWidth, const T& low, const T& high)
    {
        return math::wrap<Phase>(phase + phaseOffset, 0, 1) < pulseWidth ? high : low;
    }
    
    //! Generates a bipolar square wave
    template <typename T>
    class BipolarSquare : public PhaseGenerator<T>
    {
    public:
        BipolarSquare(float pulseWidth = 0.5f) :
        pulseWidth(pulseWidth)
        {
            
        }
        
        //! Change the pulse width
        /*! @param recompute Recompute the y to return from read() */
        void setPulseWidth(float pulseWidth, bool recompute)
        {
            if (pulseWidth == this->pulseWidth)
                return;
            
            this->pulseWidth = pulseWidth;
            
            if (recompute)
                this->recomputeY();
        }
        
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY() final { return dsp::generateSquare<T>(this->phase, this->phaseOffset, pulseWidth, -1, 1); }
        
    private:
        //! The pulse width used to generate the square
        float pulseWidth = 0.5f;
    };
    
    //! Generates a bipolar square wave
    template <typename T>
    class UnipolarSquare : public PhaseGenerator<T>
    {
    public:
        UnipolarSquare(float pulseWidth = 0.5f) :
        pulseWidth(pulseWidth)
        {
        }
        
        //! Change the pulse width
        /*! @param recompute Recompute the y to return from read() */
        void setPulseWidth(float pulseWidth, bool recompute)
        {
            if (pulseWidth == this->pulseWidth)
                return;
            
            this->pulseWidth = pulseWidth;
            
            if (recompute)
                this->recomputeY();
        }
        
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY() final { return dsp::generateSquare<T>(this->phase, this->phaseOffset, pulseWidth, 0, 1); }
        
    private:
        //! The pulse width used to generate the square
        float pulseWidth = 0.5f;
    };
    
    
    //! Generates a bipolar saw wave using the polyBLEP algorithm for anti aliasing
    template <typename T>
    class BipolarSquareBlep : public PhaseGenerator<T>
    {
    public:
        BipolarSquareBlep() = default;
        
        BipolarSquareBlep(float pulseWidth) : pulseWidth(pulseWidth)
        {
        }
        
        //! Change the pulse width
        /*! @param recompute Recompute the y to return from read() */
        void setPulseWidth(float pulseWidth, bool recompute)
        {
            if (pulseWidth == this->pulseWidth)
                return;
            
            this->pulseWidth = pulseWidth;
            
            if (recompute)
                this->convertPhaseToY();
        }
        
    private:
        void adjustForSync() final
        {
            const auto master = this->getMaster();
            if (master != nullptr)
            {
                const auto masterPhase = master->getPhase();
                const auto masterIncrement = master->getIncrement();
                
                if (masterPhase < masterIncrement)
                    syncAdjust = std::make_unique<T>(afterReset(masterPhase, masterIncrement));
                else if (masterPhase > 1.0l - masterIncrement)
                    syncAdjust = std::make_unique<T>(beforeReset(masterPhase, masterIncrement));
                else
                    syncAdjust.reset();
            }
        }
        
        //! Recompute the most recently computed value
        T convertPhaseToY() final
        {
            // Compute the y without any anti aliasing
            auto y = dsp::generateSquare<T>(this->phase, this->phaseOffset, pulseWidth, -1, 1);
            
            if (syncAdjust != nullptr)
            {
                y -= *syncAdjust;
                
                return y;
            }
            
            // If there's a syncAdjust value, it shoud never perform a 'normal' blep
            assert(syncAdjust == nullptr);
            
            // Compute the increment (phase - previous) and adjust y using polyBLEP
            y += polyBlep<long double>(math::wrap<long double>(this->getPhase() + this->phaseOffset, 0.0, 1.0), this->increment_);
            y -= polyBlep<long double>(math::wrap<long double>(this->getPhase() + this->phaseOffset + (1 - pulseWidth), 0, 1), this->increment_);
            
            return y;
        }
        
        T beforeReset(long double masterPhase, long double masterIncrement)
        {
            const auto ratio = this->increment_ / masterIncrement;
            
            // hoever verschilt de phase tot precies 1 waar reset echt plaats vindt
            const long double phaseDiffMasterToEnd = 1 - masterPhase;
            
            // gebruik dit verschil om bij de slave op te tellen
            // vermenigvuldig met de ratio want slave gaat sneller
            // dit is de phase waar de slave exact zou resetten
            const long double phaseEndOfSlave = this->phase + (phaseDiffMasterToEnd * ratio);
            
            // Hoeveel phase moet de slave nog tot hij bij bovenstaand einde is
            const long double phaseDifffSlaveToEnd = phaseEndOfSlave - this->phase;
            
            // bereken de hoogte door de slave phase end in te vullen in de saw generator
            const auto begin = generateSquare<T>(0.0l, this->phaseOffset, pulseWidth, -1, 1);
            const auto end = generateSquare<T>(phaseEndOfSlave, this->phaseOffset, pulseWidth, -1, 1);
            
            // Bereken de scaling relatief tot de master
            // Je deelt omdat je normaliter van -1 tot 1 gaat
            blepScale = (end - begin) / 2;
            
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
        
    private:
        std::unique_ptr<T> syncAdjust = nullptr;
        
        long double blepScale = 0;
        
        //! The pulse width used to generate the square
        float pulseWidth = 0.5f;
    };
}

#endif /* GRIZZLY_SQUARE_HPP */

