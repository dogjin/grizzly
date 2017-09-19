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
        T convertPhaseToY(long double phase) final { return dsp::generateSquare<T>(phase, pulseWidth, -1, 1); }
        
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
        T convertPhaseToY(long double phase) final { return dsp::generateSquare<T>(phase, pulseWidth, 0, 1); }
        
    private:
        //! The pulse width used to generate the square
        float pulseWidth = 0.5f;
    };
    
    //    //! Generates a bipolar square wave
    //    template <typename T>
    //    class BipolarSquareBlep : public PhaseGenerator<T>
    //    {
    //    public:
    //        BipolarSquareBlep(float pulseWidth = 0.5f) :
    //            pulseWidth(pulseWidth)
    //        {
    //        }
    //
    //        //! Change the pulse width
    //        /*! @param recompute Recompute the y to return from read() */
    //        void setPulseWidth(float pulseWidth, bool recompute)
    //        {
    //            if (pulseWidth == this->pulseWidth)
    //                return;
    //
    //            this->pulseWidth = pulseWidth;
    //
    //            if (recompute)
    //                this->recomputeY();
    //        }
    //
    //    private:
    //        //! Recompute the most recently computed value
    //        T convertPhaseToY(long double phase) final
    //        {
    //            // Compute the y without any anti aliasing
    //            auto y = dsp::generateSquare<T>(phase, pulseWidth, -1, 1);
    //
    //            // Compute the increment (phase - previous) and adjust y using polyBLEP
    //            const auto increment = phase - previousPhase;
    //            y += polyBlep<long double>(phase, increment);
    //            y -= polyBlep<long double>(math::wrap<long double>(this->getPhase() + (1 - pulseWidth), 0, 1), increment);
    //
    //            // Update the previous phase
    //            previousPhase = phase;
    //
    //            return y;
    //        }
    //
    //    private:
    //        //! The pulse width used to generate the square
    //        float pulseWidth = 0.5f;
    //
    //        long double previousPhase = 0;
    //    };
    
    
    
    
    
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
        void preIncrement() final
        {
            const auto master = this->getMaster();
            if (master != nullptr)
            {
                const auto masterPhase = master->getPhase();
                const auto masterIncrement = master->getIncrement();
                
                if (masterPhase < masterIncrement)
                    adjustValue = afterReset(masterPhase, masterIncrement);
                else if (masterPhase > 1.0l - masterIncrement)
                    adjustValue = beforeReset(masterPhase, masterIncrement);
                else
                    adjustValue = 0;
            }
        }
        
        //! Recompute the most recently computed value
        T convertPhaseToY() final
        {
            // Compute the y without any anti aliasing
            auto y = dsp::generateSquare<T>(this->phase, this->phaseOffset, pulseWidth, -1, 1);
            
            if (adjustValue != 0)
            {
                if (adjustValue > 0)
                {
                    // does the slave become 1 or -1 at phase 0?
                    source = y;
                    destination = dsp::generateSquare<T>(0.0l, this->phaseOffset, pulseWidth, -1, 1);
                }
                
                if (destination == source)
                    return y;
                
                if (source < destination)
                    y += adjustValue;
                else
                    y -= adjustValue;
                        
                return y;
            }
            
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
            
            // Doe een echte blep step, alsof van -1 naar 1...
            return insertPolyBlepBeforeReset(1.l - phaseDifffSlaveToEnd, this->increment_);
        }
        
        T afterReset(long double masterPhase, long double masterIncrement)
        {
            const auto ratio = this->increment_ / masterIncrement;
            
            this->phase = math::wrap<long double>(masterPhase * ratio, 0, 1);
            
            return insertPolyBlepAfterReset(this->phase, this->increment_);
        }
        
    private:
        T adjustValue = 0;
        
        T source = 0;
        T destination = 0;
        
        //! The pulse width used to generate the square
        float pulseWidth = 0.5f;
    };
}

#endif /* GRIZZLY_SQUARE_HPP */
