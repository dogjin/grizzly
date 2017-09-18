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

#ifndef GRIZZLY_SAW_HPP
#define GRIZZLY_SAW_HPP

#include <moditone/math/interpolation.hpp>
#include <type_traits>

#include "phase_generator.hpp"
#include "poly_blep.hpp"

namespace dsp
{
    //! Generate a bipolar saw wave given a normalized phase
    template <typename T, typename Phase>
    constexpr T generateBipolarSaw(Phase phase, Phase phaseOffset)
    {
        return math::wrap<std::common_type_t<Phase, T>>(phase + phaseOffset, 0, 1) * 2 - 1;
    }
    
    //! Generate a unipolar saw wave given a normalized phase
    template <typename T, typename Phase>
    constexpr T generateUnipolarSaw(Phase phase, Phase phaseOffset)
    {
        return math::wrap<std::common_type_t<Phase, T>>(phase + phaseOffset, 0, 1);
    }
    
    //! Generates a bipolar saw wave
    template <typename T>
    class BipolarSaw : public PhaseGenerator<T>
    {
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY() final override { return dsp::generateBipolarSaw<T>(this->phase, this->phaseOffset); }
    };
    
    //! Generates a unipolar saw wave
    template <typename T>
    class UnipolarSaw : public PhaseGenerator<T>
    {
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY() final override { return dsp::generateUnipolarSaw<T>(this->phase, this->phaseOffset); }
    };
    
    //! Generates a bipolar saw wave using the polyBLEP algorithm for anti aliasing
    template <typename T>
    class BipolarSawBlep : public PhaseGenerator<T>
    {
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY() final override
        {
            // Compute the y without any anti aliasing
            auto y = dsp::generateBipolarSaw<T>(this->phase, this->phaseOffset);
            
            // Compute the increment (phase - previous) and adjust y using polyBLEP
            y -= polyBlep<long double>(math::wrap<long double>(this->getPhase() + this->phaseOffset, 0.0, 1.0), this->increment_);
            
            return y;
        }
    };
    
    template <typename T>
    class BiploarSawBlepSlave : public BlepSlave<T>
    {
    public:
        void beforeReset(long double masterPhase, long double masterIncrement) final
        {
            auto r = this->increment_ / masterIncrement;
            
            // hoever verschilt de phase tot precies 1 waar reset echt plaats vindt
            long double phaseDiffMasterToEnd = 1 - masterPhase;
            
            // gebruik dit verschil om bij de slave op te tellen
            // vermenigvuldig met de ratio want slave gaat sneller
            // dit is de phase waar de slave exact zou resetten
            long double phaseEndOfSlave = this->phase + (phaseDiffMasterToEnd * r);
            
            
            long double phaseDifffSlaveToEnd = phaseEndOfSlave - this->phase;
            
            // bereken de hoogte door de slave phase end in te vullen in de saw generator
            peak = generateBipolarSaw<T>(phaseEndOfSlave, this->phaseOffset);
            
            long double x = insertPolyBlepBeforeReset(1.l - phaseDifffSlaveToEnd, this->increment_);
            
            this->adjustValue = math::scale<long double>(x, -1.l, 1.l, generateBipolarSaw<long double>(0.l, this->phaseOffset), peak);
            
        }
        
        void afterReset(long double masterPhase, long double masterIncrement) final
        {
            auto r = this->increment_ / masterIncrement;
            
            this->setPhase(masterPhase * r);
            
            auto x = insertPolyBlepAfterReset(this->phase, this->increment_);
            
            this->adjustValue = math::scale<long double>(x, -1.l, 1.l, generateBipolarSaw<long double>(0.l, this->phaseOffset), peak);
        }
        
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY() final
        {
            // Compute the y without any anti aliasing
            auto y = dsp::generateBipolarSaw<T>(this->phase, this->phaseOffset);
            
            // Compute the increment (phase - previous) and adjust y using polyBLEP
            y -= polyBlep<long double>(math::wrap<long double>(this->getPhase() + this->phaseOffset, 0.0, 1.0), this->increment_);
            
            if (this->adjustValue != 0)
            {
                y -= this->adjustValue;
                this->adjustValue = 0;
            }
            
            return y;
        }
        
    private:
        long double peak = 0;
    };
    
    template <typename T>
    class BipolarSawBlepMaster : public BlepMaster<T>
    {
    private:
        //! Recompute the most recently computed value
        T convertPhaseToY() final
        {
            // Compute the y without any anti aliasing
            auto y = dsp::generateBipolarSaw<T>(this->phase, this->phaseOffset);
            
            // Compute the increment (phase - previous) and adjust y using polyBLEP
            y -= polyBlep<long double>(math::wrap<long double>(this->getPhase() + this->phaseOffset, 0.0, 1.0), this->increment_);
            
            return y;
        }
    };
}

#endif /* GRIZZLY_SAW_HPP */
