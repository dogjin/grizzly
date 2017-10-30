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
#include <moditone/unit/hertz.hpp>
#include <set>
#include <vector>

namespace dsp
{
    class GeneratorBase;
    
    class Phasor
    {
        friend class GeneratorBase;
        
    public:
        void setIncrement(long double increment);
        void setIncrement(unit::hertz<long double> frequency, unit::hertz<long double> sampleRate);
        
        void doIncrement();
        long double doIncrementAndGetPhase();
        long double getIncrement() const;
        
        void setPhase(long double phase, bool recomputeGenerators);
        long double getPhase() const;
        
        template <typename... Args>
        Phasor& createSlave(Args&&... args)
        {
            return addSlave(std::make_unique<Phasor>(std::forward<Args&&>(args)...));
        }
        
        Phasor& addSlave(std::unique_ptr<Phasor> slave);
        
        const Phasor* getMaster() const;
        bool hasMaster() const;
        
    protected:
        std::set<GeneratorBase*> generators;
        
    private:
        void incrementUnwrappedPhases();
        void computeNewPhases();
        void recomputeGenerators();
        void resetSlaves(Phasor& m);
        
    private:
        Phasor* master = nullptr;
        
        long double phase = 0;
        long double unwrappedPhase = 0;
        long double increment = 0;
        
        std::vector<std::unique_ptr<Phasor>> slaves;
    };
}
