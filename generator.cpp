//
//  generator.cpp
//  Sandbox
//
//  Created by Stijn Frishert on 20/10/2017.
//  Copyright © 2017 Stijn Frishert. All rights reserved.
//

#include "generator.hpp"

namespace dsp
{
    GeneratorBase::GeneratorBase(Phasor& phasor) :
        phasor(phasor)
    {
        phasor.generators.emplace(this);
    }
    
    GeneratorBase::~GeneratorBase()
    {
        phasor.generators.erase(this);
    }
    
    long double GeneratorBase::getPhase() const
    {
        return phasor.getPhase();
    }
    
    long double GeneratorBase::getIncrement() const
    {
        return phasor.getIncrement();
    }
    
    void GeneratorBase::setPhaseOffset(long double offset)
    {
        phaseOffset = offset;
    }
    
    long double GeneratorBase::getPhaseOffset() const
    {
        return phaseOffset;
    }
    
    const Phasor* GeneratorBase::getMaster() const
    {
        return phasor.getMaster();
    }
    
    bool GeneratorBase::hasMaster() const
    {
        return phasor.hasMaster();
    }
}
