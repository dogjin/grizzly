//
//  generator.cpp
//  Sandbox
//
//  Created by Stijn Frishert on 20/10/2017.
//  Copyright © 2017 Stijn Frishert. All rights reserved.
//

#include <stdexcept>

#include "generator.hpp"

namespace dsp
{
    GeneratorBase::GeneratorBase(Phasor& phasor) :
        phasor(&phasor)
    {
        phasor.generators.emplace(this);
    }
    
    GeneratorBase::~GeneratorBase()
    {
        if (phasor)
            phasor->generators.erase(this);
    }
    
    void GeneratorBase::setPhasor(Phasor* phasor)
    {
        if (this->phasor)
            this->phasor->generators.erase(this);
        
        this->phasor = phasor;
        
        if (this->phasor)
            phasor->generators.emplace(this);
        
        recompute();
    }
    
    long double GeneratorBase::getPhase() const
    {
        if (!phasor)
            throw std::runtime_error("generator is not attached to a phasor");
        
        return phasor->getPhase();
    }
    
    long double GeneratorBase::getIncrement() const
    {
        if (!phasor)
            throw std::runtime_error("generator is not attached to a phasor");
        
        return phasor->getIncrement();
    }
    
    void GeneratorBase::setPhaseOffset(long double offset, bool recompute)
    {
        phaseOffset = offset;
        
        if (recompute)
            this->recompute();
    }
    
    long double GeneratorBase::getPhaseOffset() const
    {
        return phaseOffset;
    }
    
    const Phasor* GeneratorBase::getMaster() const
    {
        if (!phasor)
            throw std::runtime_error("generator is not attached to a phasor");
        
        return phasor->getMaster();
    }
    
    bool GeneratorBase::hasMaster() const
    {
        if (!phasor)
            throw std::runtime_error("generator is not attached to a phasor");
        
        return phasor->hasMaster();
    }
}
