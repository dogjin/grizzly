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
    
    void GeneratorBase::setPhaseOffset(long double offset, bool recompute)
    {
        phaseOffset = offset;
        
        if (recompute)
            this->recompute();
    }
    
    void GeneratorBase::recompute() noexcept
    {
        if (phasor)
            recomputeRequested();
    }
}
