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
        
    void GeneratorBase::attachToPhasor(Phasor& phasor)
    {
        // Detach from the current phasor if there is one
        detachFromPhasor();
        
        // Set the phasor
        this->phasor = &phasor;
        
        // Register this generator to the phasor
        phasor.generators.emplace(this);
        
        // Set the waveform of the generator to the current state of the phasor
        recompute();
    }
    
    void GeneratorBase::detachFromPhasor()
    {
        if (phasor)
            phasor->generators.erase(this);
    }
    
    void GeneratorBase::setPhaseOffset(long double offset, bool recompute)
    {
        phaseOffset = offset;
        
        if (recompute)
            this->recompute();
    }
    
    void GeneratorBase::recompute()
    {
        if (phasor)
            recomputeRequested();
    }
}
