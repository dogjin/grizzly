//
//  frequency.hpp
//  Grizzly
//
//  Created by Stijn Frishert on 12/11/17.
//

#ifndef GRIZZLY_FREQUENCY_HPP
#define GRIZZLY_FREQUENCY_HPP

#include <vector>

namespace dsp
{
    //! Converts a frequency in hertz to a midi pitch
    template <typename T>
    T convertFrequencyToMidiPitch(const T& frequency)
    {
        if (frequency == 0)
            return 0;
        
        return 69 + 12 * std::log2(frequency / 440.0);
    }
    
    template <typename InputIterator, typename OutputIterator>
    void convertFrequenciesToMidiPitches(InputIterator begin, InputIterator end, OutputIterator outBegin)
    {
        for (auto it = begin; it != end; ++it)
            *outBegin++ = convertFrequencyToMidiPitch(*it);
    }
    
    template <typename Iterator>
    std::vector<typename Iterator::value_type> convertFrequenciesToMidiPitches(Iterator begin, Iterator end)
    {
        std::vector<typename Iterator::value_type> semitones;
        semitones.reserve(std::distance(begin, end));
        convertFrequenciesToMidiPitches(begin, end, std::back_inserter(semitones));
        return semitones;
    }
}

#endif /* GRIZZLY_FREQUENCY_HPP */
