/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/dsperados/grizzly for more information.
 
 Copyright (C) 2016 Dsperados <info@dsperados.com>
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>
 
 --------------------------------------------------------------------
 
 If you would like to use Grizzly for commercial or closed-source
 purposes, please contact us for a commercial license.
 
 */

#ifndef GRIZZLY_SEGMENT_ENVELOPE_HPP
#define GRIZZLY_SEGMENT_ENVELOPE_HPP

#include <cassert>
#include <cstddef>
#include <dsperados/math/utility.hpp>
#include <dsperados/math/ease.hpp>
#include <dsperados/math/interpolation.hpp>
#include <experimental/optional>
#include <functional>
#include <initializer_list>
#include <numeric>
#include <unit/time.hpp>
#include <utility>
#include <vector>

namespace dsp
{
    //! Segment Envelope
    /*! Envelope generator using sequential segments */
    template <class Value, class Time = float>
    class SegmentEnvelope
    {
    public:
        //! Segment class which describes the individual paths of the envelope
        class Segment
        {
        public:
            //! Construct a segment
            /*! @param amplitude: The amplitude value, starting from the pervious amplitude or zero at envelope start
                @param duration: The duration in seconds to get to the destination amplitude.
                @param ease An: easing function to alther the shape of the segment */
            Segment(SegmentEnvelope& envelope, Value amplitude, unit::second<Time> duration, std::function<double(double)> ease = nullptr);
            
            //! Set the duration
            /*! Any negative duration will be clamped to 0 */
            void setDuration(unit::second<Time> duration);
            
            //! Get the duration
            const unit::second<Time>& getDuration() const { return duration; }
            
            //! Retrieve the absolute time of the segment
            unit::second<Time> getAbsoluteTime() const;
            
            //! Given a time within the segment and its starting value, compute an interpolated value
            Value interpolate(unit::second<Time> time, const Value& startValue);
            
        public:
            //! Destination amplitude
            Value amplitude = 0;
            
            //! Ease function to shape the segment
            std::function<double(double)> ease = [](auto x){ return math::easeLinear(x); };
            
        private:
            //! Return the eased proportion along the time axis of this axis
            double computeTimeProportion(unit::second<Time> time);
            
        private:
            //! The envelope that owns this segment
            SegmentEnvelope& envelope;
            
            //! Duration
            unit::second<Time> duration = 0;
        };
        
    public:
        //! Construct an envelope without any segments
        SegmentEnvelope() = default;
        
        //! Construct an envelope with segments
        SegmentEnvelope(std::initializer_list<std::tuple<Value, unit::second<Time>>> segments);
        
        //! Move constructor
        SegmentEnvelope(SegmentEnvelope&& rhs) { operator=(std::move(rhs)); }
            
        //! Move assignment
        SegmentEnvelope& operator=(SegmentEnvelope&& rhs);
        
        //! Increment the internal time of the envelope
        void increment(unit::second<Time> increment);
        
        //! Return the current envelope value
        Value read();
        
        //! Set the state of the envelope
        /*! Jump directly to a certain point, changing the current segment accordingly. */
        void setTime(unit::second<Time> to);
        
        //! Reset the envelope to its starting position, and enable its hold
        void reset();
        
    // --- Segment insertion/removal --- //
        
        //! Add a segment
        template <typename... Args>
        void emplace(Args&&... args);
        
        //! Insert a segment
        template <typename... Args>
        void insert(std::size_t index, Args&&... args);
        
        //! Erase a segment
        void eraseSegment(std::size_t index);
        
    // --- Hold point manipulation --- //
        
        //! Set a hold point (and enable it on default)
        void setAndEnableHoldPoint(unit::second<Time> at);
        
        //! Remove hold point
        void removeHoldPoint();
        
        //! Enable the hold point, if there is one
        void enableHold();
        
        //! Disable the hold point, if there is one
        void disableHold();
        
        //! Retrieve the hold point
        std::experimental::optional<Time> getHold() const;
        
    // --- Access to the segments --- //
        
        //! Retrieve the previous segment
        const Segment* previous(const Segment& segment) const;
        
        //! Retrieve the next segment
        const Segment* next(const Segment& segment) const;
        
        //! Get a segment
        auto& operator[](size_t index) { return segments[index]; }
        
        //! Get a segment, const
        const auto& operator[](size_t index) const { return segments[index]; }
        
        // Begin and end for ranged for-loop
        auto begin() { return segments.begin(); }
        auto begin() const { return segments.begin(); }
        auto end() { return segments.end(); }
        auto end() const { return segments.end(); }
        
    // --- Utility construction functions --- //
        
        //! Create an attack, sustain, release envelope
        static SegmentEnvelope ar(unit::second<Time> attack, unit::second<Time> release, bool hold = true);
        
        //! Create an attack, decay, sustain, release envelope
        static SegmentEnvelope adsr(unit::second<Time> attack, unit::second<Time> decay, Value sustain, unit::second<Time> release);
        
    private:
        //! Hold point
        /*! Contraints a time point from which the envelope stops incrementing and a boolian to indicate whether the hold is enabled or not. */
        struct Hold
        {
            unit::second<Time> timePoint = 0;
            bool enabled = false;
        };
        
    private:
        //! All the segments in the envelope (in sequential order)
        std::vector<Segment> segments;
        
        //! The segment index
        size_t index = 0;
        
        //! The current time in the current segment
        unit::second<Time> segmentTime = 0;
        
        //! The current time in the entire envelope
        unit::second<Time> envelopeTime = 0;
        
        //! Optional hold point
        /*! The envelope will remain at this point until the hold point is disable */
        std::experimental::optional<Hold> hold = std::experimental::nullopt;
    };
    
    template <class Value, class Time>
    SegmentEnvelope<Value, Time>::SegmentEnvelope(std::initializer_list<std::tuple<Value, unit::second<Time>>> segments)
    {
        for (auto& segment : segments)
            this->segments.emplace_back(*this, std::get<0>(segment), std::get<1>(segment));
    }
        
    template <class Value, class Time>
    SegmentEnvelope<Value, Time>& SegmentEnvelope<Value, Time>::operator=(SegmentEnvelope&& rhs)
    {
        segments.clear();
        for (auto& segment : rhs.segments)
            segments.emplace_back(*this, segment.amplitude, segment.getDuration(), segment.ease);
        rhs.segments.clear();
        
        index = rhs.index;
        rhs.index = 0;
        
        segmentTime = rhs.segmentTime;
        rhs.segmentTime = 0;
        
        envelopeTime = rhs.envelopeTime;
        rhs.envelopeTime = 0;
        
        hold = rhs.hold;
        rhs.hold = std::experimental::nullopt;
        
        return *this;
    }
    
    template <class Value, class Time>
    void SegmentEnvelope<Value, Time>::increment(unit::second<Time> increment)
    {
        // If we've reached the last segment, bail out
        if (index >= segments.size())
            return;
        
        // Increment the overall time, and the time within the segment
        segmentTime += increment;
        envelopeTime += increment;
        
        // If we've got a hold, it's enabled, pause at the hold time
        if (hold && hold->enabled && envelopeTime >= hold->timePoint)
        {
            segmentTime -= envelopeTime - hold->timePoint;
            envelopeTime = hold->timePoint;
        }
        
        // Make sure we're at the correct index
        while (index < segments.size() && segmentTime >= segments[index].getDuration())
        {
            segmentTime -= segments[index].getDuration();
            ++index;
        }
    }
    
    template <class Value, class Time>
    Value SegmentEnvelope<Value, Time>::read()
    {
        // Make sure we're at the correct index
        while (index < segments.size() && segmentTime >= segments[index].getDuration())
        {
            segmentTime -= segments[index].getDuration();
            ++index;
        }
        
        // If we've reached the last segment, return the last amplitude
        if (index >= segments.size())
            return segments.empty() ? 0 : segments.back().amplitude;
        
        return segments[index].interpolate(segmentTime, index == 0 ? Value(0) : segments[index - 1].amplitude);
    }
    
    template <class Value, class Time>
    void SegmentEnvelope<Value, Time>::setTime(unit::second<Time> to)
    {
        if (segments.empty() || to.value <= 0)
        {
            envelopeTime = 0;
            segmentTime = 0;
            index = 0;
            return;
        }
        
        // accumulate all segments durations
        auto envelopeDuration = std::accumulate(segments.begin(), segments.end(), unit::second<Time>(0) , [](const auto& acc, const auto& segment) { return acc + segment.getDuration(); } );
        
        if (to >= envelopeDuration)
        {
            envelopeTime = envelopeDuration;
            segmentTime = segments.back().getDuration();
            index = segments.size();
            return;
        }
        
        envelopeTime = to;
        
        unit::second<Time> partialTime = 0;
        for (auto i = 0; i < segments.size(); ++i)
        {
            if (partialTime + segments[i].getDuration() < envelopeTime)
            {
                partialTime += segments[i].getDuration();
                continue;
            }
            
            index = i;
            segmentTime = envelopeTime - partialTime;
            break;
        }
    }
    
    template <class Value, class Time>
    void SegmentEnvelope<Value, Time>::reset()
    {
        setTime(0);
        enableHold();
    }
    
    template <class Value, class Time>
    template <typename... Args>
    void SegmentEnvelope<Value, Time>::emplace(Args&&... args)
    {
        segments.emplace_back(*this, std::forward<Args&&>(args)...);
    }
    
    template <class Value, class Time>
    template <typename... Args>
    void SegmentEnvelope<Value, Time>::insert(std::size_t index, Args&&... args)
    {
        segments.insert(segments.begin() + index, *this, std::forward<Args&&>(args)...);
        setTime(envelopeTime);
    }
    
    template <class Value, class Time>
    void SegmentEnvelope<Value, Time>::eraseSegment(std::size_t index)
    {
        if (segments.empty() || index >= segments.size())
            return;
        
        segments.erase(segments.begin() + index);
        setTime(envelopeTime);
    }
    
    template <class Value, class Time>
    void SegmentEnvelope<Value, Time>::setAndEnableHoldPoint(unit::second<Time> at)
    {
        hold = Hold{at, true};
    }
    
    template <class Value, class Time>
    void SegmentEnvelope<Value, Time>::removeHoldPoint()
    {
        hold = std::experimental::nullopt;
    }
    
    template <class Value, class Time>
    void SegmentEnvelope<Value, Time>::enableHold()
    {
        if (hold)
            hold->enabled = true;
    }
    
    template <class Value, class Time>
    void SegmentEnvelope<Value, Time>::disableHold()
    {
        if (hold)
            hold->enabled = false;
    }
    
    template <class Value, class Time>
    std::experimental::optional<Time> SegmentEnvelope<Value, Time>::getHold() const
    {
        if (hold)
            return hold->timePoint.value;
        else
            return std::experimental::nullopt;
    }
    
    template <class Value, class Time>
    const typename SegmentEnvelope<Value, Time>::Segment* SegmentEnvelope<Value, Time>::previous(const Segment& segment) const
    {
        auto it = std::find_if(segments.rbegin(), segments.rend(), [&](const auto& s){ return &s == &segment; });
        if (it == segments.rend())
            return nullptr;
        
        std::advance(it, 1);
        if (it == segments.rend())
            return nullptr;
        
        return &*it;
    }
    
    template <class Value, class Time>
    const typename SegmentEnvelope<Value, Time>::Segment* SegmentEnvelope<Value, Time>::next(const Segment& segment) const
    {
        auto it = std::find_if(segments.begin(), segments.end(), [&](const auto& s){ return &s == &segment; });
        if (it == segments.end())
            return nullptr;
        
        std::advance(it, 1);
        if (it == segments.end())
            return nullptr;
        
        return &*it;
    }
    
// --- Static constructors --- //
    
    template <class Value, class Time>
    SegmentEnvelope<Value, Time> SegmentEnvelope<Value, Time>::ar(unit::second<Time> attack, unit::second<Time> release, bool hold)
    {
        SegmentEnvelope<Value, Time> env = {{1, attack}, {0, release}};
        if (hold)
            env.setAndEnableHoldPoint(attack);
        return env;
    }
    
    template <class Value, class Time>
    SegmentEnvelope<Value, Time> SegmentEnvelope<Value, Time>::adsr(unit::second<Time> attack, unit::second<Time> decay, Value sustain, unit::second<Time> release)
    {
        SegmentEnvelope<Value, Time> env = {{1, attack}, {sustain, decay}, {0, release}};
        env.setAndEnableHoldPoint(attack + decay);
        return env;
    }
    
// --- Segment --- //
    
    template <class Value, class Time>
    SegmentEnvelope<Value, Time>::Segment::Segment(SegmentEnvelope& envelope, Value amplitude, unit::second<Time> duration, std::function<double(double)> ease) :
        amplitude(amplitude),
        ease(ease),
        envelope(envelope),
        duration(duration)
    {
        
    }
    
    template <class Value, class Time>
    void SegmentEnvelope<Value, Time>::Segment::setDuration(unit::second<Time> duration)
    {
        this->duration = duration.value < 0 ? 0 : duration;
    }
    
    template <class Value, class Time>
    Value SegmentEnvelope<Value, Time>::Segment::interpolate(unit::second<Time> time, const Value& startValue)
    {
        return math::interpolateLinear(computeTimeProportion(time), startValue, amplitude);
    }
    
    template <class Value, class Time>
    double SegmentEnvelope<Value, Time>::Segment::computeTimeProportion(unit::second<Time> time)
    {
        // This function should never be called with a duration of 0
        assert(duration.value != 0);
        
        double x = time.value / static_cast<double>(duration.value);
        return ease ? ease(x) : x;
    }
    
    template <class Value, class Time>
    unit::second<Time> SegmentEnvelope<Value, Time>::Segment::getAbsoluteTime() const
    {
        auto prev = envelope.previous(*this);
        return prev ? (prev->getAbsoluteTime() + prev->getDuration()) : 0;
    }
}


#endif
