/*
 
 This file is a part of Grizzly, a modern C++ library for digital signal
 processing. See https://github.com/moditone/grizzly for more information.
 
 Copyright (C) 2016-2018 Moditone <info@moditone.com>
 
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

#ifndef GRIZZLY_MID_SIDE_HPP
#define GRIZZLY_MID_SIDE_HPP

#include <utility>
#include <vector>

namespace dsp
{
    //! Convert a left and right stereo sample to mid
    template <class T>
    constexpr T stereo2mid(const T& left, const T& right)
    {
        return static_cast<T>((left + right) * 0.5);
    }
    
    //! Convert a left and right stereo sample to side
    template <class T>
    constexpr T stereo2side(const T& left, const T& right)
    {
        return static_cast<T>((left - right) * 0.5);
    }
    
    //! Convert mid-side sample to a stereo left
    template <class T>
    constexpr T midSide2left(const T& mid, const T& side)
    {
        return mid + side;
    }
    
    //! Convert mid-side sample to a stereo right
    template <class T>
    constexpr T midSide2right(const T& mid, const T& side)
    {
        return mid - side;
    }
    
    //! Sample with a left and right channel
    template <class T>
    struct Stereo
    {
    public:
        Stereo() = default;
        Stereo(const T& left, const T& right) : left(left), right(right) { }

        T left; //!< The left channel
        T right; //!< The right channel
    };

    //! Compare two stereos for equality
    template <class T>
    constexpr bool operator==(const Stereo<T>& lhs, const Stereo<T>& rhs)
    {
        return lhs.left == rhs.left && lhs.right == rhs.right;
    }

    //! Compare two stereos for inequality
    template <class T>
    constexpr bool operator!=(const Stereo<T>& lhs, const Stereo<T>& rhs)
    {
        return !(lhs == rhs);
    }

    //! Sample with a mid and side channel
    template <class T>
    struct MidSide
    {
        MidSide() = default;
        MidSide(const T& mid, const T& side) : mid(mid), side(side) { }

        T mid; //!< The mid channel
        T side; //!< The side channel
    };

    //! Compare two mid-sides for equality
    template <class T>
    constexpr bool operator==(const MidSide<T>& lhs, const MidSide<T>& rhs)
    {
        return lhs.mid == rhs.mid && lhs.side == rhs.side;
    }

    //! Compare two mid-sides for inequality
    template <class T>
    constexpr bool operator!=(const MidSide<T>& lhs, const MidSide<T>& rhs)
    {
        return !(lhs == rhs);
    }

    //! Convert a left and right stereo sample to mid-side
    template <class T>
    constexpr MidSide<T> stereo2midSide(const T& left, const T& right)
    {
        return {stereo2mid(left, right), stereo2side(left, right)};
    }

    //! Convert a left and right stereo sample to mid-side
    template <class T>
    constexpr MidSide<T> stereo2midSide(const Stereo<T>& stereo)
    {
        return stereo2ms(stereo.left, stereo.right);
    }

    //! Convert mid-side sample to a stereo left-right
    template <class T>
    constexpr Stereo<T> midSide2stereo(const T& mid, const T& side)
    {
        return {midSide2left(mid, side), midSide2right(mid, side)};
    }

    //! Convert mid-side sample to a stereo left-right
    template <class T>
    constexpr Stereo<T> midSide2stereo(const MidSide<T>& ms)
    {
        return midSide2stereo(ms.mid, ms.side);
    }
}

#endif
