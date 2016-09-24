//
//  Spline.hpp
//  libbear
//
//  Created by Stijn Frishert on 04/02/16.
//  Copyright © 2016 FrisHertz. All rights reserved.
//

#ifndef BEAR_CORE_SPLINE_HPP
#define BEAR_CORE_SPLINE_HPP

#include <gsl/span.h>
#include <vector>

#include "Parallel.hpp"

namespace bear
{
    //! Cubic spline with control points
    /*! Utility class for generating cubic splines. One can add points along the spline, and then
        retrieve interpolated values.
     
        @code{cpp}
        CubicSpline spline;
     
        spline.emplace(0, 1);
        spline.emplace(1, 8);
        spline.emplace(2, -3);
     
        cout << spline[1.124] << endl;
        @endcode */
    class CubicSpline
    {
    public:
        //! Add a point to the spline
        /*! Every time a point is added, the coefficients will be recalculated.
            @warning If you'll add more than one point, use the version of emplace() taking spans */
        void emplace(float x, float y)
        {
            emplacePoint(x, y);
            
            recomputeCoefficients();
        }
        
        //! Emplace points and their values
        template <class U>
        void emplace(gsl::span<const U> x, gsl::span<const float> y)
        {
            auto n = std::min(x.size(), y.size());
            for (auto i = 0; i < n; ++i)
                emplacePoint(x[i], y[i]);
            
            recomputeCoefficients();
        }
        
        //! Emplace points and their values by index
        /*! @param indices Indexes into the value span
            @param values y-values, per 1 x */
        template <class U>
        void emplaceByIndex(gsl::span<const U> indices, gsl::span<const float> values)
        {
            auto n = std::min(indices.size(), values.size());
            for (auto i = 0; i < n; ++i)
                emplacePoint(indices[i], values[indices[i]]);
            
            recomputeCoefficients();
        }
        
        //! Access one of the points on the spline
        /*! @param x The index being a floating-point, the output will automatically be interpolated */
        float operator[](double x) const
        {
            // If there are not points (and coefficients, return 0)
            if (points.empty())
                return 0;
            
            // If we're before the first point, just return that point's y value
            if (x < points.front().x)
                return points.front().a;
            
            auto last = std::prev(points.end());
            
            // Find the right coefficients
            auto it = find_if(std::next(points.rbegin()), points.rend(), [&](const auto& point){ return x >= point.x; });
            if (it == points.rend())
                return last->a;
            
            // Compute the fraction and fraction powered
            const float f = x - it->x;
            const auto f2 = f * f;
            
            // Return the spline interpolation
            return it->a + (it->b * f) + (it->c * f2) + (it->d * f * f2);
        }
        
        // Access a range of points on the spline
        std::vector<float> span(ptrdiff_t offset, size_t length)
        {
            std::vector<float> out(length);
            
            for (auto i = 0; i < length; ++i)
                out[i] = (*this)[offset + i];
            
            return out;
        }
        
        //! Begin for ranged for-loops
        inline auto begin() { return points.begin(); }
        
        //! Begin for ranged for-loops
        inline auto begin() const { return points.begin(); }
        
        //! End for ranged for-loops
        inline auto end() { return points.end(); }
        
        //! End for ranged for-loops
        inline auto end() const { return points.end(); }
        
    private:
        //! A point in the spline
        struct Point
        {
        public:
            Point(float x, float y) : x(x), a(y) { }
            
            bool operator<(const Point& rhs) const { return x < rhs.x; }
            
        public:
            float x = 0;
            
            float a = 0; // The same as y
            float b = 0;
            float c = 0;
            float d = 0;
        };
        
    private:
        //! Emplace a new point, but don't recompute the coefficients
        void emplacePoint(float x, float y)
        {
            auto it = find_if(points.begin(), points.end(), [&](const auto& point){ return point.x == x; });
            if (it == points.end())
                points.emplace_back(x, y);
            else
                it->a = y;
        }
        
        //! Recompute the coefficients
        void recomputeCoefficients()
        {
            if (points.size() <= 1)
                return;
            
            // Sort the points
            std::sort(points.begin(), points.end());
            
            auto n = points.size() - 1;
            
            dx.resize(n);
            for (auto i = 0; i < n; ++i)
                dx[i] = points[i + 1].x - points[i].x;
            
            alpha.resize(n);
            for (auto i = 1; i < n; ++i)
                alpha[i] = 3.0 * (points[i + 1].a - points[i].a) / dx[i] - 3.0 * (points[i].a - points[i - 1].a) / dx[i - 1];
            
            l.resize(n + 1);
            mu.resize(n + 1);
            z.resize(n + 1);
            
            l[0] = l[n] = 1;
            mu[0] = z[0] = z[n] = 0;
            
            for (auto i = 1; i < n; ++i)
            {
                l[i] = 2.0 * (points[i + 1].x - points[i - 1].x) - dx[i - 1] * mu[i - 1];
                mu[i] = dx[i] / l[i];
                z[i] = (alpha[i] - dx[i - 1] * z[i - 1]) / l[i];
            }
            
            points[n].c = 0;
            
            for (int i = int(n - 1); i >= 0; --i)
            {
                points[i].c = z[i] - mu[i] * points[i + 1].c;
                points[i].b = (points[i + 1].a - points[i].a) / dx[i] - dx[i] * (points[i + 1].c + 2.0 * points[i].c) / 3.0;
                points[i].d = (points[i + 1].c - points[i].c) / (3 * dx[i]);
            }
        }
        
    private:
        //! The points in the spline
        std::vector<Point> points;
        
        //! Utility vectors for recomputation of coefficients
        std::vector<float> alpha;
        std::vector<float> dx;
        std::vector<float> l;
        std::vector<float> mu;
        std::vector<float> z;
    };
    
    //! Generate the minima spline of a span
    template <typename T>
    inline static std::vector<T> minimaSpline(gsl::span<T> x)
    {
        auto min = localMinima(x);

        CubicSpline spline;
        spline.emplaceByIndex(gsl::span<const size_t>(min), x);
        
        return spline.span(0, x.size());
    }
    
    //! Generate the maxima spline of a span
    template <typename T, long N>
    inline static std::vector<T> maximaSpline(gsl::span<T> x)
    {
        auto max = localMaxima(x);

        CubicSpline spline;
        spline.emplaceByIndex(gsl::span<const size_t>(max), x);
        
        return spline.span(0, x.size());
    }
}

#endif /* BEAR_CORE_SPLINE_HPP */
