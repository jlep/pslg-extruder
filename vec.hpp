#pragma once

#include <cmath>
#include <iostream>

template <typename T>
struct Vec2 {
    union {
        struct {
            T x;
            T y;
        };
        T c[2];
    };

    Vec2() noexcept : x(0), y(0) {}
    Vec2(T x, T y) noexcept : x(x), y(y) {}

    bool operator==(const Vec2& v) const {
        return x == v.x && y == v.y;
    }
    bool operator!=(const Vec2& v) const {
        return !(*this == v);
    }

    Vec2 operator-(const Vec2& v) const {
        return {x - v.x, y - v.y};
    }

    Vec2 operator+(const Vec2& v) const {
        return {x + v.x, y + v.y};
    }

    Vec2 operator+=(const Vec2& v) {
        x += v.x;
        y += v.y;
        return *this;
    }

    Vec2 operator-() const {
        return {-x, -y};
    }

    static constexpr Vec2<T> nil() {
        return {std::numeric_limits<T>::quiet_NaN(), 0};
    }

    T& operator[](size_t i) {
        return c[i];
    }

    const T& operator[](size_t i) const {
        return c[i];
    }
};

template <typename T>
bool isnil(const Vec2<T>& v) {
    return isnan(v.x);
}

template <typename T>
Vec2<T> operator*(const T& a, const Vec2<T>& v) {
    return {a * v.x, a * v.y};
}

template <typename T>
Vec2<T> operator/(const Vec2<T>& v, const T& d) {
    return {v.x / d, v.y / d};
}

template <typename T>
T det(const Vec2<T>& v1, const Vec2<T>& v2) {
    return v1.x * v2.y - v2.x * v1.y;
}

template <typename T>
T dot(const Vec2<T>& v1, const Vec2<T>& v2) {
    return v1.x * v2.x + v1.y * v2.y;
}

template <typename T>
Vec2<T> rot90ccw(const Vec2<T>& v) {
    return {-v.y, v.x};
}

template <typename T>
Vec2<T> rot90cw(const Vec2<T>& v) {
    return {v.y, -v.x};
}

template <typename T>
T sqnorm2(const Vec2<T>& v) {
    return dot(v, v);
}

template <typename T>
T norm2(const Vec2<T>& v) {
    return sqrt(sqnorm2(v));
}

template <typename T>
T sqdist(const Vec2<T>& v1, const Vec2<T>& v2) {
    return sqnorm2(v2 - v1);
}

template <typename T>
T dist(const Vec2<T>& v1, const Vec2<T>& v2) {
    return sqrt(sqdist(v1, v2));
}

template <typename T>
Vec2<T> normalized(const Vec2<T>& v) {
    T d = norm2(v);
    return {v.x / d, v.y / d};
}

// 0-1: 0-90
// 1-2: 90-180
// 2-3: 180-270
// 3-0: 270-360
template <typename T>
int quadrant(const Vec2<T>& a, const Vec2<T>& b) {
    T dott = dot(a, b);
    T dett = det(a, b);
    if (dett >= 0) {
        return (dott >= 0) ? 1 : 2;
    }
    return (dott <= 0) ? 3 : 4;
}

// solve [a b] * x = c
template <typename T>
Vec2<T> solve2x2(const Vec2<T>& a, const Vec2<T>& b, const Vec2<T>& c) {
    T d = det(a, b);
    if (d == 0) return Vec2<T>::nil();
    T x = det(c, b);
    T y = det(a, c);
    return {x / d, y / d};
}

template <typename T>
std::ostream& operator<<(std::ostream& o, const Vec2<T>& v) {
    return o << '(' << v.x << ", " << v.y << ')';
}

// line intersection:
//
// solve r from
// v1 + r1 * n1 = v2 + r2 * n2
//
//        x
//
//
//    /n1    \n2
//   /        \
//  v1        v2

template <typename T>
Vec2<T> lineIntersect(const Vec2<T>& v1, const Vec2<T>& n1,
                      const Vec2<T>& v2, const Vec2<T>& n2) {
    return solve2x2(n1, -n2, v2 - v1);
}

// distance of point q from line p + r * n
template <typename T>
T linePointDistance(const Vec2<T>& p, const Vec2<T>& n, const Vec2<T>& q) {
    return abs(det(q - p, n)) / norm2(n);
}

// line p + r * n
template <typename T>
struct Line2 {
    Vec2<T> p;
    Vec2<T> n;

    T distance(const Vec2<T>& p) const;
};

template <typename T>
Vec2<T> lineIntersect(const Line2<T>& l1, const Line2<T>& l2) {
    return lineIntersect(l1.p, l1.n, l2.p, l2.n);
}

template <typename T>
T linePointDistance(const Line2<T>& l, const Vec2<T>& q) {
    return linePointDistance(l.p, l.n, q);
}

template <typename T>
T Line2<T>::distance(const Vec2<T>& p) const {
    return ::linePointDistance(*this, p);
}

template <typename T>
T diamondAngle(T y, T x)
{
    if (y >= 0) {
        return (x >= 0) ? (y / (x + y)) : (1 - x / (-x + y));
    } else {
        return (x < 0) ? (2 - y / (-x - y)) : (3 + x / (x - y));
    }
}

template <typename T>
T diamondAngle(const Vec2<T>& v) {
    return diamondAngle(v.y, v.x);
}

template <typename T>
T angle(const Vec2<T>& v) {
    return atan2(v.y, v.x);
}

/*               Return a positive value if the points pa, pb, and pc occur  */
/*               in counterclockwise order; a negative value if they occur   */
/*               in clockwise order; and zero if they are collinear.  The    */
/*               result is also a rough approximation of twice the signed    */
/*               area of the triangle defined by the three points.           */
template <typename T>
double orient2dfast(const Vec2<T>& pa, const Vec2<T>& pb, const Vec2<T>& pc)
{
    double acx, bcx, acy, bcy;

    acx = pa.x - pc.x;
    bcx = pb.x - pc.x;
    acy = pa.y - pc.y;
    bcy = pb.y - pc.y;
    return acx * bcy - acy * bcx;
}

/*               Return a positive value if the point pd lies inside the     */
/*               circle passing through pa, pb, and pc; a negative value if  */
/*               it lies outside; and zero if the four points are cocircular.*/
/*               The points pa, pb, and pc must be in counterclockwise       */
/*               order, or the sign of the result will be reversed.          */
template <typename T>
double incirclefast(const Vec2<T>& pa, const Vec2<T>& pb, const Vec2<T>& pc, const Vec2<T>& pd)
{
    double adx, ady, bdx, bdy, cdx, cdy;
    double abdet, bcdet, cadet;
    double alift, blift, clift;

    adx = pa.x - pd.x;
    ady = pa.y - pd.y;
    bdx = pb.x - pd.x;
    bdy = pb.y - pd.y;
    cdx = pc.x - pd.x;
    cdy = pc.y - pd.y;

    abdet = adx * bdy - bdx * ady;
    bcdet = bdx * cdy - cdx * bdy;
    cadet = cdx * ady - adx * cdy;
    alift = adx * adx + ady * ady;
    blift = bdx * bdx + bdy * bdy;
    clift = cdx * cdx + cdy * cdy;

    return alift * bcdet + blift * cadet + clift * abdet;
}

template <typename T>
bool incircle(const Vec2<T>& pa, const Vec2<T>& pb, const Vec2<T>& pc, const Vec2<T>& pd) {
    return incirclefast(pa, pb, pc, pd) * orient2dfast(pa, pb, pc) > 0;
}

// This is getting out of hand

template <typename B, typename P>
void addTriangle(B& triangulation, const P& p1, const P& p2, const P& p3) {
}

// assume that the quad is convex
template <typename B, typename T>
void triangulateQuad(B& triangulation,
                     const Vec2<T> &p1,
                     const Vec2<T> &p2,
                     const Vec2<T> &p3,
                     const Vec2<T> &p4
) {
    if (incircle(p1, p2, p3, p4)) {
        addTriangle(triangulation, p2, p4, p1);
        addTriangle(triangulation, p4, p2, p3);
    } else {
        addTriangle(triangulation, p1, p3, p4);
        addTriangle(triangulation, p3, p1, p2);
    }
}

template <std::size_t I, typename T, std::size_t N>
struct GetMod {
    const std::array<T, N>& array;
    typedef typename std::array<T, N>::const_reference Value;
    explicit GetMod(const std::array<T, N>& array): array(array) {}
};

template <std::size_t J, std::size_t I, typename T, std::size_t N>
typename GetMod<I, T, N>::Value
constexpr getMod(const GetMod<I, T, N>& g) {
    return std::get<(I + J) % N>(g.array);
};

template <std::size_t I, typename B, typename P>
bool tryTriangulate(B& triangulation, const std::array<const P*, 5>& points) {
    GetMod<I, const P*, 5> p(points);
    if (!incircle(*getMod<0>(p), *getMod<1>(p), *getMod<3>(p), *getMod<2>(p)) &&
        !incircle(*getMod<0>(p), *getMod<1>(p), *getMod<3>(p), *getMod<4>(p))) {
        addTriangle(triangulation, *getMod<0>(p), *getMod<1>(p), *getMod<3>(p));
        addTriangle(triangulation, *getMod<1>(p), *getMod<2>(p), *getMod<3>(p));
        addTriangle(triangulation, *getMod<3>(p), *getMod<4>(p), *getMod<0>(p));
        return true;
    }
    return false;
}

// assume that the pentagon is convex
template <typename B, typename P>
void triangulatePentagon(B& triangulation,
                         const P& p1,
                         const P& p2,
                         const P& p3,
                         const P& p4,
                         const P& p5) {
    std::array<const P*, 5> p { &p1, &p2, &p3, &p4, &p5 };
    if (tryTriangulate<0>(triangulation, p)) return;
    if (tryTriangulate<1>(triangulation, p)) return;
    if (tryTriangulate<2>(triangulation, p)) return;
    if (tryTriangulate<3>(triangulation, p)) return;
    if (tryTriangulate<4>(triangulation, p)) return;
    // should not reach this point
    assert(false);
}
