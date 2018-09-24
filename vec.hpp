#pragma once

template <typename T>
struct Vec2 {
    union {
        struct {
            T x;
            T y;
        };
        T c[2];
    };

    Vec2(): x(0), y(0) {}
    Vec2(T x, T y): x(x), y(y) {}

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

using Vec2f = Vec2<float>;
using Line2f = Line2<float>;
