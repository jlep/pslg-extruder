#pragma once

#include "vec.hpp"
#include "fixedvector.hpp"

#include <algorithm>
#include <type_traits>
#include <vector>
#include <array>
#include <cmath>

template <typename T, typename V, typename E> struct PslgVertex;
template <typename T, typename V, typename E> struct PslgEdge;
template <typename T, typename V, typename E> struct Pslg;

struct Empty {};

template <typename T>
struct DefaultEdge {
    T thickness;
    explicit DefaultEdge(T thickness) :
            thickness(std::move(thickness)) {
    }
};

template <typename T, typename E>
struct EdgeTraits {
};

template <typename T>
struct EdgeTraits<T, DefaultEdge<T>> {
    using Edge = DefaultEdge<T>;
    static T getThickness(const Edge& edge) {
        return edge.thickness;
    }
    static void setThickness(Edge& edge, const T& thickness) {
        edge.thickness = thickness;
    }
};

// edges have ccw ordering in right-handed coordinate system
template <typename T = float, typename V = Empty, typename E = DefaultEdge<T>>
struct PslgVertex {

    using Edge = PslgEdge<T, V, E>;

    Vec2<T> p;
    std::vector<Edge*> edges;
    V v;

    explicit PslgVertex(float x, float y, V vertex = V()) :
            p(x, y), v(std::move(vertex)) {
    }

    V& vertex() {
        return v;
    }

    const V& vertex() const {
        return v;
    }

    void addEdge(Edge *edge) {
        edges.emplace_back(edge);
        sortEdges();
    }

    Edge *nextEdge(const Edge *edge) const {
        if (edges.empty()) return nullptr;
        if (edges.back() == edge) return edges.front();
        for (size_t i = 0; i < edges.size() - 1; ++i) {
            if (edges[i] == edge) return edges[i + 1];
        }
        return nullptr;
    }

    Edge *prevEdge(const Edge *edge) const {
        if (edges.empty()) return nullptr;
        if (edges.front() == edge) return edges.back();
        for (size_t i = 1; i < edges.size(); ++i) {
            if (edges[i] == edge) return edges[i - 1];
        }
        return nullptr;
    }

    void setPoint(const Vec2<T> &p) {
        if (this->p == p) return;
        this->p = p;
        sortEdges();
        for (Edge *e : edges) {
            e->otherVertex(this)->sortEdges();
        }
    }

    void sortEdges() {
        std::sort(edges.begin(), edges.end(), [this](const Edge *e1, const Edge *e2) {
            return e1->diamondAngle(this) < e2->diamondAngle(this);
        });
    }
};

template <typename T = float, typename V = Empty, typename E = DefaultEdge<T>>
struct VertexPair {

    using Vertex = PslgVertex<T, V, E>;
    using Edge = PslgEdge<T, V, E>;

    Vertex* v1;
    Vertex* v2;
};


template <typename T = float, typename V = Empty, typename E = DefaultEdge<T>>
struct PslgEdge {

    using Vertex = PslgVertex<T, V, E>;
    using Edge = PslgEdge<T, V, E>;

    Vertex* v1;
    Vertex* v2;
    E e;

    PslgEdge(Vertex* v1, Vertex* v2, E edge = E()) :
            v1(v1), v2(v2), e(edge) {
        v1->addEdge(this);
        v2->addEdge(this);
    }

    E& edge() {
        return e;
    }

    const E& edge() const {
        return e;
    }

    const Vec2<T>& p1() const {
        return v1->p;
    }

    const Vec2<T>& p2() const {
        return v2->p;
    }

    Vec2<T> diff() const {
        return p2() - p1();
    }

    T diamondAngle(const Vertex* v) const {
        if (v == v1) return ::diamondAngle(diff());
        if (v == v2) return ::diamondAngle(-diff());
        return std::numeric_limits<T>::quiet_NaN();
    }

    Vertex* otherVertex(const Vertex* v) const {
        if (v == v1) return v2;
        if (v == v2) return v1;
        return nullptr;
    }

    std::array<Vec2<T>, 2> midPoints() const {
        auto thickness = getThickness();
        auto dp = diff();
        auto mp = p1() + T(0.5) * dp;
        auto n1 = thickness * normalized(rot90ccw(dp));
        return {
                mp + n1,
                mp - n1,
        };
    };

    std::array<Vec2<T>, 4> cornerPoints() const {
        auto thickness = getThickness();
        auto n1 = thickness * normalized(rot90ccw(diff()));
        return {
                p1() + n1,
                p2() + n1,
                p2() - n1,
                p1() - n1
        };
    };

    Line2<T> line() const {
        return {p1(), p2() - p1()};
    }

    T getThickness() const {
        return EdgeTraits<T, E>::getThickness(e);
    }

    void setThickness(const T& thickness) {
        EdgeTraits<T, E>::setThickness(e, thickness);
    }

    enum Side {
        LEFT,
        RIGHT
    };

    //         x (r)
    //      u1  u2
    //       \  /
    // v1 --- v0 --- v2
    //
    // Origin side is left if v0 is on the left side of the edge
    // we're interested in.
    template <Side originSide>
    static std::array<Vec2<T>, 2> miteredCorner(const Vec2<T>& v0, const Vec2<T>& v1, const Vec2<T>& v2,
                                                T thickness1, T thickness2) {
        constexpr T miterLimit = 2.0;
        auto dv1 = v1 - v0;
        auto dv2 = v2 - v0;
        auto du1 = thickness1 * normalized(rot90cw(dv1));
        auto du2 = thickness2 * normalized(rot90ccw(dv2));
        auto u1 = v0 + du1;
        auto u2 = v0 + du2;
        auto det0 = det(dv2, dv1);
        auto mlimit = miterLimit * fmax(thickness1, thickness2);
        if (det0 == 0) {
            if (originSide == LEFT) {
                return {v0, u2};
            } else {
                return {v0, u1};
            }
        }
        auto r = lineIntersect(u1, dv1, u2, dv2);
        auto x = u1 + r.x * dv1;
        if (det0 < 0) {
            // > 180 deg
            if (r.x >= 0) {
                if (originSide == LEFT) {
                    auto s = lineIntersect(v0, du1, u2, dv2);
                    return {v0, v0 + s.x * du1};
                } else {
                    return {v0, u1};
                }
            } else if (r.y >= 0) {
                if (originSide == LEFT) {
                    return {v0, u2};
                } else {
                    auto s = lineIntersect(v0, du2, u1, dv1);
                    return {v0, v0 + s.x * du2};
                }
            } else {
                if (dist(v0, x) > mlimit) {
                    auto dx = x - v0;
                    auto s = lineIntersect(v0, dx, u1, u2 - u1);
                    auto y = v0 + s.x * dx;
                    if (originSide == LEFT) {
                        return {y, u2};
                    } else {
                        return {y, u1};
                    }
                }
            }
        } else {
            auto dot0 = dot(dv2, dv1);
            if (dot0 <= 0) {
                // 90-180 deg
                if (r.x <= 0) {
                    if (originSide == LEFT) {
                        auto s = lineIntersect(v0, du1, u2, dv2);
                        return {v0, v0 + s.x * du1};
                    } else {
                        return {v0, u1};
                    }
                } else if (r.y <= 0) {
                    if (originSide == LEFT) {
                        return {v0, u2};
                    } else {
                        auto s = lineIntersect(v0, du2, u1, dv1);
                        return {v0, v0 + s.x * du2};
                    }
                }
            }
        }
        return {v0, x};
    };

    //         q1
    //      e1 |   e0  |  e2
    // u1 ---- v1 ---- v2 --- u2
    template <size_t N>
    void miteredEdge(FixedVector<Vec2<T>, N>& pts,
                     const Vertex* v1, const Vertex* v2) const {
        auto thickness = getThickness();
        auto e1 = v1->nextEdge(this);
        auto e2 = v2->prevEdge(this);
        auto u1 = e1->otherVertex(v1);
        auto u2 = e2->otherVertex(v2);
        auto c1 = miteredCorner<LEFT>(v1->p, u1->p, v2->p, e1->getThickness(), thickness);
        auto c2 = miteredCorner<RIGHT>(v2->p, v1->p, u2->p, thickness, e2->getThickness());
        auto dv = v2->p - v1->p;
        auto q1 = v1->p + thickness * normalized(rot90ccw(dv));
        auto dc1 = c1[1] - c1[0];
        auto dc2 = c2[1] - c2[0];

        // check constraints
        // this is pretty horrible and could probably be simplified
        if (c1[0] == v1->p) {
            // intersection with other
            auto r1 = lineIntersect(c1[0], dc1, c2[0], dc2);
            if (isnil(r1) || r1.x <= 0) {
                // a "cap"
                pts.push_back(c1[1]);
                pts.push_back(c2[1]);
                if (c2[0] != v2->p) pts.push_back(c2[0]);
                return;
            }
            // intersection with stroke
            auto r2 = lineIntersect(c1[0], dc1, q1, dv);
            if (r1.x >= r2.x) {
                pts.push_back(c1[1]);
                pts.push_back(c2[1]);
                if (c2[0] != v2->p) pts.push_back(c2[0]);
                return;
            }
            // intersection with extra segment
            if (c2[0] != v2->p) {
                auto r3 = lineIntersect(c1[0], dc1, v2->p, c2[0] - v2->p);
                if (r3.x > 0 && r3.x < r1.x) {
                    auto x = c1[0] + r3.x * dc1;
                    pts.push_back(x);
                    return;
                }
            }
            auto x = c1[0] + r1.x * dc1;
            pts.push_back(x);
            if (c2[0] != v2->p) pts.push_back(c2[0]);
            return;
        } else if (c2[0] == v2->p) {
            // there is an extra segment on the other side now
            // intersection with other
            auto r1 = lineIntersect(c2[0], dc2, c1[0], dc1);
            if (isnil(r1) || r1.x <= 0) {
                // a "cap"
                pts.push_back(c1[0]);
                pts.push_back(c1[1]);
                pts.push_back(c2[1]);
                return;
            }
            // intersection with stroke
            auto r2 = lineIntersect(c2[0], dc2, q1, dv);
            if (r1.x >= r2.x) {
                pts.push_back(c1[0]);
                pts.push_back(c1[1]);
                pts.push_back(c2[1]);
                return;
            }
            // intersection with extra segment
            auto r3 = lineIntersect(c2[0], dc2, v1->p, c1[0] - v1->p);
            if (r3.x > 0 && r3.x < r1.x) {
                auto x = c2[0] + r3.x * dc2;
                pts.push_back(x);
                return;
            }
            auto x = c2[0] + r1.x * dc2;
            pts.push_back(c1[0]);
            pts.push_back(x);
            return;
        } else {
            // both joints have an extra segment
            auto r1 = lineIntersect(c2[0], dc2, c1[0], dc1);
            // intersection with stroke
            auto r2 = lineIntersect(c2[0], dc2, q1, dv);
            if (isnil(r1) || r1.x >= r2.x) {
                pts.push_back(c1[0]);
                pts.push_back(c1[1]);
                pts.push_back(c2[1]);
                pts.push_back(c2[0]);
                return;
            }
            auto x = c2[0] + r1.x * dc2;
            pts.push_back(c1[0]);
            pts.push_back(x);
            pts.push_back(c2[0]);
            return;
        }
    }

    FixedVector<Vec2<T>, 10> miteredPoints() const {
        FixedVector<Vec2<T>, 10> pts;
        pts.push_back(p1());
        miteredEdge(pts, v1, v2);
        pts.push_back(p2());
        miteredEdge(pts, v2, v1);
        pts.push_back(p1());
        return pts;
    };
};

template <typename T = float, typename V = Empty, typename E = DefaultEdge<T>>
struct Pslg {

    using Vertex = PslgVertex<T, V, E>;
    using Edge = PslgEdge<T, V, E>;

    std::vector<std::unique_ptr<Vertex>> verts;
    std::vector<std::unique_ptr<Edge>> edges;

    Vertex* addVertex(float x, float y, V vertex = V()) {
        return verts.emplace_back(std::make_unique<Vertex>(x, y, std::move(vertex))).get();
    }

    Edge* addEdge(Vertex* v1, Vertex* v2, E edge = E()) {
        return edges.emplace_back(std::make_unique<Edge>(v1, v2, std::move(edge))).get();
    }

    Edge* addEdge(Vertex* v1, Vertex* v2, T thickness) {
        return edges.emplace_back(std::make_unique<Edge>(v1, v2, E(std::move(thickness)))).get();
    }
};
