//#include "Pslg.hpp"
//#include "linmath.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <nanovg.h>
#define NANOVG_GL3_IMPLEMENTATION
#include <nanovg_gl.h>

#include <iostream>
#include <memory>
#include <vector>
#include <algorithm>
#include <limits>
#include <array>
#include <typeinfo>
#include <optional>

#include "vec.hpp"
//#include "pslg.hpp"

GLFWwindow* window;
NVGcontext* vg;

constexpr int nvertices = 16;
Vec2f polygon[nvertices] = {};
constexpr float PI = 3.1415926f;

void initVertices() {
    for (int i = 0; i < nvertices; ++i) {
        auto& v = polygon[i];
        float a = i * 2 * PI / nvertices;
        v.x = 600 + cos(a) * 100;
        v.y = 200 + sin(a) * 100;
    }
}

// Pslg things

struct PslgVertex;
struct PslgEdge;
struct Pslg;

struct PslgJoint {
    PslgVertex* lv;
    PslgEdge* le;
    PslgVertex* v;
    PslgEdge* re;
    PslgVertex* rv;
    PslgJoint leftJoint() const;
    PslgJoint rightJoint() const;
};

// edges have ccw ordering in right-handed coordinate system
struct PslgVertex {

    Vec2f p;
    std::vector<PslgEdge*> edges;

    explicit PslgVertex(float x, float y) :
            p(x, y)
    {
    }

    void addEdge(PslgEdge* edge) {
        edges.emplace_back(edge);
        sortEdges();
    }

    PslgEdge* nextEdge(const PslgEdge* edge) const {
        if (edges.empty()) return nullptr;
        if (edges.back() == edge) return edges.front();
        for (int i = 0; i < edges.size() - 1; ++i) {
            if (edges[i] == edge) return edges[i + 1];
        }
        return nullptr;
    }

    PslgEdge* prevEdge(const PslgEdge* edge) const {
        if (edges.empty()) return nullptr;
        if (edges.front() == edge) return edges.back();
        for (int i = 1; i < edges.size(); ++i) {
            if (edges[i] == edge) return edges[i - 1];
        }
        return nullptr;
    }

    PslgJoint withRightEdge(PslgEdge* re) const;

    void setPoint(const Vec2f& p);
    void sortEdges();
};

template <typename T, size_t N>
struct FixedVector {

    size_t size = 0;
    std::array<T, N> arr;
    using Size = typename std::array<T, N>::size_type;

    void push_back(T value) {
        arr[size++] = std::move(value);
    }

    const T& operator[](Size i) const {
        return arr[i];
    }

    auto begin() {
        return arr.begin();
    }

    auto end() {
        return arr.begin() + size;
    }
};

struct PslgEdge {

    PslgVertex* v1;
    PslgVertex* v2;
    float thickness;

    PslgEdge(PslgVertex* v1, PslgVertex* v2, float thickness):
            v1(v1), v2(v2), thickness(thickness)
    {
        v1->addEdge(this);
        v2->addEdge(this);
    }

    Vec2f diff() const {
        return v2->p - v1->p;
    }

    float diamondAngle(const PslgVertex* v) const {
        if (v == v1) return ::diamondAngle(diff());
        if (v == v2) return ::diamondAngle(-diff());
        return std::numeric_limits<float>::quiet_NaN();
    }

    float angle(const PslgVertex* v) const {
        if (v == v1) return ::angle(diff());
        if (v == v2) return ::angle(-diff());
        return std::numeric_limits<float>::quiet_NaN();
    }

    PslgVertex* otherVertex(const PslgVertex* v) const {
        if (v == v1) return v2;
        if (v == v2) return v1;
        return nullptr;
    }

    std::array<Vec2f, 2> midPoints() const {
        auto dp = diff();
        auto mp = v1->p + 0.5f * dp;
        auto n1 = thickness * normalized(rot90ccw(dp));
        return {
            mp + n1,
            mp - n1,
        };
    };

    std::array<Vec2f, 4> cornerPoints() const {
        auto n1 = thickness * normalized(rot90ccw(diff()));
        return {
            v1->p + n1,
            v2->p + n1,
            v2->p - n1,
            v1->p - n1
        };
    };

    Line2f lineFrom(const PslgVertex* v) const {
        return { v->p, otherVertex(v)->p - v->p };
    }

    Line2f prevLine(const PslgVertex* v) const {
        return v->prevEdge(this)->lineFrom(v);
    }

    Line2f nextLine(const PslgVertex* v) const {
        return v->nextEdge(this)->lineFrom(v);
    }

    PslgVertex* prevVertex(const PslgVertex* v) const {
        return v->prevEdge(this)->otherVertex(v);
    }

    PslgVertex* nextVertex(const PslgVertex* v) const {
        return v->prevEdge(this)->otherVertex(v);
    }

    FixedVector<Vec2f, 6> miteredPoints2() const {
        //this makes no sense
        FixedVector<Vec2f, 6> ret;
        ret.push_back(v1->p);

        // one side at a time
        //
        // e1                                 e2
        //     <-- m1  n1 --> <-- n2  m2 -->
        // q1 ----- v1 ----------- v2 ----- q2
        //          / \           / \
        //        p1  u1         u2  p2
        //          x r1           x r2
        auto n1 = v2->p - v1->p;
        auto n2 = v1->p - v2->p;
        auto u1 = v1->p + thickness * normalized(rot90cw(n1));
        auto u2 = v2->p + thickness * normalized(rot90ccw(n2));

        auto e1 = v1->prevEdge(this);
        auto q1 = e1->otherVertex(v1);
        auto m1 = q1->p - v1->p;
        auto p1 = v1->p;
        if (det(m1, n1) == 0) {
            // flat cap
            m1 = rot90cw(n1);
        } else {
            p1 += e1->thickness * normalized(rot90ccw(m1));
        }

        auto e2 = v2->nextEdge(this);
        auto q2 = e2->otherVertex(v2);
        auto m2 = q2->p - v2->p;
        auto p2 = v2->p;
        if (det(n2, m2) == 0) {
            // flat cap
            m2 = rot90ccw(n2);
        } else {
            p2 += e2->thickness * normalized(rot90cw(m2));
        }

        // intersections
        auto r1 = lineIntersect(p1, m1, u1, n1);
        auto r2 = lineIntersect(u2, n2, p2, m2);

        // forms cup ?
        //if

        auto x1 = u1 + r1[1] * n1;
        auto x2 = u2 + r2[0] * n2;

        ret.push_back(x1);
        ret.push_back(x2);
        ret.push_back(v2->p);
        return ret;
    };

    FixedVector<Vec2f, 6> miteredPoints3() const;

    FixedVector<Vec2f, 10> miteredPoints() const;

    Line2f line() const {
        return {v1->p, v2->p - v1->p};
    }
};

PslgJoint PslgJoint::leftJoint() const {
    auto e = lv->nextEdge(le);
    return {
            .lv = e->otherVertex(lv),
            .le = e,
            .v = lv,
            .re = le,
            .rv = v
    };
}

PslgJoint PslgJoint::rightJoint() const {
    auto e = rv->prevEdge(re);
    return {
            .lv = v,
            .le = re,
            .v = rv,
            .re = e,
            .rv = e->otherVertex(rv)
    };
}

PslgJoint PslgVertex::withRightEdge(PslgEdge* re) const {
    auto le = nextEdge(re);
    return {};
}

float miterLimit = 2.0f;

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
std::array<Vec2f, 2> miteredCorner(const Vec2f& v0, const Vec2f& v1, const Vec2f& v2,
                                   float thickness1, float thickness2) {
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
void miteredEdge(FixedVector<Vec2f, N>& pts, const PslgEdge* e0,
                 const PslgVertex* v1, const PslgVertex* v2) {
    auto e1 = v1->nextEdge(e0);
    auto e2 = v2->prevEdge(e0);
    auto u1 = e1->otherVertex(v1);
    auto u2 = e2->otherVertex(v2);
    auto c1 = miteredCorner<LEFT>(v1->p, u1->p, v2->p, e1->thickness, e0->thickness);
    auto c2 = miteredCorner<RIGHT>(v2->p, v1->p, u2->p, e0->thickness, e2->thickness);
    auto dv = v2->p - v1->p;
    auto q1 = v1->p + e0->thickness * normalized(rot90ccw(dv));
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
            if (r3.x < r1.x) {
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
        auto r3 = lineIntersect(c2[0], dc2, v2->p, c2[0] - v2->p);
        if (r3.x < r1.x) {
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

FixedVector<Vec2f, 10> PslgEdge::miteredPoints() const {
    FixedVector<Vec2f, 10> pts;
    pts.push_back(v1->p);
    miteredEdge(pts, this, v1, v2);
    pts.push_back(v2->p);
    miteredEdge(pts, this, v2, v1);
    pts.push_back(v1->p);
    return pts;
};

FixedVector<Vec2f, 6> PslgEdge::miteredPoints3() const {
    FixedVector<Vec2f, 6> ret;
    ret.push_back(v1->p);
    // one side at a time
    //
    // e1                                 e2
    //     <-- m1  n1 --> <-- n2  m2 -->
    // q1 ----- v1 ----------- v2 ----- q2
    //          / \           / \
    //        p1  u1         u2  p2
    //          x r1           x r2
    //
    //
    //          x1 r1          x2 r2
    //  e1    p1 u1          u2 p1  e2
    //         \ /            \ /
    // q1 ----- v1 ----------- v2 ------ q2
    //   <--- m1  n1 --> <-- n2   m2 -->
    //
    auto n1 = v2->p - v1->p;
    auto n2 = v1->p - v2->p;
    auto du1 = thickness * normalized(rot90ccw(n1));
    auto du2 = thickness * normalized(rot90cw(n2));
    auto u1 = v1->p + du1;
    auto u2 = v2->p + du2;

    auto e1 = v1->nextEdge(this);
    auto q1 = e1->otherVertex(v1);
    auto m1 = q1->p - v1->p;
    // holy grail, x1 direction
    auto x1 = u1;

    auto dp1 = e1->thickness * normalized(rot90cw(m1));
    auto p1 = v1-> p + dp1;
    auto r1 = lineIntersect(u1, n1, p1, m1);

    // z1 is the starting point of the constraining line to x1
    auto z1 = v1->p;
    auto det1 = det(n1, m1);
    if (det1 == 0) {
        // do nothing
    } else if (det1 < 0) {
        if (r1.x >= 0) {
            x1 = u1;
        } else if (r1.y >= 0) {
            auto s1 = lineIntersect(u1, n1, v1->p, dp1);
            x1 = u1 + s1.x * n1;
        } else {
            x1 = u1 + r1.x * n1;
            if (dist(v1->p, x1) > miterLimit * fmax(thickness, e1->thickness)) {
                auto y1 = 0.5f * (p1 + u1);
                z1 = y1;
                x1 = u1;
            }
        }
    } else {
        auto dot1 = dot(n1, m1);
        if (dot1 <= 0) {
            // 90-180 degree
            if (r1.x <= 0) {
                x1 = u1;
            } else if (r1.y <= 0) {
                auto s1 = lineIntersect(u1, n1, v1->p, dp1);
                x1 = u1 + s1.x * n1;
            } else {
                x1 = u1 + r1.x * n1;
            }
        } else {
            x1 = u1 + r1.x * n1;
        }
    }

    // neeeext...

    auto e2 = v2->prevEdge(this);
    auto q2 = e2->otherVertex(v2);
    auto m2 = q2->p - v2->p;
    auto x2 = u2;

    auto dp2 = e2->thickness * normalized(rot90ccw(m2));
    auto p2 = v2->p + dp2;
    auto r2 = lineIntersect(u2, n2, p2, m2);

    auto z2 = v2->p;
    auto det2 = det(m2, n2);
    if (det2 == 0) {
        // do nothing
    } else if (det2 < 0) {
        if (r2.x >= 0) {
            x2 = u2;
        } else if (r2.y >= 0) {
            auto s2 = lineIntersect(u2, n2, v2->p, dp2);
            x2 = u2 + s2.x * n2;
        } else {
            x2 = u2 + r2.x * n2;
            if (dist(v2->p, x2) > miterLimit * fmax(thickness, e2->thickness)) {
                auto y2 = 0.5f * (p2 + u2);
                z2 = y2;
                x2 = u2;
            }
        }
    } else {
        auto dot2 = dot(m2, n2);
        if (dot2 <= 0) {
            // 90-180 degree
            if (r2.x <= 0) {
                x2 = u2;
            } else if (r2.y <= 0) {
                auto s2 = lineIntersect(u2, n2, v2->p, dp2);
                x2 = u2 + s2.x * n2;
            } else {
                x2 = u2 + r2.x * n2;
            }
        } else {
            x2 = u2 + r2.x * n2;
        }
    }

    // check constraints
    auto dz1 = x1 - z1;
    auto dz2 = x2 - z2;
    auto s2 = lineIntersect(z1, dz1, z2, dz2);
    if (!isnil(s2) && s2.x > 0) {
        auto s1 = lineIntersect(z1, dz1, u1, n1);
        if (s1.x > s2.x) {
            // TODO: check if z1-x1 etc. is intersected (hard to trigger)
            if (z1 != v1->p) ret.push_back(z1);
            ret.push_back(z1 + s2.x * dz1);
            if (z2 != v2->p) ret.push_back(z2);
            ret.push_back(v2->p);
            return ret;
        }
    }
    // stroke intersects closer, we're good
    if (z1 != v1->p) ret.push_back(z1);
    ret.push_back(x1);
    ret.push_back(x2);
    if (z2 != v2->p) ret.push_back(z2);
    ret.push_back(v2->p);
    return ret;
}

void PslgVertex::setPoint(const Vec2f& p) {
    if (this->p == p) return;
    this->p = p;
    sortEdges();
    for (PslgEdge* e : edges) {
        e->otherVertex(this)->sortEdges();
    }
}

void PslgVertex::sortEdges() {
    std::sort(edges.begin(), edges.end(), [this](const PslgEdge* e1, const PslgEdge* e2) {
        return e1->diamondAngle(this) < e2->diamondAngle(this);
    });
}

struct Pslg {

    std::vector<std::unique_ptr<PslgVertex>> verts;
    std::vector<std::unique_ptr<PslgEdge>> edges;

    PslgVertex* addVertex(float x, float y) {
        return verts.emplace_back(std::make_unique<PslgVertex>(x, y)).get();
    }

    PslgEdge* addEdge(PslgVertex* v1, PslgVertex* v2, float thickness) {
        return edges.emplace_back(std::make_unique<PslgEdge>(v1, v2, thickness)).get();
    }
};

Pslg pslg;

void initPslg() {
    auto v0 = pslg.addVertex(200, 100);
    auto v1 = pslg.addVertex(100, 100);
    auto v2 = pslg.addVertex(200, 200);
    auto v3 = pslg.addVertex(300, 100);
    auto v4 = pslg.addVertex(400, 100);
    auto v5 = pslg.addVertex(400, 200);
    auto v6 = pslg.addVertex(100, 200);
    auto e1 = pslg.addEdge(v0, v1, 30);
    auto e2 = pslg.addEdge(v0, v2, 30);
    auto e3 = pslg.addEdge(v0, v3, 30);
    auto e4 = pslg.addEdge(v3, v4, 30);
    auto e5 = pslg.addEdge(v4, v5, 30);
    auto e6 = pslg.addEdge(v2, v5, 30);
    auto e7 = pslg.addEdge(v1, v6, 30);
}

class Handle {
public:
    virtual ~Handle() = default;
    virtual void startMoving(const Vec2f& origin) = 0;
    virtual void move(const Vec2f& delta) = 0;
    virtual Vec2f get() const = 0;
    virtual const std::type_info& typeInfo() const = 0;
    bool selected = false;
};

template <typename T>
class AbstractHandle : public Handle {
public:
    const std::type_info& typeInfo() const override {
        return typeinfo;
    }
    static const std::type_info& typeinfo;
};

template <typename T>
const std::type_info& AbstractHandle<T>::typeinfo = typeid(T);

enum ToolAction {
    HOVERING,
    PRESSING_MOVING,
    MOVING,
    PRESSING_SELECTING,
    SELECTING
};

ToolAction toolAction = HOVERING;
bool dirty = true;
std::vector<std::unique_ptr<Handle>> handles;
constexpr float handleRadius = 7;
constexpr float handleRadius2 = handleRadius * handleRadius;
Handle* activeHandle = nullptr;
Handle* controlHandle = nullptr;
Vec2f selection_p1;
Vec2f selection_p2;

class PslgVertexHandle : public AbstractHandle<PslgVertexHandle> {
public:
    explicit PslgVertexHandle(PslgVertex* v): v(v) {}
    void startMoving(const Vec2f& p) override {
        p0 = p;
        v0 = v->p;
    }
    void move(const Vec2f& p) override {
        v->setPoint(v0 + p - p0);
    }
    Vec2f get() const override {
        return v->p;
    }
    PslgVertex* v;
    Vec2f p0;
    Vec2f v0;
};

template <size_t I>
class PslgThicknessHandle : public AbstractHandle<PslgThicknessHandle<I>> {
public:
    explicit PslgThicknessHandle(PslgEdge* e): e(e) {}
    void startMoving(const Vec2f& p) override {}
    void move(const Vec2f& p) override {
        // don't do anything if there are other types of handles
        // this is a pretty nasty way to do this...
        for (const auto& h : handles) {
            if (!h->selected) continue;
            auto& t = h->typeInfo();
            if (t == PslgThicknessHandle<0>::typeinfo) continue;
            if (t == PslgThicknessHandle<1>::typeinfo) continue;
            return;
        }
        float d = e->line().distance(p);
        e->thickness = d;
    }
    Vec2f get() const override {
        return std::get<I>(e->midPoints());
    }
    PslgEdge* e;
};

class Vec2fHandle : public AbstractHandle<Vec2fHandle> {
public:
    explicit Vec2fHandle(Vec2f* v): v(v) {}
    void startMoving(const Vec2f& p) override {
        p0 = p;
        v0 = *v;
    }
    void move(const Vec2f& p) override {
        *v = v0 + p - p0;
    }
    Vec2f get() const override {
        return *v;
    }
    Vec2f* v;
    Vec2f p0;
    Vec2f v0;
};

void initHandles() {
    for (auto* v = polygon; v != polygon + nvertices; ++v) {
        handles.emplace_back(std::make_unique<Vec2fHandle>(v));
    }
    for (const auto& e: pslg.edges) {
        handles.emplace_back(std::make_unique<PslgThicknessHandle<0>>(e.get()));
        handles.emplace_back(std::make_unique<PslgThicknessHandle<1>>(e.get()));
    }
    for (const auto& v : pslg.verts) {
        handles.emplace_back(std::make_unique<PslgVertexHandle>(v.get()));
    }
}

void draw();

void errorcb(int error, const char* desc)
{
    printf("GLFW error %d: %s\n", error, desc);
}

void framebufferSizeCallback(GLFWwindow *window, int width, int height)
{
    dirty = true;
    draw();
}

struct bbox {
    Vec2f pmin;
    Vec2f pmax;

    bbox(const Vec2f& p1, const Vec2f& p2):
            pmin(fmin(p1.x, p2.x), fmin(p1.y, p2.y)),
            pmax(fmax(p1.x, p2.x), fmax(p1.y, p2.y))
    {
    }

    bool intersects(const Vec2f& p) const {
        return (pmin.x <= p.x) && (p.x <= pmax.x) &&
               (pmin.y <= p.y) && (p.y <= pmax.y);
    }

    float x() const {
        return pmin.x;
    }

    float y() const {
        return pmin.y;
    }

    float w() const {
        return pmax.x - pmin.x;
    }

    float h() const {
        return pmax.y - pmin.y;
    }

};

template <typename T>
struct Reverse {
    T& iterable;
};

template <typename T>
auto begin(Reverse<T> r) {
    return std::rbegin(r.iterable);
}

template <typename T>
auto end(Reverse<T> r) {
    return std::rend(r.iterable);
}

template <typename T>
Reverse<T> reverse(T& iterable) {
    return {iterable};
}

void cursorPosCallback(GLFWwindow *window, double mx, double my)
{
    Vec2f mp = {(float) mx, (float) my};
    switch (toolAction) {
    case HOVERING: {
        Handle *newActiveHandle = nullptr;
        for (const auto& h : reverse(handles)) {
            if (sqdist(h->get(), mp) > handleRadius2) continue;
            newActiveHandle = h.get();
            break;
        }
        if (newActiveHandle != activeHandle) {
            activeHandle = newActiveHandle;
            dirty = true;
        }
        break;
    }
    case PRESSING_SELECTING:
        toolAction = SELECTING;
    case SELECTING: {
        selection_p2 = mp;
        bbox bb(selection_p1, selection_p2);
        for (const auto& h : handles) {
            h->selected = bb.intersects(h->get());
        }
        activeHandle = nullptr;
        dirty = true;
        break;
    }
    case PRESSING_MOVING:
        toolAction = MOVING;
    case MOVING:
        for (const auto& h : handles) {
            if (h->selected) h->move(mp);
        }
        dirty = true;
        break;
    }
    draw();
}

void deselectAll() {
    for (const auto& h : handles) {
        h->selected = false;
    }
}

void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods)
{
    double mx, my;
    glfwGetCursorPos(window, &mx, &my);
    Vec2f mp = {(float) mx, (float) my};
    switch (toolAction) {
    case HOVERING:
        // action == GLFW_PRESS
        if (activeHandle) {
            if (!activeHandle->selected) {
                deselectAll();
                activeHandle->selected = true;
            }
            for (const auto& h : handles) {
                if (h->selected) h->startMoving(mp);
            }
            controlHandle = activeHandle;
            toolAction = PRESSING_MOVING;
        } else {
            deselectAll();
            selection_p1 = mp;
            toolAction = PRESSING_SELECTING;
        }
        dirty = true;
        break;
    case PRESSING_MOVING:
        // action == GLFW_RELEASE
        deselectAll();
        activeHandle->selected = true;
    default:
        controlHandle = nullptr;
        toolAction = HOVERING;
        dirty = true;
        break;
    }
    draw();
}

void moveTo(const Vec2f& v) {
    nvgMoveTo(vg, v.x, v.y);
}

void lineTo(const Vec2f& v) {
    nvgLineTo(vg, v.x, v.y);
}

void circle(const Vec2f& v, float radius) {
    nvgCircle(vg, v.x, v.y, radius);
}

void drawObjects(float mx, float my) {
    Vec2f mp = {mx, my};
    nvgStrokeWidth(vg, 1);

    // draw polygon
    nvgBeginPath(vg);
    auto& v0 = polygon[0];
    nvgMoveTo(vg, v0.x, v0.y);
    for (int i = 1; i < nvertices; ++i) {
        auto& v = polygon[i];
        nvgLineTo(vg, v.x, v.y);
    }
    nvgLineTo(vg, v0.x, v0.y);
    nvgFillColor(vg, nvgRGBA(255,192,0,128));
    nvgStrokeColor(vg, nvgRGBA(255,192,0,255));
    nvgFill(vg);
    nvgStroke(vg);

    // pslg
    nvgStrokeWidth(vg, 1);

    // rectangles
    for (const auto& edge : pslg.edges) {
        nvgBeginPath(vg);
        auto rect = edge->cornerPoints();
        moveTo(rect[0]);
        lineTo(rect[1]);
        lineTo(rect[2]);
        lineTo(rect[3]);
        lineTo(rect[0]);
        nvgFillColor(vg, nvgRGBA(100, 100, 255, 100));
        nvgStrokeColor(vg, nvgRGBA(100, 100, 255, 200));
        //nvgFill(vg);
        //nvgStroke(vg);
    }

    // extrusion
    for (const auto& edge : pslg.edges) {
        nvgBeginPath(vg);
        auto pts = edge->miteredPoints();
        moveTo(pts[0]);
        for (size_t i = 1; i < pts.size; ++i) {
            lineTo(pts[i]);
        }
        nvgFillColor(vg, nvgRGBA(200, 200, 255, 100));
        nvgStrokeColor(vg, nvgRGBA(200, 200, 255, 200));
        nvgFill(vg);
        nvgStroke(vg);
    }

    // segments

    for (const auto& edge : pslg.edges) {
        nvgStrokeWidth(vg, 1);
        nvgBeginPath(vg);
        moveTo(edge->v1->p);
        lineTo(edge->v2->p);
        nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 255));
        nvgStroke(vg);
    }

    // handles
    nvgStrokeWidth(vg, 2);
    for (const auto& h : handles) {
        nvgBeginPath(vg);
        circle(h->get(), handleRadius);
        if (h->selected) {
            nvgFillColor(vg, nvgRGBA(50, 50, 255, 255));
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 255));
        } else {
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 255));
            nvgStrokeColor(vg, nvgRGBA(100, 100, 255, 255));
        }
        if (h.get() == activeHandle) {
            nvgStrokeColor(vg, nvgRGBA(255, 100, 100, 255));
        }
        if (h.get() == controlHandle) {
            nvgStrokeColor(vg, nvgRGBA(0, 0, 255, 255));
        }
        nvgFill(vg);
        nvgStroke(vg);
    }

    // selection rectangle
    if (toolAction == SELECTING) {
        nvgBeginPath(vg);
        bbox bb(selection_p1, selection_p2);
        nvgRect(vg, bb.x(), bb.y(), bb.w(), bb.h());
        nvgStrokeColor(vg, nvgRGBA(100, 100, 255, 255));
        nvgStroke(vg);
    }
}

void draw() {
    if (!dirty) return;
    dirty = false;

    double mx, my;
    int winWidth, winHeight;
    int fbWidth, fbHeight;
    float pxRatio;

    glfwGetCursorPos(window, &mx, &my);
    glfwGetWindowSize(window, &winWidth, &winHeight);
    glfwGetFramebufferSize(window, &fbWidth, &fbHeight);

    // Calculate pixel ration for hi-dpi devices.
    pxRatio = (float)fbWidth / (float)winWidth;

    // Update and render
    glViewport(0, 0, fbWidth, fbHeight);
    glClearColor(0.3f, 0.3f, 0.32f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT|GL_STENCIL_BUFFER_BIT);

    nvgBeginFrame(vg, winWidth, winHeight, pxRatio);
    drawObjects(mx, my);
    nvgEndFrame(vg);

    glfwSwapBuffers(window);
}

int main()
{

    if (!glfwInit()) {
        printf("Failed to init GLFW.");
        return -1;
    }

    glfwSetErrorCallback(errorcb);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    //glfwWindowHint(GLFW_SAMPLES, 4);

    window = glfwCreateWindow(1000, 600, "PSLG Extruder", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return -1;
    }

    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
        fprintf(stderr, "Failed to load GL\n");
        glfwTerminate();
        return -1;
    }

    vg = nvgCreateGL3(NVG_ANTIALIAS | NVG_STENCIL_STROKES | NVG_DEBUG);
    if (vg == nullptr) {
        printf("Could not init nanovg.\n");
        return -1;
    }

    glfwSwapInterval(1);
    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
    glfwSetCursorPosCallback(window, cursorPosCallback);
    glfwSetMouseButtonCallback(window, mouseButtonCallback);

    initVertices();
    initPslg();
    initHandles();
    draw();
    while (!glfwWindowShouldClose(window))
    {
        glfwWaitEvents();
    }

    nvgDeleteGL3(vg);

    glfwTerminate();
    return 0;
}
