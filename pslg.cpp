#include "pslg.hpp"

PslgVertex::PslgVertex(float x, float y) :
        p(x, y)
{
}

void PslgVertex::addEdge(PslgEdge* edge) {
    edges.emplace_back(edge);
    sortEdges();
}

PslgEdge* PslgVertex::nextEdge(const PslgEdge* edge) const {
    if (edges.empty()) return nullptr;
    if (edges.back() == edge) return edges.front();
    for (int i = 0; i < edges.size() - 1; ++i) {
        if (edges[i] == edge) return edges[i + 1];
    }
    return nullptr;
}

PslgEdge* PslgVertex::prevEdge(const PslgEdge* edge) const {
    if (edges.empty()) return nullptr;
    if (edges.front() == edge) return edges.back();
    for (int i = 1; i < edges.size(); ++i) {
        if (edges[i] == edge) return edges[i - 1];
    }
    return nullptr;
}

PslgEdge::PslgEdge(PslgVertex* v1, PslgVertex* v2, float thickness):
        v1(v1), v2(v2), thickness(thickness)
{
    v1->addEdge(this);
    v2->addEdge(this);
}

Vec2f PslgEdge::diff() const {
    return v2->p - v1->p;
}

float PslgEdge::diamondAngle(const PslgVertex* v) const {
    if (v == v1) return ::diamondAngle(diff());
    if (v == v2) return ::diamondAngle(-diff());
    return std::numeric_limits<float>::quiet_NaN();
}

float PslgEdge::angle(const PslgVertex* v) const {
    if (v == v1) return ::angle(diff());
    if (v == v2) return ::angle(-diff());
    return std::numeric_limits<float>::quiet_NaN();
}

PslgVertex* PslgEdge::otherVertex(const PslgVertex* v) const {
    if (v == v1) return v2;
    if (v == v2) return v1;
    return nullptr;
}

std::array<Vec2f, 2> PslgEdge::midPoints() const {
    auto dp = diff();
    auto mp = v1->p + 0.5f * dp;
    auto n1 = thickness * normalized(rot90ccw(dp));
    return {
            mp + n1,
            mp - n1,
    };
};

std::array<Vec2f, 4> PslgEdge::cornerPoints() const {
    auto n1 = thickness * normalized(rot90ccw(diff()));
    return {
            v1->p + n1,
            v2->p + n1,
            v2->p - n1,
            v1->p - n1
    };
};

Line2f PslgEdge::lineFrom(const PslgVertex* v) const {
    return { v->p, otherVertex(v)->p - v->p };
}

Line2f PslgEdge::prevLine(const PslgVertex* v) const {
    return v->prevEdge(this)->lineFrom(v);
}

Line2f PslgEdge::nextLine(const PslgVertex* v) const {
    return v->nextEdge(this)->lineFrom(v);
}

PslgVertex* PslgEdge::prevVertex(const PslgVertex* v) const {
    return v->prevEdge(this)->otherVertex(v);
}

PslgVertex* PslgEdge::nextVertex(const PslgVertex* v) const {
    return v->prevEdge(this)->otherVertex(v);
}

Line2f PslgEdge::line() const {
    return {v1->p, v2->p - v1->p};
}

enum Side {
    LEFT,
    RIGHT
};

constexpr float miterLimit = 2.0f;

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

FixedVector<Vec2f, 10> PslgEdge::miteredPoints() const {
    FixedVector<Vec2f, 10> pts;
    pts.push_back(v1->p);
    miteredEdge(pts, this, v1, v2);
    pts.push_back(v2->p);
    miteredEdge(pts, this, v2, v1);
    pts.push_back(v1->p);
    return pts;
};

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

PslgVertex* Pslg::addVertex(float x, float y) {
    return verts.emplace_back(std::make_unique<PslgVertex>(x, y)).get();
}

PslgEdge* Pslg::addEdge(PslgVertex* v1, PslgVertex* v2, float thickness) {
    return edges.emplace_back(std::make_unique<PslgEdge>(v1, v2, thickness)).get();
}

