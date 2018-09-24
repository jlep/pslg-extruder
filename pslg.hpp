#pragma once

#include "vec.hpp"
#include "fixedvector.hpp"

#include <vector>
#include <array>
#include <cmath>

struct PslgVertex;
struct PslgEdge;
struct Pslg;

// edges have ccw ordering in right-handed coordinate system
struct PslgVertex {

    Vec2f p;
    std::vector<PslgEdge*> edges;

    explicit PslgVertex(float x, float y);
    void addEdge(PslgEdge* edge);
    PslgEdge* nextEdge(const PslgEdge* edge) const;
    PslgEdge* prevEdge(const PslgEdge* edge) const;
    void setPoint(const Vec2f& p);
    void sortEdges();
};

struct PslgEdge {

    PslgVertex* v1;
    PslgVertex* v2;
    float thickness;

    PslgEdge(PslgVertex* v1, PslgVertex* v2, float thickness);
    Vec2f diff() const;
    float diamondAngle(const PslgVertex* v) const;
    PslgVertex* otherVertex(const PslgVertex* v) const;
    std::array<Vec2f, 2> midPoints() const;
    std::array<Vec2f, 4> cornerPoints() const;
    FixedVector<Vec2f, 10> miteredPoints() const;
    Line2f line() const;
};

struct Pslg {

    std::vector<std::unique_ptr<PslgVertex>> verts;
    std::vector<std::unique_ptr<PslgEdge>> edges;

    PslgVertex* addVertex(float x, float y);

    PslgEdge* addEdge(PslgVertex* v1, PslgVertex* v2, float thickness);
};
