#pragma once
#include "predicates.h"

#include <iostream>
#include <fstream>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <boost/qvm/mat.hpp>
#include <boost/qvm/vec.hpp>
#include <boost/qvm/vec_access.hpp>
#include <boost/qvm/vec_operations.hpp>
#include <boost/qvm/mat_operations.hpp>
#include <boost/qvm/vec_mat_operations.hpp>
#include <boost/format.hpp>

#include <algorithm>
#include <limits>

using namespace boost::qvm;

using Vec2 = vec<double, 2>;
using Mat2 = mat<double, 2, 2>;
using Point = boost::geometry::model::d2::point_xy<double>;

Point toPoint(const Vec2& v) {
    return {X(v), Y(v)};
}

std::ostream& operator<<(std::ostream& o, const Vec2& v) {
    using namespace boost::qvm;
    return o << '[' << X(v) << ", " << Y(v) << ']';
}

struct Vertex;
struct Edge;
struct Pslg;

Vec2 rotate270(const Vec2& v) {
    return {Y(v), -X(v)};
}

Vec2 rotate90(const Vec2& v) {
    return {-Y(v), X(v)};
}

constexpr Vec2 nilvec = {std::numeric_limits<double>::quiet_NaN(), 0};

bool isnil(const Vec2& v) {
    return isnan(X(v));
}

// det of |a b|
double det(const Vec2& a, const Vec2& b) {
    return X(a) * Y(b) - X(b) * Y(a);
}

double dot(const Vec2& a, const Vec2& b) {
    return X(a) * X(b) + Y(a) * Y(b);
}

// 1: 0-90
// 2: 90-180
// 3: 180-270
// 4: 270-360
int quadrant(const Vec2& a, const Vec2& b) {
    double dott = dot(a, b);
    double dett = det(a, b);
    if (dett >= 0) {
        return (dott >= 0) ? 1 : 2;
    }
    return (dott <= 0) ? 3 : 4;
}

// solve [a b] * x = c
Vec2 solve2x2(Vec2 a, Vec2 b, Vec2 c) {
    double d = det(a, b);
    if (d == 0) return nilvec;
    double x = det(c, b);
    double y = det(a, c);
    return {x / d, y / d};
}

// line intersection:
//
// solve r from
// p1 + r1 * n1 = p2 + r2 * n2
//
//        x
//
//
//    /n1    \n2
//   /        \
//  p1        p2

Vec2 line_intersect(const Vec2& p1, const Vec2& n1,
                    const Vec2& p2, const Vec2& n2) {
    return solve2x2(n1, -n2, p2 - p1);
}

// extrusion from a joint:
//
//
//
//     (x1,x2)
//
//     p1  p2
//      \  /
//       p0
//
std::pair<Vec2, Vec2> extrusion2(const Vec2& p0, const Vec2& p1, const Vec2& p2,
                                 double thickness1, double thickness2) {
    // difference vectors
    Vec2 n1 = p1 - p0;
    Vec2 n2 = p2 - p0;
    Vec2 m1 = rotate270(n1);
    Vec2 m2 = rotate90(n2);
    // parallel starting points
    Vec2 q1 = p0 + thickness1 * normalized(m1);
    Vec2 q2 = p0 + thickness2 * normalized(m2);
    Vec2 r = line_intersect(q1, n1, q2, n2);
    if (isnil(r)) {
        // parallel lines, return parallel points
        return {q1, q2};
    }
    // quadrante is ccw while order here is cw
    auto q = quadrant(n2, n1);
    if (q == 1) {
        Vec2 r11 = line_intersect(q1, n1, p1, m1);
        Vec2 r12 = line_intersect(q1, n1, p2, m2);
        double r1 = fmin(fmin(X(r11), X(r12)), X(r));
        Vec2 r21 = line_intersect(q2, n2, p1, m1);
        Vec2 r22 = line_intersect(q2, n2, p2, m2);
        double r2 = fmin(fmin(X(r21), X(r22)), Y(r));
        return {
                //q1 + r1 * n1,
                //q2 + r2 * n2
                q1 + X(r) * n1,
                q2 + Y(r) * n2
        };
    }
    if (q == 4) {
        // > 270 degrees, do something about the spikes
        return {
                q1 + X(r) * n1,
                q2 + Y(r) * n2
        };
    }
    // we're safe
    return {
            q1 + X(r) * n1,
            q2 + Y(r) * n2
    };
}

// extrusion from a joint:
//
//

std::pair<Vec2, Vec2> extrusion(const Vec2& v1, const Vec2& v2, const Vec2& v3, double thickness1, double thickness2) {
    Vec2 n1 = v2 - v1;
    Vec2 n2 = v3 - v2;
    Vec2 p1 = v1 + thickness1 * rotate270(normalized(n1));
    Vec2 p2 = v2 + thickness2 * rotate270(normalized(n2));
    // solve intersection between p1 + r1 * n1 and p2 + r2 * n2 from
    // [n1 -n2] * r = c with Cramer's rule
    Vec2 c = p2 - p1;
    double det = X(n2) * Y(n1) - X(n1) * Y(n2);
    if (det == 0) {
        // return two different points
        return {p1 + n1, p2};
    }
    double det2 = X(n1) * Y(c) - X(c) * Y(n1);
    double r2 = det2 / det;
    Vec2 x = p2 + r2 * n2;
    return {x, x};
}

struct Vertex {

    Vec2 point;
    // in ccw order
    std::vector<Edge*> edges;

    Vertex(double x, double y): point{x, y} {}

    void sort();

    Edge* prev(const Edge* edge) const {
        if (edges.empty()) return nullptr;
        if (edges.front() == edge) return edges.back();
        for (size_t i = 1; i < edges.size(); ++i) {
            if (edges[i] == edge) return edges[i - 1];
        }
        return nullptr;
    }

    Edge* next(const Edge* edge) const {
        if (edges.empty()) return nullptr;
        if (edges.back() == edge) return edges.front();
        for (size_t i = 0; i < edges.size() - 1; ++i) {
            if (edges[i] == edge) return edges[i + 1];
        }
        return nullptr;
    }

    const double* cs() const {
        return point.a;
    }

    double x() const {
        return X(point);
    }

    double y() const {
        return Y(point);
    }

    Point p() const {
        return toPoint(point);
    }
};

std::pair<Vec2, Vec2> extrusion(Vertex* v1, Vertex* v2, Vertex* v3, double thickness1, double thickness2) {
    return extrusion(v1->point, v2->point, v3->point, thickness1, thickness2);
}

std::pair<Vec2, Vec2> extrusion2(Vertex* v0, Vertex* v1, Vertex* v2, double thickness1, double thickness2) {
    return extrusion2(v0->point, v1->point, v2->point, thickness1, thickness2);
}

double orient2d(const Point& p1, const Point& p2, const Point& p3) {
    double c[] = {p1.x(), p1.y(), p2.x(), p2.y(), p3.x(), p3.y()};
    return predicates::orient2d(c, c + 2, c + 4);
}

void mapTriangle(boost::geometry::svg_mapper<Point>& mapper, const std::string& style,
                 const Point& p1, const Point& p2, const Point& p3) {
    namespace bg = boost::geometry;
    // skip slivers
    if (orient2d(p1, p2, p3) <= 0) {
        return;
    }
    bg::model::polygon<Point> t1;
    bg::append(t1.outer(), p1);
    bg::append(t1.outer(), p2);
    bg::append(t1.outer(), p3);
    bg::append(t1.outer(), p1);
    mapper.map(t1, style);
}

double diamondAngle(double y, double x)
{
    if (y >= 0) {
        return (x >= 0) ? (y / (x + y)) : (1 - x / (-x + y));
    } else {
        return (x < 0) ? (2 - y / (-x - y)) : (3 + x / (x - y));
    }
}

// undirected edge
struct Edge {

    Vertex* v1;
    Vertex* v2;
    double thickness;

    Edge(Vertex* v1, Vertex* v2, double thickness): v1(v1), v2(v2), thickness(thickness) {}

    double diamondAngle(Vertex* v) const
    {
        double dx = xdiff();
        double dy = ydiff();
        if (v2 == v) {
            dx = -dx;
            dy = -dy;
        }
        return ::diamondAngle(dy, dx);
    }

    double angle(Vertex* v) const {
        double dx = xdiff();
        double dy = ydiff();
        if (v2 == v) {
            dx = -dx;
            dy = -dy;
        }
        return atan2(dy, dx);
    }

    double xdiff() const {
        return X(v2->point) - X(v1->point);
    }

    double ydiff() const {
        return Y(v2->point) - Y(v1->point);
    }

    template <typename P>
    void map(boost::geometry::svg_mapper<P>& mapper) const {
        namespace bg = boost::geometry;
        boost::geometry::model::linestring<P> c;
        const auto& p1 = v1->point;
        const auto& p2 = v2->point;
        c.push_back(P(X(p1), Y(p1)));
        c.push_back(P(X(p2), Y(p2)));
        mapper.map(c, "fill:none;stroke:rgb(0,0,0);stroke-width:1");

        bg::model::polygon<Point> p;
        bg::append(p.outer(), v1->p());
        {
            Edge *e = v1->next(this);
            Vertex *q0 = v1;
            Vertex *q1 = e->otherVertex(v1);
            Vertex *q2 = v2;
            auto r = extrusion2(q0, q1, q2, e->thickness, thickness);
            auto q = r.second;
            bg::append(p.outer(), toPoint(q));
            std::cout << q << std::endl;
        }
        {
            Edge *e = v2->prev(this);
            Vertex *q0 = v2;
            Vertex *q1 = v1;
            Vertex *q2 = e->otherVertex(v2);
            auto r = extrusion2(q0, q1, q2, thickness, e->thickness);
            auto q = r.first;
            bg::append(p.outer(), toPoint(q));
            std::cout << q << std::endl;
        }
        bg::append(p.outer(), v2->p());
        {
            Edge *e = v2->next(this);
            Vertex *q0 = v2;
            Vertex *q1 = e->otherVertex(v2);
            Vertex *q2 = v1;
            auto r = extrusion2(q0, q1, q2, e->thickness, thickness);
            auto q = r.second;
            bg::append(p.outer(), toPoint(q));
            std::cout << q << std::endl;
        }
        {
            Edge *e = v1->prev(this);
            Vertex *q0 = v1;
            Vertex *q1 = v2;
            Vertex *q2 = e->otherVertex(v1);;
            auto r = extrusion2(q0, q1, q2, thickness, e->thickness);
            auto q = r.first;
            bg::append(p.outer(), toPoint(q));
            std::cout << q << std::endl;
        }

        bg::append(p.outer(), v1->p());
        int r = rand() % 255;
        int g = rand() % 255;
        int b = rand() % 255;
        std::cout << "c:" << r << "," << g << "," << b << "\n";
        auto s = boost::str(boost::format("fill-opacity:0.3;fill:rgb(%1%,%2%,%3%);stroke:rgb(%1%,%2%,%3%);stroke-width:1;stroke-linecap=round;stroke-linejoin=round") % r % g % b);
        //mapper.map(p, s);
        // triangles
        const auto& ring = p.outer();
        mapTriangle(mapper, s, ring[0], ring[5], ring[1]);
        mapTriangle(mapper, s, ring[3], ring[2], ring[4]);
        mapTriangle(mapper, s, ring[1], ring[5], ring[2]);
        mapTriangle(mapper, s, ring[2], ring[5], ring[4]);
    }

    Vertex* commonVertex(const Edge* edge) const {
        if (v1 == edge->v1) return v1;
        if (v1 == edge->v2) return v1;
        if (v2 == edge->v1) return v2;
        if (v2 == edge->v2) return v2;
        return nullptr;
    }

    Vertex* otherVertex(const Vertex* vertex) const {
        if (v1 == vertex) return v2;
        if (v2 == vertex) return v1;
        return nullptr;
    }
};

void Vertex::sort() {
    std::sort(edges.begin(), edges.end(), [this](Edge* e1, Edge* e2) {
        return e1->diamondAngle(this) < e2->diamondAngle(this);
    });
}

struct Pslg {

    std::vector<std::unique_ptr<Vertex>> vertices;
    std::vector<std::unique_ptr<Edge>> edges;

    Vertex* addVertex(double x, double y) {
        return vertices.emplace_back(std::make_unique<Vertex>(x, y)).get();
    }

    Edge* addEdge(Vertex* v1, Vertex* v2, double thickness) {
        Edge* edge = edges.emplace_back(std::make_unique<Edge>(v1, v2, thickness)).get();
        v1->edges.push_back(edge);
        v2->edges.push_back(edge);
        return edge;
    }

    void build() {
        for (const auto& v : vertices) {
            v->sort();
        }
    }

    template <typename P>
    void map(boost::geometry::svg_mapper<P>& mapper) const {
        for (const auto& e : edges) {
            e->map(mapper);
        }
    }
};

int mmain(int argc, char** argv) {

    srand(0);
    Pslg pslg;

    auto v0 = pslg.addVertex(50, 50);
    auto v1 = pslg.addVertex(80, 70);
    auto v2 = pslg.addVertex(60, 60);
    auto v3 = pslg.addVertex(60, 90);
    auto e1 = pslg.addEdge(v0, v1, 2);
    auto e2 = pslg.addEdge(v0, v2, 2);
    auto e3 = pslg.addEdge(v2, v3, 2);
    //auto e3 = pslg.addEdge(v3, v4, 2);
    /*
    auto v1 = pslg.addVertex(40, 50);
    auto v2 = pslg.addVertex(50, 50);
    auto v3 = pslg.addVertex(90, 90);
    auto v4 = pslg.addVertex(50, 10);
    auto v5 = pslg.addVertex(90, 50);
    auto v6 = pslg.addVertex(10, 90);
    auto e1 = pslg.addEdge(v1, v2, 10);
    auto e2 = pslg.addEdge(v3, v2, 10);
    auto e3 = pslg.addEdge(v4, v2, 8);
    auto e4 = pslg.addEdge(v5, v2, 5);
    auto e5 = pslg.addEdge(v1, v6, 15);
     */
    pslg.build();

    typedef boost::geometry::model::d2::point_xy<double> point_type;


    // Declare a stream and an SVG mapper
    std::ofstream svg("my_map.svg");
    boost::geometry::svg_mapper<point_type> mapper(svg, 800, 800);
    boost::geometry::model::polygon<point_type> b;
    boost::geometry::read_wkt("POLYGON((0 0,0 100,100 100,100 0,0 0))", b);
    mapper.add(b);
    pslg.map(mapper);

    //pslg.edges[1]->map(mapper);

    return 0;
}

