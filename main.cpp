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

#include "pslg.hpp"

GLFWwindow* window;
NVGcontext* vg;

using Float = float;
using Size = size_t;
using PslgVertexf = PslgVertex<Float>;
using PslgEdgef = PslgEdge<Float>;
using Pslgf = Pslg<Float>;
using Vec2f = Vec2<Float>;

constexpr size_t nvertices = 16;
Vec2f polygon[nvertices] = {};
constexpr Float PI = Float(3.1415926);

void initVertices() {
    for (Size i = 0; i < nvertices; ++i) {
        auto& v = polygon[i];
        Float a = i * 2 * PI / nvertices;
        v.x = 600 + cos(a) * 100;
        v.y = 200 + sin(a) * 100;
    }
}

Pslgf pslg;

void initPslg() {
    auto v11 = pslg.addVertex(100, 100);
    auto v12 = pslg.addVertex(200, 100);
    auto v13 = pslg.addVertex(300, 100);
    auto v14 = pslg.addVertex(400, 100);
    auto e11 = pslg.addEdge(v12, v11, 30);
    auto e12 = pslg.addEdge(v13, v12, 30);
    auto e13 = pslg.addEdge(v14, v13, 30);

    auto v21 = pslg.addVertex(100, 200);
    auto v22 = pslg.addVertex(200, 200);
    auto v23 = pslg.addVertex(300, 200);
    auto v24 = pslg.addVertex(400, 200);
    auto e21 = pslg.addEdge(v21, v22, 30);
    auto e22 = pslg.addEdge(v22, v23, 30);
    auto e23 = pslg.addEdge(v23, v24, 30);

    auto e3 = pslg.addEdge(v12, v22, 30);
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
constexpr Float handleRadius = 7;
constexpr Float handleRadius2 = handleRadius * handleRadius;
Handle* activeHandle = nullptr;
Handle* controlHandle = nullptr;
Vec2f selection_p1;
Vec2f selection_p2;

class PslgVertexHandle : public AbstractHandle<PslgVertexHandle> {
public:
    explicit PslgVertexHandle(PslgVertexf* v): v(v) {}
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
    PslgVertexf* v;
    Vec2f p0;
    Vec2f v0;
};

template <size_t I>
class PslgThicknessHandle : public AbstractHandle<PslgThicknessHandle<I>> {
public:
    explicit PslgThicknessHandle(PslgEdgef* e): e(e) {}
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
        Float d = e->line().distance(p);
        e->setThickness(d);
    }
    Vec2f get() const override {
        return std::get<I>(e->midPoints());
    }
    PslgEdgef* e;
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

    Float x() const {
        return pmin.x;
    }

    Float y() const {
        return pmin.y;
    }

    Float w() const {
        return pmax.x - pmin.x;
    }

    Float h() const {
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
    Vec2f mp = {(Float) mx, (Float) my};
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
    Vec2f mp = {(Float) mx, (Float) my};
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

void circle(const Vec2f& v, Float radius) {
    nvgCircle(vg, v.x, v.y, radius);
}

void drawObjects(Float mx, Float my) {
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
        moveTo(edge->p1());
        lineTo(edge->p2());
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
