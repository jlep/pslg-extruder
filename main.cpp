#include "pslg.hpp"
//#include "linmath.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <nanovg.h>
#define NANOVG_GL3_IMPLEMENTATION
#include <nanovg_gl.h>

GLFWwindow* window;
NVGcontext* vg;

struct point {
    union {
        struct {
            float x;
            float y;
        };
        float cs[2];
    };

    point(): x(0), y(0) {}
    point(float x, float y): x(x), y(y) {}

    bool operator==(const point& p) const {
        return x == p.x && y == p.y;
    }
    bool operator!=(const point& p) const {
        return !(*this == p);
    }

    point operator-(const point& p) const {
        return {x - p.x, y - p.y};
    }

    point operator+(const point& p) const {
        return {x + p.x, y + p.y};
    }
};

std::ostream& operator<<(std::ostream& o, const point& p) {
    return o << '(' << p.x << ", " << p.y << ')';
}

struct vertex {
    point p;
    // used for moving
    point prev_p;
    bool selected;
};

std::ostream& operator<<(std::ostream& o, const vertex& v) {
    return o << "{point=" << v.p << ", selected=" << (v.selected ? "true": "false") << '}';
}

constexpr int nvertices = 16;
vertex polygon[nvertices] = {};
constexpr float PI = 3.1415926f;

void initVertices() {
    for (int i = 0; i < nvertices; ++i) {
        auto& v = polygon[i];
        float a = i * 2 * PI / nvertices;
        v.p.x = 200 + cos(a) * 100;
        v.p.y = 200 + sin(a) * 100;
        v.selected = false;
    }
}

enum tool_action_t {
    HOVERING,
    PRESSING_MOVING,
    MOVING,
    PRESSING_SELECTING,
    SELECTING
};
tool_action_t tool_action = HOVERING;

bool dirty = true;
constexpr float handle_radius = 7;
constexpr float handle_radius2 = handle_radius * handle_radius;
vertex* active_vertex = nullptr;
vertex* control_vertex = nullptr;
point move_start;
point selection_p1;
point selection_p2;

void draw();

float dot(const point& p1, const point& p2) {
    return p1.x * p2.x + p1.y * p1.y;
}

float dist2(const point& p1, const point& p2) {
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    return dx * dx + dy * dy;
}

void errorcb(int error, const char* desc)
{
    printf("GLFW error %d: %s\n", error, desc);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    dirty = true;
    draw();
}

struct bbox {
    point pmin;
    point pmax;

    bbox(const point& p1, const point& p2):
            pmin(fmin(p1.x, p2.x), fmin(p1.y, p2.y)),
            pmax(fmax(p1.x, p2.x), fmax(p1.y, p2.y))
    {
    }

    bool intersects(const point& p) const {
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

void cursor_pos_callback(GLFWwindow* window, double mx, double my)
{
    point mp = {(float) mx, (float) my};
    switch (tool_action) {
    case HOVERING: {
        vertex *new_active_vertex = nullptr;
        for (int i = nvertices - 1; i >= 0; --i) {
            auto* v = polygon + i;
            if (dist2(v->p, mp) > handle_radius2) continue;
            new_active_vertex = v;
            break;
        }
        if (new_active_vertex != active_vertex) {
            active_vertex = new_active_vertex;
            dirty = true;
        }
        break;
    }
    case PRESSING_SELECTING:
        tool_action = SELECTING;
    case SELECTING: {
        selection_p2 = mp;
        bbox bb(selection_p1, selection_p2);
        for (auto *v = polygon; v != polygon + nvertices; ++v) {
            v->selected = bb.intersects(v->p);
        }
        active_vertex = nullptr;
        dirty = true;
        break;
    }
    case PRESSING_MOVING:
        tool_action = MOVING;
    case MOVING:
        auto dp = mp - move_start;
        for (vertex* v = polygon; v != polygon + nvertices; ++v) {
            if (!v->selected) continue;
            v->p = v->prev_p + dp;
        }
        dirty = true;
        break;
    }
    draw();
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
    double mx, my;
    glfwGetCursorPos(window, &mx, &my);
    point mp = {(float) mx, (float) my};
    switch (tool_action) {
    case HOVERING:
        // action == GLFW_PRESS
        if (active_vertex) {
            if (!active_vertex->selected) {
                for (auto& v : polygon) {
                    v.selected = false;
                }
                active_vertex->selected = true;
            }
            for (auto& v : polygon) {
                v.prev_p = v.p;
            }
            move_start = mp;
            control_vertex = active_vertex;
            tool_action = PRESSING_MOVING;
        } else {
            for (auto& v : polygon) {
                v.selected = false;
            }
            selection_p1 = mp;
            tool_action = PRESSING_SELECTING;
        }
        dirty = true;
        break;
    case PRESSING_MOVING:
        // action == GLFW_RELEASE
        for (auto& v : polygon) {
            v.selected = false;
        }
        active_vertex->selected = true;
    default:
        control_vertex = nullptr;
        tool_action = HOVERING;
        dirty = true;
        break;
    }
    draw();
}


void drawObjects(float mx, float my) {
    point mp = point{mx, my};
    nvgStrokeWidth(vg, 1);

    // draw polygon
    nvgBeginPath(vg);
    auto& v0 = polygon[0];
    nvgMoveTo(vg, v0.p.x, v0.p.y);
    for (int i = 1; i < nvertices; ++i) {
        auto& v = polygon[i];
        nvgLineTo(vg, v.p.x, v.p.y);
    }
    nvgLineTo(vg, v0.p.x, v0.p.y);
    nvgFillColor(vg, nvgRGBA(255,192,0,128));
    nvgStrokeColor(vg, nvgRGBA(255,192,0,255));
    nvgFill(vg);
    nvgStroke(vg);

    // handles
    nvgStrokeWidth(vg, 2);
    for (auto* v = polygon; v != polygon + nvertices; ++v) {
        nvgBeginPath(vg);
        nvgCircle(vg, v->p.x, v->p.y, handle_radius);

        if (v->selected) {
            nvgFillColor(vg, nvgRGBA(50, 50, 255, 255));
            nvgStrokeColor(vg, nvgRGBA(255, 255, 255, 255));
        } else {
            nvgFillColor(vg, nvgRGBA(255, 255, 255, 255));
            nvgStrokeColor(vg, nvgRGBA(100, 100, 255, 255));
        }
        if (v == active_vertex) {
            nvgStrokeColor(vg, nvgRGBA(255, 100, 100, 255));
        }
        if (v == control_vertex) {
            nvgStrokeColor(vg, nvgRGBA(0, 0, 255, 255));
        }
        nvgFill(vg);
        nvgStroke(vg);
    }

    // selection rectangle
    if (tool_action == SELECTING) {
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

    window = glfwCreateWindow(1000, 600, "NanoVG", nullptr, nullptr);
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
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, cursor_pos_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    initVertices();
    draw();
    while (!glfwWindowShouldClose(window))
    {
        glfwWaitEvents();
    }

    nvgDeleteGL3(vg);

    glfwTerminate();
    return 0;
}
