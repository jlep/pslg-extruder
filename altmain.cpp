#include "pslg.hpp"
//#include "linmath.h"

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/glm.hpp>
#include <glm/ext.hpp>

extern const char *vertex_shader_str;
extern const char *fragment_shader_str;

#include "shader.hpp"

GLFWwindow *window;
GLuint programID;
GLuint vertexbuffer;

size_t renderCount = 0;

// An array of 3 vectors which represents 3 vertices
const GLfloat g_vertex_buffer_data[] = {
        0.0f, 0.0f, 0.0f,
        100.0f, 0.0f, 0.0f,
        0.0f, 100.0f, 0.0f,
};

void draw() {
    std::cout << "rendering... " << renderCount++ << std::endl;

    int fbw, fbh;
    int w, h;
    glfwGetFramebufferSize(window, &fbw, &fbh);
    glfwGetWindowSize(window, &w, &h);

    std::cout << fbw << fbh << w << h << std::endl;
    // Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
    //glm::mat4 Projection = glm::perspective(glm::radians(45.0f), aspect, 0.1f, 100.0f);

    // Or, for an ortho camera :
    glm::mat4 Projection = glm::ortho(0.0f, (float) w, (float) h, 0.0f); // In world coordinates

    // Camera matrix
//        glm::mat4 View = glm::lookAt(
//                glm::vec3(4,3,3), // Camera is at (4,3,3), in World Space
//                glm::vec3(0,0,0), // and looks at the origin
//                glm::vec3(0,1,0)  // Head is up (set to 0,-1,0 to look upside-down)
//        );

    // Model matrix : an identity matrix (model will be at the origin)
    glm::mat4 Model = glm::mat4(1.0f);

    // Our ModelViewProjection : multiplication of our 3 matrices
    glm::mat4 mvp = Projection * Model; // Remember, matrix multiplication is the other way around

    // Get a handle for our "MVP" uniform
    // Only during the initialisation
    GLint MatrixID = glGetUniformLocation(programID, "MVP");

    // Send our transformation to the currently bound shader, in the "MVP" uniform
    // This is done in the main loop since each model will have a different MVP matrix (At least for the M part)
    glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &mvp[0][0]);


    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glUseProgram(programID);

    // 1st attribute buffer : vertices
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glVertexAttribPointer(
            0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
            3,                  // size
            GL_FLOAT,           // type
            GL_FALSE,           // normalized?
            0,                  // stride
            nullptr             // array buffer offset
    );
    // Draw the triangle !
    glDrawArrays(GL_LINE_STRIP, 0, 3); // Starting from vertex 0; 3 vertices total -> 1 triangle
    glDisableVertexAttribArray(0);

    // Swap buffers
    glfwSwapBuffers(window);
}

void error_callback(int error, const char *description) {
    puts(description);
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    draw();
}

int main() {

    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return -1;
    }

    glfwSetErrorCallback(error_callback);

    glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL

    // Open a window and create its OpenGL context

    window = glfwCreateWindow(800, 600, "Tutorial 01", nullptr, nullptr);
    if (window == nullptr) {
        fprintf(stderr,
                "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n");
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    // load gl functions
    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
        fprintf(stderr, "Failed to load GL\n");
        glfwTerminate();
        return -1;
    }
    printf("OpenGL %d.%d\n", GLVersion.major, GLVersion.minor);

    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    // Generate 1 buffer, put the resulting identifier in vertexbuffer
    glGenBuffers(1, &vertexbuffer);

    // The following commands will talk about our 'vertexbuffer' buffer
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);

    // Give our vertices to OpenGL.
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

    // Create and compile our GLSL program from the shaders
    programID = loadShaders(vertex_shader_str, fragment_shader_str);
    glfwSwapInterval(1);

    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);



    // drawing just once doesn't seem to work
    draw();
    draw();
    do {
        glfwWaitEvents();
    } while (!glfwWindowShouldClose(window));

    return 0;
}
