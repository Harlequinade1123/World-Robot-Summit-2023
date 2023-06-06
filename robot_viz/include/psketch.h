#ifndef PSKETCH_H
#define PSKETCH_H

#define GLEW_STATIC
#define GLM_ENABLE_EXPERIMENTAL
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/exterior_product.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <cmath>
#include <thread>

void key_callback(GLFWwindow * window, int key, int scancode, int action, int mode);
void mouse_button_callback(GLFWwindow * window, int button, int action, int mode);
void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);

enum ColorMode { RGB, HSB };
enum LineMode { ROUND, BEVEL, MILTER, SQUARE };
enum GraphicsMode { P2D, P3D };
enum UnitMode { MKS, CGS, MMGS, IPS };

struct PTransformations {
    glm::vec2 translate = glm::vec2(0.0f, 0.0f);
    float rotate = 0.0f;
    glm::vec2 scale = glm::vec2(1.0f, 1.0f);
};

struct PStyle
{
    glm::vec4 fillColor = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
    glm::vec4 stroke = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
    float strokeWeight = 1;
    LineMode strokeCap = SQUARE;
    LineMode strokeJoin = MILTER;
    bool noStroke = false;
};

class PSketch
{
public:
    PSketch();
    ~PSketch();
    virtual void run() final;
    virtual void setup(){};
    virtual void draw(){};
    virtual void keyEvent(int key, int action) {};
    virtual void mouseButtonEvent(int button, int action) {};
    virtual void cursorPosEvent(double xpos, double ypos) {};
    virtual void scrollEvent(double xoffset, double yoffset) {};
    virtual void parallelTask1() {};
    virtual void parallelTask2() {};
    virtual void parallelTask3() {};

    virtual void background(int r, int g, int b) final;
    virtual void background(int c) final;
    virtual void frameRate(int frameRate) final;
    virtual void size(int width, int height, GraphicsMode mode) final;
    virtual void noLoop() final;
    virtual void fill(int r, int g, int b) final;
    virtual void fill(int c) final;
    virtual void strokeWeight(float value) final;
    virtual void translate(float x, float y) final;
    virtual void translate(float x, float y, float z) final;
    virtual void rotate(float angle) final;
    virtual void rotate(float angle, glm::vec3 axis) final;
    virtual void rotateX(float angle) final;
    virtual void rotateY(float angle) final;
    virtual void rotateZ(float angle) final;
    virtual void scale(float xRate, float yRate, float zRate) final;
    virtual void scale(float xRate, float yRate) final;
    virtual void scale(float rate) final;
    virtual void pushMatrix() final;
    virtual void popMatrix() final;
    virtual void translateCamera(float distance, float height) final;
    virtual void rotateCamera(float radian) final;
    virtual void setCamera(float distance, float height, float radian) final;
    virtual float map(float value, float start1, float stop1, float start2, float stop2) final;
    virtual float radians(float degree) final;
    virtual void rectMode(int mode) final;
    virtual void ellipseMode(int mode) final;
    virtual void colorMode(ColorMode mode) final;
    virtual void stroke(int r, int g, int b) final;
    virtual void stroke(int c) final;
    virtual void noCursor() final;
    virtual void cursor() final;
    virtual void redraw() final;
    virtual void strokeCap(LineMode kind) final;
    virtual void strokeJoin(LineMode kind) final;
    virtual void pushStyle() final;
    virtual void popStyle() final;
    virtual void point(float x, float y) final;
    virtual void point(float x, float y, float z) final;
    virtual void line(float x1, float y1, float x2, float y2) final;
    virtual void line(float x1, float y1, float z1, float x2, float y2, float z2) final;
    virtual void rect(float x, float y, float width, float height) final;
    virtual void ellipse(float x, float y, float width, float height) final;
    virtual void circle(float x, float y, float size) final;

    virtual void box(float size) final;
    virtual void box(float w, float h, float d) final;
    virtual void cylinder(float r, float d) final;

protected:
    GLFWwindow * window = nullptr;
    int WIDTH = 100;
    int HEIGHT = 100;
    int FRAMERATE = 0;
    double MOUSEX = 0;
    double MOUSEY = 0;
    const int CENTER = 0;
    const int CORNER = 1;

private:
    std::thread parallelThread1;
    std::thread parallelThread2;
    std::thread parallelThread3;

    int target_fps = 60;

    glm::mat4 transformationMat = glm::mat4(1.0);
    std::vector<glm::mat4> transformationMat_stack = std::vector<glm::mat4>();
    std::vector<PStyle> style_stack = std::vector<PStyle>();
    bool needRedraw = false;

    glm::vec3 HSBtoRGB(int h, int s, int b);

    PStyle style;
    int RECTMODE = 1;
    int ELLIPSEMODE = 0;
    ColorMode COLORMODE = RGB;
    GraphicsMode GRAPHICSMODE = P2D;
    float cameraDistance = 600.0f;
    float cameraRotation = 0.0f;
    float cameraHeight   = 250.0f;
    UnitMode UNITMODE = MMGS;
};

#endif
