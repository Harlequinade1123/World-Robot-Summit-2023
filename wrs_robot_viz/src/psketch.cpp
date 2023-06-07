#include "psketch.h"

PSketch* thisPointerForCallback;
PSketch::PSketch()
{
    // rand init
    srand(time(NULL));

    thisPointerForCallback = this;
}

PSketch::~PSketch()
{}

void key_callback(GLFWwindow * window, int key, int scancode, int action, int mode)
{
    thisPointerForCallback->keyEvent(key, action);
}

void mouse_button_callback(GLFWwindow * window, int button, int action, int mode)
{
    thisPointerForCallback->mouseButtonEvent(button, action);
}

void cursor_pos_callback(GLFWwindow* window, double xpos, double ypos)
{
    thisPointerForCallback->cursorPosEvent(xpos, ypos);
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    thisPointerForCallback->scrollEvent(xoffset, yoffset);
}

void PSketch::run()
{
    // GLFW を初期化する
    if (!glfwInit())
    {
        std::cerr << "Can't initialize GLFW" << std::endl;
        exit(1);
    }

    atexit(glfwTerminate);
    
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    glfwWindowHint(GLFW_SAMPLES, 4);

    this->window = glfwCreateWindow(this->WIDTH, this->HEIGHT, "processing", nullptr, nullptr);
    if (window == NULL)
    {
        std::cerr << "Can't create GLFW window." << std::endl;
        exit(1);
    }

    glfwSetKeyCallback(this->window, key_callback);
    glfwSetMouseButtonCallback(this->window, mouse_button_callback);
    glfwSetCursorPosCallback(this->window, cursor_pos_callback);
    glfwSetScrollCallback(this->window, scroll_callback);

    glewExperimental = GL_TRUE;
    if (!glewInit())
    {
        std::cerr << "Can't initialize GLEW" << std::endl;
        exit(1);
    }
    
    int widthW, heightW;
    glfwGetFramebufferSize(this->window, &widthW, &heightW);
    glViewport(0, 0, widthW, heightW);
    glfwMakeContextCurrent(this->window);
    glfwSwapInterval(1);

    glm::mat4 projection;
    if (GRAPHICSMODE == P2D)
    {
        projection = glm::ortho(static_cast<GLfloat>(-WIDTH * 0.5), static_cast<GLfloat>(WIDTH * 0.5), 
                                static_cast<GLfloat>(-HEIGHT * 0.5), static_cast<GLfloat>(HEIGHT * 0.5), -5000.0f, 5000.0f);
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(glm::value_ptr(projection));
    }
    else
    {
        glClearDepth(1.0);
        glDepthFunc(GL_LESS);
        glEnable(GL_DEPTH_TEST);
        projection = glm::perspective(glm::radians(60.0f), (float)WIDTH / (float)HEIGHT, 0.2f, 5000.0f);
        glm::mat4 look = glm::lookAt(
            glm::vec3(cameraDistance * cos(cameraRotation), cameraDistance * sin(cameraRotation), cameraHeight),
            glm::vec3(0, 0, 0),
            glm::vec3(0, 0, 1));
        projection = projection * look;
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(glm::value_ptr(projection));
    }

    this->setup();
    
    // framerate regulation init
    float interval = 1.0 / this->target_fps;
    float previousTime = glfwGetTime();
    // fps count init
    float previousCount = glfwGetTime();
    int frameCount = 0;

    parallelThread1 = std::thread([this]() { this->parallelTask1(); });
    parallelThread2 = std::thread([this]() { this->parallelTask2(); });
    parallelThread3 = std::thread([this]() { this->parallelTask3(); });

    while (!glfwWindowShouldClose(this->window))
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glfwGetCursorPos(window, &MOUSEX, &MOUSEY);
        // reverts the mouseY axis
        MOUSEY = HEIGHT - MOUSEY;
        
        MOUSEX -= WIDTH * 0.5;
        MOUSEY -= HEIGHT * 0.5;
        
        if (GRAPHICSMODE == P3D)
        {
            glClearDepth(1.0);
            glDepthFunc(GL_LESS);
            glEnable(GL_DEPTH_TEST);
            projection = glm::perspective(glm::radians(60.0f), (float)WIDTH / (float)HEIGHT, 0.2f, 5000.0f);
            glm::mat4 look = glm::lookAt(
                glm::vec3(cameraDistance * cos(cameraRotation), cameraDistance * sin(cameraRotation), cameraHeight),
                glm::vec3(0, 0, 0),
                glm::vec3(0, 0, 1));
            projection = projection * look;
            glMatrixMode(GL_PROJECTION);
            glLoadMatrixf(glm::value_ptr(projection));
        }
        
        this->draw();

        //Reset matrix
        if(transformationMat_stack.size() != 0)
        {
            std::cerr << "ERROR::SKETCH::A MATRIX WAS NOT POPED AT THE END OF THE DRAW FUNCTION" << std::endl;
            exit(-1);
        }

        // resets the transformation
        this->transformationMat = glm::mat4(1.0);
        this->transformationMat_stack.clear();
        
        glfwSwapBuffers(this->window);
        glfwPollEvents();

        // framerate regulation
        while(glfwGetTime() - previousTime < interval && !glfwWindowShouldClose(this->window) && !needRedraw)
        {
            glfwPollEvents();
        }
        previousTime = glfwGetTime();
    
        // fps count
        frameCount++;
        if(glfwGetTime() - previousCount > 1.0)
        {
            this->FRAMERATE = frameCount;
            previousCount = glfwGetTime();
            frameCount = 0;
        }
        
        //Resets the redraw flag
        needRedraw = false;
    }

    parallelThread1.join();
    parallelThread2.join();
    parallelThread3.join();

    glfwTerminate();
}

void PSketch::background(int r, int g, int b)
{
    glClearColor(float(r) / 255.0, float(g) / 255.0, float(b) / 255.0, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
}

void PSketch::background(int c)
{
    background(c, c, c);
}

void PSketch::frameRate(int frameRate)
{
    this->target_fps = frameRate;
}

void PSketch::fill(int r, int g, int b)
{
    if(COLORMODE == HSB)
    {
        glm::vec3 hsb = HSBtoRGB(r, g, b);
        this->style.fillColor = glm::vec4(float(hsb.r) / 255.0, float(hsb.g) / 255.0, float(hsb.b) / 255.0, 1.0);
    }
    else
    {
        this->style.fillColor = glm::vec4(float(r) / 255.0, float(g) / 255.0, float(b) / 255.0, 1.0);
    }
}

void PSketch::fill(int c)
{
    if(COLORMODE != HSB)
    {
        fill(c, c, c);
    }
    else
    {
        std::cerr << "ERROR::SKETCH::CANNOT USE 1 VALUE FOR A HSB COLOR" << std::endl;
        exit(-1);
    }
}

void PSketch::rect(float x, float y, float width, float height)
{
    glm::mat4 model = this->transformationMat;
    model = glm::translate(model, glm::vec3(x + RECTMODE*(width / 2), y + RECTMODE*(width / 2), 0));
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(model));
    
    glColor4f(this->style.fillColor[0], this->style.fillColor[1], this->style.fillColor[2], this->style.fillColor[3]);
    glBegin(GL_QUADS);
        glVertex3f(x + width / 2, y + width / 2, 0.0);
        glVertex3f(x + width / 2, y - width / 2, 0.0);
        glVertex3f(x - width / 2, y - width / 2, 0.0);
        glVertex3f(x - width / 2, y + width / 2, 0.0);
    glEnd();

    glColor4f(this->style.stroke[0], this->style.stroke[1], this->style.stroke[2], this->style.stroke[3]);
    glBegin(GL_LINES);
        glVertex3f(x + width / 2, y + width / 2, 0.0);
        glVertex3f(x + width / 2, y - width / 2, 0.0);

        glVertex3f(x + width / 2 + this->style.strokeWeight * 0.5, y - width / 2, 0.0);
        glVertex3f(x - width / 2 - this->style.strokeWeight * 0.5, y - width / 2, 0.0);

        glVertex3f(x - width / 2, y - width / 2 , 0.0);
        glVertex3f(x - width / 2, y + width / 2 , 0.0);

        glVertex3f(x - width / 2 - this->style.strokeWeight * 0.5, y + width / 2, 0.0);
        glVertex3f(x + width / 2 + this->style.strokeWeight * 0.5, y + width / 2, 0.0);
    glEnd();
}

void PSketch::noLoop()
{
    frameRate(0);
}

void PSketch::point(float x, float y)
{
    glm::mat4 model = this->transformationMat;
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(model));
    
    glPointSize(this->style.strokeWeight);
    glColor4f(this->style.stroke[0], this->style.stroke[1], this->style.stroke[2], this->style.stroke[3]);

    glBegin(GL_POINTS);
        glVertex3f(x, y, 0.0);
    glEnd();
}

void PSketch::point(float x, float y, float z)
{
    glm::mat4 model = this->transformationMat;
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(model));
    
    glPointSize(this->style.strokeWeight);
    glColor4f(this->style.stroke[0], this->style.stroke[1], this->style.stroke[2], this->style.stroke[3]);

    glBegin(GL_POINTS);
        glVertex3f(x, y, z);
    glEnd();
}

void PSketch::strokeWeight(float value)
{
    this->style.strokeWeight = value;
    if(value > 8)
    {
        this->style.strokeWeight = 8;
    }
}

void PSketch::size(int width, int height, GraphicsMode mode)
{
    if(this->window == nullptr)
    {
        this->WIDTH = width;
        this->HEIGHT = height;
        this->GRAPHICSMODE = mode;
    }
    else
    {
        std::cerr << "ERROR::SKETCH::SIZE SHOULD BE CALLED FROM THE SKETCH CONSTRUCTOR" << std::endl;
        exit(-1);
    }
}

void PSketch::ellipse(float x, float y, float width, float height)
{
    glm::mat4 model = this->transformationMat;
    model = glm::translate(model, glm::vec3(x + ELLIPSEMODE*(width / 2), y + ELLIPSEMODE*(height / 2), 0));
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(model));
    
    glColor4f(this->style.fillColor[0], this->style.fillColor[1], this->style.fillColor[2], this->style.fillColor[3]);
    glBegin(GL_POLYGON);
        for (int i = 0; i <= 360; i++)
        {
            glVertex3f(width * cos(M_PI / 180.0 * static_cast<float>(i)), height * sin(M_PI / 180.0 * static_cast<float>(i)), 0.0);
        }
    glEnd();

    glColor4f(this->style.stroke[0], this->style.stroke[1], this->style.stroke[2], this->style.stroke[3]);
    glBegin(GL_LINE_LOOP);
        for (int i = 0; i <= 360; i++)
        {
            glVertex3f(width * cos(M_PI / 180.0 * static_cast<float>(i)), height * sin(M_PI / 180.0 * static_cast<float>(i)), 0.0);
        }
    glEnd();
}

void PSketch::circle(float x, float y, float size)
{
    this->ellipse(x, y, size, size);
}

void PSketch::translate(float x, float y)
{
    this->transformationMat = glm::translate(this->transformationMat, glm::vec3(x, y, 0));
}

void PSketch::translate(float x, float y, float z)
{
    this->transformationMat = glm::translate(this->transformationMat, glm::vec3(x, y, z));
}

void PSketch::rotate(float angle)
{
    this->transformationMat = glm::rotate(this->transformationMat, angle, glm::vec3(0, 0, 1));
}

void PSketch::rotate(float angle, glm::vec3 axis)
{
    this->transformationMat = glm::rotate(this->transformationMat, angle, glm::normalize(axis));
}

void PSketch::rotateX(float angle)
{
    this->transformationMat = glm::rotate(this->transformationMat, angle, glm::vec3(1, 0, 0));
}

void PSketch::rotateY(float angle)
{
    this->transformationMat = glm::rotate(this->transformationMat, angle, glm::vec3(0, 1, 0));
}

void PSketch::rotateZ(float angle)
{
    this->transformationMat = glm::rotate(this->transformationMat, angle, glm::vec3(0, 0, 1));
}

void PSketch::scale(float xRate, float yRate, float zRate)
{
    this->transformationMat = glm::scale(this->transformationMat, glm::vec3(xRate, yRate, zRate));
}

void PSketch::scale(float xRate, float yRate)
{
    this->transformationMat = glm::scale(this->transformationMat, glm::vec3(xRate, yRate, 1));
}

void PSketch::scale(float rate)
{
    scale(rate, rate);
}

void PSketch::pushMatrix()
{
    this->transformationMat_stack.push_back(this->transformationMat);
}

void PSketch::popMatrix()
{
    if(this->transformationMat_stack.size() == 0)
    {
        std::cerr << "ERROR::SKETCH::PUSHMATRIX SHOULD BE CALLED BEFORE POPMATRIX" << std::endl;
        exit(-1);
    }
    this->transformationMat = this->transformationMat_stack.back();
    this->transformationMat_stack.pop_back();
}

void PSketch::ellipseMode(int mode)
{
    if(mode == CENTER || mode == CORNER)
    {
        ELLIPSEMODE = mode;
    }
    else
    {
        std::cerr << "ERROR::SKETCH::THIS IS NOT A VALID ELLIPSEMODE" << std::endl;
        exit(-1);
    }
}

void PSketch::rectMode(int mode)
{
    if(mode == CENTER || mode == CORNER)
    {
        RECTMODE = mode;
    }
    else
    {
        std::cerr << "ERROR::SKETCH::THIS IS NOT A VALID RECTMODE" << std::endl;
        exit(-1);
    }
}

void PSketch::colorMode(ColorMode mode)
{
    if(mode == HSB || mode == RGB)
    {
        this->COLORMODE = mode;
    }
    else
    {
        std::cerr << "ERROR::SKETCH::THIS IS NOT A VALID COLORMODE" << std::endl;
        exit(-1);
    }
}

glm::vec3 PSketch::HSBtoRGB(int h, int s, int b)
{
    glm::vec3 rgb;
    glm::vec3 hsb = glm::vec3(float(h) / 255.0, float(s) / 255.0, float(b) / 255.0);

    int i = floor(hsb.r * 6);
    float f = hsb.r * 6 - i;
    float p = hsb.b * (1 - hsb.g);
    float q = hsb.b * (1 - f * hsb.g);
    float t = hsb.b * (1 - (1 - f) * hsb.g);

    switch(i % 6)
    {
        case 0: rgb.r = hsb.b, rgb.g = t, rgb.b = p; break;
        case 1: rgb.r = q, rgb.g = hsb.b, rgb.b = p; break;
        case 2: rgb.r = p, rgb.g = hsb.b, rgb.b = t; break;
        case 3: rgb.r = p, rgb.g = q, rgb.b = hsb.b; break;
        case 4: rgb.r = t, rgb.g = p, rgb.b = hsb.b; break;
        case 5: rgb.r = hsb.b, rgb.g = p, rgb.b = q; break;
    }
    rgb.r *= 255;
    rgb.g *= 255;
    rgb.b *= 255;
    return rgb;
}

void PSketch::stroke(int r, int g, int b)
{
    if(COLORMODE == HSB)
    {
        glm::vec3 hsb = HSBtoRGB(r, g, b);
        this->style.stroke = glm::vec4(float(hsb.r) / 255.0, float(hsb.g) / 255.0, float(hsb.b) / 255.0, 1.0);
    }
    else
    {
        this->style.stroke = glm::vec4(float(r) / 255.0, float(g) / 255.0, float(b) / 255.0, 1.0);
    }
}

void PSketch::stroke(int c)
{
    if(COLORMODE != HSB)
    {
        stroke(c, c, c);
    }
    else
    {
        std::cerr << "ERROR::SKETCH::CANNOT USE 1 VALUE FOR A HSB COLOR" << std::endl;
        exit(-1);
    }
}

void PSketch::noCursor()
{
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_HIDDEN);
}

void PSketch::cursor()
{
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
}

void PSketch::redraw()
{
    needRedraw = true;
}

void PSketch::strokeCap(LineMode kind)
{
    if(kind == ROUND || kind == SQUARE)
    {
        this->style.strokeCap = kind;
    }
    else
    {
        std::cerr << "ERROR::SKETCH::THIS IS NOT A VALID STROKECAP" << std::endl;
        exit(-1);
    }
}

void PSketch::strokeJoin(LineMode kind)
{
    if(kind == ROUND || kind == MILTER || kind == BEVEL)
    {
        this->style.strokeJoin = kind;
    }
    else
    {
        std::cerr << "ERROR::SKETCH::THIS IS NOT A VALID STROKEJOIN" << std::endl;
        exit(-1);
    }
}

void PSketch::pushStyle()
{
    style_stack.push_back(style);
}

void PSketch::popStyle()
{
    if(style_stack.size() == 0)
    {
        std::cerr << "ERROR::SKETCH::PUSHSTYLE SHOULD BE CALLED BEFORE POPSTYLE" << std::endl;
        exit(-1);
    }
    style = style_stack.back();
    style_stack.pop_back();
}

void PSketch::line(float x1, float y1, float x2, float y2)
{
    glm::mat4 model = this->transformationMat;
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(model));
    
    glLineWidth(this->style.strokeWeight);
    glColor4f(this->style.stroke[0], this->style.stroke[1], this->style.stroke[2], this->style.stroke[3]);

    glBegin(GL_LINES);
        glVertex3f(x1, y1, 0.0f);
        glVertex3f(x2, y2, 0.0f);
    glEnd();
    
    if(style.strokeCap == ROUND)
    {
        point(x1, y1);
        point(x2, y2);
    }
}

void PSketch::line(float x1, float y1, float z1, float x2, float y2, float z2)
{
    glm::mat4 model = this->transformationMat;
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(model));
    
    glLineWidth(this->style.strokeWeight);
    glColor4f(this->style.stroke[0], this->style.stroke[1], this->style.stroke[2], this->style.stroke[3]);

    glBegin(GL_LINES);
        glVertex3f(x1, y1, z1);
        glVertex3f(x2, y2, z2);
    glEnd();
    
    if(style.strokeCap == ROUND)
    {
        point(x1, y1, z1);
        point(x2, y2, z2);
    }
}

float PSketch::map(float value, float start1, float stop1, float start2, float stop2)
{
    if(start1 <= value && value <= stop1)
    {
        return (stop2 - start2) / (stop1 - start1) * value;	
    }
    else
    {
        std::cerr << "ERROR::MAP::VALUE BOUNDARIES ARE NOT CORRECT" << std::endl;
        exit(-1);
    }
}

float PSketch::radians(float degree)
{
	return degree * M_PI / 180.0;
}

void PSketch::translateCamera(float distance, float height)
{
    this->cameraDistance -= distance;
    this->cameraHeight   += height;
    if (this->cameraDistance < 0.1)
    {
        this->cameraDistance = 0.1;
    }
    if (1000 < this->cameraDistance)
    {
        this->cameraDistance = 1000;
    }
    if (this->cameraHeight < 10)
    {
        this->cameraHeight = 10;
    }
    if (1000 < this->cameraHeight)
    {
        this->cameraHeight = 1000;
    }
}

void PSketch::rotateCamera(float radian)
{
    this->cameraRotation += radian;
}

void PSketch::setCamera(float distance, float height, float radian)
{
    this->cameraDistance = distance;
    this->cameraHeight   = height;
    this->cameraRotation = radian;
    if (this->cameraDistance < 0.1)
    {
        this->cameraDistance = 0.1;
    }
    if (1000 < this->cameraDistance)
    {
        this->cameraDistance = 1000;
    }
    if (this->cameraHeight < 10)
    {
        this->cameraHeight = 10;
    }
    if (1000 < this->cameraHeight)
    {
        this->cameraHeight = 1000;
    }
}

void PSketch::box(float size)
{
    this->box(size, size, size);
}

void PSketch::box(float w, float h, float d)
{
    glm::mat4 model = this->transformationMat;
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(model));
    
    glColor4f(this->style.fillColor[0], this->style.fillColor[1], this->style.fillColor[2], this->style.fillColor[3]);
    glBegin(GL_QUADS);
        glVertex3f(w / 2, h / 2, d / 2);
        glVertex3f(w / 2, h / 2,-d / 2);
        glVertex3f(w / 2,-h / 2,-d / 2);
        glVertex3f(w / 2,-h / 2, d / 2);

        glVertex3f(-w / 2, h / 2, d / 2);
        glVertex3f(-w / 2, h / 2,-d / 2);
        glVertex3f(-w / 2,-h / 2,-d / 2);
        glVertex3f(-w / 2,-h / 2, d / 2);

        glVertex3f( w / 2, h / 2, d / 2);
        glVertex3f(-w / 2, h / 2, d / 2);
        glVertex3f(-w / 2, h / 2,-d / 2);
        glVertex3f( w / 2, h / 2,-d / 2);

        glVertex3f( w / 2, -h / 2, d / 2);
        glVertex3f(-w / 2, -h / 2, d / 2);
        glVertex3f(-w / 2, -h / 2,-d / 2);
        glVertex3f( w / 2, -h / 2,-d / 2);

        glVertex3f( w / 2, h / 2, d / 2);
        glVertex3f(-w / 2, h / 2, d / 2);
        glVertex3f(-w / 2,-h / 2, d / 2);
        glVertex3f( w / 2,-h / 2, d / 2);

        glVertex3f( w / 2, h / 2,-d / 2);
        glVertex3f(-w / 2, h / 2,-d / 2);
        glVertex3f(-w / 2,-h / 2,-d / 2);
        glVertex3f( w / 2,-h / 2,-d / 2);
    glEnd();

    glColor4f(this->style.stroke[0], this->style.stroke[1], this->style.stroke[2], this->style.stroke[3]);
    glBegin(GL_LINES);
        glVertex3f(w / 2, h / 2, d / 2);
        glVertex3f(w / 2, h / 2,-d / 2);

        glVertex3f(w / 2, h / 2 + this->style.strokeWeight * 0.5,-d / 2);
        glVertex3f(w / 2,-h / 2 - this->style.strokeWeight * 0.5,-d / 2);

        glVertex3f(w / 2,-h / 2,-d / 2);
        glVertex3f(w / 2,-h / 2, d / 2);

        glVertex3f(w / 2,-h / 2 - this->style.strokeWeight * 0.5, d / 2);
        glVertex3f(w / 2, h / 2 + this->style.strokeWeight * 0.5, d / 2);

        glVertex3f(-w / 2, h / 2, d / 2);
        glVertex3f(-w / 2, h / 2,-d / 2);

        glVertex3f(-w / 2, h / 2 + this->style.strokeWeight * 0.5,-d / 2);
        glVertex3f(-w / 2,-h / 2 - this->style.strokeWeight * 0.5,-d / 2);

        glVertex3f(-w / 2,-h / 2,-d / 2);
        glVertex3f(-w / 2,-h / 2, d / 2);

        glVertex3f(-w / 2,-h / 2 - this->style.strokeWeight * 0.5, d / 2);
        glVertex3f(-w / 2, h / 2 + this->style.strokeWeight * 0.5, d / 2);

        glVertex3f( w / 2, h / 2, d / 2);
        glVertex3f(-w / 2, h / 2, d / 2);

        glVertex3f(-w / 2, h / 2,-d / 2);
        glVertex3f( w / 2, h / 2,-d / 2);

        glVertex3f( w / 2,-h / 2, d / 2);
        glVertex3f(-w / 2,-h / 2, d / 2);

        glVertex3f(-w / 2,-h / 2,-d / 2);
        glVertex3f( w / 2,-h / 2,-d / 2);
    glEnd();
}

void PSketch::cylinder(float r, float d)
{
    glm::mat4 model = this->transformationMat;
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(glm::value_ptr(model));
    
    glColor4f(this->style.fillColor[0], this->style.fillColor[1], this->style.fillColor[2], this->style.fillColor[3]);
    glBegin(GL_QUAD_STRIP);
        for (int i = 0; i <= 360; i++)
        {
            glVertex3f(r * cos(M_PI / 180.0 * static_cast<float>(i)), r * sin(M_PI / 180.0 * static_cast<float>(i)),-d / 2.0);
            glVertex3f(r * cos(M_PI / 180.0 * static_cast<float>(i)), r * sin(M_PI / 180.0 * static_cast<float>(i)), d / 2.0);
        }
    glEnd();

    glColor4f(this->style.stroke[0], this->style.stroke[1], this->style.stroke[2], this->style.stroke[3]);
    translate(0.0, 0.0, d / 2.0);
    this->circle(0.0, 0.0, r);
    translate(0.0, 0.0,-d);
    this->circle(0.0, 0.0, r);
    translate(0.0, 0.0, d / 2.0);
}