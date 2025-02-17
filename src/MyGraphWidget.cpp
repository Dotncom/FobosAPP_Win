#include "MyGraphWidget.h"

MyGraphWidget::MyGraphWidget(QWidget *parent)
    : QOpenGLWidget(parent), xMin(60e6), xMax(140e6), yMin(0), yMax(100), fftLength(65536), initialized(false) {
}

MyGraphWidget::~MyGraphWidget() {
}

void MyGraphWidget::initializeGL() {
    initializeOpenGLFunctions();

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width(), height(), 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    initialized = true;
}

void MyGraphWidget::resizeGL(int w, int h) {
    glViewport(0, 0, w, h);
}

void MyGraphWidget::setData(const std::vector<float> &xData, const std::vector<float> &yData, double xMin, double xMax, int fftLength, bool colorf) {
    this->xData = xData;
    this->yData = yData;
    this->xMin = xMin;
    this->xMax = xMax;
    this->fftLength = fftLength;
    this->colorf = colorf;

    yMin = *std::min_element(yData.begin(), yData.end());
    yMax = *std::max_element(yData.begin(), yData.end());
    update();
}

void MyGraphWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glViewport(0, 0, width(), height());
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(xMin, xMax, 0, height(), -1, 1);
        glColor3f(1.0f, 1.0f, 1.0f);
        glBegin(GL_LINE_STRIP);

        for (int i = 0; i < fftLength && i < xData.size(); ++i) {
            float intensity = yData[(i + fftLength / 2) % fftLength];
            if (colorf == true) {
                QColor color = valueToColor(intensity);
                    glColor3f(color.redF(), color.greenF(), color.blueF());} else {
                glColor3f(0.0f, 1.0f, 0.0f);
            }
            float yPos = (intensity - yMin) / (yMax - yMin) * (height() * 3 / 4);
            glVertex2f(xData[i], yPos);
        }
        glEnd();
}

void MyGraphWidget::wheelEvent(QWheelEvent *event) {
    emit scaleChanged(event->angleDelta().y() > 0 ? 1 : -1);
    event->accept();
}

QColor MyGraphWidget::valueToColor(float value) {
    value = qBound(0.0f, value, 1.0f);
    int r, g, b;
    if (value < 0.16f) {
        float ratio = value / 0.2f;
        r = 0;
        g = 0;
        b = static_cast<int>(255 * ratio);
    } else if (value < 0.33f) {
        float ratio = (value - 0.16f) / 0.2f;
        r = 0;
        g = static_cast<int>(255 * (1 - ratio));
        b = 255;
    } else if (value < 0.5f) {
        float ratio = (value - 0.33f) / 0.2f;
        r = 0;
        g = 255;
        b = static_cast<int>(255 * (1 - ratio));
    } else if (value < 0.66f) {
        float ratio = (value - 0.5f) / 0.2f;
        r = static_cast<int>(255 * ratio);
        g = 255;
        b = 0;
    } else if (value < 0.83f) {
        float ratio = (value - 0.66f) / 0.2f;
        r = 255;
        g = static_cast<int>(255 * (1 - 0.5f * ratio));
        b = 0;
    } else {
        float ratio = (value - 0.83f) / 0.2f;
        r = 255;
        g = static_cast<int>(128 * (1 - ratio));
        b = 0;
    }
    return QColor(r, g, b);
}
