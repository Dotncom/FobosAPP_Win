#include "MyGraphWidget.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr int GRAPH_LEFT_MARGIN = 0;
constexpr int GRAPH_RIGHT_MARGIN = 8;
constexpr int GRAPH_TOP_MARGIN = 8;
constexpr int GRAPH_BOTTOM_MARGIN = 20;
constexpr int GRAPH_Y_TICKS = 6;
}

MyGraphWidget::MyGraphWidget(QWidget *parent)
    : QOpenGLWidget(parent), xMin(60e6), xMax(140e6), yMin(-120), yMax(0), fftLength(32768), initialized(false) {
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
    this->fftLength = std::max(0, fftLength);
    this->colorf = colorf;
    update();
}

void MyGraphWidget::setLevelRange(float minLevel, float maxLevel) {
    if (!std::isfinite(minLevel) || !std::isfinite(maxLevel) || maxLevel <= minLevel) {
        return;
    }
    yMin = minLevel;
    yMax = maxLevel;
    update();
}

void MyGraphWidget::clearData() {
    xData.clear();
    yData.clear();
    fftLength = 0;
    update();
}

void MyGraphWidget::paintGL() {
    QPainter painter(this);
    painter.beginNativePainting();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    const bool canDrawData = !xData.empty() && !yData.empty() && fftLength > 0 &&
                             !qFuzzyCompare(xMin, xMax) && !qFuzzyCompare(yMin, yMax);
    if (canDrawData) {
        glViewport(0, 0, width(), height());
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(0, width(), 0, height(), -1, 1);
        glColor3f(1.0f, 1.0f, 1.0f);
        glBegin(GL_LINE_STRIP);

        const float plotLeft = static_cast<float>(GRAPH_LEFT_MARGIN);
        const float plotBottom = static_cast<float>(GRAPH_BOTTOM_MARGIN);
        const float plotWidth = static_cast<float>((std::max)(1, width() - GRAPH_LEFT_MARGIN - GRAPH_RIGHT_MARGIN));
        const float plotHeight = static_cast<float>((std::max)(1, height() - GRAPH_TOP_MARGIN - GRAPH_BOTTOM_MARGIN));
        const int dataCount = std::min({fftLength, static_cast<int>(xData.size()), static_cast<int>(yData.size())});
        for (int i = 0; i < dataCount; ++i) {
            if (!std::isfinite(xData[i])) {
                continue;
            }
            const float xNorm = static_cast<float>((xData[i] - xMin) / (xMax - xMin));
            if (xNorm < 0.0f || xNorm > 1.0f) {
                continue;
            }
            float intensity = yData[(i + dataCount / 2) % dataCount];
            if (!std::isfinite(intensity)) {
                intensity = 0.0f;
            }
            const float level = normalizedLevel(intensity);
            if (colorf == true) {
                QColor color = valueToColor(level);
                    glColor3f(color.redF(), color.greenF(), color.blueF());} else {
                glColor3f(0.0f, 1.0f, 0.0f);
            }
            const float xPos = plotLeft + xNorm * plotWidth;
            const float yPos = plotBottom + level * plotHeight;
            glVertex2f(xPos, yPos);
        }
        glEnd();
    }
    painter.endNativePainting();
    drawYAxis(painter);
}

void MyGraphWidget::wheelEvent(QWheelEvent *event) {
    emit scaleChanged(event->angleDelta().y() > 0 ? 1 : -1);
    event->accept();
}

QColor MyGraphWidget::valueToColor(float value) {
    if (!std::isfinite(value)) {
        value = 0.0f;
    }
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

float MyGraphWidget::normalizedLevel(float value) const {
    if (!std::isfinite(value) || qFuzzyCompare(yMin, yMax)) {
        return 0.0f;
    }
    return qBound(0.0f, static_cast<float>((value - yMin) / (yMax - yMin)), 1.0f);
}

void MyGraphWidget::drawYAxis(QPainter &painter) const {
    const int plotLeft = GRAPH_LEFT_MARGIN;
    const int plotRight = (std::max)(plotLeft + 1, width() - GRAPH_RIGHT_MARGIN);
    const int plotTop = GRAPH_TOP_MARGIN;
    const int plotBottom = (std::max)(plotTop + 1, height() - GRAPH_BOTTOM_MARGIN);
    const int plotHeight = plotBottom - plotTop;

    painter.setRenderHint(QPainter::Antialiasing, false);
    painter.setPen(QColor(55, 80, 70, 170));
    painter.drawRect(plotLeft, plotTop, plotRight - plotLeft, plotHeight);

    for (int tick = 0; tick <= GRAPH_Y_TICKS; ++tick) {
        const double fraction = static_cast<double>(tick) / GRAPH_Y_TICKS;
        const double level = yMin + (yMax - yMin) * fraction;
        const int y = plotBottom - static_cast<int>(std::round(fraction * plotHeight));
        painter.setPen(QColor(45, 65, 58, 150));
        painter.drawLine(plotLeft, y, plotRight, y);
        painter.setPen(QColor(185, 210, 195));
        const QString label = QString::number(level, 'f', 0);
        painter.drawText(4, y + 4, label);
    }

    painter.setPen(QColor(185, 210, 195));
    painter.drawText(4, height() - 4, "dBFS");
}
