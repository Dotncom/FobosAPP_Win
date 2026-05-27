#include "MyWaterfallWidget.h"

#include <algorithm>
#include <cmath>
#include <limits>

bool changebit=false;

MyWaterfallWidget::MyWaterfallWidget(QWidget *parent)
    : QOpenGLWidget(parent), xMin(60000000), xMax(140000000), yMin(-120), yMax(0), contrast(10), sensitivity(10),
      levelMin(-120), levelMax(0), fftLength(32768), initialized(false), secondGraph(false) {
		waterfallTexture = 0;
}

MyWaterfallWidget::~MyWaterfallWidget() {
    makeCurrent();
    if (waterfallTexture != 0) {
        glDeleteTextures(1, &waterfallTexture);
        waterfallTexture = 0;
    }
    waterfallVbo.release();
    doneCurrent();
}


void MyWaterfallWidget::wheelEvent(QWheelEvent *event) {
    emit scaleChanged(event->angleDelta().y() > 0 ? 1 : -1);
    event->accept();
}

void MyWaterfallWidget::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::RightButton) {
        emit tuneContextRequested(frequencyAtX(event->x()), event->globalPos());
        event->accept();
        return;
    }
    QOpenGLWidget::mousePressEvent(event);
}

double MyWaterfallWidget::frequencyAtX(int x) const {
    if (width() <= 0 || qFuzzyCompare(xMin, xMax)) {
        return xMin;
    }
    const double normalized = std::clamp(static_cast<double>(x) / (std::max)(1, width()), 0.0, 1.0);
    return xMin + normalized * (xMax - xMin);
}

void MyWaterfallWidget::ensureLineBuffer() {
    const int lineWidth = std::max(1, width());
    const size_t requiredSize = static_cast<size_t>(lineWidth) * 3;
    if (lineData.size() != requiredSize) {
        lineData.assign(requiredSize, 0);
    }
}

void MyWaterfallWidget::initializeGL() {
qDebug() << "MyWaterfallWidget::initializeGL start";
initializeOpenGLFunctions();


    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); 
    ensureLineBuffer();
    glGenTextures(1, &waterfallTexture);
    resetWaterfallTexture(width(), height());
    waterfallVbo.create();
    waterfallVbo.bind();
    waterfallVbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);
    waterfallVbo.allocate(std::max(1, width()) * 3);
    waterfallVbo.release();
	initialized = true;
qDebug() << "MyWaterfallWidget::initializeGL done";
}

void MyWaterfallWidget::resizeGL(int w, int h) {
        glViewport(0, 0, w, h);
        ensureLineBuffer();
        resetWaterfallTexture(w, h);
        if (waterfallVbo.isCreated()) {
            waterfallVbo.bind();
            waterfallVbo.allocate(std::max(1, w) * 3);
            waterfallVbo.release();
        }
        qDebug() << "resizeGL done";
}

void MyWaterfallWidget::resetWaterfallTexture(int w, int h) {
    textureWidth = std::max(1, w);
    textureHeight = std::max(1, h);
    waterfallWriteRow = 0;

    std::vector<unsigned char> blank(static_cast<size_t>(textureWidth) * textureHeight * 3, 0);
    glBindTexture(GL_TEXTURE_2D, waterfallTexture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, textureWidth, textureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, blank.data());
    glBindTexture(GL_TEXTURE_2D, 0);
}

void MyWaterfallWidget::setData(const std::vector<float> &xData, const std::vector<float> &yData, double xMin, double xMax, int fftLength, bool secondGraph, float contrast, float sensitivity, float levelMin, float levelMax) {
    {
        QMutexLocker locker(&mutex);
        this->xData = xData;
        this->yData = yData;
        this->xMin = xMin;
        this->xMax = xMax;
        this->fftLength = std::max(0, fftLength);
        this->secondGraph = secondGraph;
        this->contrast = contrast;
        this->sensitivity = sensitivity;
        if (std::isfinite(levelMin) && std::isfinite(levelMax) && levelMax > levelMin) {
            this->levelMin = levelMin;
            this->levelMax = levelMax;
            yMin = levelMin;
            yMax = levelMax;
        }

        ensureLineBuffer();
    }

    computeLineData();
}

void MyWaterfallWidget::setLevelRange(float minLevel, float maxLevel) {
    {
        QMutexLocker locker(&mutex);
        if (!std::isfinite(minLevel) || !std::isfinite(maxLevel) || maxLevel <= minLevel) {
            return;
        }
        levelMin = minLevel;
        levelMax = maxLevel;
        yMin = minLevel;
        yMax = maxLevel;
    }
    computeLineData();
}

void MyWaterfallWidget::clearData() {
    {
        QMutexLocker locker(&mutex);
        xData.clear();
        yData.clear();
        fftLength = 0;
        std::fill(lineData.begin(), lineData.end(), 0);
        pendingTextureLine = false;
        textureClearRequested = true;
    }
    update();
}

void MyWaterfallWidget::computeLineData() {
	QMutexLocker locker(&mutex);
    ensureLineBuffer();
    if (lineData.empty() || xData.empty() || yData.empty() || fftLength <= 0 || qFuzzyCompare(xMin, xMax)) {
        QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
        return;
    }
    std::fill(lineData.begin(), lineData.end(), 0);

	float contrastFactor = contrast/10;
	float sensitivityFactor = sensitivity/10;
    const int lineWidth = std::max(1, width());
    const int dataCount = std::min({fftLength, static_cast<int>(xData.size()), static_cast<int>(yData.size())});
    std::vector<float> pixelMax(static_cast<size_t>(lineWidth),
                                std::numeric_limits<float>::quiet_NaN());
    for (int id = 0; id < dataCount; ++id) {
        if (!std::isfinite(xData[id])) {
            continue;
        }
        const int x1 = static_cast<int>((xData[id] - xMin) * lineWidth / (xMax - xMin));
        if (x1 < 0 || x1 >= lineWidth) {
            continue;
        }
        const int shiftedIndex = (id + dataCount / 2) % dataCount;
        const float value = yData[shiftedIndex];
        if (std::isfinite(value)) {
            float &pixelValue = pixelMax[static_cast<size_t>(x1)];
            pixelValue = std::isfinite(pixelValue) ? (std::max)(pixelValue, value) : value;
        }
    }

    int previousFilled = -1;
    float previousValue = levelMin;
    for (int x = 0; x < lineWidth; ++x) {
        const float value = pixelMax[static_cast<size_t>(x)];
        if (!std::isfinite(value)) {
            continue;
        }

        if (previousFilled < 0) {
            for (int fill = 0; fill < x; ++fill) {
                pixelMax[static_cast<size_t>(fill)] = value;
            }
        } else if (x - previousFilled > 1) {
            const int gap = x - previousFilled;
            for (int fill = previousFilled + 1; fill < x; ++fill) {
                const float t = static_cast<float>(fill - previousFilled) / static_cast<float>(gap);
                pixelMax[static_cast<size_t>(fill)] = previousValue + (value - previousValue) * t;
            }
        }

        previousFilled = x;
        previousValue = value;
    }

    if (previousFilled < 0) {
        std::fill(pixelMax.begin(), pixelMax.end(), levelMin);
    } else {
        for (int fill = previousFilled + 1; fill < lineWidth; ++fill) {
            pixelMax[static_cast<size_t>(fill)] = previousValue;
        }
    }

    for (int x = 0; x < lineWidth; ++x) {
        QColor color = valueToColor(normalizedLevel(pixelMax[static_cast<size_t>(x)]), contrastFactor, sensitivityFactor);
        lineData[x * 3 + 0] = static_cast<unsigned char>(color.red());
        lineData[x * 3 + 1] = static_cast<unsigned char>(color.green());
        lineData[x * 3 + 2] = static_cast<unsigned char>(color.blue());
    }
    pendingTextureLine = true;
    QMetaObject::invokeMethod(this, "update", Qt::QueuedConnection);
}

void MyWaterfallWidget::paintGL() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    QMutexLocker locker(&mutex);
    if (width() <= 0 || height() <= 0) {
        return;
    }

    if (textureClearRequested) {
        ensureLineBuffer();
        std::fill(lineData.begin(), lineData.end(), 0);
        resetWaterfallTexture(width(), height());
        pendingTextureLine = false;
        textureClearRequested = false;
    }

	if (secondGraph == true) {
    if (qFuzzyCompare(xMin, xMax)) {
        return;
    }
	//additional color graph
	glViewport(0, 0, width(), height());
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(xMin, xMax, 0, height(), -1, 1);

    glColor3f(1.0f, 1.0f, 1.0f);
	glBegin(GL_LINE_STRIP);
    const int dataCount = std::min({fftLength, static_cast<int>(xData.size()), static_cast<int>(yData.size())});
	for (int i = 0; i < dataCount; ++i) {
    if (!std::isfinite(xData[i])) {
        continue;
    }
    float intensityS = yData[(i + dataCount / 2) % dataCount];
    if (!std::isfinite(intensityS)) {
        intensityS = 0.0f;
    }
    QColor colors = valueToColors(normalizedLevel(intensityS));
    glColor3f(colors.redF(), colors.greenF(), colors.blueF());
    float yPos = normalizedLevel(intensityS) * (height() * 3 / 4);
    glVertex2f(xData[i], yPos); 
	}
	glEnd();
	} else {
    //waterfall
  
    glViewport(0, 0, width(), height());
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, width(), 0, height(), -1, 1);
   
	glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, waterfallTexture);
    glColor3f(1.0f, 1.0f, 1.0f);
    
    ensureLineBuffer();
    const int texWidth = std::max(1, width());
    const int texHeight = std::max(1, height());
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    if (texWidth != textureWidth || texHeight != textureHeight) {
        resetWaterfallTexture(texWidth, texHeight);
    }
    if (pendingTextureLine && !lineData.empty()) {
        waterfallWriteRow = (waterfallWriteRow + textureHeight - 1) % textureHeight;
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, waterfallWriteRow, textureWidth, 1, GL_RGB, GL_UNSIGNED_BYTE, lineData.data());
        pendingTextureLine = false;
    }
    const float vStart = static_cast<float>(waterfallWriteRow) / static_cast<float>(textureHeight);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, vStart + 1.0f); glVertex2f(0, 0);
    glTexCoord2f(1.0f, vStart + 1.0f); glVertex2f(width(), 0);
    glTexCoord2f(1.0f, vStart); glVertex2f(width(), height());
    glTexCoord2f(0.0f, vStart); glVertex2f(0, height());
        glEnd();
        glBindTexture(GL_TEXTURE_2D, 0);
	}
}

QColor MyWaterfallWidget::valueToColor(float value, float contrastFactor, float sensitivityFactor) {
    if (!std::isfinite(value)) {
        value = 0.0f;
    }
    value = qBound(0.0f, value * sensitivityFactor, 1.0f);
    QVector<QColor> colorMap = {
        QColor("#000020"),
        QColor("#000050"),
        QColor("#000090"),
        QColor("#0000F0"),
        QColor("#0000FF"),
        QColor("#50F030"),
        QColor("#1E90FF"),
        QColor("#FFFFFF"),
        QColor("#FFFF00"),
        QColor("#FE6D16"),
        QColor("#FE6D16"),
        QColor("#FF0000"),
        QColor("#FF0000"),
        QColor("#C60000"),
        QColor("#9F0000"),
        QColor("#750000"),
        QColor("#4A0000"),
        //QColor("#F000F0")
    };
    int index = static_cast<int>(value * (colorMap.size() - 1));
    index = std::clamp(index, 0, colorMap.size() - 1);
    QColor baseColor = colorMap[index];
    int blue = (baseColor.green() + baseColor.blue())/3;
    int r = static_cast<int>(baseColor.red() * contrastFactor + blue * (1.0f - contrastFactor));
    int g = static_cast<int>(baseColor.green() * contrastFactor + blue * (1.0f - contrastFactor));
    int b = static_cast<int>(baseColor.blue() * contrastFactor + blue * (1.0f - contrastFactor));
    return QColor(r, g, b);
}

float MyWaterfallWidget::normalizedLevel(float value) const {
    if (!std::isfinite(value) || qFuzzyCompare(levelMin, levelMax)) {
        return 0.0f;
    }
    return qBound(0.0f, (value - levelMin) / (levelMax - levelMin), 1.0f);
}


 QColor MyWaterfallWidget::valueToColors(float value) {
    if (!std::isfinite(value)) {
        value = 0.0f;
    }
    value = qBound(0.0f, value, 1.0f);
    int r, g, b;
    if (value < 0.12f) {
        float ratio = value / 0.2f;
        r = 0;
        g = 0;
        b = static_cast<int>(255 * ratio);
    } else if (value < 0.26f) {
        float ratio = (value - 0.12f) / 0.2f;
        r = 0;
        g = static_cast<int>(255 * (1 - ratio));
        b = 255;
    } else if (value < 0.40f) {
        float ratio = (value - 0.28f) / 0.2f;
        r = 0;
        g = 255;
        b = static_cast<int>(255 * (1 - ratio));
    } else if (value < 0.54f) {
        float ratio = (value - 0.40f) / 0.2f;
        r = static_cast<int>(255 * ratio);
        g = 255;
        b = 0;
    } else if (value < 0.68f) {
        float ratio = (value - 0.54f) / 0.2f;
        r = 255;
        g = static_cast<int>(255 * (1 - 0.5f * ratio));
        b = 0;    
    } else if (value < 0.86f) {
        float ratio = (value - 0.68f) / 0.2f;
        r = 255;
        g = static_cast<int>(128 * (1 - 0.5f * ratio));
        b = 0;
    } else {
        float ratio = (value - 0.86f) / 0.2f;
        r = 255;
        g = 0;
        b = static_cast<int>(255 * (1 - ratio));
    }
    return QColor(r, g, b);
}
