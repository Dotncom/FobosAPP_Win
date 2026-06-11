#include "MyWaterfallWidget.h"

#include <algorithm>
#include <cmath>
#include <limits>

bool changebit=false;

namespace {
constexpr double AUTO_TUNE_WINDOW_FRACTION = 1.0 / 80.0;
constexpr double AUTO_TUNE_MIN_WINDOW_HZ = 300.0;
constexpr double AUTO_TUNE_MAX_WINDOW_HZ = 500000.0;

struct SignalSample {
    double frequency = 0.0;
    float level = 0.0f;
};
}

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
    if (event->button() == Qt::MiddleButton) {
        emit autoTuneRequested(signalCenterNearFrequency(frequencyAtX(event->x())));
        event->accept();
        return;
    }
    QOpenGLWidget::mousePressEvent(event);
}

void MyWaterfallWidget::mouseDoubleClickEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        emit autoTuneRequested(signalCenterNearFrequency(frequencyAtX(event->x())));
        event->accept();
        return;
    }
    QOpenGLWidget::mouseDoubleClickEvent(event);
}

double MyWaterfallWidget::displayFrequencyAtX(int x) const {
    if (width() <= 0 || qFuzzyCompare(xMin, xMax)) {
        return xMin;
    }
    const double normalized = std::clamp(static_cast<double>(x) / (std::max)(1, width()), 0.0, 1.0);
    return xMin + normalized * (xMax - xMin);
}

double MyWaterfallWidget::actualFrequencyForDisplayFrequency(double displayFrequency) const {
    for (const ScanVisualSegment &segment : scanSegments) {
        if (!std::isfinite(segment.startHz) ||
            !std::isfinite(segment.endHz) ||
            !std::isfinite(segment.actualStartHz) ||
            !std::isfinite(segment.actualEndHz) ||
            segment.endHz <= segment.startHz ||
            displayFrequency < segment.startHz ||
            displayFrequency > segment.endHz) {
            continue;
        }
        const double ratio = (displayFrequency - segment.startHz) / (segment.endHz - segment.startHz);
        return segment.actualStartHz +
               std::clamp(ratio, 0.0, 1.0) * (segment.actualEndHz - segment.actualStartHz);
    }
    return displayFrequency;
}

double MyWaterfallWidget::displayFrequencyForActualFrequency(double actualFrequency) const {
    for (const ScanVisualSegment &segment : scanSegments) {
        if (!std::isfinite(segment.startHz) ||
            !std::isfinite(segment.endHz) ||
            !std::isfinite(segment.actualStartHz) ||
            !std::isfinite(segment.actualEndHz) ||
            segment.endHz <= segment.startHz ||
            segment.actualEndHz <= segment.actualStartHz ||
            actualFrequency < segment.actualStartHz ||
            actualFrequency > segment.actualEndHz) {
            continue;
        }
        const double ratio = (actualFrequency - segment.actualStartHz) /
                             (segment.actualEndHz - segment.actualStartHz);
        return segment.startHz + std::clamp(ratio, 0.0, 1.0) * (segment.endHz - segment.startHz);
    }
    return actualFrequency;
}

double MyWaterfallWidget::frequencyAtX(int x) const {
    return actualFrequencyForDisplayFrequency(displayFrequencyAtX(x));
}

double MyWaterfallWidget::signalCenterNearFrequency(double frequency) {
    QMutexLocker locker(&mutex);
    if (!std::isfinite(frequency) || qFuzzyCompare(xMin, xMax) || xData.empty() || yData.empty() || fftLength <= 0) {
        return frequency;
    }

    const int dataCount = std::min({fftLength, static_cast<int>(xData.size()), static_cast<int>(yData.size())});
    if (dataCount <= 0) {
        return frequency;
    }

    const double targetDisplayFrequency = displayFrequencyForActualFrequency(frequency);
    const double visibleSpan = std::abs(xMax - xMin);
    const double halfWindow = (std::clamp)(visibleSpan * AUTO_TUNE_WINDOW_FRACTION,
                                           AUTO_TUNE_MIN_WINDOW_HZ,
                                           AUTO_TUNE_MAX_WINDOW_HZ);
    std::vector<SignalSample> samples;
    samples.reserve(256);
    for (int i = 0; i < dataCount; ++i) {
        const double sampleFrequency = xData[i];
        if (!std::isfinite(sampleFrequency) ||
            std::abs(sampleFrequency - targetDisplayFrequency) > halfWindow) {
            continue;
        }
        const float level = yData[(i + dataCount / 2) % dataCount];
        if (!std::isfinite(level)) {
            continue;
        }
        samples.push_back({sampleFrequency, level});
    }
    if (samples.empty()) {
        return frequency;
    }

    std::sort(samples.begin(), samples.end(), [](const SignalSample &a, const SignalSample &b) {
        return a.frequency < b.frequency;
    });

    const int sampleCount = static_cast<int>(samples.size());
    const int smoothRadius = (std::clamp)(sampleCount / 160, 1, 6);
    std::vector<float> smoothedLevels(static_cast<std::size_t>(sampleCount), -160.0f);
    std::vector<float> sortedSmoothedLevels;
    sortedSmoothedLevels.reserve(static_cast<std::size_t>(sampleCount));
    for (int i = 0; i < sampleCount; ++i) {
        double sum = 0.0;
        int count = 0;
        for (int j = (std::max)(0, i - smoothRadius);
             j <= (std::min)(sampleCount - 1, i + smoothRadius);
             ++j) {
            sum += samples[j].level;
            ++count;
        }
        smoothedLevels[static_cast<std::size_t>(i)] =
            count > 0 ? static_cast<float>(sum / count) : samples[i].level;
        sortedSmoothedLevels.push_back(smoothedLevels[static_cast<std::size_t>(i)]);
    }
    std::sort(sortedSmoothedLevels.begin(), sortedSmoothedLevels.end());
    const float baseline = sortedSmoothedLevels[sortedSmoothedLevels.size() / 4];

    int peakIndex = 0;
    double bestScore = -std::numeric_limits<double>::infinity();
    for (int i = 0; i < sampleCount; ++i) {
        const double distanceRatio = std::abs(samples[i].frequency - targetDisplayFrequency) /
                                     (std::max)(1.0, halfWindow);
        const double score = smoothedLevels[static_cast<std::size_t>(i)] - distanceRatio * 8.0;
        if (score > bestScore) {
            bestScore = score;
            peakIndex = i;
        }
    }
    const float peakLevel = smoothedLevels[static_cast<std::size_t>(peakIndex)];
    const double dynamicRange = (std::max)(0.0, static_cast<double>(peakLevel - baseline));
    if (dynamicRange < 2.0) {
        return samples[peakIndex].frequency;
    }

    const float threshold = (std::max)(baseline + 3.0f,
                                       peakLevel - static_cast<float>((std::clamp)(dynamicRange * 0.42, 4.0, 10.0)));
    int left = peakIndex;
    while (left > 0 && smoothedLevels[static_cast<std::size_t>(left - 1)] >= threshold) {
        --left;
    }
    int right = peakIndex;
    while (right + 1 < sampleCount && smoothedLevels[static_cast<std::size_t>(right + 1)] >= threshold) {
        ++right;
    }

    if (right <= left) {
        return samples[peakIndex].frequency;
    }

    auto interpolatedEdge = [&](int inside, int outside) {
        if (outside < 0 || outside >= sampleCount) {
            return samples[inside].frequency;
        }
        const double insideLevel = smoothedLevels[static_cast<std::size_t>(inside)];
        const double outsideLevel = smoothedLevels[static_cast<std::size_t>(outside)];
        const double denom = insideLevel - outsideLevel;
        if (std::abs(denom) < 0.0001) {
            return samples[inside].frequency;
        }
        const double ratio = (insideLevel - threshold) / denom;
        return samples[inside].frequency +
               (samples[outside].frequency - samples[inside].frequency) *
                   (std::clamp)(ratio, 0.0, 1.0);
    };

    const double leftFrequency = interpolatedEdge(left, left - 1);
    const double rightFrequency = interpolatedEdge(right, right + 1);
    const double centerFrequency = (leftFrequency + rightFrequency) * 0.5;
    const double actualCenterFrequency = actualFrequencyForDisplayFrequency(centerFrequency);
    return std::isfinite(actualCenterFrequency)
               ? actualCenterFrequency
               : actualFrequencyForDisplayFrequency(samples[peakIndex].frequency);
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
        resizeWaterfallTexturePreserve(w, h);
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

void MyWaterfallWidget::resizeWaterfallTexturePreserve(int w, int h) {
    const int newWidth = std::max(1, w);
    const int newHeight = std::max(1, h);
    if (newWidth == textureWidth && newHeight == textureHeight) {
        return;
    }

    if (waterfallTexture == 0 || textureWidth <= 0 || textureHeight <= 0) {
        resetWaterfallTexture(newWidth, newHeight);
        return;
    }

    const int oldWidth = textureWidth;
    const int oldHeight = textureHeight;
    const int oldWriteRow = waterfallWriteRow;
    std::vector<unsigned char> oldPixels(static_cast<size_t>(oldWidth) * oldHeight * 3, 0);

    glBindTexture(GL_TEXTURE_2D, waterfallTexture);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    while (glGetError() != GL_NO_ERROR) {
    }
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RGB, GL_UNSIGNED_BYTE, oldPixels.data());
    if (glGetError() != GL_NO_ERROR) {
        glBindTexture(GL_TEXTURE_2D, 0);
        resetWaterfallTexture(newWidth, newHeight);
        return;
    }

    std::vector<unsigned char> resized(static_cast<size_t>(newWidth) * newHeight * 3, 0);
    for (int y = 0; y < newHeight; ++y) {
        const int sourceVisualY = std::min(oldHeight - 1, static_cast<int>((static_cast<long long>(y) * oldHeight) / newHeight));
        const int sourceRow = (oldWriteRow + sourceVisualY) % oldHeight;
        for (int x = 0; x < newWidth; ++x) {
            const int sourceX = std::min(oldWidth - 1, static_cast<int>((static_cast<long long>(x) * oldWidth) / newWidth));
            const size_t sourceIndex = (static_cast<size_t>(sourceRow) * oldWidth + sourceX) * 3;
            const size_t destIndex = (static_cast<size_t>(y) * newWidth + x) * 3;
            resized[destIndex + 0] = oldPixels[sourceIndex + 0];
            resized[destIndex + 1] = oldPixels[sourceIndex + 1];
            resized[destIndex + 2] = oldPixels[sourceIndex + 2];
        }
    }

    textureWidth = newWidth;
    textureHeight = newHeight;
    waterfallWriteRow = 0;
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, textureWidth, textureHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, resized.data());
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

void MyWaterfallWidget::setScanSegments(const QVector<ScanVisualSegment> &segments) {
    {
        QMutexLocker locker(&mutex);
        scanSegments = segments;
    }
    update();
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
    {
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
            resizeWaterfallTexturePreserve(texWidth, texHeight);
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
    QPainter painter(this);
    drawScanSegments(painter);
}

void MyWaterfallWidget::drawScanSegments(QPainter &painter) const {
    QMutexLocker locker(&mutex);
    if (scanSegments.size() < 2 || width() <= 0 || height() <= 0 || qFuzzyCompare(xMin, xMax)) {
        return;
    }

    painter.save();
    painter.setRenderHint(QPainter::Antialiasing, false);
    QPen edgePen(QColor(255, 255, 255, 42));
    edgePen.setWidth(1);
    painter.setPen(edgePen);
    for (const ScanVisualSegment &segment : scanSegments) {
        if (!std::isfinite(segment.startHz) || !std::isfinite(segment.endHz)) {
            continue;
        }
        const int left = static_cast<int>(std::round((segment.startHz - xMin) * width() / (xMax - xMin)));
        const int right = static_cast<int>(std::round((segment.endHz - xMin) * width() / (xMax - xMin)));
        if (right < 0 || left > width() || right <= left) {
            continue;
        }
        painter.drawLine(left, 0, left, height());
        painter.drawLine(right, 0, right, height());
    }
    painter.restore();
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
