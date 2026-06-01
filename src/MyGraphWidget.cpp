#include "MyGraphWidget.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr int GRAPH_LEFT_MARGIN = 0;
constexpr int GRAPH_RIGHT_MARGIN = 8;
constexpr int GRAPH_TOP_MARGIN = 8;
constexpr int GRAPH_BOTTOM_MARGIN = 20;
constexpr int GRAPH_COMPACT_BAND_BOTTOM_MARGIN = 30;
constexpr int GRAPH_Y_TICKS = 6;
constexpr double AUTO_TUNE_WINDOW_FRACTION = 1.0 / 80.0;
constexpr double AUTO_TUNE_MIN_WINDOW_HZ = 300.0;
constexpr double AUTO_TUNE_MAX_WINDOW_HZ = 500000.0;

struct SignalSample {
    double frequency = 0.0;
    float level = 0.0f;
};
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

void MyGraphWidget::setOverlayData(const std::vector<float> &yData, bool enabled) {
    overlayYData = enabled ? yData : std::vector<float>();
    overlayEnabled = enabled && !overlayYData.empty();
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

void MyGraphWidget::setBandMarkersEnabled(bool generalEnabled, bool amateurEnabled) {
    if (generalBandMarkersEnabled == generalEnabled && amateurBandMarkersEnabled == amateurEnabled) {
        return;
    }
    generalBandMarkersEnabled = generalEnabled;
    amateurBandMarkersEnabled = amateurEnabled;
    update();
}

void MyGraphWidget::setBandMarkersCompact(bool compact) {
    if (compactBandMarkersEnabled == compact) {
        return;
    }
    compactBandMarkersEnabled = compact;
    update();
}

void MyGraphWidget::setBandMarkers(const QVector<GraphBandMarker> &markers) {
    bandMarkers = markers;
    update();
}

void MyGraphWidget::clearData() {
    xData.clear();
    yData.clear();
    overlayYData.clear();
    overlayEnabled = false;
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
        const float plotBottom = static_cast<float>(bottomMargin());
        const float plotWidth = static_cast<float>((std::max)(1, width() - GRAPH_LEFT_MARGIN - GRAPH_RIGHT_MARGIN));
        const float plotHeight = static_cast<float>((std::max)(1, height() - GRAPH_TOP_MARGIN - bottomMargin()));
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

        if (overlayEnabled && !overlayYData.empty()) {
            glColor3f(1.0f, 0.22f, 0.16f);
            glBegin(GL_LINE_STRIP);
            const int overlayCount = std::min({dataCount, static_cast<int>(overlayYData.size())});
            for (int i = 0; i < overlayCount; ++i) {
                if (!std::isfinite(xData[i])) {
                    continue;
                }
                const float xNorm = static_cast<float>((xData[i] - xMin) / (xMax - xMin));
                if (xNorm < 0.0f || xNorm > 1.0f) {
                    continue;
                }
                float intensity = overlayYData[(i + overlayCount / 2) % overlayCount];
                if (!std::isfinite(intensity)) {
                    intensity = yMin;
                }
                const float level = normalizedLevel(intensity);
                const float xPos = plotLeft + xNorm * plotWidth;
                const float yPos = plotBottom + level * plotHeight;
                glVertex2f(xPos, yPos);
            }
            glEnd();
        }
    }
    painter.endNativePainting();
    drawBandMarkers(painter);
    drawYAxis(painter);
}

void MyGraphWidget::wheelEvent(QWheelEvent *event) {
    emit scaleChanged(event->angleDelta().y() > 0 ? 1 : -1);
    event->accept();
}

void MyGraphWidget::mousePressEvent(QMouseEvent *event) {
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

void MyGraphWidget::mouseDoubleClickEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        emit autoTuneRequested(signalCenterNearFrequency(frequencyAtX(event->x())));
        event->accept();
        return;
    }
    QOpenGLWidget::mouseDoubleClickEvent(event);
}

double MyGraphWidget::frequencyAtX(int x) const {
    if (width() <= 0 || qFuzzyCompare(xMin, xMax)) {
        return xMin;
    }
    const double plotLeft = GRAPH_LEFT_MARGIN;
    const double plotWidth = (std::max)(1, width() - GRAPH_LEFT_MARGIN - GRAPH_RIGHT_MARGIN);
    const double normalized = std::clamp((static_cast<double>(x) - plotLeft) / plotWidth, 0.0, 1.0);
    return xMin + normalized * (xMax - xMin);
}

double MyGraphWidget::signalCenterNearFrequency(double frequency) const {
    if (!std::isfinite(frequency) || qFuzzyCompare(xMin, xMax) || xData.empty() || yData.empty() || fftLength <= 0) {
        return frequency;
    }

    const int dataCount = std::min({fftLength, static_cast<int>(xData.size()), static_cast<int>(yData.size())});
    if (dataCount <= 0) {
        return frequency;
    }

    const double visibleSpan = std::abs(xMax - xMin);
    const double halfWindow = (std::clamp)(visibleSpan * AUTO_TUNE_WINDOW_FRACTION,
                                           AUTO_TUNE_MIN_WINDOW_HZ,
                                           AUTO_TUNE_MAX_WINDOW_HZ);
    std::vector<SignalSample> samples;
    samples.reserve(256);
    std::vector<float> levels;
    levels.reserve(256);
    for (int i = 0; i < dataCount; ++i) {
        const double sampleFrequency = xData[i];
        if (!std::isfinite(sampleFrequency) || std::abs(sampleFrequency - frequency) > halfWindow) {
            continue;
        }
        const float level = yData[(i + dataCount / 2) % dataCount];
        if (!std::isfinite(level)) {
            continue;
        }
        samples.push_back({sampleFrequency, level});
        levels.push_back(level);
    }
    if (samples.empty()) {
        return frequency;
    }

    std::sort(samples.begin(), samples.end(), [](const SignalSample &a, const SignalSample &b) {
        return a.frequency < b.frequency;
    });
    std::sort(levels.begin(), levels.end());
    const float baseline = levels[levels.size() / 4];

    int peakIndex = 0;
    for (int i = 1; i < static_cast<int>(samples.size()); ++i) {
        if (samples[i].level > samples[peakIndex].level) {
            peakIndex = i;
        }
    }
    const float peakLevel = samples[peakIndex].level;
    const double dynamicRange = (std::max)(0.0, static_cast<double>(peakLevel - baseline));
    if (dynamicRange < 2.0) {
        return samples[peakIndex].frequency;
    }

    const float threshold = baseline + static_cast<float>((std::max)(3.0, dynamicRange * 0.35));
    int left = peakIndex;
    while (left > 0 && samples[left - 1].level >= threshold) {
        --left;
    }
    int right = peakIndex;
    while (right + 1 < static_cast<int>(samples.size()) && samples[right + 1].level >= threshold) {
        ++right;
    }

    double weightedFrequency = 0.0;
    double weightSum = 0.0;
    for (int i = left; i <= right; ++i) {
        const double weight = std::pow(10.0, (samples[i].level - threshold) / 20.0);
        weightedFrequency += samples[i].frequency * weight;
        weightSum += weight;
    }
    if (weightSum <= 0.0 || !std::isfinite(weightedFrequency)) {
        return samples[peakIndex].frequency;
    }
    return weightedFrequency / weightSum;
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

int MyGraphWidget::bottomMargin() const {
    return compactBandMarkersEnabled && (generalBandMarkersEnabled || amateurBandMarkersEnabled)
               ? GRAPH_COMPACT_BAND_BOTTOM_MARGIN
               : GRAPH_BOTTOM_MARGIN;
}

void MyGraphWidget::drawBandMarkers(QPainter &painter) const {
    if (bandMarkers.isEmpty() ||
        (!generalBandMarkersEnabled && !amateurBandMarkersEnabled) ||
        width() <= 0 ||
        qFuzzyCompare(xMin, xMax)) {
        return;
    }

    const int plotLeft = GRAPH_LEFT_MARGIN;
    const int plotRight = (std::max)(plotLeft + 1, width() - GRAPH_RIGHT_MARGIN);
    const int plotTop = GRAPH_TOP_MARGIN;
    const int plotBottom = (std::max)(plotTop + 1, height() - bottomMargin());
    const int markerTop = compactBandMarkersEnabled ? plotBottom : plotTop;
    const int markerBottom = compactBandMarkersEnabled ? height() : plotBottom;
    const int markerHeight = (std::max)(1, markerBottom - markerTop);
    const double viewStart = (std::min)(xMin, xMax);
    const double viewEnd = (std::max)(xMin, xMax);
    const double span = xMax - xMin;
    const double plotWidth = static_cast<double>((std::max)(1, plotRight - plotLeft));

    auto frequencyToX = [&](double frequency) {
        return plotLeft + ((frequency - xMin) / span) * plotWidth;
    };

    auto drawLayer = [&](bool amateurLayer) {
        if (amateurLayer && !amateurBandMarkersEnabled) {
            return;
        }
        if (!amateurLayer && !generalBandMarkersEnabled) {
            return;
        }

        const QColor fillColor = compactBandMarkersEnabled
                                     ? (amateurLayer ? QColor(255, 198, 66, 128)
                                                     : QColor(76, 162, 255, 112))
                                     : (amateurLayer ? QColor(255, 198, 66, 42)
                                                     : QColor(76, 162, 255, 32));
        const QColor edgeColor = compactBandMarkersEnabled
                                     ? (amateurLayer ? QColor(255, 220, 96, 220)
                                                     : QColor(112, 196, 255, 200))
                                     : (amateurLayer ? QColor(255, 220, 96, 110)
                                                     : QColor(112, 196, 255, 90));
        const QColor textColor = compactBandMarkersEnabled
                                     ? (amateurLayer ? QColor(255, 236, 170, 245)
                                                     : QColor(210, 236, 255, 235))
                                     : (amateurLayer ? QColor(255, 232, 150, 210)
                                                     : QColor(176, 224, 255, 190));

        for (const GraphBandMarker &marker : bandMarkers) {
            if (marker.amateur != amateurLayer ||
                !std::isfinite(marker.startHz) ||
                !std::isfinite(marker.endHz) ||
                marker.endHz <= marker.startHz ||
                marker.endHz < viewStart ||
                marker.startHz > viewEnd) {
                continue;
            }

            const double clippedStart = (std::max)(marker.startHz, viewStart);
            const double clippedEnd = (std::min)(marker.endHz, viewEnd);
            int x1 = static_cast<int>(std::floor(frequencyToX(clippedStart)));
            int x2 = static_cast<int>(std::ceil(frequencyToX(clippedEnd)));
            if (x2 < x1) {
                std::swap(x1, x2);
            }
            x1 = (std::clamp)(x1, plotLeft, plotRight);
            x2 = (std::clamp)(x2, plotLeft, plotRight);
            const int markerWidth = (std::max)(1, x2 - x1);

            painter.fillRect(QRect(x1, markerTop, markerWidth, markerHeight), fillColor);
            painter.setPen(edgeColor);
            painter.drawLine(x1, markerTop, x1, markerBottom);
            if (markerWidth > 2) {
                painter.drawLine(x1 + markerWidth, markerTop, x1 + markerWidth, markerBottom);
            }

            if (markerWidth >= 44 && !marker.label.trimmed().isEmpty()) {
                const int textY = compactBandMarkersEnabled
                                      ? markerTop + (std::max)(2, (markerHeight - 15) / 2)
                                      : markerTop + 2;
                const QRect textRect(x1 + 3, textY, markerWidth - 6, 15);
                const QString label = painter.fontMetrics().elidedText(marker.label.trimmed(),
                                                                       Qt::ElideRight,
                                                                       textRect.width());
                painter.setPen(textColor);
                painter.drawText(textRect, Qt::AlignLeft | Qt::AlignVCenter, label);
            }
        }
    };

    painter.save();
    painter.setRenderHint(QPainter::Antialiasing, false);
    if (compactBandMarkersEnabled) {
        painter.fillRect(QRect(plotLeft, markerTop, plotRight - plotLeft, markerHeight), QColor(0, 0, 0, 210));
        painter.setPen(QColor(80, 96, 92, 160));
        painter.drawLine(plotLeft, markerTop, plotRight, markerTop);
    }
    painter.setClipRect(QRect(plotLeft, markerTop, plotRight - plotLeft, markerHeight));
    drawLayer(false);
    drawLayer(true);
    painter.restore();
}

void MyGraphWidget::drawYAxis(QPainter &painter) const {
    const int plotLeft = GRAPH_LEFT_MARGIN;
    const int plotRight = (std::max)(plotLeft + 1, width() - GRAPH_RIGHT_MARGIN);
    const int plotTop = GRAPH_TOP_MARGIN;
    const int plotBottom = (std::max)(plotTop + 1, height() - bottomMargin());
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
