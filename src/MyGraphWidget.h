#ifndef MYGRAPHWIDGET_H
#define MYGRAPHWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QVector>
#include <QColor>
#include <QString>
#include <vector>
#include <QWheelEvent>
#include <QPainter>
#include <QMouseEvent>
#include <QPoint>

struct GraphBandMarker {
    double startHz = 0.0;
    double endHz = 0.0;
    QString label;
    bool amateur = false;
};

class MyGraphWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    explicit MyGraphWidget(QWidget *parent = nullptr);
    ~MyGraphWidget();

    void setData(const std::vector<float> &xData, const std::vector<float> &yData, double xMin, double xMax, int fftLength, bool colorf);
    void setOverlayData(const std::vector<float> &yData, bool enabled);
    void setLevelRange(float minLevel, float maxLevel);
    void setBandMarkersEnabled(bool generalEnabled, bool amateurEnabled);
    void setBandMarkersCompact(bool compact);
    void setBandMarkers(const QVector<GraphBandMarker> &markers);
    void clearData();

signals:
    void scaleChanged(int direction);
    void tuneContextRequested(double frequency, const QPoint &globalPos);
    void autoTuneRequested(double frequency);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;

private:
    void drawBandMarkers(QPainter &painter) const;
    void drawYAxis(QPainter &painter) const;
    float normalizedLevel(float value) const;
    int bottomMargin() const;
    double frequencyAtX(int x) const;
    double signalCenterNearFrequency(double frequency) const;

    double xMin, xMax, yMin, yMax;
    int fftLength;
    bool initialized;
    bool colorf;

    std::vector<float> xData, yData, overlayYData;
    bool overlayEnabled = false;
    bool generalBandMarkersEnabled = false;
    bool amateurBandMarkersEnabled = false;
    bool compactBandMarkersEnabled = false;
    QVector<GraphBandMarker> bandMarkers;
    QColor valueToColor(float value);
};

#endif // MYGRAPHWIDGET_H
