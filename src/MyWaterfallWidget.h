#ifndef MYWATERFALLWIDGET_H
#define MYWATERFALLWIDGET_H

#include <QObject>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <vector>
#include <QColor>
#include <QOpenGLTexture>
#include <QMutex>
#include <QWheelEvent>
#include <QPainter>
#include <QMouseEvent>
#include <QPoint>
#include <cmath>

class MyWaterfallWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit MyWaterfallWidget(QWidget *parent = nullptr);
    ~MyWaterfallWidget();
    bool initialized;
    void setData(const std::vector<float> &xData, const std::vector<float> &yData, double minFrequency, double maxFrequency, int fftLength, bool secondGraph, float contrast, float sensitivity, float levelMin, float levelMax);
    void setLevelRange(float minLevel, float maxLevel);
    void clearData();
    void computeLineData();
signals:
    void scaleChanged(int delta);
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
    void ensureLineBuffer();
    void resetWaterfallTexture(int w, int h);
    void resizeWaterfallTexturePreserve(int w, int h);
    double frequencyAtX(int x) const;
    double signalCenterNearFrequency(double frequency);
    QMutex mutex;
	QOpenGLBuffer waterfallVbo;
    GLuint waterfallTexture;
    std::vector<unsigned char> lineData;
    QColor valueToColor(float value, float contrastFactor, float sensitivityFactor);
    QColor valueToColors(float value);
    float normalizedLevel(float value) const;
    std::vector<float> xData;
    std::vector<float> yData;
    float yMin, yMax, contrast, sensitivity, levelMin, levelMax;
    double xMin, xMax;
    int fftLength;
    int textureWidth = 0;
    int textureHeight = 0;
    int waterfallWriteRow = 0;
    bool secondGraph;
    bool changebit;
    bool pendingTextureLine = false;
    bool textureClearRequested = false;
};

#endif // MYWATERFALLWIDGET_H
