#ifndef MYWATERFALLWIDGET_H
#define MYWATERFALLWIDGET_H

#include <QObject>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <vector>
#include <QColor>
#include <QThread>
#include <QOpenGLTexture>
#include <QMutex>
#include <QWheelEvent>
#include <QPainter>
#include <cmath>

class MyWaterfallWidget;

// Declare the ComputeThread class
class ComputeThread : public QThread {
    Q_OBJECT
public:
    explicit ComputeThread(MyWaterfallWidget *widget);
    void run() override;

private:
    MyWaterfallWidget *widget;
};

class MyWaterfallWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT
public:
    explicit MyWaterfallWidget(QWidget *parent = nullptr);
    ~MyWaterfallWidget();
    bool initialized;
    void setData(const std::vector<float> &xData, const std::vector<float> &yData, double minFrequency, double maxFrequency, int fftLength, bool secondGraph, float contrast, float sensitivity);
    void computeLineData();
signals:
    void scaleChanged(int delta);
protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
	void wheelEvent(QWheelEvent *event) override;
private:
QMutex mutex;
	QOpenGLBuffer waterfallVbo;
    GLuint waterfallTexture;
    unsigned char* lineData;
    QColor valueToColor(float value, float contrastFactor, float sensitivityFactor);
    QColor valueToColors(float value);
    std::vector<float> xData;
    std::vector<float> yData;
    float yMin, yMax, contrast, sensitivity;
    double xMin, xMax;
    int fftLength;
    bool secondGraph;
    bool changebit;
    ComputeThread *computeThread; 
};

#endif // MYWATERFALLWIDGET_H
