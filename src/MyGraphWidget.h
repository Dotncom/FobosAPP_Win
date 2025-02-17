#ifndef MYGRAPHWIDGET_H
#define MYGRAPHWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QVector>
#include <QColor>
#include <vector>
#include <QWheelEvent>

class MyGraphWidget : public QOpenGLWidget, protected QOpenGLFunctions {
    Q_OBJECT

public:
    explicit MyGraphWidget(QWidget *parent = nullptr);
    ~MyGraphWidget();

    void setData(const std::vector<float> &xData, const std::vector<float> &yData, double xMin, double xMax, int fftLength, bool colorf);

signals:
    void scaleChanged(int direction);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;
    void wheelEvent(QWheelEvent *event) override;

private:
    double xMin, xMax, yMin, yMax;
    int fftLength;
    bool initialized;
    bool colorf;

    std::vector<float> xData, yData;
    QColor valueToColor(float value);
};

#endif // MYGRAPHWIDGET_H
