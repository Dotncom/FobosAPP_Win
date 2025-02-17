#ifndef SCALEWIDGET_H
#define SCALEWIDGET_H

#include <QObject>
#include <QWidget>
#include <QPainter>
#include <QPen>
#include <QMouseEvent>
#include <QWheelEvent>

extern double globalBandwidth;
extern double listeningFrequency;
extern double globalFrequency; 

class ScaleWidget : public QWidget {
    Q_OBJECT

public:
    explicit ScaleWidget(QWidget *parent = nullptr);
	
    void setRange(double min, double max);
    void setMarkerPosition(double position);

    void paintEvent(QPaintEvent *event) override;
	void mousePressEvent(QMouseEvent *event) override;
	void wheelEvent(QWheelEvent *event) override;
	void mouseMoveEvent(QMouseEvent *event) override;
	void mouseReleaseEvent(QMouseEvent *event) override;
signals:
    void frequencyChanged();
signals:
    void centralFrequencyChanged();
private:
    double minValue;
    double maxValue;
    double markerPosition;
    
    bool dragging = false;
    int lastMouseX = 0;
};

#endif // SCALEWIDGET_H
