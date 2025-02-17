#include "scalewidget.h"

extern double listeningFrequency;
extern double globalBandwidth;
extern double globalFrequency; 

ScaleWidget::ScaleWidget(QWidget *parent)
    : QWidget(parent), minValue(0), maxValue(100), markerPosition(0.5) {}

void ScaleWidget::setRange(double min, double max) {
    minValue = min;
    maxValue = max;
    setMarkerPosition((listeningFrequency - minValue) / (maxValue - minValue));
    update();  
}

void ScaleWidget::setMarkerPosition(double position) {
    markerPosition = position;
    update();
}

void ScaleWidget::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        double newMarkerPos = static_cast<double>(event->x()) / width();
        setMarkerPosition(newMarkerPos);
        listeningFrequency = minValue + newMarkerPos * (maxValue - minValue);
        emit frequencyChanged(); 
    } else if (event->button() == Qt::RightButton) {
		double newMPos = static_cast<double>(event->x()) / width();
        globalFrequency = minValue + newMPos * (maxValue - minValue);
        emit centralFrequencyChanged();
        dragging = true;
        lastMouseX = event->x();
    }
    
}

void ScaleWidget::mouseMoveEvent(QMouseEvent *event) {
    if (dragging) {
        int deltaX = event->x() - lastMouseX;
        double frequencyShift = deltaX * (maxValue - minValue) / width();
        globalFrequency -= frequencyShift;
        lastMouseX = event->x();
        emit centralFrequencyChanged(); 
        update();
    }
}

void ScaleWidget::mouseReleaseEvent(QMouseEvent *event) {
    if (event->button() == Qt::RightButton) {
        dragging = false;
    }
}

void ScaleWidget::wheelEvent(QWheelEvent *event) {
    int numSteps = event->angleDelta().y() / 120; 
    double stepSize = (maxValue - minValue) / 100; 
    listeningFrequency += numSteps * stepSize;
    if (listeningFrequency < minValue) {
        listeningFrequency = minValue;
    } else if (listeningFrequency > maxValue) {
        listeningFrequency = maxValue;
    }
    setMarkerPosition((listeningFrequency - minValue) / (maxValue - minValue));
    emit frequencyChanged();
}


void ScaleWidget::paintEvent(QPaintEvent *event) {
    QPainter painter(this);
    int width = this->width();
    int height = this->height();
    painter.drawLine(0, height / 2, width, height / 2);
    int numTicks = 10; 
    int numMinorTicks = 9;
    //double tickInterval = (maxValue - minValue) / numTicks;
    //double minorTickInterval = tickInterval / (numMinorTicks + 1);
    for (int i = 0; i <= numTicks; ++i) {
        double value = minValue + i * (maxValue - minValue) / numTicks;
        double pos = static_cast<double>(i) / numTicks * width;
        QString label = QString::number(value / 1e6, 'f', 3) + " MHz";
        painter.drawText(pos - 20, height - 5, label);
        painter.drawLine(pos, height / 2 - 10, pos, height / 2 + 10);
            for (int j = 1; j <= numMinorTicks; ++j) {
            double minorPos = pos + j * width / (numTicks * (numMinorTicks + 1));
            painter.drawLine(minorPos, height / 2 - 3, minorPos, height / 2 + 3);
        }
    }
    int markerX = static_cast<int>(markerPosition * width);
    double bandwidthFraction = globalBandwidth / (maxValue - minValue);
    int bandwidthPixels = static_cast<int>(bandwidthFraction * width);

    QColor green(0, 255, 0, 100);
    painter.setBrush(green);
    painter.setPen(Qt::NoPen);

    int bandwidthX = markerX - bandwidthPixels / 2;
    painter.drawRect(bandwidthX, 0, bandwidthPixels, height);
    painter.setPen(Qt::red);
    painter.drawLine(markerX, 0, markerX, height);
}
