#ifndef FREQUENCYCONTROL_H
#define FREQUENCYCONTROL_H

#include <QWidget>
#include <QVector>
#include <QPair>

class QComboBox;
class QDoubleSpinBox;
class QPushButton;

class FrequencyControl : public QWidget {
    Q_OBJECT

public:
    explicit FrequencyControl(QWidget *parent = nullptr);

    double valueHz() const;
    void setValueHz(double valueHz);
    void setRangeHz(double minimumHz, double maximumHz);
    void setStepPresets(const QVector<QPair<QString, double>> &stepsHz);
    void setValuePresets(const QVector<QPair<QString, double>> &valuesHz);

signals:
    void valueCommitted(double valueHz);

private:
    double unitFactor() const;
    int unitIndexForValue(double valueHz) const;
    void updateSpinRange();
    void updateDisplayedValue();
    void commitCurrentValue();
    void adjustByStep(int direction);

    QDoubleSpinBox *valueSpin = nullptr;
    QComboBox *unitCombo = nullptr;
    QComboBox *stepCombo = nullptr;
    QComboBox *presetCombo = nullptr;
    QPushButton *minusButton = nullptr;
    QPushButton *plusButton = nullptr;

    double currentValueHz = 0.0;
    double minimumValueHz = 0.0;
    double maximumValueHz = 6000000000.0;
    bool updatingUi = false;
};

#endif // FREQUENCYCONTROL_H
