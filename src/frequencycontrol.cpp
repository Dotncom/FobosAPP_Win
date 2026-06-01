#include "frequencycontrol.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSignalBlocker>

#include <algorithm>
#include <cmath>
#include <iterator>

namespace {

struct UnitInfo {
    const char *label;
    double factor;
    int decimals;
};

constexpr UnitInfo UNITS[] = {
    {"Hz", 1.0, 0},
    {"kHz", 1000.0, 3},
    {"MHz", 1000000.0, 6},
    {"GHz", 1000000000.0, 9},
};

QVector<QPair<QString, double>> defaultSteps() {
    return {
        {"1 Hz", 1.0},
        {"10 Hz", 10.0},
        {"100 Hz", 100.0},
        {"1 kHz", 1000.0},
        {"5 kHz", 5000.0},
        {"10 kHz", 10000.0},
        {"100 kHz", 100000.0},
        {"1 MHz", 1000000.0},
        {"5 MHz", 5000000.0},
        {"10 MHz", 10000000.0},
        {"50 MHz", 50000000.0},
        {"100 MHz", 100000000.0},
    };
}

} // namespace

FrequencyControl::FrequencyControl(QWidget *parent)
    : QWidget(parent) {
    valueSpin = new QDoubleSpinBox(this);
    valueSpin->setKeyboardTracking(false);
    valueSpin->setButtonSymbols(QAbstractSpinBox::NoButtons);
    valueSpin->setAlignment(Qt::AlignRight);
    valueSpin->setMinimumWidth(125);

    unitCombo = new QComboBox(this);
    for (const UnitInfo &unit : UNITS) {
        unitCombo->addItem(unit.label, unit.factor);
    }

    stepCombo = new QComboBox(this);
    setStepPresets(defaultSteps());

    presetCombo = new QComboBox(this);
    presetCombo->addItem("Preset", QVariant());
    presetCombo->setEnabled(false);

    minusButton = new QPushButton("-", this);
    plusButton = new QPushButton("+", this);
    minusButton->setFixedWidth(28);
    plusButton->setFixedWidth(28);

    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(4);
    layout->addWidget(minusButton);
    layout->addWidget(valueSpin, 1);
    layout->addWidget(unitCombo);
    layout->addWidget(stepCombo);
    layout->addWidget(plusButton);
    layout->addWidget(presetCombo);

    connect(valueSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) {
        if (!updatingUi) {
            commitCurrentValue();
        }
    });
    connect(unitCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
        updateSpinRange();
        updateDisplayedValue();
    });
    connect(stepCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
        updateSpinRange();
    });
    connect(minusButton, &QPushButton::clicked, this, [this]() {
        adjustByStep(-1);
    });
    connect(plusButton, &QPushButton::clicked, this, [this]() {
        adjustByStep(1);
    });
    connect(presetCombo, QOverload<int>::of(&QComboBox::activated), this, [this](int index) {
        const QVariant data = presetCombo->itemData(index);
        if (!data.isValid()) {
            return;
        }
        setValueHz(data.toDouble());
        emit valueCommitted(currentValueHz);
    });

    unitCombo->setCurrentIndex(2);
    updateSpinRange();
    updateDisplayedValue();
}

double FrequencyControl::valueHz() const {
    return currentValueHz;
}

void FrequencyControl::setValueHz(double valueHz) {
    currentValueHz = (std::clamp)(valueHz, minimumValueHz, maximumValueHz);
    const int preferredUnit = unitIndexForValue(currentValueHz);
    if (unitCombo->currentIndex() != preferredUnit) {
        QSignalBlocker unitBlocker(unitCombo);
        unitCombo->setCurrentIndex(preferredUnit);
        updateSpinRange();
    }
    updateDisplayedValue();
}

void FrequencyControl::setRangeHz(double minimumHz, double maximumHz) {
    minimumValueHz = std::isfinite(minimumHz) ? minimumHz : 0.0;
    maximumValueHz = std::isfinite(maximumHz) ? maximumHz : minimumValueHz + 1.0;
    if (maximumValueHz <= minimumValueHz) {
        maximumValueHz = minimumValueHz + 1.0;
    }
    currentValueHz = (std::clamp)(currentValueHz, minimumValueHz, maximumValueHz);
    updateSpinRange();
    updateDisplayedValue();
}

void FrequencyControl::setStepPresets(const QVector<QPair<QString, double>> &stepsHz) {
    QSignalBlocker blocker(stepCombo);
    stepCombo->clear();
    for (const auto &step : stepsHz) {
        if (step.second > 0.0) {
            stepCombo->addItem(step.first, step.second);
        }
    }
    if (stepCombo->count() == 0) {
        stepCombo->addItem("1 kHz", 1000.0);
    }
}

void FrequencyControl::setValuePresets(const QVector<QPair<QString, double>> &valuesHz) {
    QSignalBlocker blocker(presetCombo);
    presetCombo->clear();
    presetCombo->addItem("Preset", QVariant());
    for (const auto &preset : valuesHz) {
        if (std::isfinite(preset.second)) {
            presetCombo->addItem(preset.first, preset.second);
        }
    }
    presetCombo->setEnabled(presetCombo->count() > 1);
}

double FrequencyControl::unitFactor() const {
    const int index = (std::clamp)(unitCombo->currentIndex(), 0, static_cast<int>(std::size(UNITS)) - 1);
    return UNITS[index].factor;
}

int FrequencyControl::unitIndexForValue(double valueHz) const {
    const double absValue = std::abs(valueHz);
    if (absValue >= 1000000000.0) {
        return 3;
    }
    if (absValue >= 1000000.0) {
        return 2;
    }
    if (absValue >= 1000.0) {
        return 1;
    }
    return 0;
}

void FrequencyControl::updateSpinRange() {
    const int index = (std::clamp)(unitCombo->currentIndex(), 0, static_cast<int>(std::size(UNITS)) - 1);
    const double factor = UNITS[index].factor;
    const double stepHz = stepCombo->currentData().toDouble();
    QSignalBlocker blocker(valueSpin);
    valueSpin->setDecimals(UNITS[index].decimals);
    valueSpin->setRange(minimumValueHz / factor, maximumValueHz / factor);
    valueSpin->setSingleStep((std::max)(stepHz / factor, std::pow(10.0, -UNITS[index].decimals)));
}

void FrequencyControl::updateDisplayedValue() {
    updatingUi = true;
    QSignalBlocker blocker(valueSpin);
    valueSpin->setValue(currentValueHz / unitFactor());
    updatingUi = false;
}

void FrequencyControl::commitCurrentValue() {
    currentValueHz = (std::clamp)(valueSpin->value() * unitFactor(), minimumValueHz, maximumValueHz);
    updateDisplayedValue();
    emit valueCommitted(currentValueHz);
}

void FrequencyControl::adjustByStep(int direction) {
    const double stepHz = stepCombo->currentData().toDouble();
    if (stepHz <= 0.0) {
        return;
    }
    setValueHz(currentValueHz + direction * stepHz);
    emit valueCommitted(currentValueHz);
}
