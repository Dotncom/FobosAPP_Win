#ifndef SPECTRUMHUNTERCONTROLS_H
#define SPECTRUMHUNTERCONTROLS_H

#include <QGroupBox>

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;

class SpectrumHunterControls : public QGroupBox {
    Q_OBJECT

public:
    explicit SpectrumHunterControls(const QString &title,
                                    const QString &detectTooltip,
                                    const QString &widthSuffix,
                                    double minWidthRange,
                                    double maxWidthRange,
                                    double minWidthValue,
                                    double maxWidthValue,
                                    double minThresholdDb,
                                    double maxThresholdDb,
                                    double thresholdDb,
                                    QWidget *parent = nullptr);

    void addPreset(const QString &name, const QString &spec);
    QString currentPresetSpec() const;

    void setDetectChecked(bool checked);
    void setWidthValues(double minWidth, double maxWidth, double thresholdDb);
    void setControlsEnabled(bool enabled);
    void setTuneEnabled(bool enabled);
    void setFollowVisible(bool visible);
    void setFollowChecked(bool checked);
    void setFollowEnabled(bool enabled);
    void setCandidateNavigationVisible(bool visible);
    void setCandidateNavigationEnabled(bool enabled);
    void setCandidateIndex(int index, int count);
    void setStatusText(const QString &text);
    void setUiText(const QString &title,
                   const QString &detectTooltip,
                   const QString &detectText,
                   const QString &useScanText,
                   const QString &useScanTooltip,
                   const QString &tuneText,
                   const QString &tuneTooltip,
                   const QString &followText,
                   const QString &followTooltip,
                   const QString &previousTooltip,
                   const QString &nextTooltip,
                   const QString &minWidthText,
                   const QString &maxWidthText,
                   const QString &thresholdText);

signals:
    void detectToggled(bool checked);
    void minWidthChanged(double value);
    void maxWidthChanged(double value);
    void thresholdChanged(double value);
    void applyPresetRequested();
    void tuneRequested();
    void followToggled(bool checked);
    void previousCandidateRequested();
    void nextCandidateRequested();

private:
    QCheckBox *detectCheckbox = nullptr;
    QComboBox *presetCombo = nullptr;
    QPushButton *applyPresetButton = nullptr;
    QPushButton *tuneButton = nullptr;
    QCheckBox *followCheckbox = nullptr;
    QPushButton *previousCandidateButton = nullptr;
    QPushButton *nextCandidateButton = nullptr;
    QLabel *candidateIndexLabel = nullptr;
    QLabel *minWidthLabel = nullptr;
    QLabel *maxWidthLabel = nullptr;
    QLabel *thresholdLabel = nullptr;
    QDoubleSpinBox *minWidthSpin = nullptr;
    QDoubleSpinBox *maxWidthSpin = nullptr;
    QDoubleSpinBox *thresholdSpin = nullptr;
    QLabel *statusLabel = nullptr;
};

#endif // SPECTRUMHUNTERCONTROLS_H
