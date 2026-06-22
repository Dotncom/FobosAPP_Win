#include "main.h"

#include "appconstants.h"
#include "apphelp.h"
#include "appsettingsutils.h"
#include "diagnosticlogging.h"

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QVBoxLayout>

#include <algorithm>

void YourClassName::openApplicationHelp() {
    QWidget *parentWidget = QApplication::activeWindow();
    QDialog dialog(parentWidget ? parentWidget : static_cast<QWidget*>(this));
    dialog.setWindowTitle(uiText(QStringLiteral("program_help_title"),
                                 QStringLiteral("FobosAPP feature guide")));
    dialog.resize(760, 640);

    QVBoxLayout *rootLayout = new QVBoxLayout(&dialog);
    QPlainTextEdit *helpText = new QPlainTextEdit(&dialog);
    helpText->setReadOnly(true);
    helpText->setLineWrapMode(QPlainTextEdit::WidgetWidth);
    helpText->setPlainText(applicationHelpText(uiLanguage));
    rootLayout->addWidget(helpText, 1);

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);
    if (QPushButton *closeButton = buttonBox->button(QDialogButtonBox::Close)) {
        closeButton->setText(uiText(QStringLiteral("close"), QStringLiteral("Close")));
    }
    rootLayout->addWidget(buttonBox);
    connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    dialog.exec();
}

void YourClassName::openApplicationSettings() {
    QDialog dialog(this);
    dialog.setWindowTitle(uiText(QStringLiteral("settings"), QStringLiteral("Settings...")));
    dialog.setMinimumWidth(420);

    QVBoxLayout *rootLayout = new QVBoxLayout(&dialog);

    QFormLayout *generalLayout = new QFormLayout();
    QComboBox *languageCombo = new QComboBox(&dialog);
    populateLanguageCombo(languageCombo);
    languageCombo->setCurrentIndex(languageCombo->findData(uiLanguage));
    if (languageCombo->currentIndex() < 0) {
        languageCombo->setCurrentIndex(0);
    }

    QComboBox *fineTuneModeCombo = new QComboBox(&dialog);
    fineTuneModeCombo->addItem(uiText(QStringLiteral("fine_tune_scale"), QStringLiteral("Horizontal scale (mouse wheel)")),
                               FINE_TUNE_MODE_SCALE);
    fineTuneModeCombo->addItem(uiText(QStringLiteral("fine_tune_dial"), QStringLiteral("Round dial")),
                               FINE_TUNE_MODE_DIAL);
    const int fineTuneIndex = fineTuneModeCombo->findData(fineTuneControlMode);
    fineTuneModeCombo->setCurrentIndex(fineTuneIndex >= 0 ? fineTuneIndex : 0);

    QSpinBox *spectrumUpdateSpin = new QSpinBox(&dialog);
    spectrumUpdateSpin->setRange(SPECTRUM_UPDATE_AUTO_MS, SPECTRUM_UPDATE_MAX_MS);
    spectrumUpdateSpin->setSpecialValueText(uiText(QStringLiteral("auto"), QStringLiteral("Auto")));
    spectrumUpdateSpin->setSuffix(QStringLiteral(" ms"));
    spectrumUpdateSpin->setSingleStep(1);
    spectrumUpdateSpin->setValue(spectrumUpdateIntervalMs);
    spectrumUpdateSpin->setToolTip(uiText(
        QStringLiteral("spectrum_update_interval_tooltip"),
        QStringLiteral("Spectrum and waterfall update interval. Auto keeps the FFT-dependent default.")));

    QSpinBox *waterfallRowsSpin = new QSpinBox(&dialog);
    waterfallRowsSpin->setRange(WATERFALL_ROWS_PER_FRAME_MIN, WATERFALL_ROWS_PER_FRAME_MAX);
    waterfallRowsSpin->setSuffix(QStringLiteral(" rows/frame"));
    waterfallRowsSpin->setSingleStep(1);
    waterfallRowsSpin->setValue((std::clamp)(waterfallRowsPerFrame,
                                             WATERFALL_ROWS_PER_FRAME_MIN,
                                             WATERFALL_ROWS_PER_FRAME_MAX));
    waterfallRowsSpin->setToolTip(uiText(
        QStringLiteral("waterfall_speed_tooltip"),
        QStringLiteral("Visual waterfall scroll speed. Higher values move more rows per FFT frame without increasing FFT load.")));

    QSpinBox *scanMeasurementUpdateSpin = new QSpinBox(&dialog);
    scanMeasurementUpdateSpin->setRange(SCAN_MEASUREMENT_MIN_UPDATE_MS,
                                        SCAN_MEASUREMENT_MAX_UPDATE_MS);
    scanMeasurementUpdateSpin->setSuffix(QStringLiteral(" ms"));
    scanMeasurementUpdateSpin->setSingleStep(20);
    scanMeasurementUpdateSpin->setValue((std::clamp)(scanMeasurementUpdateIntervalMs,
                                                     SCAN_MEASUREMENT_MIN_UPDATE_MS,
                                                     SCAN_MEASUREMENT_MAX_UPDATE_MS));
    scanMeasurementUpdateSpin->setToolTip(uiText(
        QStringLiteral("scan_measurement_update_interval_tooltip"),
        QStringLiteral("How often the spectrum measurement overlay accumulates bins. Higher values reduce CPU/UI load and keep the waterfall smoother.")));

    QSpinBox *agileLiveRetuneIntervalSpin = new QSpinBox(&dialog);
    agileLiveRetuneIntervalSpin->setRange(AGILE_LIVE_RETUNE_MIN_COMMAND_INTERVAL_MS,
                                          AGILE_LIVE_RETUNE_MAX_COMMAND_INTERVAL_MS);
    agileLiveRetuneIntervalSpin->setSuffix(QStringLiteral(" ms"));
    agileLiveRetuneIntervalSpin->setSingleStep(20);
    agileLiveRetuneIntervalSpin->setValue((std::clamp)(agileLiveRetuneCommandIntervalMs,
                                                       AGILE_LIVE_RETUNE_MIN_COMMAND_INTERVAL_MS,
                                                       AGILE_LIVE_RETUNE_MAX_COMMAND_INTERVAL_MS));
    agileLiveRetuneIntervalSpin->setToolTip(uiText(
        QStringLiteral("agile_live_retune_interval_tooltip"),
        QStringLiteral("Minimum interval between live Agile center-frequency commands. Lower values make tuning more responsive but can overload USB and UI.")));

    QPushButton *helpButton = new QPushButton(
        uiText(QStringLiteral("program_help"), QStringLiteral("Feature guide...")),
        &dialog);
    helpButton->setToolTip(uiText(
        QStringLiteral("program_help_tooltip"),
        QStringLiteral("Open a practical guide to FobosAPP controls, scanning, recordings, GNSS/QTH, network mode and mouse shortcuts.")));

    QPushButton *fobosDetailsButton = new QPushButton(
        uiText(QStringLiteral("show_fobos_details"), QStringLiteral("Show Fobos Details")),
        &dialog);
    fobosDetailsButton->setToolTip(uiText(
        QStringLiteral("show_fobos_details_tooltip"),
        QStringLiteral("Show Standard and Agile Fobos API versions and the currently detected receiver list.")));

    generalLayout->addRow(uiText(QStringLiteral("language"), QStringLiteral("Lang:")), languageCombo);
    generalLayout->addRow(uiText(QStringLiteral("fine_tune"), QStringLiteral("Fine tune")), fineTuneModeCombo);
    generalLayout->addRow(uiText(QStringLiteral("spectrum_update_interval"), QStringLiteral("Spectrum/waterfall update")), spectrumUpdateSpin);
    generalLayout->addRow(uiText(QStringLiteral("waterfall_speed"), QStringLiteral("Waterfall speed")), waterfallRowsSpin);
    generalLayout->addRow(uiText(QStringLiteral("scan_measurement_update_interval"), QStringLiteral("Measurement accumulation")), scanMeasurementUpdateSpin);
    generalLayout->addRow(uiText(QStringLiteral("agile_live_retune_interval"), QStringLiteral("Agile live retune interval")), agileLiveRetuneIntervalSpin);
    generalLayout->addRow(helpButton);
    generalLayout->addRow(fobosDetailsButton);
    rootLayout->addLayout(generalLayout);

    QGroupBox *settingsBackupBox = new QGroupBox(
        uiText(QStringLiteral("settings_backup"), QStringLiteral("Settings backup")),
        &dialog);
    QVBoxLayout *settingsBackupLayout = new QVBoxLayout(settingsBackupBox);
    QLabel *settingsBackupHint = new QLabel(
        uiText(QStringLiteral("settings_backup_hint"),
               QStringLiteral("Export FobosAPP.ini before updating the app to keep custom presets, scan lists, map markers and UI settings.")),
        settingsBackupBox);
    settingsBackupHint->setWordWrap(true);
    QHBoxLayout *settingsBackupButtons = new QHBoxLayout();
    QPushButton *exportSettingsButton = new QPushButton(
        uiText(QStringLiteral("export_settings"), QStringLiteral("Export settings...")),
        settingsBackupBox);
    QPushButton *importSettingsButton = new QPushButton(
        uiText(QStringLiteral("import_settings"), QStringLiteral("Import settings...")),
        settingsBackupBox);
    settingsBackupButtons->addWidget(exportSettingsButton);
    settingsBackupButtons->addWidget(importSettingsButton);
    settingsBackupButtons->addStretch(1);
    settingsBackupLayout->addWidget(settingsBackupHint);
    settingsBackupLayout->addLayout(settingsBackupButtons);
    rootLayout->addWidget(settingsBackupBox);

    QGroupBox *quickOptionsBox = new QGroupBox(uiText(QStringLiteral("quick_options"), QStringLiteral("Quick options")), &dialog);
    QGridLayout *quickOptionsLayout = new QGridLayout(quickOptionsBox);
    QCheckBox *audioOption = new QCheckBox(uiText(QStringLiteral("audio"), QStringLiteral("Audio")), quickOptionsBox);
    QCheckBox *syncOption = new QCheckBox(uiText(QStringLiteral("sync"), QStringLiteral("Sync")), quickOptionsBox);
    QCheckBox *spectrum2Option = new QCheckBox(uiText(QStringLiteral("spectrum2"), QStringLiteral("Spectr 2")), quickOptionsBox);
    QCheckBox *colorOption = new QCheckBox(uiText(QStringLiteral("colorful"), QStringLiteral("Color spectrum")), quickOptionsBox);
    QCheckBox *generalBandMarkersOption = new QCheckBox(uiText(QStringLiteral("general_band_markers"), QStringLiteral("Band markers")), quickOptionsBox);
    QCheckBox *amateurBandMarkersOption = new QCheckBox(uiText(QStringLiteral("amateur_band_markers"), QStringLiteral("HAM bands")), quickOptionsBox);
    QCheckBox *compactBandMarkersOption = new QCheckBox(uiText(QStringLiteral("compact_band_markers"), QStringLiteral("Collapsed bands")), quickOptionsBox);
    QCheckBox *gpuWaterfallOption = new QCheckBox(uiText(QStringLiteral("gpu_waterfall"), QStringLiteral("GPU waterfall")), quickOptionsBox);
    gpuWaterfallOption->setToolTip(uiText(
        QStringLiteral("gpu_waterfall_tooltip"),
        QStringLiteral("Experimental: prepare the waterfall for GPU-backed rendering. CPU texture rendering remains the safe fallback.")));
    QCheckBox *gnssUbxAutoEnableOption = new QCheckBox(uiText(QStringLiteral("gnss_ubx_auto_enable"),
                                                              QStringLiteral("Auto-enable UBX")),
                                                       quickOptionsBox);
    gnssUbxAutoEnableOption->setToolTip(uiText(
        QStringLiteral("gnss_ubx_auto_enable_tooltip"),
        QStringLiteral("After opening a GNSS serial port, automatically request u-blox NAV-PVT, NAV-SAT and NAV-DOP output.")));
    QCheckBox *loggingOption = new QCheckBox(uiText(QStringLiteral("logging"), QString::fromUtf8("Р›РѕРіСѓРІР°РЅРЅСЏ")), quickOptionsBox);
    loggingOption->setToolTip(uiText(QStringLiteral("logging_tooltip"),
                                     QStringLiteral("Write detailed diagnostic logs and DMR dumps")));
    audioOption->setChecked(audioCheckbox && audioCheckbox->isChecked());
    syncOption->setChecked(syncCheckbox && syncCheckbox->isChecked());
    syncOption->setEnabled(false);
    syncOption->setToolTip(syncCheckbox ? syncCheckbox->toolTip() : QString());
    spectrum2Option->setChecked(graphCheckbox && graphCheckbox->isChecked());
    colorOption->setChecked(colorCheckbox && colorCheckbox->isChecked());
    generalBandMarkersOption->setChecked(showGeneralBandMarkers);
    amateurBandMarkersOption->setChecked(showAmateurBandMarkers);
    compactBandMarkersOption->setChecked(compactBandMarkers);
    gpuWaterfallOption->setChecked(experimentalGpuWaterfall);
    gnssUbxAutoEnableOption->setChecked(gnssUbxAutoEnable);
    loggingOption->setChecked(diagnosticVerboseLogging);
    quickOptionsLayout->addWidget(audioOption, 0, 0);
    quickOptionsLayout->addWidget(syncOption, 0, 1);
    quickOptionsLayout->addWidget(spectrum2Option, 1, 0);
    quickOptionsLayout->addWidget(colorOption, 1, 1);
    quickOptionsLayout->addWidget(generalBandMarkersOption, 2, 0);
    quickOptionsLayout->addWidget(amateurBandMarkersOption, 2, 1);
    quickOptionsLayout->addWidget(compactBandMarkersOption, 2, 2);
    quickOptionsLayout->addWidget(gpuWaterfallOption, 3, 0);
    quickOptionsLayout->addWidget(gnssUbxAutoEnableOption, 3, 1);
    quickOptionsLayout->addWidget(loggingOption, 3, 2);
    rootLayout->addWidget(quickOptionsBox);

    auto applyLanguage = [this, languageCombo]() {
        const QString nextLanguage = languageCombo->currentData().toString();
        uiLanguage = normalizedUiLanguage(nextLanguage);
        applyUiLanguage();
        savePersistentSettings();
    };
    auto applyFineTuneMode = [this, fineTuneModeCombo]() {
        fineTuneControlMode = fineTuneModeCombo->currentData().toInt();
        if (fineTuneControlMode != FINE_TUNE_MODE_DIAL) {
            fineTuneControlMode = FINE_TUNE_MODE_SCALE;
        }
        updateFineTuneControlMode();
        savePersistentSettings();
    };
    auto applySpectrumUpdateInterval = [this, spectrumUpdateSpin]() {
        int value = spectrumUpdateSpin->value();
        if (value > SPECTRUM_UPDATE_AUTO_MS && value < SPECTRUM_UPDATE_MIN_MS) {
            value = SPECTRUM_UPDATE_MIN_MS;
            QSignalBlocker blocker(spectrumUpdateSpin);
            spectrumUpdateSpin->setValue(value);
        }
        spectrumUpdateIntervalMs = value;
        updateSpectrumTimerInterval();
        savePersistentSettings();
    };
    auto applyWaterfallRowsPerFrame = [this, waterfallRowsSpin]() {
        waterfallRowsPerFrame = (std::clamp)(waterfallRowsSpin->value(),
                                             WATERFALL_ROWS_PER_FRAME_MIN,
                                             WATERFALL_ROWS_PER_FRAME_MAX);
        if (waterfallWidget) {
            waterfallWidget->setRowsPerFrame(waterfallRowsPerFrame);
        }
        savePersistentSettings();
    };
    auto applyScanMeasurementUpdateInterval = [this, scanMeasurementUpdateSpin]() {
        scanMeasurementUpdateIntervalMs =
            (std::clamp)(scanMeasurementUpdateSpin->value(),
                         SCAN_MEASUREMENT_MIN_UPDATE_MS,
                         SCAN_MEASUREMENT_MAX_UPDATE_MS);
        scanMeasurementUpdateClock.invalidate();
        scanMeasurementOverlayClock.invalidate();
        scanMeasurementOverlayCache.clear();
        savePersistentSettings();
    };
    auto applyAgileLiveRetuneInterval = [this, agileLiveRetuneIntervalSpin]() {
        agileLiveRetuneCommandIntervalMs =
            (std::clamp)(agileLiveRetuneIntervalSpin->value(),
                         AGILE_LIVE_RETUNE_MIN_COMMAND_INTERVAL_MS,
                         AGILE_LIVE_RETUNE_MAX_COMMAND_INTERVAL_MS);
        savePersistentSettings();
        qDebug() << "[LiveTune] Agile live retune command interval"
                 << agileLiveRetuneCommandIntervalMs;
    };

    connect(languageCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), &dialog, [applyLanguage](int) {
        applyLanguage();
    });
    connect(fineTuneModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), &dialog, [applyFineTuneMode](int) {
        applyFineTuneMode();
    });
    connect(spectrumUpdateSpin, QOverload<int>::of(&QSpinBox::valueChanged), &dialog, [applySpectrumUpdateInterval](int) {
        applySpectrumUpdateInterval();
    });
    connect(waterfallRowsSpin, QOverload<int>::of(&QSpinBox::valueChanged), &dialog, [applyWaterfallRowsPerFrame](int) {
        applyWaterfallRowsPerFrame();
    });
    connect(scanMeasurementUpdateSpin, QOverload<int>::of(&QSpinBox::valueChanged), &dialog, [applyScanMeasurementUpdateInterval](int) {
        applyScanMeasurementUpdateInterval();
    });
    connect(agileLiveRetuneIntervalSpin, QOverload<int>::of(&QSpinBox::valueChanged), &dialog, [applyAgileLiveRetuneInterval](int) {
        applyAgileLiveRetuneInterval();
    });
    connect(helpButton, &QPushButton::clicked, &dialog, [this]() {
        openApplicationHelp();
    });
    connect(fobosDetailsButton, &QPushButton::clicked, &dialog, [this]() {
        listFobosDevices();
    });
    connect(exportSettingsButton, &QPushButton::clicked, &dialog, [this]() {
        exportSettingsBackup();
    });
    connect(importSettingsButton, &QPushButton::clicked, &dialog, [this]() {
        importSettingsBackup();
    });
    connect(audioOption, &QCheckBox::toggled, &dialog, [this](bool checked) {
        if (audioCheckbox) {
            audioCheckbox->setChecked(checked);
        }
        savePersistentSettings();
    });
    connect(spectrum2Option, &QCheckBox::toggled, &dialog, [this](bool checked) {
        if (graphCheckbox) {
            graphCheckbox->setChecked(checked);
        }
        savePersistentSettings();
    });
    connect(colorOption, &QCheckBox::toggled, &dialog, [this](bool checked) {
        if (colorCheckbox) {
            colorCheckbox->setChecked(checked);
        }
        savePersistentSettings();
    });
    connect(generalBandMarkersOption, &QCheckBox::toggled, &dialog, [this](bool checked) {
        showGeneralBandMarkers = checked;
        updateGraphBandMarkers();
        savePersistentSettings();
    });
    connect(amateurBandMarkersOption, &QCheckBox::toggled, &dialog, [this](bool checked) {
        showAmateurBandMarkers = checked;
        updateGraphBandMarkers();
        savePersistentSettings();
    });
    connect(compactBandMarkersOption, &QCheckBox::toggled, &dialog, [this](bool checked) {
        compactBandMarkers = checked;
        updateGraphBandMarkers();
        savePersistentSettings();
    });
    connect(gpuWaterfallOption, &QCheckBox::toggled, &dialog, [this](bool checked) {
        experimentalGpuWaterfall = checked;
        if (waterfallWidget) {
            waterfallWidget->setRenderBackend(checked
                                                  ? MyWaterfallWidget::RenderBackend::GpuPrepared
                                                  : MyWaterfallWidget::RenderBackend::CpuTexture);
        }
        savePersistentSettings();
    });
    connect(gnssUbxAutoEnableOption, &QCheckBox::toggled, &dialog, [this](bool checked) {
        gnssUbxAutoEnable = checked;
        savePersistentSettings();
    });
    connect(loggingOption, &QCheckBox::toggled, &dialog, [this](bool checked) {
        diagnosticVerboseLogging = checked;
        setFobosVerboseLoggingEnabled(checked);
        qDebug() << "[Log] Verbose diagnostic logging"
                 << (checked ? "enabled" : "disabled");
        savePersistentSettings();
    });

    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);
    if (QPushButton *closeButton = buttonBox->button(QDialogButtonBox::Close)) {
        closeButton->setText(uiText(QStringLiteral("close"), QStringLiteral("Close")));
    }
    rootLayout->addWidget(buttonBox);

    connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    dialog.exec();
}

void YourClassName::exportSettingsBackup() {
    savePersistentSettings();

    const QString sourcePath = persistentSettingsFilePath();
    QFileInfo sourceInfo(sourcePath);
    if (!sourceInfo.exists()) {
        QMessageBox::warning(this,
                             uiText(QStringLiteral("settings_backup"), QStringLiteral("Settings backup")),
                             uiText(QStringLiteral("settings_export_failed"),
                                    QStringLiteral("Settings export failed: %1"))
                                 .arg(sourcePath));
        return;
    }

    const QString defaultName =
        QStringLiteral("FobosAPP-settings-%1.ini")
            .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd-HHmmss")));
    const QString defaultPath = QDir(sourceInfo.absolutePath()).absoluteFilePath(defaultName);
    const QString targetPath = QFileDialog::getSaveFileName(
        this,
        uiText(QStringLiteral("export_settings"), QStringLiteral("Export settings...")),
        defaultPath,
        uiText(QStringLiteral("settings_ini_filter"), QStringLiteral("INI settings (*.ini);;All files (*.*)")));
    if (targetPath.isEmpty()) {
        return;
    }

    QFile::remove(targetPath);
    if (!QFile::copy(sourcePath, targetPath)) {
        QMessageBox::warning(this,
                             uiText(QStringLiteral("settings_backup"), QStringLiteral("Settings backup")),
                             uiText(QStringLiteral("settings_export_failed"),
                                    QStringLiteral("Settings export failed: %1"))
                                 .arg(targetPath));
        return;
    }

    QMessageBox::information(this,
                             uiText(QStringLiteral("settings_backup"), QStringLiteral("Settings backup")),
                             uiText(QStringLiteral("settings_export_done"),
                                    QStringLiteral("Settings exported: %1"))
                                 .arg(QDir::toNativeSeparators(targetPath)));
}

void YourClassName::importSettingsBackup() {
    const QString currentPath = persistentSettingsFilePath();
    const QString sourcePath = QFileDialog::getOpenFileName(
        this,
        uiText(QStringLiteral("import_settings"), QStringLiteral("Import settings...")),
        QFileInfo(currentPath).absolutePath(),
        uiText(QStringLiteral("settings_ini_filter"), QStringLiteral("INI settings (*.ini);;All files (*.*)")));
    if (sourcePath.isEmpty()) {
        return;
    }

    if (QFileInfo(sourcePath).canonicalFilePath() == QFileInfo(currentPath).canonicalFilePath()) {
        QMessageBox::information(this,
                                 uiText(QStringLiteral("settings_backup"), QStringLiteral("Settings backup")),
                                 uiText(QStringLiteral("settings_import_same_file"),
                                        QStringLiteral("Selected file is already the active settings file.")));
        return;
    }

    const QMessageBox::StandardButton answer =
        QMessageBox::question(this,
                              uiText(QStringLiteral("settings_import_confirm_title"),
                                     QStringLiteral("Import settings?")),
                              uiText(QStringLiteral("settings_import_confirm"),
                                     QStringLiteral("Importing settings will replace the current FobosAPP.ini. A timestamped backup of the current file will be created first.")),
                              QMessageBox::Yes | QMessageBox::No,
                              QMessageBox::No);
    if (answer != QMessageBox::Yes) {
        return;
    }

    savePersistentSettings();

    const QFileInfo currentInfo(currentPath);
    if (!currentInfo.absoluteDir().exists()) {
        currentInfo.absoluteDir().mkpath(QStringLiteral("."));
    }

    QString backupPath;
    if (QFileInfo::exists(currentPath)) {
        backupPath = QDir(currentInfo.absolutePath()).absoluteFilePath(
            QStringLiteral("FobosAPP-settings-before-import-%1.ini")
                .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd-HHmmss"))));
        if (!QFile::copy(currentPath, backupPath)) {
            QMessageBox::warning(this,
                                 uiText(QStringLiteral("settings_backup"), QStringLiteral("Settings backup")),
                                 uiText(QStringLiteral("settings_backup_failed"),
                                        QStringLiteral("Could not create current settings backup: %1"))
                                     .arg(backupPath));
            return;
        }
    }

    QFile::remove(currentPath);
    if (!QFile::copy(sourcePath, currentPath)) {
        if (!backupPath.isEmpty()) {
            QFile::copy(backupPath, currentPath);
        }
        QMessageBox::warning(this,
                             uiText(QStringLiteral("settings_backup"), QStringLiteral("Settings backup")),
                             uiText(QStringLiteral("settings_import_failed"),
                                    QStringLiteral("Settings import failed: %1"))
                                 .arg(sourcePath));
        return;
    }

    loadPersistentSettings();
    publishSettingsToGlobals();
    updateUiFromPendingSettings();
    settingRange();
    updateGraphBandMarkers();
    updateFrequencyPresetControls();
    updateQthControls();

    QString message = uiText(QStringLiteral("settings_import_done"),
                             QStringLiteral("Settings imported. Backup created: %1"))
                          .arg(backupPath.isEmpty()
                                   ? uiText(QStringLiteral("none"), QStringLiteral("none"))
                                   : QDir::toNativeSeparators(backupPath));
    if (!isIdle()) {
        message += QStringLiteral("\n\n");
        message += uiText(QStringLiteral("settings_import_restart_hint"),
                          QStringLiteral("Some receiver settings are applied fully after Stop/Start or app restart."));
    }
    QMessageBox::information(this,
                             uiText(QStringLiteral("settings_backup"), QStringLiteral("Settings backup")),
                             message);
}
