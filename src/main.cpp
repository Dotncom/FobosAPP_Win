#include "main.h"
#include "iqbuffer.h"
#include "diagnosticlogging.h"

#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFile>
#include <QFormLayout>
#include <QJsonArray>
#include <QJsonObject>
#include <QMessageLogContext>
#include <QMutexLocker>
#include <QSpinBox>
#include <QTextStream>
#include <limits>
#include <psapi.h>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <new>

#ifdef _MSC_VER
#pragma comment(lib, "psapi.lib")
#endif

fobos_dev_t* device = nullptr;
float* iqData = nullptr; 
float* dataq = nullptr;
//size_t dataqSize = DEFAULT_BUF_LEN/8;
double globalFrequency = 100000000; 
double actualFrequency = 100000000; 
double listeningFrequency = 100000000; 
double globalSampleRate = 80000000;
double globalBandwidth = 10000;
double minFrequency = 60000000;
double maxFrequency = 140000000;
int globalModulationType = 0;
int globalMode = 0;
std::vector<float> fftMagnitudes;
std::vector<float> fftFrequencies;
int fftLength = 32768;
int DEFAULT_BUF_LEN = 32768;
double currentScale = 100.0;
bool secondGraph = false;
bool syncWariable = false;
float sensitivity = 10;
float contrast = 10;
bool colorf = false;
int deviceID = 0;

namespace {

constexpr double RF_MIN_CENTER_FREQUENCY = 50000000.0;
constexpr double RF_MIN_LISTENING_FREQUENCY = 25000000.0;
constexpr double DIRECT_MIN_FREQUENCY = 1.0;
constexpr int SCALE_SLIDER_FACTOR = 10;
constexpr double MIN_SCALE_PERCENT = 0.1;
constexpr double MAX_SCALE_PERCENT = 100.0;
constexpr int LEVEL_SLIDER_FACTOR = 10;
constexpr int MIN_LEVEL_SLIDER_VALUE = -1600;
constexpr int MAX_LEVEL_SLIDER_VALUE = 200;
constexpr float MIN_LEVEL_GAP = 0.1f;
constexpr int NETWORK_SPECTRUM_MAX_BINS = 2048;
constexpr int NETWORK_SPECTRUM_INTERVAL_MS = 100;
constexpr qint64 NETWORK_IQ_MAX_PENDING_BYTES = 8 * 1024 * 1024;
constexpr uint64_t NETWORK_IQ_DROP_LOG_INTERVAL = 200;
constexpr double NETWORK_AUDIO_PREBUFFER_SECONDS = 0.55;
constexpr qint64 NETWORK_SPECTRUM_MAX_PENDING_BYTES = 512 * 1024;

QMutex gLogMutex;
QFile gLogFile;

const char *messageTypeName(QtMsgType type) {
    switch (type) {
    case QtDebugMsg:
        return "DEBUG";
    case QtInfoMsg:
        return "INFO";
    case QtWarningMsg:
        return "WARN";
    case QtCriticalMsg:
        return "CRITICAL";
    case QtFatalMsg:
        return "FATAL";
    }
    return "LOG";
}

const char *runStateName(RadioRunState state) {
    switch (state) {
    case RadioRunState::Idle:
        return "Idle";
    case RadioRunState::Starting:
        return "Starting";
    case RadioRunState::Running:
        return "Running";
    case RadioRunState::Stopping:
        return "Stopping";
    }
    return "Unknown";
}

double directMaxFrequency(double sampleRate) {
    return (std::max)(DIRECT_MIN_FREQUENCY, sampleRate / 2.0 - 1.0);
}

void normalizeTuning(RadioSettings &settings, bool preserveCenter = false) {
    if (settings.sampleRate <= 0.0) {
        return;
    }

    if (settings.inputMode == 0) {
        const double halfRate = settings.sampleRate / 2.0;
        settings.centerFrequency = (std::max)(RF_MIN_CENTER_FREQUENCY, settings.centerFrequency);
        settings.listeningFrequency = (std::max)(RF_MIN_LISTENING_FREQUENCY, settings.listeningFrequency);

        if (!preserveCenter && settings.listeningFrequency < settings.centerFrequency - halfRate) {
            settings.centerFrequency = (std::max)(RF_MIN_CENTER_FREQUENCY,
                                                  settings.listeningFrequency + halfRate);
        } else if (!preserveCenter && settings.listeningFrequency > settings.centerFrequency + halfRate) {
            settings.centerFrequency = (std::max)(RF_MIN_CENTER_FREQUENCY,
                                                  settings.listeningFrequency - halfRate);
        }

        const double low = (std::max)(RF_MIN_LISTENING_FREQUENCY,
                                      settings.centerFrequency - halfRate);
        const double high = (std::max)(low, settings.centerFrequency + halfRate);
        settings.listeningFrequency = (std::clamp)(settings.listeningFrequency, low, high);
    } else {
        settings.centerFrequency = 0.0;
        settings.actualFrequency = 0.0;
        settings.listeningFrequency = (std::clamp)(settings.listeningFrequency,
                                                   DIRECT_MIN_FREQUENCY,
                                                   directMaxFrequency(settings.sampleRate));
    }
}

int scalePercentToSliderValue(double scalePercent) {
    const double clamped = (std::clamp)(scalePercent, MIN_SCALE_PERCENT, MAX_SCALE_PERCENT);
    return static_cast<int>(std::lround(clamped * SCALE_SLIDER_FACTOR));
}

double sliderValueToScalePercent(int sliderValue) {
    const int clamped = (std::clamp)(sliderValue,
                                     scalePercentToSliderValue(MIN_SCALE_PERCENT),
                                     scalePercentToSliderValue(MAX_SCALE_PERCENT));
    return clamped / static_cast<double>(SCALE_SLIDER_FACTOR);
}

QString formatScalePercent(double scalePercent) {
    if (std::abs(scalePercent - std::round(scalePercent)) < 0.05) {
        return QString::number(scalePercent, 'f', 0);
    }
    return QString::number(scalePercent, 'f', 1);
}

QString scaleLabelText(double scalePercent) {
    return QString("Scale: %1").arg(formatScalePercent(scalePercent));
}

float sliderValueToLevel(int sliderValue) {
    const int clamped = (std::clamp)(sliderValue, MIN_LEVEL_SLIDER_VALUE, MAX_LEVEL_SLIDER_VALUE);
    return clamped / static_cast<float>(LEVEL_SLIDER_FACTOR);
}

int levelToSliderValue(float level) {
    const int value = static_cast<int>(std::lround(level * LEVEL_SLIDER_FACTOR));
    return (std::clamp)(value, MIN_LEVEL_SLIDER_VALUE, MAX_LEVEL_SLIDER_VALUE);
}

QString levelLabelText(const QString &name, float level) {
    return QString("%1: %2").arg(name, QString::number(level, 'f', 1));
}

double defaultBandwidthForModulation(int modulationType) {
    switch (modulationType) {
    case MOD_NFM:
        return 12500.0;
    case MOD_SAM:
        return 6000.0;
    case MOD_USB:
    case MOD_LSB:
        return 2700.0;
    case MOD_DSB:
        return 6000.0;
    case MOD_CW:
        return 500.0;
    case MOD_WFM:
        return 200000.0;
    case MOD_FT8:
    case MOD_RTTY:
    case MOD_FSK:
    case MOD_PSK:
        return 3000.0;
    case MOD_AM:
    default:
        return 10000.0;
    }
}

QString formatBandwidthHz(double bandwidth) {
    return QString::number(bandwidth, 'f', 0);
}

void diagnosticMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &message) {
    const QString line = QString("%1 [%2] [tid 0x%3] %4%5")
                             .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss.zzz"))
                             .arg(messageTypeName(type))
                             .arg(reinterpret_cast<quintptr>(QThread::currentThreadId()), 0, 16)
                             .arg(message)
                             .arg(context.file ? QString(" (%1:%2)").arg(context.file).arg(context.line) : QString());

    {
        QMutexLocker lock(&gLogMutex);
        if (gLogFile.isOpen()) {
            QTextStream stream(&gLogFile);
            stream << line << '\n';
            stream.flush();
            gLogFile.flush();
        }
    }

    const QByteArray localLine = line.toLocal8Bit();
    std::fprintf(stderr, "%s\n", localLine.constData());
    std::fflush(stderr);

#ifdef _WIN32
    const std::wstring debugLine = (line + "\n").toStdWString();
    OutputDebugStringW(debugLine.c_str());
#endif

    if (type == QtFatalMsg) {
        std::abort();
    }
}

void installDiagnosticLogger() {
    const QString logPath = QDir(QCoreApplication::applicationDirPath()).filePath("FobosAPP_diagnostic.log");
    gLogFile.setFileName(logPath);
    gLogFile.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append);
    qInstallMessageHandler(diagnosticMessageHandler);
    qDebug() << "[Log] ===== Diagnostic session started =====";
    qDebug() << "[Log] Diagnostic log path:" << QDir::toNativeSeparators(logPath)
             << "fileOpen" << gLogFile.isOpen();
    qDebug() << "[Log] Verbose diagnostic logging"
             << (fobosVerboseLoggingEnabled() ? "enabled" : "disabled");
}

QString modulePathForAddress(void *address) {
    if (!address) {
        return QString();
    }

    MEMORY_BASIC_INFORMATION memoryInfo;
    ZeroMemory(&memoryInfo, sizeof(memoryInfo));
    if (!VirtualQuery(address, &memoryInfo, sizeof(memoryInfo)) || !memoryInfo.AllocationBase) {
        return QString();
    }

    wchar_t modulePath[MAX_PATH] = {};
    const DWORD length = GetModuleFileNameW(static_cast<HMODULE>(memoryInfo.AllocationBase),
                                            modulePath,
                                            MAX_PATH);
    if (length == 0) {
        return QString();
    }
    return QString::fromWCharArray(modulePath, static_cast<int>(length));
}

LONG WINAPI diagnosticUnhandledExceptionFilter(EXCEPTION_POINTERS *exceptionInfo) {
    if (!exceptionInfo || !exceptionInfo->ExceptionRecord) {
        qCritical() << "[Crash] unhandled Windows exception without exception record";
        return EXCEPTION_EXECUTE_HANDLER;
    }

    EXCEPTION_RECORD *record = exceptionInfo->ExceptionRecord;
    void *address = record->ExceptionAddress;
    const QString modulePath = modulePathForAddress(address);
    qCritical() << "[Crash] unhandled Windows exception"
                << "code" << QString("0x%1").arg(static_cast<quint32>(record->ExceptionCode), 8, 16, QChar('0'))
                << "address" << address
                << "module" << (modulePath.isEmpty() ? QString("unknown") : QDir::toNativeSeparators(modulePath))
                << "parameters" << static_cast<quint32>(record->NumberParameters);
    return EXCEPTION_EXECUTE_HANDLER;
}

void diagnosticTerminateHandler() {
    qCritical() << "[Crash] std::terminate called";
    std::abort();
}

void installCrashLogger() {
    SetUnhandledExceptionFilter(diagnosticUnhandledExceptionFilter);
    std::set_terminate(diagnosticTerminateHandler);
    qDebug() << "[Log] Crash logger installed";
}

void logFobosApiInfo() {
    char libVersion[256] = {};
    char driverVersion[256] = {};
    const int result = fobos_rx_get_api_info(libVersion, driverVersion);
    if (result == FOBOS_ERR_OK) {
        qDebug() << "[FobosLifecycle] libfobos info"
                 << "library" << libVersion
                 << "driver" << driverVersion;
    } else {
        qDebug() << "[FobosLifecycle] libfobos info unavailable"
                 << "result" << result;
    }
}

int closeFobosDeviceSafely(fobos_dev_t *dev) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_close(dev);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_close(dev);
#endif
}

int setFobosClockSourceSafely(fobos_dev_t *dev, unsigned int value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_set_clk_source(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_set_clk_source(dev, value);
#endif
}

int setFobosDirectSamplingSafely(fobos_dev_t *dev, unsigned int enabled) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_set_direct_sampling(dev, enabled);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_set_direct_sampling(dev, enabled);
#endif
}

int setFobosSampleRateSafely(fobos_dev_t *dev, double value, double *actual) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_set_samplerate(dev, value, actual);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_set_samplerate(dev, value, actual);
#endif
}

int setFobosFrequencySafely(fobos_dev_t *dev, double value, double *actual) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_set_frequency(dev, value, actual);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_set_frequency(dev, value, actual);
#endif
}

int setFobosLnaGainSafely(fobos_dev_t *dev, unsigned int value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_set_lna_gain(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_set_lna_gain(dev, value);
#endif
}

int setFobosVgaGainSafely(fobos_dev_t *dev, unsigned int value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_set_vga_gain(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_set_vga_gain(dev, value);
#endif
}

int setFobosGpoSafely(fobos_dev_t *dev, uint8_t value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_set_user_gpo(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_set_user_gpo(dev, value);
#endif
}

void logMemorySnapshot(const char *tag) {
    if (!fobosVerboseLoggingEnabled()) {
        return;
    }

    PROCESS_MEMORY_COUNTERS_EX counters;
    ZeroMemory(&counters, sizeof(counters));
    counters.cb = sizeof(counters);

    MEMORYSTATUSEX memoryStatus;
    ZeroMemory(&memoryStatus, sizeof(memoryStatus));
    memoryStatus.dwLength = sizeof(memoryStatus);

    const bool processOk = GetProcessMemoryInfo(GetCurrentProcess(),
                                                reinterpret_cast<PROCESS_MEMORY_COUNTERS*>(&counters),
                                                sizeof(counters)) != 0;
    const bool systemOk = GlobalMemoryStatusEx(&memoryStatus) != 0;
    if (!processOk && !systemOk) {
        qDebug() << "[Memory]" << tag << "unavailable";
        return;
    }

    const auto toMb = [](quint64 bytes) {
        return static_cast<double>(bytes) / (1024.0 * 1024.0);
    };

    qDebug() << "[Memory]" << tag
             << "workingSetMB" << (processOk ? toMb(counters.WorkingSetSize) : -1.0)
             << "privateMB" << (processOk ? toMb(counters.PrivateUsage) : -1.0)
             << "availPhysMB" << (systemOk ? toMb(memoryStatus.ullAvailPhys) : -1.0)
             << "memoryLoadPct" << (systemOk ? static_cast<int>(memoryStatus.dwMemoryLoad) : -1);
}

} // namespace

QString formatSampleRate(double sampleRate);

YourClassName::YourClassName(QWidget *parent) 
    : QMainWindow(parent), deviceOpened(false)
    {

    resize(1920, 1000);
    setMinimumSize(1180, 720);

    QStringList devices = getFobosDevices();

    QWidget *centralWidget = new QWidget(this);
    QScrollArea *scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);
    scrollArea->setWidget(centralWidget);
    setCentralWidget(scrollArea);
    QHBoxLayout *scaleLayout = new QHBoxLayout();
    QVBoxLayout *contrastLayout = new QVBoxLayout();
    QVBoxLayout *sensLayout = new QVBoxLayout();
    QVBoxLayout *levelMinLayout = new QVBoxLayout();
    QVBoxLayout *levelMaxLayout = new QVBoxLayout();
     QHBoxLayout *hboxLayout = new QHBoxLayout();
    QVBoxLayout *layout = new QVBoxLayout();
    QGridLayout *checkboxLayout = new QGridLayout();
     QVBoxLayout *graphLayout = new QVBoxLayout();
    
    for (int i = 0; i < 8; ++i) {
        checkBoxes[i] = new QCheckBox(QString("GPIO %1").arg(i + 1), this);
        checkboxLayout->addWidget(checkBoxes[i], i / 4, i % 4);
        connect(checkBoxes[i], &QCheckBox::stateChanged, this, &YourClassName::onCheckboxStateChanged);
    }
    fftComboBox = new QComboBox(this);
    fftComboBox->addItem("2048");
    fftComboBox->addItem("4096");
    fftComboBox->addItem("8192");
    fftComboBox->addItem("16384");
    fftComboBox->addItem("32768");
    fftComboBox->addItem("65536");
    fftComboBox->addItem("131072");
    fftComboBox->addItem("262144");
    fftComboBox->addItem("524288");
    fftComboBox->setCurrentIndex(4);
    
    lnaGainSlider = new QSlider(Qt::Horizontal, this);
    lnaGainSlider->setRange(1, 3);
    lnaGainSlider->setValue(1);
    
    vgaGainSlider = new QSlider(Qt::Horizontal, this);
    vgaGainSlider->setRange(0, 15);
    vgaGainSlider->setValue(3);
    
    lnaGainLabel = new QLabel("LNA Gain: 1", this);
    vgaGainLabel = new QLabel("VGA Gain: 3", this);
    
    scaleSlider = new QSlider(Qt::Horizontal, this); 
    scaleSlider->setRange(scalePercentToSliderValue(MIN_SCALE_PERCENT),
                          scalePercentToSliderValue(MAX_SCALE_PERCENT));
    scaleSlider->setSingleStep(1);
    scaleSlider->setPageStep(10);
    scaleSlider->setValue(scalePercentToSliderValue(currentScale));
    scaleLabel = new QLabel(scaleLabelText(currentScale), this);
    
    audioDeviceComboBox = new QComboBox(this);
    comboBox = new QComboBox(this);
    modeBox = new QComboBox(this);
    sampleBox = new QComboBox(this);
    clkBox = new QComboBox(this);
    
    audioCheckbox = new QCheckBox("Audio", this);
    syncCheckbox = new QCheckBox("Sync", this);
    syncCheckbox->setChecked(false);
    syncCheckbox->setEnabled(false);
    syncCheckbox->setToolTip("Async reader is forced for continuous streaming tests.");
    graphCheckbox = new QCheckBox("Spectr 2", this);
    colorCheckbox = new QCheckBox("Colorful", this);

    QHBoxLayout* chckbox = new QHBoxLayout();
    chckbox->addWidget(audioCheckbox);
    chckbox->addWidget(syncCheckbox);
    chckbox->addWidget(graphCheckbox);
    chckbox->addWidget(colorCheckbox);

    comboBox->addItems(getFobosDevices());
    modeBox->addItem("RF", 0);
    modeBox->addItem("HF1 + HF2", 1);
    modeBox->addItem("HF1", 2);
    modeBox->addItem("HF2", 3);
    
    clkBox->addItem("Internal", 0);
    clkBox->addItem("External", 1);

    processor = new DataProcessor( this);
    audioProcessor = new AudioProcessor(this);
    networkController = new NetworkController(this);
    remoteAudioPlayer = new RemoteAudioPlayer(this);
    connectDataProcessorSignals();
    
    graphWidget = new MyGraphWidget(this);
    graphWidget->setMinimumSize(760, 180);
    graphWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    
    scaleWidget = new ScaleWidget(this);
    scaleWidget->setMinimumWidth(760);
    scaleWidget->setFixedHeight(50);
    scaleWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    
    waterfallWidget = new MyWaterfallWidget(this);
    waterfallWidget->setMinimumSize(760, 300);
    waterfallWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    
    refreshButton = new QPushButton("Refresh USB Devices", this);
    fobosButton = new QPushButton("Show Fobos Details", this);
    networkButton = new QPushButton("Network", this);
    startButton = new QPushButton("Start", this);
    stopButton = new QPushButton("Stop", this);
    
    contrastSlider = new QSlider(Qt::Horizontal, this);
    contrastSlider->setRange(1, 15);
    contrastSlider->setValue(10);
    contrastLabel = new QLabel(QString("Contrast: %1").arg(contrast), this);
    
    sensitivitySlider = new QSlider(Qt::Horizontal, this);
    sensitivitySlider->setRange(1, 30);
    sensitivitySlider->setValue(10);
    sensitivityLabel = new QLabel(QString("Sensitivity: %1").arg(sensitivity), this);

    levelMinSlider = new QSlider(Qt::Horizontal, this);
    levelMinSlider->setRange(MIN_LEVEL_SLIDER_VALUE, MAX_LEVEL_SLIDER_VALUE);
    levelMinSlider->setValue(levelToSliderValue(displayLevelMin));
    levelMinLabel = new QLabel(levelLabelText("Min", displayLevelMin), this);

    levelMaxSlider = new QSlider(Qt::Horizontal, this);
    levelMaxSlider->setRange(MIN_LEVEL_SLIDER_VALUE, MAX_LEVEL_SLIDER_VALUE);
    levelMaxSlider->setValue(levelToSliderValue(displayLevelMax));
    levelMaxLabel = new QLabel(levelLabelText("Max", displayLevelMax), this);
    
    sensLayout->addWidget(sensitivityLabel);
    sensLayout->addWidget(sensitivitySlider);
    contrastLayout->addWidget(contrastLabel);
    contrastLayout->addWidget(contrastSlider);
    levelMinLayout->addWidget(levelMinLabel);
    levelMinLayout->addWidget(levelMinSlider);
    levelMaxLayout->addWidget(levelMaxLabel);
    levelMaxLayout->addWidget(levelMaxSlider);
    
    QLabel *centralFrequencyLabel = new QLabel("Central Frequency (Hz):", this);
    frequencyLineEdit = new QLineEdit(this);
    frequencyLineEdit->setPlaceholderText("Enter frequency (Hz)");
    frequencyLineEdit->setText("100000000");
    
    QLabel *listeningFrequencyLabel = new QLabel("Listening Frequency (Hz):", this);
    listeningFrequencyLineEdit = new QLineEdit(this);
    listeningFrequencyLineEdit->setPlaceholderText("Enter listening frequency (Hz)");
    listeningFrequencyLineEdit->setText("100000000");
    
    QLabel *fftLabel = new QLabel("FFT Length", this);
    
    bandwidthLineEdit = new QLineEdit(this);
    bandwidthLineEdit->setText(formatBandwidthHz(defaultBandwidthForModulation(MOD_AM)));
   
    modulationButtonGroup = new QButtonGroup(this);
    QStringList modulationNames = {"AM", "NFM", "SAM", "USB", "LSB", "DSB", "CW", "WFM", "FT8", "RTTY", "FSK", "PSK"};
    QVector<int> modulationIds = {MOD_AM, MOD_NFM, MOD_SAM, MOD_USB, MOD_LSB, MOD_DSB,
                                  MOD_CW, MOD_WFM, MOD_FT8, MOD_RTTY, MOD_FSK, MOD_PSK};
    
    QHBoxLayout* row1 = new QHBoxLayout();
    QHBoxLayout* row2 = new QHBoxLayout();
    QHBoxLayout* row3 = new QHBoxLayout();
    
    scaleLayout->addLayout(contrastLayout);
    scaleLayout->addLayout(sensLayout);
    scaleLayout->addLayout(levelMinLayout);
    scaleLayout->addLayout(levelMaxLayout);
    
    graphLayout->addWidget(graphWidget);
    graphLayout->addWidget(scaleWidget); 
    graphLayout->addWidget(waterfallWidget);
    graphLayout->addLayout(scaleLayout);
    
    for (int i = 0; i < modulationNames.size(); ++i) {
        QRadioButton* radioButton = new QRadioButton(modulationNames[i]);
        modulationButtonGroup->addButton(radioButton, modulationIds[i]);
        
        if (i < 4) {
            row1->addWidget(radioButton);
        } else if (i < 8) {
        row2->addWidget(radioButton);
        } else {
            row3->addWidget(radioButton);
        }
        
        if (i == 0) {
            radioButton->setChecked(true); 
        }
    }
    
    layout->addWidget(refreshButton);
    layout->addWidget(comboBox);
    layout->addWidget(clkBox);
    layout->addWidget(modeBox);
    layout->addWidget(sampleBox);
    layout->addLayout(checkboxLayout);
    layout->addWidget(centralFrequencyLabel);
    layout->addWidget(frequencyLineEdit);
    layout->addWidget(listeningFrequencyLabel);
    layout->addWidget(listeningFrequencyLineEdit);
    layout->addWidget(fftLabel);
    layout->addWidget(fftComboBox);
    layout->addWidget(scaleLabel);
    layout->addWidget(scaleSlider);
    layout->addWidget(fobosButton);
    layout->addWidget(networkButton);
    layout->addWidget(startButton);
    layout->addWidget(stopButton);
    layout->addWidget(lnaGainLabel);
    layout->addWidget(lnaGainSlider);
    layout->addWidget(vgaGainLabel);
    layout->addWidget(vgaGainSlider);
    layout->addLayout(chckbox);
    layout->addWidget(audioDeviceComboBox);
    layout->addWidget(bandwidthLineEdit);
    layout->addLayout(row1);
    layout->addLayout(row2);
    layout->addLayout(row3);
    
    hboxLayout->addLayout(layout, 0);
    hboxLayout->addLayout(graphLayout, 1);
    graphLayout->setStretch(0, 2);
    graphLayout->setStretch(1, 0);
    graphLayout->setStretch(2, 5);
    graphLayout->setStretch(3, 0);
    centralWidget->setLayout(hboxLayout);
    
    scaleWidget->setTuning(listeningFrequency, globalFrequency, globalBandwidth, globalModulationType);
    scaleWidget->setMarkerPosition(0.5);
    scaleWidget->setRange(minFrequency, maxFrequency);
    
    updateTimer = new QTimer(this);
    updateSpectrumTimerInterval();
    stopPollTimer = new QTimer(this);
    stopPollTimer->setInterval(100);
    streamWatchdogTimer = new QTimer(this);
    streamWatchdogTimer->setInterval(250);
    
    connect(updateTimer, &QTimer::timeout, this, &YourClassName::updateSpectrum);
    connect(stopPollTimer, &QTimer::timeout, this, &YourClassName::pollStopCompletion);
    connect(streamWatchdogTimer, &QTimer::timeout, this, &YourClassName::checkStreamStartup);
    connect(graphCheckbox, &QCheckBox::toggled, this, &YourClassName::doubleGraphEnable);
    connect(colorCheckbox, &QCheckBox::toggled, this, &YourClassName::colorGraphEnable);
    connect(syncCheckbox, &QCheckBox::toggled, this, &YourClassName::syncEnable);
    connect(audioDeviceComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onAudioDeviceChanged(int)));
    connect(modulationButtonGroup, QOverload<int>::of(&QButtonGroup::idClicked), this, &YourClassName::onModulationChanged);
    connect(scaleSlider, &QSlider::valueChanged, this, &YourClassName::onScaleChanged);
    connect(frequencyLineEdit, &QLineEdit::returnPressed, this, &YourClassName::onFrequencyEntered);
    connect(fftComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onfftLengthEntered()));
    connect(listeningFrequencyLineEdit, &QLineEdit::returnPressed, this, &YourClassName::onListeningFrequencyEntered);
    connect(lnaGainSlider, &QSlider::valueChanged, this, &YourClassName::onLnaGainChanged);
    connect(vgaGainSlider, &QSlider::valueChanged, this, &YourClassName::onVgaGainChanged);
    connect(contrastSlider, &QSlider::valueChanged, this, &YourClassName::onContrastChanged);
    connect(sensitivitySlider, &QSlider::valueChanged, this, &YourClassName::onSensitivityChanged);
    connect(levelMinSlider, &QSlider::valueChanged, this, &YourClassName::onLevelMinChanged);
    connect(levelMaxSlider, &QSlider::valueChanged, this, &YourClassName::onLevelMaxChanged);
    connect(scaleSlider, &QSlider::sliderReleased, this, [this]() {
        if (isNetworkClientMode() && !isFullIqProcessingMode()) {
            sendRemoteControlCommand("settings");
        }
    });
    connect(startButton, &QPushButton::clicked, this, &YourClassName::startFobosProcessing);
    connect(stopButton, &QPushButton::clicked, this, &YourClassName::stopFobosProcessing);
    connect(modeBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &YourClassName::onDirectSamplingChanged);
    connect(clkBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &YourClassName::onClkChanged);
    connect(refreshButton, &QPushButton::clicked, [this]() {
        comboBox->clear();
        comboBox->addItems(getFobosDevices());
    });
    connect(fobosButton, &QPushButton::clicked, this, &YourClassName::listFobosDevices);
    connect(networkButton, &QPushButton::clicked, this, &YourClassName::openNetworkSettingsDialog);
    connect(networkController, &NetworkController::statusChanged, this, &YourClassName::onNetworkStatusChanged);
    connect(networkController, &NetworkController::channelReady, this, [this](const QString &status) {
        onNetworkStatusChanged(status);
        if (isNetworkClientMode()) {
            QTimer::singleShot(0, this, [this]() {
                sendRemoteControlCommand("settings");
            });
        }
    });
    connect(networkController, &NetworkController::controlCommandReceived, this, &YourClassName::onNetworkControlCommandReceived);
    connect(audioProcessor,
            &AudioProcessor::audioFrameReady,
            this,
            [this](const QByteArray &pcmData) {
                sendNetworkAudioFrame(pcmData);
            },
            Qt::QueuedConnection);
    connect(sampleBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &YourClassName::onSampleRateChanged);
    connect(bandwidthLineEdit, &QLineEdit::textChanged, this, &YourClassName::onBandwidthChanged);
    connect(scaleWidget, SIGNAL(frequencyChanged()), this, SLOT(updateFrequency()));
    connect(scaleWidget, SIGNAL(centralFrequencyChanged()), this, SLOT(updateCentralFrequency()));
    connect(scaleWidget, &ScaleWidget::tuningChanged, this, &YourClassName::updateTuningFromScale);
    //connect(scaleWidget, &ScaleWidget::frequencyChanged, this, &YourClassName::updateFrequency);
    //connect(scaleWidget, &ScaleWidget::centralFrequencyChanged, this, &YourClassName::updateCentralFrequency);
    //connect(waterfallWidget, SIGNAL(scaleChanged(int delta)), this, SLOT(onWaterfallScaleChanged(int delta)));
    //connect(graphWidget, SIGNAL(scaleChanged(int delta)), this, SLOT(onWaterfallScaleChanged(int delta)));
    connect(waterfallWidget, &MyWaterfallWidget::scaleChanged, this, &YourClassName::onWaterfallScaleChanged);
    connect(graphWidget, &MyGraphWidget::scaleChanged, this, &YourClassName::onWaterfallScaleChanged);
    onFrequencyEntered();
    onVgaGainChanged(3);
    onLnaGainChanged(1);
    populateSampleRates();
    populateAudioDevices();
    refreshSettingsFromUi();
    publishSettingsToGlobals();
    updateUiForRunState();
}

YourClassName::~YourClassName() {
    if (stopPollTimer) {
        stopPollTimer->stop();
    }
    if (streamWatchdogTimer) {
        streamWatchdogTimer->stop();
    }
    pendingAudioStartAfterStreamReady = false;
    pendingNetworkAudioStartAfterIqPrebuffer = false;
    if (updateTimer) {
        updateTimer->stop();
    }
    if (remoteAudioPlayer) {
        remoteAudioPlayer->stop();
    }
    if (networkController) {
        networkController->stop();
    }
    if (audioProcessor) {
        audioProcessor->stopDemodulation();
        audioProcessor->setLocalPlaybackEnabled(true);
    }
    if (processor) {
        processor->requestStop();
        if (processor->isRunning() && !processor->wait(1500)) {
            processor->forceStop(1000);
        }
        processor->finalizeStopped();
    }

    if (audioProcessor) { 
    delete audioProcessor;
    audioProcessor = nullptr;
    }
    if (processor) {
        delete processor;
        processor = nullptr;
    }

    if (device) {
        if (fobosCloseKnownUnsafe) {
            qDebug() << "[FobosLifecycle] destructor: skipping fobos_rx_close because a previous close failed or hung";
        } else {
            closeFobosDeviceSafely(device);
        }
        device = nullptr;
    }
    if (iqData) {
        iqData = nullptr;
    }
}

uint8_t YourClassName::currentGpoValue() const {
    uint8_t value = 0;
    for (int i = 0; i < 8; ++i) {
        if (checkBoxes[i] && checkBoxes[i]->isChecked()) {
            value |= (1 << i);
        }
    }
    return value;
}

bool YourClassName::isIdle() const {
    return runState == RadioRunState::Idle;
}

bool YourClassName::isRunningOrTransitioning() const {
    return runState != RadioRunState::Idle;
}

void YourClassName::refreshSettingsFromUi() {
    if (comboBox) {
        pendingSettings.deviceIndex = std::max(0, comboBox->currentIndex());
    }
    if (clkBox) {
        pendingSettings.clockSource = clkBox->currentData().toInt();
    }
    if (modeBox) {
        pendingSettings.inputMode = modeBox->currentData().toInt();
    }
    if (sampleBox) {
        bool ok = false;
        const double sampleRate = sampleBox->currentData().toDouble(&ok);
        if (ok && sampleRate > 0.0) {
            pendingSettings.sampleRate = sampleRate;
        }
    }
    if (frequencyLineEdit) {
        bool ok = false;
        double frequency = frequencyLineEdit->text().toDouble(&ok);
        if (ok) {
            if (pendingSettings.inputMode == 0 && frequency < 50000000.0) {
                frequency = 50000000.0;
            }
            pendingSettings.centerFrequency = pendingSettings.inputMode == 0 ? frequency : 0.0;
        }
    }
    if (listeningFrequencyLineEdit) {
        bool ok = false;
        const double frequency = listeningFrequencyLineEdit->text().toDouble(&ok);
        if (ok) {
            pendingSettings.listeningFrequency = frequency;
        }
    }
    if (bandwidthLineEdit) {
        bool ok = false;
        const double bandwidth = bandwidthLineEdit->text().toDouble(&ok);
        if (ok && bandwidth > 0.0) {
            pendingSettings.bandwidth = bandwidth;
        }
    }
    if (fftComboBox) {
        const int selectedFftLength = fftComboBox->currentText().toInt();
        if (selectedFftLength > 0) {
            pendingSettings.fftLength = selectedFftLength;
        }
    }
    if (lnaGainSlider) {
        pendingSettings.lnaGain = lnaGainSlider->value();
    }
    if (vgaGainSlider) {
        pendingSettings.vgaGain = vgaGainSlider->value();
    }
    if (audioDeviceComboBox) {
        pendingSettings.audioDeviceId = audioDeviceComboBox->currentData().toInt();
    }
    if (audioCheckbox) {
        pendingSettings.audioEnabled = audioCheckbox->isChecked();
    }
    if (syncCheckbox) {
        if (syncCheckbox->isChecked()) {
            syncCheckbox->blockSignals(true);
            syncCheckbox->setChecked(false);
            syncCheckbox->blockSignals(false);
        }
        pendingSettings.syncEnabled = false;
    }
    pendingSettings.gpoValue = currentGpoValue();
    normalizeTuning(pendingSettings);
}

void YourClassName::publishSettingsToGlobals() {
    globalMode = pendingSettings.inputMode;
    globalFrequency = pendingSettings.centerFrequency;
    actualFrequency = pendingSettings.actualFrequency;
    listeningFrequency = pendingSettings.listeningFrequency;
    globalSampleRate = pendingSettings.sampleRate;
    globalBandwidth = pendingSettings.bandwidth;
    globalModulationType = pendingSettings.modulationType;
    fftLength = pendingSettings.fftLength;
    syncWariable = pendingSettings.syncEnabled;
    deviceID = pendingSettings.audioDeviceId;
    if (audioProcessor) {
        audioProcessor->configure(pendingSettings);
    }
}

bool YourClassName::isNetworkClientMode() const {
    return networkMode == NetworkMode::Client;
}

bool YourClassName::isClientIqProcessingMode() const {
    return isChannelIqProcessingMode() || isFullIqProcessingMode();
}

bool YourClassName::isChannelIqProcessingMode() const {
    return networkProcessingMode == NetworkProcessingMode::ChannelIqClientSide;
}

bool YourClassName::isFullIqProcessingMode() const {
    return networkProcessingMode == NetworkProcessingMode::FullIqClientSide;
}

void YourClassName::appendNetworkState(QJsonObject &command) const {
    command["processingMode"] = static_cast<int>(networkProcessingMode);
    command["serverDisableLocalVisualAudio"] = serverDisableLocalVisualAudio;
}

void YourClassName::applyNetworkStateFromCommand(const QJsonObject &command) {
    if (command.contains("processingMode")) {
        networkProcessingMode = static_cast<NetworkProcessingMode>(command.value("processingMode").toInt(
            static_cast<int>(networkProcessingMode)));
    }
    if (command.contains("serverDisableLocalVisualAudio")) {
        serverDisableLocalVisualAudio = command.value("serverDisableLocalVisualAudio").toBool(serverDisableLocalVisualAudio);
    }
    onNetworkStatusChanged(networkController ? networkController->statusText() : QString());
}

void YourClassName::connectDataProcessorSignals() {
    if (!processor) {
        return;
    }

    connect(processor,
            &DataProcessor::iqFrameReady,
            this,
            [this](const QByteArray &iqData, double sampleRate, int sampleCount) {
                sendNetworkIqFrame(iqData, sampleRate, sampleCount);
            },
            Qt::QueuedConnection);
}

void YourClassName::startNetworkClientProcessing() {
    if (!isNetworkClientMode() || !isClientIqProcessingMode()) {
        return;
    }

    if (remoteAudioPlayer) {
        remoteAudioPlayer->stop();
    }
    if (updateTimer) {
        updateTimer->stop();
    }
    if (audioProcessor) {
        audioProcessor->stopDemodulation();
        audioProcessor->setLocalPlaybackEnabled(true);
    }

    IqBuffer::clear();
    IqBuffer::setSampleRateEstimate(pendingSettings.sampleRate);
    publishSettingsToGlobals();
    fftResult = std::make_unique<FFTResult>();
    spectrumDebugFramesRemaining = fobosVerboseLoggingEnabled() ? 12 : 0;
    updateSpectrumTimerInterval();

    if (updateTimer && isFullIqProcessingMode()) {
        updateTimer->start();
    }
    pendingNetworkAudioStartAfterIqPrebuffer = pendingSettings.audioEnabled;

    qDebug() << "[NetworkIQ] client-side IQ processing started"
             << "sampleRate" << pendingSettings.sampleRate
             << "audio" << pendingSettings.audioEnabled
             << "audioPrebuffer" << pendingNetworkAudioStartAfterIqPrebuffer;
}

void YourClassName::stopNetworkClientProcessing() {
    if (updateTimer) {
        updateTimer->stop();
    }
    if (audioProcessor) {
        audioProcessor->stopDemodulation();
    }
    if (remoteAudioPlayer) {
        remoteAudioPlayer->stop();
    }
    pendingNetworkAudioStartAfterIqPrebuffer = false;
    qDebug() << "[NetworkIQ] client-side IQ processing stopped";
}

QJsonObject YourClassName::settingsToJson() const {
    QJsonObject settings;
    settings["deviceIndex"] = pendingSettings.deviceIndex;
    settings["clockSource"] = pendingSettings.clockSource;
    settings["inputMode"] = pendingSettings.inputMode;
    settings["centerFrequency"] = pendingSettings.centerFrequency;
    settings["actualFrequency"] = pendingSettings.actualFrequency;
    settings["listeningFrequency"] = pendingSettings.listeningFrequency;
    settings["sampleRate"] = pendingSettings.sampleRate;
    settings["bandwidth"] = pendingSettings.bandwidth;
    settings["modulationType"] = pendingSettings.modulationType;
    settings["fftLength"] = pendingSettings.fftLength;
    settings["lnaGain"] = pendingSettings.lnaGain;
    settings["vgaGain"] = pendingSettings.vgaGain;
    settings["audioEnabled"] = pendingSettings.audioEnabled;
    settings["syncEnabled"] = false;
    settings["gpoValue"] = static_cast<int>(pendingSettings.gpoValue);
    settings["scalePercent"] = currentScale;
    return settings;
}

bool YourClassName::sendRemoteControlCommand(const QString &action) {
    if (!networkController || networkMode != NetworkMode::Client) {
        return false;
    }

    refreshSettingsFromUi();

    QJsonObject command;
    command["type"] = "control";
    command["action"] = action;
    appendNetworkState(command);
    command["settings"] = settingsToJson();

    const bool sent = networkController->sendControlCommand(command);
    if (!sent) {
        qDebug() << "[Network] remote command could not be sent" << action;
    } else {
        qDebug() << "[Network] remote command sent" << action;
    }
    return sent;
}

void YourClassName::applySettingsFromJson(const QJsonObject &settingsJson) {
    auto readInt = [&settingsJson](const char *key, int currentValue) {
        return settingsJson.contains(key) ? settingsJson.value(key).toInt(currentValue) : currentValue;
    };
    auto readDouble = [&settingsJson](const char *key, double currentValue) {
        return settingsJson.contains(key) ? settingsJson.value(key).toDouble(currentValue) : currentValue;
    };
    auto readBool = [&settingsJson](const char *key, bool currentValue) {
        return settingsJson.contains(key) ? settingsJson.value(key).toBool(currentValue) : currentValue;
    };

    pendingSettings.deviceIndex = readInt("deviceIndex", pendingSettings.deviceIndex);
    pendingSettings.clockSource = readInt("clockSource", pendingSettings.clockSource);
    pendingSettings.inputMode = readInt("inputMode", pendingSettings.inputMode);
    pendingSettings.centerFrequency = readDouble("centerFrequency", pendingSettings.centerFrequency);
    pendingSettings.actualFrequency = readDouble("actualFrequency", pendingSettings.actualFrequency);
    pendingSettings.listeningFrequency = readDouble("listeningFrequency", pendingSettings.listeningFrequency);
    pendingSettings.sampleRate = readDouble("sampleRate", pendingSettings.sampleRate);
    pendingSettings.bandwidth = readDouble("bandwidth", pendingSettings.bandwidth);
    pendingSettings.modulationType = readInt("modulationType", pendingSettings.modulationType);
    pendingSettings.fftLength = readInt("fftLength", pendingSettings.fftLength);
    pendingSettings.lnaGain = readInt("lnaGain", pendingSettings.lnaGain);
    pendingSettings.vgaGain = readInt("vgaGain", pendingSettings.vgaGain);
    pendingSettings.audioEnabled = readBool("audioEnabled", pendingSettings.audioEnabled);
    pendingSettings.syncEnabled = false;
    pendingSettings.gpoValue = static_cast<std::uint8_t>(readInt("gpoValue", pendingSettings.gpoValue));
    currentScale = readDouble("scalePercent", currentScale);
    normalizeTuning(pendingSettings);
}

void YourClassName::updateUiFromPendingSettings() {
    if (comboBox) {
        comboBox->blockSignals(true);
        if (pendingSettings.deviceIndex >= 0 && pendingSettings.deviceIndex < comboBox->count()) {
            comboBox->setCurrentIndex(pendingSettings.deviceIndex);
        }
        comboBox->blockSignals(false);
    }
    if (clkBox) {
        clkBox->blockSignals(true);
        const int index = clkBox->findData(pendingSettings.clockSource);
        if (index >= 0) {
            clkBox->setCurrentIndex(index);
        }
        clkBox->blockSignals(false);
    }
    if (modeBox) {
        modeBox->blockSignals(true);
        const int index = modeBox->findData(pendingSettings.inputMode);
        if (index >= 0) {
            modeBox->setCurrentIndex(index);
        }
        modeBox->blockSignals(false);
    }
    if (sampleBox) {
        sampleBox->blockSignals(true);
        int bestIndex = -1;
        double bestDelta = std::numeric_limits<double>::max();
        for (int i = 0; i < sampleBox->count(); ++i) {
            bool ok = false;
            const double value = sampleBox->itemData(i).toDouble(&ok);
            if (!ok) {
                continue;
            }
            const double delta = std::abs(value - pendingSettings.sampleRate);
            if (delta < bestDelta) {
                bestDelta = delta;
                bestIndex = i;
            }
        }
        if (bestIndex >= 0 && bestDelta <= 0.5) {
            sampleBox->setCurrentIndex(bestIndex);
        } else {
            sampleBox->addItem(formatSampleRate(pendingSettings.sampleRate), pendingSettings.sampleRate);
            sampleBox->setCurrentIndex(sampleBox->count() - 1);
        }
        sampleBox->blockSignals(false);
    }
    if (frequencyLineEdit) {
        frequencyLineEdit->setText(QString::number(pendingSettings.centerFrequency, 'f', 0));
    }
    if (listeningFrequencyLineEdit) {
        listeningFrequencyLineEdit->setText(QString::number(pendingSettings.listeningFrequency, 'f', 0));
    }
    if (bandwidthLineEdit) {
        bandwidthLineEdit->blockSignals(true);
        bandwidthLineEdit->setText(QString::number(pendingSettings.bandwidth, 'f', 0));
        bandwidthLineEdit->blockSignals(false);
    }
    if (fftComboBox) {
        fftComboBox->blockSignals(true);
        fftComboBox->setCurrentText(QString::number(pendingSettings.fftLength));
        fftComboBox->blockSignals(false);
    }
    if (modulationButtonGroup) {
        QAbstractButton *button = modulationButtonGroup->button(pendingSettings.modulationType);
        if (button) {
            modulationButtonGroup->blockSignals(true);
            button->setChecked(true);
            modulationButtonGroup->blockSignals(false);
        }
    }
    if (lnaGainSlider) {
        lnaGainSlider->blockSignals(true);
        lnaGainSlider->setValue(pendingSettings.lnaGain);
        lnaGainSlider->blockSignals(false);
    }
    if (lnaGainLabel) {
        lnaGainLabel->setText(QString("LNA Gain: %1").arg(pendingSettings.lnaGain));
    }
    if (vgaGainSlider) {
        vgaGainSlider->blockSignals(true);
        vgaGainSlider->setValue(pendingSettings.vgaGain);
        vgaGainSlider->blockSignals(false);
    }
    if (vgaGainLabel) {
        vgaGainLabel->setText(QString("VGA Gain: %1").arg(pendingSettings.vgaGain));
    }
    if (audioCheckbox) {
        audioCheckbox->blockSignals(true);
        audioCheckbox->setChecked(pendingSettings.audioEnabled);
        audioCheckbox->blockSignals(false);
    }
    for (int i = 0; i < 8; ++i) {
        if (checkBoxes[i]) {
            checkBoxes[i]->blockSignals(true);
            checkBoxes[i]->setChecked((pendingSettings.gpoValue & (1 << i)) != 0);
            checkBoxes[i]->blockSignals(false);
        }
    }
    if (scaleWidget) {
        scaleWidget->setTuning(pendingSettings.listeningFrequency,
                               pendingSettings.centerFrequency,
                               pendingSettings.bandwidth,
                               pendingSettings.modulationType);
    }
    if (scaleSlider) {
        scaleSlider->blockSignals(true);
        scaleSlider->setValue(scalePercentToSliderValue(currentScale));
        scaleSlider->blockSignals(false);
    }
    if (scaleLabel) {
        scaleLabel->setText(scaleLabelText(currentScale));
    }
    updateSpectrumTimerInterval();
    settingRange();
}

void YourClassName::applyLiveRemoteSettings(const RadioSettings &previousSettings) {
    if (!device || isIdle()) {
        return;
    }

    if (pendingSettings.inputMode == 0 &&
        std::abs(previousSettings.centerFrequency - pendingSettings.centerFrequency) > 0.5) {
        double tunedFrequency = pendingSettings.centerFrequency;
        const int result = setFobosFrequencySafely(device, pendingSettings.centerFrequency, &tunedFrequency);
        if (result == FOBOS_ERR_OK) {
            pendingSettings.actualFrequency = tunedFrequency;
            if (hardwareSettingsApplied) {
                appliedHardwareSettings.centerFrequency = pendingSettings.centerFrequency;
                appliedHardwareSettings.actualFrequency = tunedFrequency;
            }
        } else {
            qDebug() << "[Network] failed to tune remote frequency while running" << result;
        }
    }

    if (hardwareSettingsApplied && previousSettings.lnaGain != pendingSettings.lnaGain) {
        const int result = setFobosLnaGainSafely(device, static_cast<unsigned int>(pendingSettings.lnaGain));
        qDebug() << "[Network] remote LNA apply result" << result;
        if (result == FOBOS_ERR_OK) {
            appliedHardwareSettings.lnaGain = pendingSettings.lnaGain;
        }
    }
    if (hardwareSettingsApplied && previousSettings.vgaGain != pendingSettings.vgaGain) {
        const int result = setFobosVgaGainSafely(device, static_cast<unsigned int>(pendingSettings.vgaGain));
        qDebug() << "[Network] remote VGA apply result" << result;
        if (result == FOBOS_ERR_OK) {
            appliedHardwareSettings.vgaGain = pendingSettings.vgaGain;
        }
    }
    if (hardwareSettingsApplied && previousSettings.gpoValue != pendingSettings.gpoValue) {
        const int result = setFobosGpoSafely(device, pendingSettings.gpoValue);
        qDebug() << "[Network] remote GPO apply result" << result;
        if (result == FOBOS_ERR_OK) {
            appliedHardwareSettings.gpoValue = pendingSettings.gpoValue;
        }
    }

    if (std::abs(previousSettings.sampleRate - pendingSettings.sampleRate) > 0.5 ||
        previousSettings.inputMode != pendingSettings.inputMode ||
        previousSettings.clockSource != pendingSettings.clockSource ||
        previousSettings.fftLength != pendingSettings.fftLength) {
        qDebug() << "[Network] remote settings include restart-only changes; they will be applied on next server start";
    }
}

void YourClassName::onNetworkControlCommandReceived(const QJsonObject &command) {
    if (networkMode == NetworkMode::Client && command.value("type").toString() == "iq") {
        receiveNetworkIqFrame(command);
        return;
    }

    if (networkMode == NetworkMode::Client && !isClientIqProcessingMode() && command.value("type").toString() == "audio") {
        playNetworkAudioFrame(command);
        return;
    }

    if (networkMode == NetworkMode::Client && !isFullIqProcessingMode() && command.value("type").toString() == "spectrum") {
        displayNetworkSpectrumFrame(command);
        return;
    }

    if (networkMode != NetworkMode::Server || command.value("type").toString() != "control") {
        return;
    }

    const QString action = command.value("action").toString();
    const QJsonObject settingsJson = command.value("settings").toObject();
    applyNetworkStateFromCommand(command);
    qDebug() << "[Network] received remote control command" << action;

    if (action == "settings" || action == "start") {
        const RadioSettings previousSettings = pendingSettings;
        applySettingsFromJson(settingsJson);
        publishSettingsToGlobals();
        updateUiFromPendingSettings();
        applyLiveRemoteSettings(previousSettings);
        if (processor && processor->isRunning() && isClientIqProcessingMode()) {
            processor->updateNetworkIqSettings(pendingSettings, isChannelIqProcessingMode());
        }
    }

    if (action == "start") {
        if (isIdle()) {
            startFobosProcessing();
        } else {
            qDebug() << "[Network] remote start ignored because server is not idle";
        }
    } else if (action == "stop") {
        stopFobosProcessing();
    }
}

void YourClassName::sendNetworkSpectrumFrame(const std::vector<float> &frequencies, const std::vector<float> &magnitudes) {
    if (networkMode != NetworkMode::Server ||
        isFullIqProcessingMode() ||
        !networkController ||
        !networkController->isControlReady() ||
        frequencies.empty() ||
        magnitudes.empty()) {
        return;
    }

    if (networkSpectrumFrameTimer.isValid() &&
        networkSpectrumFrameTimer.elapsed() < NETWORK_SPECTRUM_INTERVAL_MS) {
        return;
    }
    networkSpectrumFrameTimer.restart();

    const int dataCount = std::min(static_cast<int>(frequencies.size()), static_cast<int>(magnitudes.size()));
    if (dataCount <= 0) {
        return;
    }
    if (isChannelIqProcessingMode() &&
        networkController->pendingBytes() > NETWORK_SPECTRUM_MAX_PENDING_BYTES) {
        return;
    }

    const int targetCount = (std::min)(NETWORK_SPECTRUM_MAX_BINS, dataCount);
    const double step = static_cast<double>(dataCount) / static_cast<double>(targetCount);
    QJsonArray frequencyArray;
    QJsonArray magnitudeArray;

    for (int i = 0; i < targetCount; ++i) {
        const int index = (std::min)(dataCount - 1, static_cast<int>(std::floor(i * step)));
        if (!std::isfinite(frequencies[index]) || !std::isfinite(magnitudes[index])) {
            continue;
        }
        frequencyArray.append(frequencies[index]);
        magnitudeArray.append(magnitudes[index]);
    }

    QJsonObject frame;
    frame["type"] = "spectrum";
    frame["sequence"] = QString::number(++networkSpectrumFrameSequence);
    frame["centerFrequency"] = pendingSettings.centerFrequency;
    frame["listeningFrequency"] = pendingSettings.listeningFrequency;
    frame["sampleRate"] = pendingSettings.sampleRate;
    frame["bandwidth"] = pendingSettings.bandwidth;
    frame["modulationType"] = pendingSettings.modulationType;
    frame["fftLength"] = targetCount;
    frame["minFrequency"] = minFrequency;
    frame["maxFrequency"] = maxFrequency;
    frame["frequencies"] = frequencyArray;
    frame["magnitudes"] = magnitudeArray;

    networkController->sendControlCommand(frame);
}

void YourClassName::displayNetworkSpectrumFrame(const QJsonObject &frame) {
    const QJsonArray frequencyArray = frame.value("frequencies").toArray();
    const QJsonArray magnitudeArray = frame.value("magnitudes").toArray();
    const int dataCount = (std::min)(frequencyArray.size(), magnitudeArray.size());
    if (dataCount <= 0) {
        return;
    }

    std::vector<float> frequencies;
    std::vector<float> magnitudes;
    frequencies.reserve(dataCount);
    magnitudes.reserve(dataCount);

    for (int i = 0; i < dataCount; ++i) {
        const double frequency = frequencyArray.at(i).toDouble(std::numeric_limits<double>::quiet_NaN());
        const double magnitude = magnitudeArray.at(i).toDouble(std::numeric_limits<double>::quiet_NaN());
        if (!std::isfinite(frequency) || !std::isfinite(magnitude)) {
            continue;
        }
        frequencies.push_back(static_cast<float>(frequency));
        magnitudes.push_back(static_cast<float>(magnitude));
    }

    if (frequencies.empty() || magnitudes.empty()) {
        return;
    }

    const double frameMinFrequency = frame.value("minFrequency").toDouble(minFrequency);
    const double frameMaxFrequency = frame.value("maxFrequency").toDouble(maxFrequency);
    const double frameCenterFrequency = frame.value("centerFrequency").toDouble(pendingSettings.centerFrequency);
    const double frameListeningFrequency = frame.value("listeningFrequency").toDouble(pendingSettings.listeningFrequency);
    const double frameBandwidth = frame.value("bandwidth").toDouble(pendingSettings.bandwidth);
    const int frameModulationType = frame.value("modulationType").toInt(pendingSettings.modulationType);

    if (!isChannelIqProcessingMode()) {
        pendingSettings.centerFrequency = frameCenterFrequency;
        pendingSettings.listeningFrequency = frameListeningFrequency;
        pendingSettings.sampleRate = frame.value("sampleRate").toDouble(pendingSettings.sampleRate);
        pendingSettings.bandwidth = frameBandwidth;
        pendingSettings.modulationType = frameModulationType;
        publishSettingsToGlobals();
    }

    if (scaleWidget) {
        scaleWidget->setTuning(frameListeningFrequency,
                               frameCenterFrequency,
                               frameBandwidth,
                               frameModulationType);
        scaleWidget->setRange(frameMinFrequency, frameMaxFrequency);
    }

    const int frameFftLength = static_cast<int>(frequencies.size());
    if (graphWidget) {
        graphWidget->setLevelRange(displayLevelMin, displayLevelMax);
        graphWidget->setData(frequencies, magnitudes, frameMinFrequency, frameMaxFrequency, frameFftLength, colorf);
    }
    if (waterfallWidget) {
        waterfallWidget->setData(frequencies, magnitudes, frameMinFrequency, frameMaxFrequency, frameFftLength,
                                 secondGraph, contrast, sensitivity, displayLevelMin, displayLevelMax);
    }
}

void YourClassName::sendNetworkAudioFrame(const QByteArray &pcmData) {
    if (networkMode != NetworkMode::Server ||
        isClientIqProcessingMode() ||
        !networkController ||
        !networkController->isControlReady() ||
        pcmData.isEmpty()) {
        return;
    }

    QJsonObject frame;
    frame["type"] = "audio";
    frame["sampleRate"] = 48000;
    frame["channels"] = 1;
    frame["sampleFormat"] = "pcm_s16le";
    frame["pcm"] = QString::fromLatin1(pcmData.toBase64());
    networkController->sendControlCommand(frame);
}

void YourClassName::playNetworkAudioFrame(const QJsonObject &frame) {
    if (networkMode != NetworkMode::Client || !remoteAudioPlayer) {
        return;
    }

    if (frame.value("sampleRate").toInt(48000) != 48000 ||
        frame.value("channels").toInt(1) != 1 ||
        frame.value("sampleFormat").toString() != QStringLiteral("pcm_s16le")) {
        qDebug() << "[NetworkAudio] unsupported remote audio frame format";
        return;
    }

    const QByteArray pcmData = QByteArray::fromBase64(frame.value("pcm").toString().toLatin1());
    remoteAudioPlayer->playPcmFrame(pcmData);
}

void YourClassName::sendNetworkIqFrame(const QByteArray &iqData, double sampleRate, int sampleCount) {
    if (networkMode != NetworkMode::Server ||
        !isClientIqProcessingMode() ||
        !networkController ||
        !networkController->isControlReady() ||
        iqData.isEmpty()) {
        return;
    }

    const qint64 pendingBytes = networkController->pendingBytes();
    if (pendingBytes > NETWORK_IQ_MAX_PENDING_BYTES) {
        ++networkIqFramesDropped;
        if (networkIqFramesDropped == 1 ||
            (networkIqFramesDropped % NETWORK_IQ_DROP_LOG_INTERVAL) == 0) {
            qDebug() << "[NetworkIQ] dropping IQ frame because TCP queue is full"
                     << "dropped" << networkIqFramesDropped
                     << "pendingBytes" << pendingBytes
                     << "frameBytes" << iqData.size();
        }
        return;
    }

    QJsonObject frame;
    frame["type"] = "iq";
    frame["sequence"] = QString::number(++networkIqFrameSequence);
    frame["sampleRate"] = sampleRate;
    frame["sourceSampleRate"] = pendingSettings.sampleRate;
    frame["sampleCount"] = sampleCount;
    frame["sampleFormat"] = isChannelIqProcessingMode() ? "channel_iq_s16le" : "iq_s8_interleaved";
    frame["channelized"] = isChannelIqProcessingMode();
    frame["centerFrequency"] = isChannelIqProcessingMode()
                                   ? pendingSettings.listeningFrequency
                                   : pendingSettings.centerFrequency;
    frame["actualFrequency"] = isChannelIqProcessingMode()
                                   ? pendingSettings.listeningFrequency
                                   : pendingSettings.actualFrequency;
    frame["listeningFrequency"] = pendingSettings.listeningFrequency;
    frame["bandwidth"] = pendingSettings.bandwidth;
    frame["modulationType"] = pendingSettings.modulationType;
    frame["inputMode"] = isChannelIqProcessingMode() ? 0 : pendingSettings.inputMode;
    frame["iq"] = QString::fromLatin1(iqData.toBase64());

    networkController->sendControlCommand(frame);
}

void YourClassName::receiveNetworkIqFrame(const QJsonObject &frame) {
    if (networkMode != NetworkMode::Client || !isClientIqProcessingMode() || runState == RadioRunState::Idle) {
        return;
    }

    const bool channelizedFrame = frame.value("channelized").toBool(false);
    const QString sampleFormat = frame.value("sampleFormat").toString();
    if ((!channelizedFrame && sampleFormat != QStringLiteral("iq_s8_interleaved")) ||
        (channelizedFrame && sampleFormat != QStringLiteral("channel_iq_s16le"))) {
        qDebug() << "[NetworkIQ] unsupported IQ frame format";
        return;
    }

    QByteArray iqBytes = QByteArray::fromBase64(frame.value("iq").toString().toLatin1());
    const int bytesPerFloat = channelizedFrame ? static_cast<int>(sizeof(qint16)) : 1;
    const int bytesPerIqSample = 2 * bytesPerFloat;
    if (iqBytes.size() < bytesPerIqSample) {
        return;
    }
    const int remainder = iqBytes.size() % bytesPerIqSample;
    if (remainder != 0) {
        iqBytes.chop(remainder);
    }

    const double frameSampleRate = frame.value("sampleRate").toDouble(pendingSettings.sampleRate);
    if (frameSampleRate <= 0.0) {
        return;
    }

    RadioSettings iqSettings = pendingSettings;
    iqSettings.sampleRate = frameSampleRate;
    iqSettings.centerFrequency = frame.value("centerFrequency").toDouble(iqSettings.centerFrequency);
    iqSettings.actualFrequency = frame.value("actualFrequency").toDouble(iqSettings.actualFrequency);
    iqSettings.listeningFrequency = frame.value("listeningFrequency").toDouble(iqSettings.listeningFrequency);
    iqSettings.bandwidth = frame.value("bandwidth").toDouble(iqSettings.bandwidth);
    iqSettings.modulationType = frame.value("modulationType").toInt(iqSettings.modulationType);
    iqSettings.inputMode = frame.value("inputMode").toInt(iqSettings.inputMode);

    if (channelizedFrame) {
        iqSettings.inputMode = 0;
        iqSettings.centerFrequency = iqSettings.listeningFrequency;
        iqSettings.actualFrequency = iqSettings.listeningFrequency;
        if (audioProcessor) {
            audioProcessor->configure(iqSettings);
        }
    } else {
        bool processingSettingsChanged = false;
        auto updateDouble = [&processingSettingsChanged](double &target, double value) {
            if (std::abs(target - value) > 0.5) {
                target = value;
                processingSettingsChanged = true;
            }
        };
        auto updateInt = [&processingSettingsChanged](int &target, int value) {
            if (target != value) {
                target = value;
                processingSettingsChanged = true;
            }
        };

        updateDouble(pendingSettings.sampleRate, iqSettings.sampleRate);
        updateDouble(pendingSettings.centerFrequency, iqSettings.centerFrequency);
        updateDouble(pendingSettings.actualFrequency, iqSettings.actualFrequency);
        updateDouble(pendingSettings.listeningFrequency, iqSettings.listeningFrequency);
        updateDouble(pendingSettings.bandwidth, iqSettings.bandwidth);
        updateInt(pendingSettings.modulationType, iqSettings.modulationType);
        updateInt(pendingSettings.inputMode, iqSettings.inputMode);

        if (processingSettingsChanged) {
            publishSettingsToGlobals();
            settingRange();
        }
    }

    std::vector<float> floatSamples(static_cast<std::size_t>(iqBytes.size() / bytesPerFloat));
    if (channelizedFrame) {
        const auto *src = reinterpret_cast<const uchar *>(iqBytes.constData());
        for (int i = 0, out = 0; i + 1 < iqBytes.size(); i += 2, ++out) {
            const qint16 value = static_cast<qint16>(src[i] | (src[i + 1] << 8));
            floatSamples[static_cast<std::size_t>(out)] = static_cast<float>(value) / 32767.0f;
        }
    } else {
        const auto *src = reinterpret_cast<const signed char *>(iqBytes.constData());
        for (int i = 0; i < iqBytes.size(); ++i) {
            floatSamples[static_cast<std::size_t>(i)] = static_cast<float>(src[i]) / 127.0f;
        }
    }

    IqBuffer::setSampleRateEstimate(frameSampleRate);
    IqBuffer::publish(floatSamples.data(), floatSamples.size(), pendingSettings.audioEnabled);

    if (pendingNetworkAudioStartAfterIqPrebuffer && pendingSettings.audioEnabled && audioProcessor) {
        const double queuedIqSamples = static_cast<double>(IqBuffer::queuedFloatCount()) / 2.0;
        const double queuedSeconds = queuedIqSamples / frameSampleRate;
        if (queuedSeconds >= NETWORK_AUDIO_PREBUFFER_SECONDS) {
            pendingNetworkAudioStartAfterIqPrebuffer = false;
            qDebug() << "[NetworkIQ] starting client demodulator after IQ prebuffer"
                     << "queuedSeconds" << queuedSeconds
                     << "queuedBlocks" << IqBuffer::queuedBlocks()
                     << "sampleRate" << frameSampleRate;
            audioProcessor->startDemodulation();
        }
    }
}

void YourClassName::updateUiForRunState() {
    const bool idle = isIdle();

    if (startButton) startButton->setEnabled(idle);
    if (stopButton) stopButton->setEnabled(runState == RadioRunState::Starting || runState == RadioRunState::Running);
    if (comboBox) comboBox->setEnabled(idle);
    if (refreshButton) refreshButton->setEnabled(idle);
    if (fobosButton) fobosButton->setEnabled(idle);
    if (modeBox) modeBox->setEnabled(idle);
    if (sampleBox) sampleBox->setEnabled(idle);
    if (clkBox) clkBox->setEnabled(idle);
    if (fftComboBox) fftComboBox->setEnabled(idle);
    if (audioDeviceComboBox) audioDeviceComboBox->setEnabled(idle);
    if (audioCheckbox) audioCheckbox->setEnabled(idle);
    if (syncCheckbox) syncCheckbox->setEnabled(false);
    if (bandwidthLineEdit) bandwidthLineEdit->setEnabled(idle);
    if (lnaGainSlider) lnaGainSlider->setEnabled(idle);
    if (vgaGainSlider) vgaGainSlider->setEnabled(idle);
    for (int i = 0; i < 8; ++i) {
        if (checkBoxes[i]) {
            checkBoxes[i]->setEnabled(idle);
        }
    }
}

void YourClassName::updateSpectrumTimerInterval() {
    if (!updateTimer) {
        return;
    }

    int intervalMs = 33;
    if (pendingSettings.fftLength >= 262144) {
        intervalMs = 80;
    } else if (pendingSettings.fftLength >= 131072) {
        intervalMs = 50;
    } else if (pendingSettings.fftLength >= 65536) {
        intervalMs = 40;
    }
    updateTimer->setInterval(intervalMs);
}

void YourClassName::revertHardwareControlsToSettings() {
    if (modeBox) {
        modeBox->blockSignals(true);
        const int index = modeBox->findData(pendingSettings.inputMode);
        if (index >= 0) {
            modeBox->setCurrentIndex(index);
        }
        modeBox->blockSignals(false);
    }
    if (clkBox) {
        clkBox->blockSignals(true);
        const int index = clkBox->findData(pendingSettings.clockSource);
        if (index >= 0) {
            clkBox->setCurrentIndex(index);
        }
        clkBox->blockSignals(false);
    }
    if (sampleBox) {
        sampleBox->blockSignals(true);
        const int index = sampleBox->findData(pendingSettings.sampleRate);
        if (index >= 0) {
            sampleBox->setCurrentIndex(index);
        }
        sampleBox->blockSignals(false);
    }
    if (fftComboBox) {
        fftComboBox->blockSignals(true);
        fftComboBox->setCurrentText(QString::number(pendingSettings.fftLength));
        fftComboBox->blockSignals(false);
    }
    if (audioDeviceComboBox) {
        audioDeviceComboBox->blockSignals(true);
        const int index = audioDeviceComboBox->findData(pendingSettings.audioDeviceId);
        if (index >= 0) {
            audioDeviceComboBox->setCurrentIndex(index);
        }
        audioDeviceComboBox->blockSignals(false);
    }
}

bool YourClassName::openFobosSession() {
    int selectedDevice = pendingSettings.deviceIndex;
    if (selectedDevice < 0) {
        selectedDevice = 0;
    }

    qDebug() << "[FobosLifecycle] openFobosSession enter"
             << "selectedDevice" << selectedDevice
             << "device" << device
             << "openedDeviceIndex" << openedDeviceIndex
             << "appliedSampleRate" << appliedSampleRate
             << "pendingSampleRate" << pendingSettings.sampleRate
             << "sampleRateReopenRequired" << sampleRateReopenRequired
             << "fobosCloseKnownUnsafe" << fobosCloseKnownUnsafe;

    if (device && openedDeviceIndex == selectedDevice) {
        qDebug() << "[FobosLifecycle] reusing idle Fobos session; settings will be applied in place"
                 << device;
        return true;
    }

    if (device) {
        qDebug() << "[FobosLifecycle] closing mismatched existing Fobos session before open";
        if (!closeFobosSession(false)) {
            qDebug() << "[FobosLifecycle] existing Fobos session could not be closed; open aborted";
            return false;
        }
        qDebug() << "[FobosLifecycle] mismatched Fobos session closed before fresh open; waiting before reopen";
        QThread::msleep(350);
    }

    qDebug() << "[FobosLifecycle] fobos_rx_open begin" << "selectedDevice" << selectedDevice;
    const int ret = fobos_rx_open(&device, selectedDevice);
    qDebug() << "[FobosLifecycle] fobos_rx_open end" << "result" << ret << "device" << device;
    if (ret != FOBOS_ERR_OK || !device) {
        qDebug() << "Failed to open Fobos device, error code:" << ret;
        device = nullptr;
        openedDeviceIndex = -1;
        return false;
    }

    openedDeviceIndex = selectedDevice;
    appliedHardwareSettings = RadioSettings{};
    hardwareSettingsApplied = false;
    sampleRateReopenRequired = false;
    fobosCloseKnownUnsafe = false;
    return true;
}

bool YourClassName::closeFobosSession(bool clearIq) {
    qDebug() << "[FobosLifecycle] closeFobosSession enter"
             << "clearIq" << clearIq
             << "device" << device
             << "openedDeviceIndex" << openedDeviceIndex;
    bool closeOk = true;
    if (clearIq) {
        qDebug() << "[FobosLifecycle] clearing IQ buffer before close";
        IqBuffer::clear();
    }

    if (device) {
        qDebug() << "[FobosLifecycle] fobos_rx_close begin" << device;
        const int closeResult = closeFobosDeviceSafely(device);
        qDebug() << "[FobosLifecycle] fobos_rx_close end" << "result" << closeResult;
        closeOk = closeResult == FOBOS_ERR_OK;
        if (!closeOk) {
            qDebug() << "Fobos close returned error code:" << closeResult;
            qDebug() << "[FobosLifecycle] abandoning Fobos session pointer after close failure; next start will try a fresh open"
                     << device;
        }
        device = nullptr;
    }
    openedDeviceIndex = -1;
    appliedSampleRate = 0.0;
    appliedHardwareSettings = RadioSettings{};
    hardwareSettingsApplied = false;
    sampleRateReopenRequired = false;
    fobosCloseKnownUnsafe = false;
    qDebug() << "[FobosLifecycle] closeFobosSession exit";
    return closeOk;
}

bool YourClassName::applyFobosSettings() {
    if (!device) {
        qDebug() << "Cannot apply Fobos settings without an active device.";
        return false;
    }

    normalizeTuning(pendingSettings);
    publishSettingsToGlobals();
    const bool firstApply = !hardwareSettingsApplied;
    auto changedDouble = [](double a, double b) {
        return std::abs(a - b) > 0.5;
    };

    qDebug() << "[FobosLifecycle] applyFobosSettings enter"
             << "device" << device
             << "firstApply" << firstApply
             << "clock" << pendingSettings.clockSource
             << "inputMode" << pendingSettings.inputMode
             << "sampleRate" << pendingSettings.sampleRate
             << "centerFrequency" << pendingSettings.centerFrequency
             << "lna" << pendingSettings.lnaGain
             << "vga" << pendingSettings.vgaGain
             << "gpo" << pendingSettings.gpoValue;

    int result = FOBOS_ERR_OK;
    if (firstApply || appliedHardwareSettings.clockSource != pendingSettings.clockSource) {
        qDebug() << "[FobosLifecycle] fobos_rx_set_clk_source begin";
        result = setFobosClockSourceSafely(device, static_cast<unsigned int>(pendingSettings.clockSource));
        qDebug() << "[FobosLifecycle] fobos_rx_set_clk_source end" << "result" << result;
        if (result != FOBOS_ERR_OK) {
            qDebug() << "Failed to set clock source, error code:" << result;
            return false;
        }
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_set_clk_source skipped unchanged";
    }

    const int libfobosMode = (pendingSettings.inputMode == 0) ? 0 : 1;
    if (firstApply || appliedHardwareSettings.inputMode != pendingSettings.inputMode) {
        qDebug() << "[FobosLifecycle] fobos_rx_set_direct_sampling begin" << "libfobosMode" << libfobosMode;
        result = setFobosDirectSamplingSafely(device, static_cast<unsigned int>(libfobosMode));
        qDebug() << "[FobosLifecycle] fobos_rx_set_direct_sampling end" << "result" << result;
        if (result != FOBOS_ERR_OK) {
            qDebug() << "Failed to set direct sampling mode, error code:" << result;
            return false;
        }
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_set_direct_sampling skipped unchanged";
    }

    if (pendingSettings.sampleRate > 0.0 &&
        (firstApply || changedDouble(appliedHardwareSettings.sampleRate, pendingSettings.sampleRate))) {
        qDebug() << "[FobosLifecycle] fobos_rx_set_samplerate begin" << "requested" << pendingSettings.sampleRate;
        result = setFobosSampleRateSafely(device, pendingSettings.sampleRate, &globalSampleRate);
        qDebug() << "[FobosLifecycle] fobos_rx_set_samplerate end"
                 << "result" << result
                 << "actual" << globalSampleRate;
        if (result != FOBOS_ERR_OK) {
            qDebug() << "Failed to set sample rate, error code:" << result;
            globalSampleRate = pendingSettings.sampleRate;
            sampleRateReopenRequired = true;
            hardwareSettingsApplied = false;
            return false;
        }
        pendingSettings.sampleRate = globalSampleRate;
        appliedSampleRate = globalSampleRate;
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_set_samplerate skipped unchanged";
        if (appliedSampleRate > 0.0) {
            globalSampleRate = appliedSampleRate;
            pendingSettings.sampleRate = appliedSampleRate;
        }
    }

    if (pendingSettings.inputMode == 0) {
        if (firstApply ||
            appliedHardwareSettings.inputMode != pendingSettings.inputMode ||
            changedDouble(appliedHardwareSettings.centerFrequency, pendingSettings.centerFrequency)) {
            qDebug() << "[FobosLifecycle] fobos_rx_set_frequency begin" << "requested" << pendingSettings.centerFrequency;
            result = setFobosFrequencySafely(device, pendingSettings.centerFrequency, &actualFrequency);
            qDebug() << "[FobosLifecycle] fobos_rx_set_frequency end"
                     << "result" << result
                     << "actual" << actualFrequency;
            if (result != FOBOS_ERR_OK) {
                qDebug() << "Failed to set frequency, error code:" << result;
            }
            pendingSettings.actualFrequency = actualFrequency;
        } else {
            qDebug() << "[FobosLifecycle] fobos_rx_set_frequency skipped unchanged";
            actualFrequency = pendingSettings.actualFrequency;
        }
    } else {
        pendingSettings.centerFrequency = 0.0;
        actualFrequency = 0.0;
        pendingSettings.actualFrequency = 0.0;
    }

    if (firstApply || appliedHardwareSettings.lnaGain != pendingSettings.lnaGain) {
        qDebug() << "[FobosLifecycle] fobos_rx_set_lna_gain begin" << pendingSettings.lnaGain;
        result = setFobosLnaGainSafely(device, static_cast<unsigned int>(pendingSettings.lnaGain));
        qDebug() << "[FobosLifecycle] fobos_rx_set_lna_gain end" << "result" << result;
        if (result != FOBOS_ERR_OK) {
            qDebug() << "Failed to set LNA gain, error code:" << result;
        }
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_set_lna_gain skipped unchanged";
    }

    if (firstApply || appliedHardwareSettings.vgaGain != pendingSettings.vgaGain) {
        qDebug() << "[FobosLifecycle] fobos_rx_set_vga_gain begin" << pendingSettings.vgaGain;
        result = setFobosVgaGainSafely(device, static_cast<unsigned int>(pendingSettings.vgaGain));
        qDebug() << "[FobosLifecycle] fobos_rx_set_vga_gain end" << "result" << result;
        if (result != FOBOS_ERR_OK) {
            qDebug() << "Failed to set VGA gain, error code:" << result;
        }
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_set_vga_gain skipped unchanged";
    }

    if (firstApply || appliedHardwareSettings.gpoValue != pendingSettings.gpoValue) {
        qDebug() << "[FobosLifecycle] fobos_rx_set_user_gpo begin" << pendingSettings.gpoValue;
        const int gpoResult = setFobosGpoSafely(device, pendingSettings.gpoValue);
        qDebug() << "[FobosLifecycle] fobos_rx_set_user_gpo end" << "result" << gpoResult;
        if (gpoResult != FOBOS_ERR_OK) {
            qDebug() << "Failed to set user GPO, error code:" << gpoResult;
        }
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_set_user_gpo skipped unchanged";
    }

    appliedHardwareSettings = pendingSettings;
    appliedHardwareSettings.sampleRate = globalSampleRate;
    appliedHardwareSettings.actualFrequency = pendingSettings.actualFrequency;
    hardwareSettingsApplied = true;
    publishSettingsToGlobals();
    if (frequencyLineEdit) {
        frequencyLineEdit->setText(QString::number(pendingSettings.centerFrequency, 'f', 0));
    }
    if (listeningFrequencyLineEdit) {
        listeningFrequencyLineEdit->setText(QString::number(pendingSettings.listeningFrequency, 'f', 0));
    }
    settingRange();
    qDebug() << "[FobosLifecycle] applyFobosSettings exit";
    return true;
}

void YourClassName::updateFrequency() {
   const double frequency = scaleWidget ? scaleWidget->currentListeningFrequency() : listeningFrequency;
   listeningFrequencyLineEdit->setText(QString::number(frequency, 'f', 0));
   onListeningFrequencyEntered();
}

void YourClassName::updateCentralFrequency() {
   if (pendingSettings.inputMode != 0) {
       normalizeTuning(pendingSettings);
       if (scaleWidget) {
           scaleWidget->setTuning(pendingSettings.listeningFrequency,
                                  pendingSettings.centerFrequency,
                                  pendingSettings.bandwidth,
                                  pendingSettings.modulationType);
       }
       return;
   }
   const double frequency = scaleWidget ? scaleWidget->currentCenterFrequency() : globalFrequency;
   frequencyLineEdit->setText(QString::number(frequency, 'f', 0));
   onFrequencyEntered();
}

void YourClassName::updateTuningFromScale(double tunedListeningFrequency, double tunedCenterFrequency) {
    pendingSettings.listeningFrequency = tunedListeningFrequency;
    pendingSettings.centerFrequency = tunedCenterFrequency;
    normalizeTuning(pendingSettings);

    if (pendingSettings.inputMode == 0) {
        if (!isIdle() && device) {
            double tunedFrequency = pendingSettings.centerFrequency;
            const int result = setFobosFrequencySafely(device, pendingSettings.centerFrequency, &tunedFrequency);
            if (result == FOBOS_ERR_OK) {
                pendingSettings.actualFrequency = tunedFrequency;
                if (hardwareSettingsApplied) {
                    appliedHardwareSettings.centerFrequency = pendingSettings.centerFrequency;
                    appliedHardwareSettings.actualFrequency = tunedFrequency;
                }
            } else {
                qDebug() << "Failed to tune frequency from scale, error code:" << result;
            }
        }
    }

    publishSettingsToGlobals();
    if (frequencyLineEdit) {
        frequencyLineEdit->setText(QString::number(pendingSettings.centerFrequency, 'f', 0));
    }
    if (listeningFrequencyLineEdit) {
        listeningFrequencyLineEdit->setText(QString::number(pendingSettings.listeningFrequency, 'f', 0));
    }
    settingRange();
    if (isNetworkClientMode()) {
        sendRemoteControlCommand("settings");
    }
}

void YourClassName::onModulationChanged(int id) {
    pendingSettings.modulationType = id;
    pendingSettings.bandwidth = defaultBandwidthForModulation(id);
    if (bandwidthLineEdit) {
        bandwidthLineEdit->blockSignals(true);
        bandwidthLineEdit->setText(formatBandwidthHz(pendingSettings.bandwidth));
        bandwidthLineEdit->blockSignals(false);
    }
    publishSettingsToGlobals();
    if (scaleWidget) {
        scaleWidget->setModulationType(id);
    }
    settingRange();
    qDebug() << "Modulation type changed to:" << id
             << "bandwidth preset" << pendingSettings.bandwidth;
    if (isNetworkClientMode()) {
        sendRemoteControlCommand("settings");
    }
}

void YourClassName::onScaleChanged(int value) {
    currentScale = sliderValueToScalePercent(value);

    scaleLabel->setText(scaleLabelText(currentScale));
    settingRange();
}

void YourClassName::onSensitivityChanged(int value) {
    sensitivity = value;
    sensitivityLabel->setText(QString("Sensitivity: %1").arg(value));
    settingRange();
}

void YourClassName::onContrastChanged(int value) {
    contrast = value;
    contrastLabel->setText(QString("Contrast: %1").arg(value));
    settingRange();
}

void YourClassName::onLevelMinChanged(int value) {
    displayLevelMin = sliderValueToLevel(value);
    if (displayLevelMin >= displayLevelMax) {
        displayLevelMin = (std::max)(sliderValueToLevel(MIN_LEVEL_SLIDER_VALUE), displayLevelMax - MIN_LEVEL_GAP);
        if (levelMinSlider) {
            levelMinSlider->blockSignals(true);
            levelMinSlider->setValue(levelToSliderValue(displayLevelMin));
            levelMinSlider->blockSignals(false);
        }
    }
    if (levelMinLabel) {
        levelMinLabel->setText(levelLabelText("Min", displayLevelMin));
    }
    if (levelMaxLabel) {
        levelMaxLabel->setText(levelLabelText("Max", displayLevelMax));
    }
    if (graphWidget) {
        graphWidget->setLevelRange(displayLevelMin, displayLevelMax);
    }
    if (waterfallWidget) {
        waterfallWidget->setLevelRange(displayLevelMin, displayLevelMax);
    }
}

void YourClassName::onLevelMaxChanged(int value) {
    displayLevelMax = sliderValueToLevel(value);
    if (displayLevelMax <= displayLevelMin) {
        displayLevelMax = (std::min)(sliderValueToLevel(MAX_LEVEL_SLIDER_VALUE), displayLevelMin + MIN_LEVEL_GAP);
        if (levelMaxSlider) {
            levelMaxSlider->blockSignals(true);
            levelMaxSlider->setValue(levelToSliderValue(displayLevelMax));
            levelMaxSlider->blockSignals(false);
        }
    }
    if (levelMinLabel) {
        levelMinLabel->setText(levelLabelText("Min", displayLevelMin));
    }
    if (levelMaxLabel) {
        levelMaxLabel->setText(levelLabelText("Max", displayLevelMax));
    }
    if (graphWidget) {
        graphWidget->setLevelRange(displayLevelMin, displayLevelMax);
    }
    if (waterfallWidget) {
        waterfallWidget->setLevelRange(displayLevelMin, displayLevelMax);
    }
}

void YourClassName::onWaterfallScaleChanged(int delta) {
    int value = scaleSlider->value();

    if (delta > 0) {
        value += scaleSlider->singleStep();
    } else {
        value -= scaleSlider->singleStep();
    }

    value = std::clamp(value, scaleSlider->minimum(), scaleSlider->maximum());

    if (value == scaleSlider->value()) {
        return;
    }

    scaleSlider->setValue(value);

    if (isNetworkClientMode() && !isFullIqProcessingMode()) {
        sendRemoteControlCommand("settings");
    }
}

void YourClassName::doubleGraphEnable(bool checked) {
    if (checked){
        secondGraph = true;
        qDebug()<<"secondgraph enabled";
    } else {
        secondGraph = false;
        qDebug()<<"secondgraph disabled";
    }
}

void YourClassName::colorGraphEnable(bool checked) {
    if (checked){
        colorf = true;
        qDebug()<<"color graph enabled";
    } else {
        colorf = false;
        qDebug()<<"color graph disabled";
    }
}

void YourClassName::syncEnable(bool checked) {
    Q_UNUSED(checked);
    pendingSettings.syncEnabled = false;
    if (syncCheckbox && syncCheckbox->isChecked()) {
        syncCheckbox->blockSignals(true);
        syncCheckbox->setChecked(false);
        syncCheckbox->blockSignals(false);
    }
    publishSettingsToGlobals();
    qDebug() << "Sync reader disabled; async reader is forced.";
}

void YourClassName::wheelEvent(QWheelEvent *event) {
    if (event->angleDelta().y() != 0) {
        QLineEdit *focusedLineEdit = qobject_cast<QLineEdit*>(focusWidget());
        if (focusedLineEdit) {
            bool ok;
            double currentValue = focusedLineEdit->text().toDouble(&ok);
            if (ok) {
                double delta = event->angleDelta().y() > 0 ? 1.0 : -1.0; 
                currentValue += delta;
                focusedLineEdit->setText(QString::number(currentValue, 'f', 0));

                focusedLineEdit->emit textEdited(focusedLineEdit->text()); 
            }
        }
    }
    QMainWindow::wheelEvent(event); 
}

void YourClassName::populateAudioDevices() {
    UINT numDevices = waveOutGetNumDevs();
    qDebug() << "Number of waveOut devices found:" << numDevices;

    for (UINT i = 0; i < numDevices; i++) {
        WAVEOUTCAPS caps;
        if (waveOutGetDevCaps(i, &caps, sizeof(WAVEOUTCAPS)) == MMSYSERR_NOERROR) {
            QString deviceName = QString::fromLocal8Bit(caps.szPname);  // Исправлено
            qDebug() << "Device" << i << ":" << deviceName;
            audioDeviceComboBox->addItem(deviceName, QVariant(i));
        }
    }
}

void YourClassName::onAudioDeviceChanged(int index) {
    if (index < 0) return;
    if (!isIdle()) {
        qDebug() << "Audio device change is locked while radio is running.";
        revertHardwareControlsToSettings();
        return;
    }

    QVariant data = audioDeviceComboBox->currentData();
    if (!data.isValid()) {
        qDebug() << "Error: Invalid audio device selected!";
        return;
    }

    // Получаем строку с именем устройства
    QString deviceName = data.toString();
    qDebug() << "Selected audio device:" << deviceName;
    pendingSettings.audioDeviceId = data.toInt();
    publishSettingsToGlobals();

    deviceID = pendingSettings.audioDeviceId;  // Передаём ID вместо имени
    qDebug() << "Selected audio device ID:" << deviceID;

    if (audioProcessor) {
        audioProcessor->setAudioDevice(deviceID);
    } else {
        qDebug() << "Error: audioProcessor is null!";
    }
}


void YourClassName::onBandwidthChanged() {
    bool ok = false;
    const double bandwidth = bandwidthLineEdit->text().toDouble(&ok);
    if (ok && bandwidth > 0.0) {
        pendingSettings.bandwidth = bandwidth;
        publishSettingsToGlobals();
        settingRange();
        if (isNetworkClientMode()) {
            sendRemoteControlCommand("settings");
        }
    }
}

void YourClassName::openNetworkSettingsDialog() {
    QDialog dialog(this);
    dialog.setWindowTitle("Network Settings");
    dialog.setMinimumWidth(420);

    QVBoxLayout *rootLayout = new QVBoxLayout(&dialog);
    QFormLayout *formLayout = new QFormLayout();

    QComboBox *modeCombo = new QComboBox(&dialog);
    modeCombo->addItem("Disabled", static_cast<int>(NetworkMode::Disabled));
    modeCombo->addItem("Server", static_cast<int>(NetworkMode::Server));
    modeCombo->addItem("Client", static_cast<int>(NetworkMode::Client));
    modeCombo->setCurrentIndex(modeCombo->findData(static_cast<int>(networkMode)));

    QComboBox *processingCombo = new QComboBox(&dialog);
    processingCombo->addItem("Server processing (spectrum/audio stream)", static_cast<int>(NetworkProcessingMode::ServerSide));
    processingCombo->addItem("Channel IQ + client demod", static_cast<int>(NetworkProcessingMode::ChannelIqClientSide));
    processingCombo->addItem("Full IQ client processing (LAN only)", static_cast<int>(NetworkProcessingMode::FullIqClientSide));
    processingCombo->setCurrentIndex(processingCombo->findData(static_cast<int>(networkProcessingMode)));

    QLineEdit *serverAddressEdit = new QLineEdit(networkServerAddress, &dialog);
    serverAddressEdit->setPlaceholderText("Server IP address");

    QLineEdit *bindAddressEdit = new QLineEdit(networkBindAddress, &dialog);
    bindAddressEdit->setPlaceholderText("0.0.0.0");

    QSpinBox *portSpin = new QSpinBox(&dialog);
    portSpin->setRange(1, 65535);
    portSpin->setValue(networkControlPort);

    QCheckBox *serverDisableLocalUiCheck = new QCheckBox("Disable local visual/audio on server when streaming is implemented", &dialog);
    serverDisableLocalUiCheck->setChecked(serverDisableLocalVisualAudio);

    QLabel *statusLabel = new QLabel(networkController ? networkController->statusText() : QString("Network controller unavailable"), &dialog);
    statusLabel->setWordWrap(true);

    formLayout->addRow("Mode:", modeCombo);
    formLayout->addRow("Processing:", processingCombo);
    formLayout->addRow("Server IP:", serverAddressEdit);
    formLayout->addRow("Bind address:", bindAddressEdit);
    formLayout->addRow("Control port:", portSpin);
    formLayout->addRow("", serverDisableLocalUiCheck);

    QPushButton *testButton = new QPushButton("Apply / Test Channel", &dialog);
    QPushButton *stopButton = new QPushButton("Stop Network", &dialog);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);

    QHBoxLayout *actionLayout = new QHBoxLayout();
    actionLayout->addWidget(testButton);
    actionLayout->addWidget(stopButton);

    rootLayout->addLayout(formLayout);
    rootLayout->addWidget(statusLabel);
    rootLayout->addLayout(actionLayout);
    rootLayout->addWidget(buttonBox);

    auto updateFieldState = [=]() {
        const auto selectedMode = static_cast<NetworkMode>(modeCombo->currentData().toInt());
        serverAddressEdit->setEnabled(selectedMode == NetworkMode::Client);
        bindAddressEdit->setEnabled(selectedMode == NetworkMode::Server);
        serverDisableLocalUiCheck->setEnabled(selectedMode != NetworkMode::Disabled);
        processingCombo->setEnabled(selectedMode != NetworkMode::Disabled);
        portSpin->setEnabled(selectedMode != NetworkMode::Disabled);
        testButton->setText(selectedMode == NetworkMode::Disabled ? "Apply" : "Apply / Test Channel");
    };
    connect(modeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), &dialog, updateFieldState);
    updateFieldState();

    QMetaObject::Connection statusConnection;
    if (networkController) {
        statusConnection = connect(networkController,
                                   &NetworkController::statusChanged,
                                   &dialog,
                                   [statusLabel](const QString &status) {
                                       statusLabel->setText(status);
                                   });
    }

    connect(testButton, &QPushButton::clicked, &dialog, [=]() {
        networkMode = static_cast<NetworkMode>(modeCombo->currentData().toInt());
        networkServerAddress = serverAddressEdit->text().trimmed().isEmpty()
                                   ? QString("127.0.0.1")
                                   : serverAddressEdit->text().trimmed();
        networkBindAddress = bindAddressEdit->text().trimmed().isEmpty()
                                 ? QString("0.0.0.0")
                                 : bindAddressEdit->text().trimmed();
        networkControlPort = static_cast<quint16>(portSpin->value());
        serverDisableLocalVisualAudio = serverDisableLocalUiCheck->isChecked();
        networkProcessingMode = static_cast<NetworkProcessingMode>(processingCombo->currentData().toInt());

        if (!networkController) {
            statusLabel->setText("Network controller unavailable");
            return;
        }

        if (networkMode == NetworkMode::Disabled) {
            if (remoteAudioPlayer) {
                remoteAudioPlayer->stop();
            }
            networkController->stop();
            return;
        }

        if (remoteAudioPlayer && networkMode != NetworkMode::Client) {
            remoteAudioPlayer->stop();
        }
        if (networkMode == NetworkMode::Server) {
            networkController->startServer(networkBindAddress, networkControlPort);
            return;
        }

        networkController->testClientConnection(networkServerAddress, networkControlPort);
    });

    connect(stopButton, &QPushButton::clicked, &dialog, [=]() {
        networkMode = NetworkMode::Disabled;
        if (modeCombo) {
            modeCombo->setCurrentIndex(modeCombo->findData(static_cast<int>(NetworkMode::Disabled)));
        }
        if (networkController) {
            networkController->stop();
        }
        if (remoteAudioPlayer) {
            remoteAudioPlayer->stop();
        }
    });

    connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    dialog.exec();
    if (networkController && statusConnection) {
        disconnect(statusConnection);
    }
}

void YourClassName::onNetworkStatusChanged(const QString &status) {
    qDebug() << "[Network]" << status;
    if (!networkButton) {
        return;
    }

    switch (networkMode) {
    case NetworkMode::Server:
        networkButton->setText(isChannelIqProcessingMode()
                                   ? "Network: Server ChIQ"
                                   : (isFullIqProcessingMode() ? "Network: Server IQ" : "Network: Server"));
        break;
    case NetworkMode::Client:
        networkButton->setText(isChannelIqProcessingMode()
                                   ? "Network: Client ChIQ"
                                   : (isFullIqProcessingMode() ? "Network: Client IQ" : "Network: Client"));
        break;
    case NetworkMode::Disabled:
    default:
        networkButton->setText("Network");
        break;
    }
}

void YourClassName::onfftLengthEntered() {
    const int newFftLength = fftComboBox->currentText().toInt();
    if (!isIdle()) {
        qDebug() << "Cannot change FFT length while processing is running.";
        revertHardwareControlsToSettings();
        return;
    }

    if (newFftLength <= 0) {
        qDebug() << "Invalid FFT length selected.";
        return;
    }

    pendingSettings.fftLength = newFftLength;
    publishSettingsToGlobals();
    updateSpectrumTimerInterval();
    fftResult = std::make_unique<FFTResult>();
    if (isNetworkClientMode()) {
        sendRemoteControlCommand("settings");
    }
}

QString formatSampleRate(double sampleRate) {
    QString formattedRate;
    if (sampleRate >= 1e9) {
        formattedRate = QString::number(sampleRate / 1e9, 'f', 2) + " GHz";
    } else if (sampleRate >= 1e6) {
        formattedRate = QString::number(sampleRate / 1e6, 'f', 2) + " MHz";
    } else if (sampleRate >= 1e3) {
        formattedRate = QString::number(sampleRate / 1e3, 'f', 2) + " kHz";
    } else {
        formattedRate = QString::number(sampleRate, 'f', 2) + " Hz";
    }

    return formattedRate;
}

void YourClassName::populateSampleRates() {
    auto addDefaultSampleRates = [this]() {
        if (!sampleBox || sampleBox->count() > 0) {
            return;
        }
        const QVector<double> defaultRates = {
            8000000.0, 10000000.0, 12500000.0, 16000000.0, 20000000.0,
            25000000.0, 32000000.0, 40000000.0, 50000000.0, 80000000.0
        };
        for (const double rate : defaultRates) {
            sampleBox->addItem(formatSampleRate(rate), rate);
        }
        const int defaultIndex = sampleBox->findData(pendingSettings.sampleRate);
        if (defaultIndex >= 0) {
            sampleBox->setCurrentIndex(defaultIndex);
        }
        qDebug() << "[Network] using fallback sample-rate list; no local Fobos device is required for client control";
    };

    fobos_dev_t* sampleRateDevice = device;
    bool openedForSampleRates = false;

    int ret = FOBOS_ERR_OK;
    if (!sampleRateDevice) {
        ret = fobos_rx_open(&sampleRateDevice, 0);
        openedForSampleRates = (ret == FOBOS_ERR_OK && sampleRateDevice);
    }
    if (ret != FOBOS_ERR_OK) {
        qDebug() << "Failed to open device, error code:" << ret;
        addDefaultSampleRates();
        return;
    }
    if (!sampleRateDevice) {
        qDebug() << "Device is not initialized.";
        addDefaultSampleRates();
        return;
    }
    double sampleRates[100];
    unsigned int count = 100;
    int result = fobos_rx_get_samplerates(sampleRateDevice, sampleRates, &count);
    if (result != FOBOS_ERR_OK) {
        qDebug() << "Failed to get sample rates, error code:" << result;
        if (openedForSampleRates) {
            closeFobosDeviceSafely(sampleRateDevice);
        }
        addDefaultSampleRates();
        return;
    }
    sampleBox->clear();
    for (unsigned int i = 0; i < count; ++i) {
        QString formattedRate = formatSampleRate(sampleRates[i]);
        sampleBox->addItem(formattedRate, sampleRates[i]);
    }
    if (openedForSampleRates) {
        closeFobosDeviceSafely(sampleRateDevice);
    }
}

void YourClassName::updateSpectrum() {
    if (!fftResult) {
        return;
    }

    const bool traceFrame = spectrumDebugFramesRemaining > 0;
    QElapsedTimer traceTimer;
    if (traceFrame) {
        traceTimer.start();
        qDebug() << "[Spectrum] update begin"
                 << "framesLeft" << spectrumDebugFramesRemaining
                 << "sampleRate" << pendingSettings.sampleRate
                 << "fftLength" << pendingSettings.fftLength
                 << "iqSnapshotFloats" << IqBuffer::size()
                 << "queuedBlocks" << IqBuffer::queuedBlocks();
    }

    auto finishTrace = [&](const char *stage,
                           const std::vector<float> &frequencies,
                           const std::vector<float> &magnitudes) {
        if (!traceFrame) {
            return;
        }
        qDebug() << "[Spectrum] update" << stage
                 << "elapsedMs" << traceTimer.elapsed()
                 << "freqCount" << frequencies.size()
                 << "magCount" << magnitudes.size()
                 << "iqSnapshotFloats" << IqBuffer::size()
                 << "queuedBlocks" << IqBuffer::queuedBlocks();
        --spectrumDebugFramesRemaining;
    };

    //dataq = new float[dataSize];
        //for (int i = 0; i < 8; ++i){
        //int setrf = fobos_rx_set_frequency(device, globalFrequency + globalSampleRate * i, &actualFrequency);
        //memcpy(iqData + i * DEFAULT_BUF_LEN/8, dataq, DEFAULT_BUF_LEN/8 * sizeof(float));
        //}
    std::vector<float> spectrumFrequencies;
    std::vector<float> spectrumMagnitudes;
    bool haveSpectrum = false;
    try {
        haveSpectrum = fftResult->storeFFTResults(pendingSettings, spectrumFrequencies, spectrumMagnitudes);
    } catch (const std::bad_alloc &error) {
        qCritical() << "[Spectrum] bad_alloc" << error.what()
                    << "sampleRate" << pendingSettings.sampleRate
                    << "fftLength" << pendingSettings.fftLength;
        updateTimer->stop();
        finishTrace("bad_alloc", spectrumFrequencies, spectrumMagnitudes);
        return;
    } catch (const std::exception &error) {
        qCritical() << "[Spectrum] exception" << error.what();
        finishTrace("exception", spectrumFrequencies, spectrumMagnitudes);
        return;
    } catch (...) {
        qCritical() << "[Spectrum] unknown exception";
        finishTrace("unknown_exception", spectrumFrequencies, spectrumMagnitudes);
        return;
    }

    if (!haveSpectrum || spectrumFrequencies.empty() || spectrumMagnitudes.empty()) {
        finishTrace("no_data", spectrumFrequencies, spectrumMagnitudes);
        return;
    }
    if (traceFrame) {
        qDebug() << "[Spectrum] before graph"
                 << "elapsedMs" << traceTimer.elapsed()
                 << "freqCount" << spectrumFrequencies.size()
                 << "magCount" << spectrumMagnitudes.size();
    }
    const bool suppressLocalVisual =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    if (!suppressLocalVisual) {
        graphWidget->setLevelRange(displayLevelMin, displayLevelMax);
        graphWidget->setData(spectrumFrequencies, spectrumMagnitudes, minFrequency, maxFrequency, pendingSettings.fftLength, colorf);
        if (traceFrame) {
            qDebug() << "[Spectrum] before waterfall" << "elapsedMs" << traceTimer.elapsed();
        }
        waterfallWidget->setData(spectrumFrequencies, spectrumMagnitudes, minFrequency, maxFrequency, pendingSettings.fftLength, secondGraph, contrast, sensitivity, displayLevelMin, displayLevelMax);
    } else if (traceFrame) {
        qDebug() << "[Spectrum] local server visual update skipped" << "elapsedMs" << traceTimer.elapsed();
    }
    sendNetworkSpectrumFrame(spectrumFrequencies, spectrumMagnitudes);
    finishTrace("end", spectrumFrequencies, spectrumMagnitudes);
    //waterfallWidget->setData(fftFrequencies, fftMagnitudes, minFrequency, maxFrequency, fftLength, secondGraph, contrast, sensitivity);
    //qDebug() << "all took" << timer.elapsed() << "milliseconds";
}
 
void YourClassName::onSampleRateChanged(int index) {
    qDebug() << "[FobosLifecycle] onSampleRateChanged enter"
             << "index" << index
             << "state" << runStateName(runState)
             << "deviceOpened" << deviceOpened
             << "processorRunning" << (processor && processor->isRunning());
    if (!isIdle()) {
        qDebug() << "Stop processing before changing sample rate.";
        revertHardwareControlsToSettings();
        return;
    }

    bool ok = false;
    double selectedSampleRate = sampleBox->currentData().toDouble(&ok);
    if (!ok) {
        qDebug() << "Invalid sample rate selected.";
        fftResult = std::make_unique<FFTResult>();

        return;
    }
    const bool sampleRateChanged = std::abs(pendingSettings.sampleRate - selectedSampleRate) > 0.5;
    qDebug() << "[FobosLifecycle] sample rate selected"
             << "previous" << pendingSettings.sampleRate
             << "selected" << selectedSampleRate
             << "changed" << sampleRateChanged
             << "device" << device;
    pendingSettings.sampleRate = selectedSampleRate;
    if (sampleRateChanged && device) {
        const bool selectedRateMatchesOpenSession =
            appliedSampleRate > 0.0 && std::abs(appliedSampleRate - selectedSampleRate) <= 0.5;
        sampleRateReopenRequired = !selectedRateMatchesOpenSession;
        qDebug() << "[FobosLifecycle] sample rate changed while Fobos session is open"
                 << "selectedRateMatchesOpenSession" << selectedRateMatchesOpenSession
                 << "sampleRateReopenRequired" << sampleRateReopenRequired
                 << "willReopenOnNextStart" << sampleRateReopenRequired;
    } else if (!device) {
        sampleRateReopenRequired = false;
    }
    publishSettingsToGlobals();
    qDebug() << "Attempting to set sample rate to:" << selectedSampleRate;
    qDebug() << "Sample rate will be applied on the next start.";
    normalizeTuning(pendingSettings);
    publishSettingsToGlobals();
    settingRange();
    if (isNetworkClientMode()) {
        sendRemoteControlCommand("settings");
    }
}

void YourClassName::onListeningFrequencyEntered() {
    if (listeningFrequencyLineEdit) {
        bool ok;
        double frequency = listeningFrequencyLineEdit->text().toDouble(&ok);
        if (!ok) {
        qDebug() << "Invalid frequency entered.";
        return;
    }
        if (ok) {
         pendingSettings.listeningFrequency = frequency;
         normalizeTuning(pendingSettings);
         publishSettingsToGlobals();
         if (frequencyLineEdit) {
             frequencyLineEdit->setText(QString::number(pendingSettings.centerFrequency, 'f', 0));
         }
         listeningFrequencyLineEdit->setText(QString::number(pendingSettings.listeningFrequency, 'f', 0));
         qDebug() << "Frequency set to" << listeningFrequency << "Hz";
     }
 settingRange();
 if (isNetworkClientMode()) {
     sendRemoteControlCommand("settings");
 }
}
}    
    
void YourClassName::onFrequencyEntered() {
    if (pendingSettings.inputMode == 0) {
    if (frequencyLineEdit) {
        bool ok;
        double frequency = frequencyLineEdit->text().toDouble(&ok);
        if (!ok) {
        qDebug() << "Invalid frequency entered.";
        return;
    }
        if (ok) {
            pendingSettings.centerFrequency = frequency;
            normalizeTuning(pendingSettings, true);
            if (!isIdle() && device) {
                double tunedFrequency = pendingSettings.centerFrequency;
                const int result = setFobosFrequencySafely(device, pendingSettings.centerFrequency, &tunedFrequency);
                if (result == FOBOS_ERR_OK) {
                    pendingSettings.actualFrequency = tunedFrequency;
                    if (hardwareSettingsApplied) {
                        appliedHardwareSettings.centerFrequency = pendingSettings.centerFrequency;
                        appliedHardwareSettings.actualFrequency = tunedFrequency;
                    }
                } else {
                    qDebug() << "Failed to tune frequency while running, error code:" << result;
                }
            }
            publishSettingsToGlobals();
            QString frequencyStr = QString::number(globalFrequency, 'f', 0);
            frequencyLineEdit->setText(frequencyStr);
            if (listeningFrequencyLineEdit) {
                listeningFrequencyLineEdit->setText(QString::number(pendingSettings.listeningFrequency, 'f', 0));
            }
            qDebug() << "Frequency set to" << globalFrequency << "Hz";
        }
        }
    }
    else { frequencyLineEdit->setText("000000000"); 
        listeningFrequencyLineEdit->setText("1250000");
        pendingSettings.listeningFrequency = 1250000;
        pendingSettings.centerFrequency = 0;
        normalizeTuning(pendingSettings);
        publishSettingsToGlobals();
        }
settingRange();
if (isNetworkClientMode()) {
    sendRemoteControlCommand("settings");
}
}

QStringList YourClassName::getFobosDevices() {
    QStringList deviceList;
    if (deviceOpened) {
        deviceList << "Stop processing before refreshing devices";
        return deviceList;
    }
    int deviceCount = fobos_rx_get_device_count();
    if (deviceCount == 0) {
        deviceList << "Failed to initialize libfobos";
        return deviceList;
    }
    QString deviceInfo = "Number of Fobos devices connected: " + QString::number(deviceCount) + "\n";
    for (int i = 0; i < deviceCount; ++i) {
        char hvrev[256], fvver[256], manuf[256], prod[256], serialNumber[256];
        fobos_dev_t* tempDevice = nullptr;
        const bool useOpenDevice = device && comboBox && i == comboBox->currentIndex();
        int result = useOpenDevice ? FOBOS_ERR_OK : fobos_rx_open(&tempDevice, i);
        fobos_dev_t* infoDevice = useOpenDevice ? device : tempDevice;
        if (result == FOBOS_ERR_OK && infoDevice) {
            if (fobos_rx_get_board_info(infoDevice, hvrev, fvver, manuf, prod, serialNumber) == FOBOS_ERR_OK) {
                deviceList << QString("Device %1: Serial Number: %2").arg(i).arg(serialNumber);
            } else {
                deviceList << QString("Failed to get serial number for device %1").arg(i);
            }
            if (tempDevice) {
                closeFobosDeviceSafely(tempDevice);
            }
        } else {
            deviceInfo += "Failed to open device " + QString::number(i) + "\n";
        }
    }
    return deviceList;
}

void YourClassName::listFobosDevices() {
    if (deviceOpened || (processor && processor->isRunning())) {
        QMessageBox::information(this, "Devices", "Stop processing before listing devices.");
        return;
    }

    int deviceCount = fobos_rx_get_device_count();
    if (deviceCount == 0) {
        QMessageBox::information(this, "Devices", "No Fobos devices connected.");
    } else {
        QString deviceInfo = "Number of Fobos devices connected: " + QString::number(deviceCount) + "\n";
        for (int i = 0; i < deviceCount; ++i) {
            char hvrev[256], fvver[256], manuf[256], prod[256], serialNumber[256], libv[256], apiv[256];
            fobos_dev_t* infoDevice = nullptr;
            int openResult = fobos_rx_open(&infoDevice, i);
            if (openResult == FOBOS_ERR_OK && infoDevice) {
                if (fobos_rx_get_board_info(infoDevice, hvrev, fvver, manuf, prod, serialNumber) == FOBOS_ERR_OK &&
                    fobos_rx_get_api_info(libv, apiv) == FOBOS_ERR_OK) {
                    deviceInfo += QString("Device %1: hardware revision %2, firmware version %3, manufacturer %4, Product %5, Serial Number: %6, library version %7, API version %8\n")
                        .arg(i).arg(hvrev).arg(fvver).arg(manuf).arg(prod).arg(serialNumber).arg(libv).arg(apiv);
                } else {
                    deviceInfo += "Failed to read device info " + QString::number(i) + "\n";
                }
                closeFobosDeviceSafely(infoDevice);
            } else {
                deviceInfo += "Failed to open device " + QString::number(i) + "\n";
            }
        }
        QMessageBox::information(this, "Fobos Devices", deviceInfo);
    }
}

void YourClassName::onDirectSamplingChanged(int index) {
    if (!isIdle()) {
        qDebug() << "Stop processing before changing input mode.";
        revertHardwareControlsToSettings();
        return;
    }

    int value = modeBox->currentData().toInt();
    pendingSettings.inputMode = value;
    if (value != 0){
    pendingSettings.centerFrequency = 0;
    pendingSettings.listeningFrequency = 1250000;
    normalizeTuning(pendingSettings);
    frequencyLineEdit->setText(QString::number(pendingSettings.centerFrequency, 'f', 0));
    listeningFrequencyLineEdit->setText(QString::number(pendingSettings.listeningFrequency, 'f', 0));
    publishSettingsToGlobals();
    }
      else  if (value == 0){
    pendingSettings.listeningFrequency = 100000000;
    pendingSettings.centerFrequency = 100000000;
    normalizeTuning(pendingSettings);
    frequencyLineEdit->setText(QString::number(pendingSettings.centerFrequency, 'f', 0));
    listeningFrequencyLineEdit->setText(QString::number(pendingSettings.listeningFrequency, 'f', 0));
    publishSettingsToGlobals();
    }
    qDebug() << "Current mode set to" << globalMode;
    qDebug() << "Input mode will be applied on the next start.";
    settingRange();
    if (isNetworkClientMode()) {
        sendRemoteControlCommand("settings");
    }
}

void YourClassName::settingRange() {
    if (!scaleWidget || globalSampleRate <= 0.0) {
        return;
    }

    double newRange = globalSampleRate * (currentScale / 100.0);
    double overallMin = DIRECT_MIN_FREQUENCY;
    double overallMax = directMaxFrequency(globalSampleRate);

    if (globalMode == 0) {
        overallMin = (std::max)(RF_MIN_LISTENING_FREQUENCY,
                                globalFrequency - globalSampleRate / 2.0);
        overallMax = (std::max)(overallMin,
                                globalFrequency + globalSampleRate / 2.0);
    }

    const double availableRange = (std::max)(1.0, overallMax - overallMin);
    newRange = (std::clamp)(newRange, 1.0, availableRange);

    double clampedListening = (std::clamp)(listeningFrequency, overallMin, overallMax);
	double newMin = clampedListening - newRange / 2.0;
    newMin = (std::clamp)(newMin, overallMin, overallMax - newRange);
    double newMax = newMin + newRange;
    minFrequency = newMin;
    maxFrequency = newMax;
    scaleWidget->setTuning(clampedListening, globalFrequency, globalBandwidth, globalModulationType);
    scaleWidget->setRange(minFrequency, maxFrequency);
}

void YourClassName::onCheckboxStateChanged(int state) {
    QCheckBox *senderCheckbox = qobject_cast<QCheckBox*>(sender());
    if (senderCheckbox) {
        const uint8_t value = currentGpoValue();
        pendingSettings.gpoValue = value;
        qDebug() << "Checkbox state changed. New GPO value:" << value;
        qDebug() << "GPO state will be applied on the next start.";
        if (isNetworkClientMode()) {
            sendRemoteControlCommand("settings");
        }
    }
}

void YourClassName::onLnaGainChanged(int value) {
    pendingSettings.lnaGain = value;
    lnaGainLabel->setText(QString("LNA Gain: %1").arg(value));
    qDebug() << "LNA gain will be applied on the next start.";
    if (isNetworkClientMode()) {
        sendRemoteControlCommand("settings");
    }
}

void YourClassName::onVgaGainChanged(int value) {
    pendingSettings.vgaGain = value;
    vgaGainLabel->setText(QString("VGA Gain: %1").arg(value));
    qDebug() << "VGA gain will be applied on the next start.";
    if (isNetworkClientMode()) {
        sendRemoteControlCommand("settings");
    }
}

void YourClassName::onClkChanged(int index) {
    if (!isIdle()) {
        qDebug() << "Stop processing before changing clock source.";
        revertHardwareControlsToSettings();
        return;
    }

    pendingSettings.clockSource = clkBox->currentData().toInt();
    qDebug() << "Clock source will be applied on the next start.";
    if (isNetworkClientMode()) {
        sendRemoteControlCommand("settings");
    }
}

void YourClassName::startFobosProcessing() {
    if (isNetworkClientMode()) {
        if (sendRemoteControlCommand("start")) {
            runState = RadioRunState::Running;
            updateUiForRunState();
            if (isClientIqProcessingMode()) {
                startNetworkClientProcessing();
            } else if (remoteAudioPlayer) {
                remoteAudioPlayer->stop();
            }
        }
        return;
    }

    const bool watchdogRestart = automaticStreamRestart;
    automaticStreamRestart = false;
    if (!watchdogRestart) {
        streamStartupRetryCount = 0;
        restartAfterStartupWatchdog = false;
    }
    qDebug() << "[FobosLifecycle] Start requested"
             << "state" << runStateName(runState)
             << "deviceOpened" << deviceOpened
             << "processorRunning" << (processor && processor->isRunning())
             << "device" << device
             << "appliedSampleRate" << appliedSampleRate
             << "pendingSampleRate" << pendingSettings.sampleRate
             << "sampleRateReopenRequired" << sampleRateReopenRequired
             << "fobosCloseKnownUnsafe" << fobosCloseKnownUnsafe
             << "watchdogRestart" << watchdogRestart
             << "startupRetryCount" << streamStartupRetryCount;
    logMemorySnapshot("before start");
    if (!isIdle() || deviceOpened || (processor && processor->isRunning())) {
        qDebug() << "Warning: Processor is already running!";
        return;
    }

    runState = RadioRunState::Starting;
    qDebug() << "[FobosLifecycle] state changed" << runStateName(runState);
    updateUiForRunState();
    refreshSettingsFromUi();
    const bool sampleRateDiffersFromOpenSession =
        device && appliedSampleRate > 0.0 &&
        std::abs(appliedSampleRate - pendingSettings.sampleRate) > 0.5;
    if (sampleRateDiffersFromOpenSession) {
        sampleRateReopenRequired = true;
        qDebug() << "[FobosLifecycle] sample rate differs from previous run; reopening Fobos session before applying"
                 << "appliedSampleRate" << appliedSampleRate
                 << "pendingSampleRate" << pendingSettings.sampleRate;
    }
    publishSettingsToGlobals();

    if (device && sampleRateReopenRequired) {
        if (fobosCloseKnownUnsafe) {
            qDebug() << "[FobosLifecycle] previous Fobos close was unsafe; abandoning stale session pointer before reopen"
                     << device;
            device = nullptr;
            openedDeviceIndex = -1;
            appliedSampleRate = 0.0;
            appliedHardwareSettings = RadioSettings{};
            hardwareSettingsApplied = false;
            sampleRateReopenRequired = false;
            fobosCloseKnownUnsafe = false;
        }
    }

    if (device && sampleRateReopenRequired) {
        if (processor && !processor->isRunning()) {
            processor->finalizeStopped();
        }
        qDebug() << "[FobosLifecycle] closing Fobos session before sample-rate change"
                 << "device" << device
                 << "appliedSampleRate" << appliedSampleRate
                 << "pendingSampleRate" << pendingSettings.sampleRate;
        if (!closeFobosSession(false)) {
            qDebug() << "[FobosLifecycle] Fobos session close returned an error before sample-rate change; trying a fresh open anyway";
        }
        qDebug() << "[FobosLifecycle] Fobos session closed for sample-rate change; waiting before reopen";
        QThread::msleep(500);
    }

    qDebug() << "[FobosLifecycle] opening Fobos session";
    if (!openFobosSession()) {
        qDebug() << "[FobosLifecycle] openFobosSession failed";
        runState = RadioRunState::Idle;
        updateUiForRunState();
        return;
    }

    qDebug() << "[FobosLifecycle] applying Fobos settings";
    if (!applyFobosSettings()) {
        qDebug() << "Start aborted because Fobos settings could not be applied; closing Fobos session before retry.";
        closeFobosSession(false);
        runState = RadioRunState::Idle;
        updateUiForRunState();
        return;
    }
    fftResult = std::make_unique<FFTResult>();
    spectrumDebugFramesRemaining = fobosVerboseLoggingEnabled() ? 12 : 0;
    updateSpectrumTimerInterval();
    fftComboBox->setEnabled(false);
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[FobosLifecycle] clearing IQ buffer before reader start";
    }
    IqBuffer::clear();

    deviceOpened = true;
    const bool serverIqStreaming = networkMode == NetworkMode::Server && isClientIqProcessingMode();
    const bool serverFullIqStreaming = networkMode == NetworkMode::Server && isFullIqProcessingMode();
    const bool serverChannelIqStreaming = networkMode == NetworkMode::Server && isChannelIqProcessingMode();
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    const bool serverLocalAudioEnabled = pendingSettings.audioEnabled && !serverIqStreaming;
    const bool queueAudioBlocks = serverLocalAudioEnabled;
    if (audioProcessor) {
        audioProcessor->setLocalPlaybackEnabled(!suppressServerLocalOutput);
    }
    if (serverIqStreaming && processor) {
        processor->updateNetworkIqSettings(pendingSettings, serverChannelIqStreaming);
    }
    qDebug() << "[FobosLifecycle] starting DataProcessor"
             << "device" << device
             << "sampleRate" << pendingSettings.sampleRate
             << "syncEnabled" << pendingSettings.syncEnabled
             << "queueAudioBlocks" << queueAudioBlocks
             << "serverIqStreaming" << serverIqStreaming
             << "serverChannelIqStreaming" << serverChannelIqStreaming;
    processor->startProcessing(device,
                               pendingSettings.syncEnabled,
                               pendingSettings.sampleRate,
                               queueAudioBlocks,
                               serverIqStreaming);
    pendingAudioStartAfterStreamReady = serverLocalAudioEnabled;
    streamStartCallbackCount = processor ? processor->callbackCount() : 0;
    streamStartElapsedTimer.restart();
    if (streamWatchdogTimer) {
        qDebug() << "[FobosLifecycle] stream startup watchdog armed"
                 << "callbackCount" << streamStartCallbackCount
                 << "retryCount" << streamStartupRetryCount;
        streamWatchdogTimer->start();
    }
    runState = RadioRunState::Running;
    qDebug() << "[FobosLifecycle] state changed" << runStateName(runState)
             << "processorRunning" << (processor && processor->isRunning());
    updateUiForRunState();
    qDebug() << "Fobos fft Started";

    if (serverLocalAudioEnabled) {
        qDebug() << "[FobosLifecycle] audio start deferred until stream produces IQ callbacks";
    } else if (serverChannelIqStreaming) {
        qDebug() << "[NetworkIQ] server is streaming channelized IQ frames; local server audio is disabled";
    } else if (serverFullIqStreaming) {
        qDebug() << "[NetworkIQ] server is streaming full IQ frames; local server audio is disabled";
    }

    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[FobosLifecycle] starting spectrum timer";
    }
    if (serverFullIqStreaming && suppressServerLocalOutput) {
        qDebug() << "[NetworkIQ] server local spectrum timer disabled in full-IQ processing mode";
    } else {
        updateTimer->start();
    }
    logMemorySnapshot("after start");
    qDebug() << "[FobosLifecycle] Start sequence complete";
}

void YourClassName::recreateDataProcessor() {
    qDebug() << "[FobosLifecycle] recreateDataProcessor enter"
             << "processor" << processor
             << "processorRunning" << (processor && processor->isRunning());
    if (processor) {
        delete processor;
    }
    processor = new DataProcessor(this);
    connectDataProcessorSignals();
    qDebug() << "[FobosLifecycle] recreateDataProcessor exit" << "processor" << processor;
}

void YourClassName::finishFobosStop(bool forcedRecovery) {
    qDebug() << "[FobosLifecycle] finishFobosStop enter"
             << "forcedRecovery" << forcedRecovery
             << "processorRunning" << (processor && processor->isRunning())
             << "device" << device;

    if (stopPollTimer) {
        stopPollTimer->stop();
    }
    if (streamWatchdogTimer) {
        streamWatchdogTimer->stop();
    }

    if (processor) {
        processor->finalizeStopped();
    }

    if (forcedRecovery) {
        qDebug() << "[FobosLifecycle] forced stop recovery: abandoning Fobos session without close and recreating DataProcessor"
                 << "device" << device;
        device = nullptr;
        openedDeviceIndex = -1;
        appliedSampleRate = 0.0;
        appliedHardwareSettings = RadioSettings{};
        hardwareSettingsApplied = false;
        sampleRateReopenRequired = false;
        fobosCloseKnownUnsafe = false;
        if (processor && !processor->isRunning()) {
            recreateDataProcessor();
        }
    } else if (device) {
        qDebug() << "[FobosLifecycle] clean stop: closing Fobos session; IQ snapshot remains visible";
        const bool closed = closeFobosSession(false);
        if (!closed) {
            qDebug() << "[FobosLifecycle] clean stop: Fobos close returned an error; stale session pointer was abandoned";
        }
        if (processor && !processor->isRunning()) {
            recreateDataProcessor();
        }
    }

    deviceOpened = false;
    runState = RadioRunState::Idle;
    qDebug() << "[FobosLifecycle] state changed" << runStateName(runState);
    updateUiForRunState();
    logMemorySnapshot("after stop");
    qDebug() << "Stop requested: complete.";

    if (restartAfterStartupWatchdog && !forcedRecovery) {
        restartAfterStartupWatchdog = false;
        automaticStreamRestart = true;
        hardwareSettingsApplied = false;
        appliedHardwareSettings = RadioSettings{};
        appliedSampleRate = 0.0;
        qDebug() << "[FobosLifecycle] scheduling automatic restart after stream startup watchdog"
                 << "retryCount" << streamStartupRetryCount;
        QTimer::singleShot(350, this, &YourClassName::startFobosProcessing);
    }
}

void YourClassName::checkStreamStartup() {
    if (runState != RadioRunState::Running || !processor || !processor->isRunning()) {
        if (streamWatchdogTimer) {
            streamWatchdogTimer->stop();
        }
        return;
    }

    const uint64_t callbackCount = processor->callbackCount();
    if (callbackCount > streamStartCallbackCount) {
        qDebug() << "[FobosLifecycle] stream startup watchdog satisfied"
                 << "callbackCount" << callbackCount
                 << "elapsedMs" << streamStartElapsedTimer.elapsed();
        if (streamWatchdogTimer) {
            streamWatchdogTimer->stop();
        }
        if (pendingAudioStartAfterStreamReady) {
            pendingAudioStartAfterStreamReady = false;
            if (deviceOpened && processor && processor->isRunning()) {
                qDebug() << "[FobosLifecycle] starting deferred audio after IQ stream became active";
                audioProcessor->startDemodulation();
                qDebug() << "Fobos audio Started";
            } else {
                qDebug() << "[FobosLifecycle] deferred audio start skipped because SDR is no longer running";
            }
        }
        return;
    }

    const qint64 elapsedMs = streamStartElapsedTimer.isValid() ? streamStartElapsedTimer.elapsed() : 0;
    if (elapsedMs < 1800) {
        return;
    }

    qDebug() << "[FobosLifecycle] stream startup watchdog: no IQ callbacks after start"
             << "elapsedMs" << elapsedMs
             << "callbackCount" << callbackCount
             << "retryCount" << streamStartupRetryCount
             << "device" << device
             << "sampleRate" << pendingSettings.sampleRate;
    if (streamWatchdogTimer) {
        streamWatchdogTimer->stop();
    }
    pendingAudioStartAfterStreamReady = false;

    if (streamStartupRetryCount < 1) {
        ++streamStartupRetryCount;
        restartAfterStartupWatchdog = true;
        qDebug() << "[FobosLifecycle] stream startup watchdog will retry once"
                 << "retryCount" << streamStartupRetryCount;
    } else {
        restartAfterStartupWatchdog = false;
        qDebug() << "[FobosLifecycle] stream startup watchdog retry already used; leaving receiver stopped";
    }

    stopFobosProcessing();
}

void YourClassName::pollStopCompletion() {
    if (!processor) {
        qDebug() << "[FobosLifecycle] pollStopCompletion: no processor";
        finishFobosStop(false);
        return;
    }

    if (!processor->isRunning()) {
        qDebug() << "[FobosLifecycle] DataProcessor stopped asynchronously"
                 << "elapsedMs" << stopElapsedTimer.elapsed();
        finishFobosStop(false);
        return;
    }

    const qint64 elapsedMs = stopElapsedTimer.isValid() ? stopElapsedTimer.elapsed() : 0;
    if (stopCancelRetryCount < 4 && elapsedMs >= (stopCancelRetryCount + 1) * 1000) {
        ++stopCancelRetryCount;
        qDebug() << "[FobosLifecycle] retrying async cancel"
                 << "retry" << stopCancelRetryCount
                 << "elapsedMs" << elapsedMs;
        processor->requestStop();
        return;
    }

    if (elapsedMs < 6000) {
        return;
    }

    qDebug() << "[FobosLifecycle] DataProcessor did not stop after async cancel; forcing recovery"
             << "elapsedMs" << elapsedMs;
    if (stopPollTimer) {
        stopPollTimer->stop();
    }

    const bool forced = processor->forceStop(1000);
    qDebug() << "[FobosLifecycle] forced recovery result" << forced;
    if (forced) {
        finishFobosStop(true);
        return;
    }

    qDebug() << "[FobosLifecycle] forced recovery failed; UI remains in Stopping to avoid unsafe restart";
    updateUiForRunState();
}
 
void YourClassName::stopFobosProcessing() {
    if (isNetworkClientMode()) {
        if (sendRemoteControlCommand("stop")) {
            if (isClientIqProcessingMode()) {
                stopNetworkClientProcessing();
            } else if (remoteAudioPlayer) {
                remoteAudioPlayer->stop();
            }
            runState = RadioRunState::Idle;
            updateUiForRunState();
        }
        return;
    }

    qDebug() << "[FobosLifecycle] Stop requested"
             << "state" << runStateName(runState)
             << "deviceOpened" << deviceOpened
             << "processorRunning" << (processor && processor->isRunning())
             << "device" << device;
    logMemorySnapshot("before stop");
    if (runState == RadioRunState::Idle && !deviceOpened && !(processor && processor->isRunning())) {
        qDebug() << "Stop ignored because radio is already idle.";
        return;
    }
    if (runState == RadioRunState::Stopping) {
        qDebug() << "Stop ignored because radio is already stopping.";
        return;
    }

    runState = RadioRunState::Stopping;
    qDebug() << "[FobosLifecycle] state changed" << runStateName(runState);
    updateUiForRunState();
    if (streamWatchdogTimer) {
        streamWatchdogTimer->stop();
    }
    pendingAudioStartAfterStreamReady = false;
    updateTimer->stop();
    qDebug() << "Stop requested: spectrum timer stopped.";

    if (audioProcessor) {
        qDebug() << "Stop requested: stopping AudioProcessor.";
        audioProcessor->stopDemodulation();
        qDebug() << "Stop requested: AudioProcessor stopped.";
    }

    if (processor && (deviceOpened || processor->isRunning())) {
        qDebug() << "Stop requested: requesting DataProcessor stop.";
        stopCancelRetryCount = 0;
        processor->requestStop();
        stopElapsedTimer.restart();
        if (stopPollTimer) {
            stopPollTimer->start();
        }
        return;
    }

    finishFobosStop(false);
}

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    installDiagnosticLogger();
    installCrashLogger();
    logFobosApiInfo();
    YourClassName window;
    window.show(); 

    qDebug() << "App started";
        SetConsoleOutputCP(CP_UTF8);  // Устанавливаем UTF-8 для вывода
        SetConsoleCP(CP_UTF8);        // Устанавливаем UTF-8 для ввода
        return app.exec();
}
