#include "main.h"
#include "iqbuffer.h"
#include "diagnosticlogging.h"

#include <fobos_sdr.h>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QFormLayout>
#include <QGridLayout>
#include <QJsonArray>
#include <QJsonObject>
#include <QList>
#include <QMenu>
#include <QMessageLogContext>
#include <QMutexLocker>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTextStream>
#include <QTextCursor>
#include <QPushButton>
#include <QKeyEvent>
#include <QAbstractButton>
#include <QColor>
#include <QCoreApplication>
#include <QSettings>
#include <cmath>
#include <limits>
#include <psapi.h>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <new>
#include <utility>

#ifdef _MSC_VER
#pragma comment(lib, "psapi.lib")
#endif

fobos_dev_t* device = nullptr;
fobos_sdr_dev_t* agileDevice = nullptr;
FobosApiKind activeFobosApiKind = FobosApiKind::Standard;
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
constexpr int NETWORK_CHANNEL_SPECTRUM_MAX_BINS = 768;
constexpr int NETWORK_SPECTRUM_INTERVAL_MS = 100;
constexpr int NETWORK_CHANNEL_SPECTRUM_INTERVAL_MS = 160;
constexpr qint64 NETWORK_IQ_MAX_PENDING_BYTES = 8 * 1024 * 1024;
constexpr uint64_t NETWORK_IQ_DROP_LOG_INTERVAL = 200;
constexpr double NETWORK_AUDIO_PREBUFFER_SECONDS = 0.55;
constexpr qint64 NETWORK_SPECTRUM_MAX_PENDING_BYTES = 4 * 1024 * 1024;
constexpr int AUDIO_LOW_PASS_SLIDER_STEP_HZ = 100;
constexpr int AUDIO_LOW_PASS_SLIDER_MAX = 200;
constexpr int AUDIO_HIGH_PASS_SLIDER_STEP_HZ = 25;
constexpr int AUDIO_HIGH_PASS_SLIDER_MAX = 40;
constexpr int VIDEO_SNAPSHOT_INTERVAL_MS = 90;
constexpr std::size_t VIDEO_SNAPSHOT_MAX_FLOATS = 262144 * 2;

QMutex gLogMutex;
QFile gLogFile;

QString persistentSettingsFilePath() {
    return QCoreApplication::applicationDirPath() + QStringLiteral("/FobosAPP.ini");
}

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

const char *fobosApiKindName(FobosApiKind kind) {
    switch (kind) {
    case FobosApiKind::Standard:
        return "standard";
    case FobosApiKind::Agile:
        return "agile";
    }
    return "unknown";
}

QString fobosApiDisplayName(FobosApiKind kind) {
    return kind == FobosApiKind::Agile ? QStringLiteral("Agile") : QStringLiteral("Standard");
}

bool firmwareLooksAgile(const QString &firmwareVersion) {
    bool ok = false;
    const int major = firmwareVersion.trimmed().section('.', 0, 0).toInt(&ok);
    return ok && major >= 3;
}

void *activeFobosDevice() {
    return activeFobosApiKind == FobosApiKind::Agile
               ? static_cast<void*>(agileDevice)
               : static_cast<void*>(device);
}

bool hasActiveFobosDevice() {
    return activeFobosDevice() != nullptr;
}

double directMaxFrequency(double sampleRate) {
    return (std::max)(DIRECT_MIN_FREQUENCY, sampleRate / 2.0 - 1.0);
}

double directMinFrequencyForMode(int inputMode, double sampleRate) {
    return inputMode == 1 ? -directMaxFrequency(sampleRate) : DIRECT_MIN_FREQUENCY;
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
        const double directMin = directMinFrequencyForMode(settings.inputMode, settings.sampleRate);
        const double directMax = directMaxFrequency(settings.sampleRate);
        settings.listeningFrequency = (std::clamp)(settings.listeningFrequency,
                                                   directMin,
                                                   directMax);
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

double clampAudioLowPassHz(double hz) {
    if (!std::isfinite(hz) || hz <= 0.0) {
        return 0.0;
    }
    return (std::clamp)(hz, static_cast<double>(AUDIO_LOW_PASS_SLIDER_STEP_HZ),
                        static_cast<double>(AUDIO_LOW_PASS_SLIDER_STEP_HZ * AUDIO_LOW_PASS_SLIDER_MAX));
}

double clampAudioHighPassHz(double hz) {
    if (!std::isfinite(hz) || hz <= 0.0) {
        return 0.0;
    }
    return (std::clamp)(hz, static_cast<double>(AUDIO_HIGH_PASS_SLIDER_STEP_HZ),
                        static_cast<double>(AUDIO_HIGH_PASS_SLIDER_STEP_HZ * AUDIO_HIGH_PASS_SLIDER_MAX));
}

double audioLowPassSliderValueToHz(int value) {
    return value <= 0 ? 0.0 : clampAudioLowPassHz(value * AUDIO_LOW_PASS_SLIDER_STEP_HZ);
}

int audioLowPassHzToSliderValue(double hz) {
    if (!std::isfinite(hz) || hz <= 0.0) {
        return 0;
    }
    return (std::clamp)(static_cast<int>(std::lround(hz / AUDIO_LOW_PASS_SLIDER_STEP_HZ)),
                        1,
                        AUDIO_LOW_PASS_SLIDER_MAX);
}

double audioHighPassSliderValueToHz(int value) {
    return value <= 0 ? 0.0 : clampAudioHighPassHz(value * AUDIO_HIGH_PASS_SLIDER_STEP_HZ);
}

int audioHighPassHzToSliderValue(double hz) {
    if (!std::isfinite(hz) || hz <= 0.0) {
        return 0;
    }
    return (std::clamp)(static_cast<int>(std::lround(hz / AUDIO_HIGH_PASS_SLIDER_STEP_HZ)),
                        1,
                        AUDIO_HIGH_PASS_SLIDER_MAX);
}

QString audioFilterFrequencyText(double hz) {
    if (!std::isfinite(hz) || hz <= 0.0) {
        return QString();
    }
    if (hz >= 1000.0) {
        const int decimals = hz >= 10000.0 ? 0 : 1;
        return QString("%1 kHz").arg(hz / 1000.0, 0, 'f', decimals);
    }
    return QString("%1 Hz").arg(static_cast<int>(std::lround(hz)));
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
        return 3000.0;
    case MOD_RTTY:
        return 2700.0;
    case MOD_PSK:
        return 2500.0;
    case MOD_FSK:
        return 12000.0;
    case MOD_DMR:
        return 12500.0;
    case MOD_ATV:
        return 5000000.0;
    case MOD_SSTV:
        return 3000.0;
    case MOD_APT:
        return 40000.0;
    case MOD_WEFAX:
        return 3000.0;
    case MOD_LRPT:
        return 140000.0;
    case MOD_AM:
    default:
        return 10000.0;
    }
}

QString formatBandwidthHz(double bandwidth) {
    return QString::number(bandwidth, 'f', 0);
}

QImage createSstvTestPattern() {
    constexpr int width = 320;
    constexpr int height = 256;
    QImage image(width, height, QImage::Format_RGB32);
    static const QRgb bars[] = {
        qRgb(255, 255, 255),
        qRgb(255, 255, 0),
        qRgb(0, 255, 255),
        qRgb(0, 255, 0),
        qRgb(255, 0, 255),
        qRgb(255, 0, 0),
        qRgb(0, 0, 255),
        qRgb(16, 16, 16),
    };

    for (int y = 0; y < height; ++y) {
        QRgb *line = reinterpret_cast<QRgb *>(image.scanLine(y));
        for (int x = 0; x < width; ++x) {
            if (y < 86) {
                line[x] = bars[(x * 8) / width];
            } else if (y < 172) {
                const int value = (x * 255) / (width - 1);
                line[x] = qRgb(value, value, value);
            } else {
                const bool checker = (((x / 16) + (y / 16)) % 2) == 0;
                const int ramp = ((x + y) * 255) / (width + height - 2);
                line[x] = checker ? qRgb(ramp, 80, 255 - ramp)
                                  : qRgb(255 - ramp, ramp, 80);
            }
        }
    }
    return image;
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

int getFobosStandardApiInfoSafely(char *libVersion, char *driverVersion) {
#ifdef _WIN32
    __try {
        return fobos_rx_get_api_info(libVersion, driverVersion);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_get_api_info(libVersion, driverVersion);
#endif
}

int getFobosAgileApiInfoSafely(char *libVersion, char *driverVersion) {
#ifdef _WIN32
    __try {
        return fobos_sdr_get_api_info(libVersion, driverVersion);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_get_api_info(libVersion, driverVersion);
#endif
}

int getFobosStandardDeviceCountSafely() {
#ifdef _WIN32
    __try {
        return fobos_rx_get_device_count();
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return 0;
    }
#else
    return fobos_rx_get_device_count();
#endif
}

int getFobosAgileDeviceCountSafely() {
#ifdef _WIN32
    __try {
        return fobos_sdr_get_device_count();
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return 0;
    }
#else
    return fobos_sdr_get_device_count();
#endif
}

int openFobosDeviceSafely(fobos_dev_t **dev, uint32_t index) {
    if (!dev) {
        return FOBOS_ERR_BAD_PARAM;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_open(dev, index);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        *dev = nullptr;
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_open(dev, index);
#endif
}

int openFobosAgileDeviceSafely(fobos_sdr_dev_t **dev, uint32_t index) {
    if (!dev) {
        return FOBOS_ERR_BAD_PARAM;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_open(dev, index);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        *dev = nullptr;
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_open(dev, index);
#endif
}

int getFobosBoardInfoSafely(fobos_dev_t *dev,
                            char *hwRevision,
                            char *firmwareVersion,
                            char *manufacturer,
                            char *product,
                            char *serial) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_get_board_info(dev, hwRevision, firmwareVersion, manufacturer, product, serial);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_get_board_info(dev, hwRevision, firmwareVersion, manufacturer, product, serial);
#endif
}

int getFobosAgileBoardInfoSafely(fobos_sdr_dev_t *dev,
                                 char *hwRevision,
                                 char *firmwareVersion,
                                 char *manufacturer,
                                 char *product,
                                 char *serial) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_get_board_info(dev, hwRevision, firmwareVersion, manufacturer, product, serial);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_get_board_info(dev, hwRevision, firmwareVersion, manufacturer, product, serial);
#endif
}

int getFobosSampleRatesSafely(fobos_dev_t *dev, double *values, unsigned int *count) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_rx_get_samplerates(dev, values, count);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_rx_get_samplerates(dev, values, count);
#endif
}

int getFobosAgileSampleRatesSafely(fobos_sdr_dev_t *dev, double *values, unsigned int *count) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_get_samplerates(dev, values, count);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_get_samplerates(dev, values, count);
#endif
}

void logFobosApiInfo() {
    char libVersion[256] = {};
    char driverVersion[256] = {};
    const int result = getFobosStandardApiInfoSafely(libVersion, driverVersion);
    if (result == FOBOS_ERR_OK) {
        qDebug() << "[FobosLifecycle] libfobos info"
                 << "library" << libVersion
                 << "driver" << driverVersion;
    } else {
        qDebug() << "[FobosLifecycle] libfobos info unavailable"
                 << "result" << result;
    }

    char agileLibVersion[256] = {};
    char agileDriverVersion[256] = {};
    const int agileResult = getFobosAgileApiInfoSafely(agileLibVersion, agileDriverVersion);
    if (agileResult == FOBOS_ERR_OK) {
        qDebug() << "[FobosLifecycle] libfobos agile info"
                 << "library" << agileLibVersion
                 << "driver" << agileDriverVersion;
    } else {
        qDebug() << "[FobosLifecycle] libfobos agile info unavailable"
                 << "result" << agileResult;
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

int closeFobosAgileDeviceSafely(fobos_sdr_dev_t *dev) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_close(dev);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_close(dev);
#endif
}

int setFobosAgileClockSourceSafely(fobos_sdr_dev_t *dev, int value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_set_clk_source(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_set_clk_source(dev, value);
#endif
}

int setFobosAgileDirectSamplingSafely(fobos_sdr_dev_t *dev, unsigned int enabled) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_set_direct_sampling(dev, enabled);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_set_direct_sampling(dev, enabled);
#endif
}

int setFobosAgileSampleRateSafely(fobos_sdr_dev_t *dev, double value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_set_samplerate(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_set_samplerate(dev, value);
#endif
}

int setFobosAgileFrequencySafely(fobos_sdr_dev_t *dev, double value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_set_frequency(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_set_frequency(dev, value);
#endif
}

int setFobosAgileLnaGainSafely(fobos_sdr_dev_t *dev, unsigned int value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_set_lna_gain(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_set_lna_gain(dev, value);
#endif
}

int setFobosAgileVgaGainSafely(fobos_sdr_dev_t *dev, unsigned int value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_set_vga_gain(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_set_vga_gain(dev, value);
#endif
}

int setFobosAgileGpoSafely(fobos_sdr_dev_t *dev, uint8_t value) {
    if (!dev) {
        return FOBOS_ERR_NOT_OPEN;
    }
#ifdef _WIN32
    __try {
        return fobos_sdr_set_user_gpo(dev, value);
    } __except(EXCEPTION_EXECUTE_HANDLER) {
        return FOBOS_ERR_LIBUSB;
    }
#else
    return fobos_sdr_set_user_gpo(dev, value);
#endif
}

int setActiveClockSourceSafely(int value) {
    if (activeFobosApiKind == FobosApiKind::Agile) {
        return setFobosAgileClockSourceSafely(agileDevice, value);
    }
    return setFobosClockSourceSafely(device, static_cast<unsigned int>(value));
}

int setActiveDirectSamplingSafely(unsigned int enabled) {
    if (activeFobosApiKind == FobosApiKind::Agile) {
        return setFobosAgileDirectSamplingSafely(agileDevice, enabled);
    }
    return setFobosDirectSamplingSafely(device, enabled);
}

int setActiveSampleRateSafely(double value, double *actual) {
    if (activeFobosApiKind == FobosApiKind::Agile) {
        if (!agileDevice) {
            return FOBOS_ERR_NOT_OPEN;
        }
        const int result = setFobosAgileSampleRateSafely(agileDevice, value);
        if (actual) {
            *actual = value;
        }
        return result;
    }
    return setFobosSampleRateSafely(device, value, actual);
}

int setActiveFrequencySafely(double value, double *actual) {
    if (activeFobosApiKind == FobosApiKind::Agile) {
        if (!agileDevice) {
            return FOBOS_ERR_NOT_OPEN;
        }
        const int result = setFobosAgileFrequencySafely(agileDevice, value);
        if (actual) {
            *actual = value;
        }
        return result;
    }
    return setFobosFrequencySafely(device, value, actual);
}

int setActiveLnaGainSafely(unsigned int value) {
    if (activeFobosApiKind == FobosApiKind::Agile) {
        return setFobosAgileLnaGainSafely(agileDevice, value);
    }
    return setFobosLnaGainSafely(device, value);
}

int setActiveVgaGainSafely(unsigned int value) {
    if (activeFobosApiKind == FobosApiKind::Agile) {
        return setFobosAgileVgaGainSafely(agileDevice, value);
    }
    return setFobosVgaGainSafely(device, value);
}

int setActiveGpoSafely(uint8_t value) {
    if (activeFobosApiKind == FobosApiKind::Agile) {
        return setFobosAgileGpoSafely(agileDevice, value);
    }
    return setFobosGpoSafely(device, value);
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

    centralWidget = new QWidget(this);
    QScrollArea *graphScrollArea = new QScrollArea(this);
    graphScrollArea->setWidgetResizable(true);
    graphScrollArea->setWidget(centralWidget);
    setCentralWidget(graphScrollArea);

    QWidget *controlsWidget = new QWidget(this);
    QScrollArea *controlsScrollArea = new QScrollArea(this);
    controlsScrollArea->setWidgetResizable(true);
    controlsScrollArea->setWidget(controlsWidget);
    controlsDock = new QDockWidget("Controls", this);
    controlsDock->setObjectName("controlsDock");
    controlsDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    controlsDock->setFeatures(QDockWidget::DockWidgetMovable |
                              QDockWidget::DockWidgetFloatable);
    controlsDock->setWidget(controlsScrollArea);
    addDockWidget(Qt::LeftDockWidgetArea, controlsDock);

    modulationButtonGroup = new QButtonGroup(this);
    auto addModulationRadioButton = [this](QWidget *parent,
                                           QHBoxLayout *targetLayout,
                                           const QString &label,
                                           int modulationId,
                                           const QString &toolTip) -> QRadioButton * {
        QRadioButton *button = new QRadioButton(label, parent);
        if (!toolTip.isEmpty()) {
            button->setToolTip(toolTip);
        }
        modulationButtonGroup->addButton(button, modulationId);
        targetLayout->addWidget(button);
        return button;
    };

    QWidget *digitalWidget = new QWidget(this);
    QVBoxLayout *digitalLayout = new QVBoxLayout(digitalWidget);
    QHBoxLayout *digitalHeaderLayout = new QHBoxLayout();
    QHBoxLayout *digitalModeLayout = new QHBoxLayout();
    QGridLayout *dmrLabLayout = new QGridLayout();
    digitalDecodeCheckbox = new QCheckBox("Decode", digitalWidget);
    digitalDecodeCheckbox->setChecked(digitalDecodeEnabled);
    dmrLabCaptureCheckbox = new QCheckBox("DMR Lab", digitalWidget);
    dmrLabCaptureCheckbox->setToolTip("Write expected DMR test metadata next to audio/IQ recordings");
    dmrLabColorCodeCombo = new QComboBox(digitalWidget);
    dmrLabColorCodeCombo->addItem("?", -1);
    for (int cc = 0; cc <= 15; ++cc) {
        dmrLabColorCodeCombo->addItem(QString::number(cc), cc);
    }
    dmrLabSlotCombo = new QComboBox(digitalWidget);
    dmrLabSlotCombo->addItem("?", 0);
    dmrLabSlotCombo->addItem("TS1", 1);
    dmrLabSlotCombo->addItem("TS2", 2);
    dmrLabCallTypeCombo = new QComboBox(digitalWidget);
    dmrLabCallTypeCombo->addItem("?", QStringLiteral("unknown"));
    dmrLabCallTypeCombo->addItem("Group", QStringLiteral("group"));
    dmrLabCallTypeCombo->addItem("Private", QStringLiteral("private"));
    dmrLabCallTypeCombo->addItem("All", QStringLiteral("all_call"));
    dmrLabSourceIdEdit = new QLineEdit(digitalWidget);
    dmrLabSourceIdEdit->setPlaceholderText(QStringLiteral("Src ID"));
    dmrLabTargetIdEdit = new QLineEdit(digitalWidget);
    dmrLabTargetIdEdit->setPlaceholderText(QStringLiteral("TG/Target"));
    dmrLabRadioEdit = new QLineEdit(digitalWidget);
    dmrLabRadioEdit->setPlaceholderText(QStringLiteral("Radio"));
    dmrLabNotesEdit = new QLineEdit(digitalWidget);
    dmrLabNotesEdit->setPlaceholderText(QStringLiteral("Test note"));
    dmrLabColorCodeCombo->setMaximumWidth(64);
    dmrLabSlotCombo->setMaximumWidth(72);
    dmrLabCallTypeCombo->setMaximumWidth(92);
    QPushButton *digitalClearButton = new QPushButton("Clear", digitalWidget);
    digitalStatusLabel = new QLabel("Digital audio decoder idle", digitalWidget);
    digitalTextEdit = new QPlainTextEdit(digitalWidget);
    digitalTextEdit->setReadOnly(true);
    digitalTextEdit->setMaximumBlockCount(2000);
    digitalTextEdit->setPlaceholderText("Decoded digital-audio text will appear here.");
    digitalModeLayout->addWidget(new QLabel("Mode:", digitalWidget));
    addModulationRadioButton(digitalWidget, digitalModeLayout, "FT8", MOD_FT8, "FT8 weak-signal decoder");
    addModulationRadioButton(digitalWidget, digitalModeLayout, "RTTY", MOD_RTTY, "AFSK RTTY decoder");
    addModulationRadioButton(digitalWidget, digitalModeLayout, "FSK", MOD_FSK, "Frequency-shift keying decoder");
    addModulationRadioButton(digitalWidget, digitalModeLayout, "PSK", MOD_PSK, "PSK audio mode placeholder");
    addModulationRadioButton(digitalWidget, digitalModeLayout, "DMR", MOD_DMR, "DMR 4FSK sync monitor");
    digitalModeLayout->addStretch();
    dmrLabLayout->addWidget(dmrLabCaptureCheckbox, 0, 0);
    dmrLabLayout->addWidget(new QLabel("CC:", digitalWidget), 0, 1);
    dmrLabLayout->addWidget(dmrLabColorCodeCombo, 0, 2);
    dmrLabLayout->addWidget(new QLabel("Slot:", digitalWidget), 0, 3);
    dmrLabLayout->addWidget(dmrLabSlotCombo, 0, 4);
    dmrLabLayout->addWidget(new QLabel("Call:", digitalWidget), 0, 5);
    dmrLabLayout->addWidget(dmrLabCallTypeCombo, 0, 6);
    dmrLabLayout->addWidget(dmrLabSourceIdEdit, 1, 0, 1, 2);
    dmrLabLayout->addWidget(dmrLabTargetIdEdit, 1, 2, 1, 2);
    dmrLabLayout->addWidget(dmrLabRadioEdit, 1, 4, 1, 3);
    dmrLabLayout->addWidget(dmrLabNotesEdit, 2, 0, 1, 7);
    digitalHeaderLayout->addWidget(digitalDecodeCheckbox);
    digitalHeaderLayout->addStretch();
    digitalHeaderLayout->addWidget(digitalClearButton);
    digitalLayout->addLayout(digitalHeaderLayout);
    digitalLayout->addLayout(digitalModeLayout);
    digitalLayout->addLayout(dmrLabLayout);
    digitalLayout->addWidget(digitalStatusLabel);
    digitalLayout->addWidget(digitalTextEdit);
    digitalDock = new QDockWidget("Digital Audio", this);
    digitalDock->setObjectName("digitalDock");
    digitalDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
    digitalDock->setFeatures(QDockWidget::DockWidgetMovable |
                             QDockWidget::DockWidgetFloatable |
                             QDockWidget::DockWidgetClosable);
    digitalDock->setWidget(digitalWidget);
    addDockWidget(Qt::RightDockWidgetArea, digitalDock);
    digitalDock->hide();

    QWidget *videoPanel = new QWidget(this);
    QVBoxLayout *videoLayout = new QVBoxLayout(videoPanel);
    QHBoxLayout *videoHeaderLayout = new QHBoxLayout();
    QHBoxLayout *videoModeLayout = new QHBoxLayout();
    videoDecodeCheckbox = new QCheckBox("Decode", videoPanel);
    videoDecodeCheckbox->setChecked(videoDecodeEnabled);
    videoDemodCombo = new QComboBox(videoPanel);
    videoDemodCombo->addItem("FM video", VideoProcessor::FmVideo);
    videoDemodCombo->addItem("AM video", VideoProcessor::AmVideo);
    videoStandardCombo = new QComboBox(videoPanel);
    videoStandardCombo->addItem("PAL 15.625 kHz", 15625.0);
    videoStandardCombo->addItem("NTSC 15.734 kHz", 15734.2657);
    videoInvertCheckbox = new QCheckBox("Invert", videoPanel);
    videoHSyncCheckbox = new QCheckBox("HSync", videoPanel);
    videoHSyncCheckbox->setChecked(true);
    videoHSyncCheckbox->setToolTip("Align video lines by the darkest horizontal sync pulse");
    videoVSyncCheckbox = new QCheckBox("VSync", videoPanel);
    videoVSyncCheckbox->setChecked(true);
    videoVSyncCheckbox->setToolTip("Reset analog video frame on broad vertical sync pulses");
    videoTestPatternCheckbox = new QCheckBox("Test", videoPanel);
    videoTestPatternCheckbox->setToolTip("Generate an internal test pattern for the selected video mode");
    videoStatusLabel = new QLabel("Video decoder disabled", videoPanel);
    videoWidget = new VideoWidget(videoPanel);
    videoModeLayout->addWidget(new QLabel("Mode:", videoPanel));
    addModulationRadioButton(videoPanel, videoModeLayout, "ATV", MOD_ATV, "Analog television video demodulator");
    addModulationRadioButton(videoPanel, videoModeLayout, "SSTV", MOD_SSTV, "Slow-scan television image decoder");
    addModulationRadioButton(videoPanel, videoModeLayout, "APT", MOD_APT, "NOAA APT weather satellite image decoder");
    addModulationRadioButton(videoPanel, videoModeLayout, "WEFAX", MOD_WEFAX, "HF weather fax image decoder");
    addModulationRadioButton(videoPanel, videoModeLayout, "LRPT", MOD_LRPT, "Meteor LRPT beta IQ monitor");
    videoModeLayout->addStretch();
    videoHeaderLayout->addWidget(videoDecodeCheckbox);
    videoHeaderLayout->addWidget(videoDemodCombo);
    videoHeaderLayout->addWidget(videoStandardCombo);
    videoHeaderLayout->addWidget(videoInvertCheckbox);
    videoHeaderLayout->addWidget(videoHSyncCheckbox);
    videoHeaderLayout->addWidget(videoVSyncCheckbox);
    videoHeaderLayout->addWidget(videoTestPatternCheckbox);
    videoHeaderLayout->addStretch();
    videoLayout->addLayout(videoModeLayout);
    videoLayout->addLayout(videoHeaderLayout);
    videoLayout->addWidget(videoStatusLabel);
    videoLayout->addWidget(videoWidget, 1);
    videoDock = new QDockWidget("Video", this);
    videoDock->setObjectName("videoDock");
    videoDock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea | Qt::BottomDockWidgetArea);
    videoDock->setFeatures(QDockWidget::DockWidgetMovable |
                           QDockWidget::DockWidgetFloatable |
                           QDockWidget::DockWidgetClosable);
    videoDock->setWidget(videoPanel);
    addDockWidget(Qt::RightDockWidgetArea, videoDock);
    videoDock->hide();

    QHBoxLayout *scaleLayout = new QHBoxLayout();
    QVBoxLayout *contrastLayout = new QVBoxLayout();
    QVBoxLayout *sensLayout = new QVBoxLayout();
    QVBoxLayout *levelMinLayout = new QVBoxLayout();
    QVBoxLayout *levelMaxLayout = new QVBoxLayout();
    QVBoxLayout *layout = new QVBoxLayout();
    QGridLayout *checkboxLayout = new QGridLayout();
    QVBoxLayout *graphLayout = new QVBoxLayout();
    QHBoxLayout *graphToolLayout = new QHBoxLayout();
    
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
    vgaGainSlider->setRange(0, 31);
    vgaGainSlider->setValue(3);

    volumeSlider = new QSlider(Qt::Horizontal, this);
    volumeSlider->setRange(0, 200);
    volumeSlider->setValue(100);

    audioLowPassSlider = new QSlider(Qt::Horizontal, this);
    audioLowPassSlider->setRange(0, AUDIO_LOW_PASS_SLIDER_MAX);
    audioLowPassSlider->setSingleStep(1);
    audioLowPassSlider->setPageStep(10);
    audioLowPassSlider->setValue(0);

    audioHighPassSlider = new QSlider(Qt::Horizontal, this);
    audioHighPassSlider->setRange(0, AUDIO_HIGH_PASS_SLIDER_MAX);
    audioHighPassSlider->setSingleStep(1);
    audioHighPassSlider->setPageStep(4);
    audioHighPassSlider->setValue(0);

    volumeLabel = new QLabel("Volume: 100%", this);
    audioLowPassLabel = new QLabel("Audio LPF: Auto", this);
    audioHighPassLabel = new QLabel("Audio HPF: Off", this);
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
    for (int i = 0; i < comboBox->count(); ++i) {
        comboBox->setItemData(i, i);
    }
    modeBox->addItem("RF", 0);
    modeBox->addItem("HF1 + HF2", 1);
    modeBox->addItem("HF1", 2);
    modeBox->addItem("HF2", 3);
    
    clkBox->addItem("Internal", 0);
    clkBox->addItem("External", 1);

    processor = new DataProcessor( this);
    audioProcessor = new AudioProcessor(this);
    digitalDecoderThread = new QThread(this);
    digitalDecoderThread->setObjectName(QStringLiteral("DigitalDecoderThread"));
    digitalDecoder = new DigitalDecoder();
    digitalDecoder->moveToThread(digitalDecoderThread);
    connect(digitalDecoderThread, &QThread::finished, digitalDecoder, &QObject::deleteLater);
    digitalDecoderThread->start();
    videoProcessorThread = new QThread(this);
    videoProcessorThread->setObjectName(QStringLiteral("VideoProcessorThread"));
    videoProcessor = new VideoProcessor();
    videoProcessor->moveToThread(videoProcessorThread);
    connect(videoProcessorThread, &QThread::finished, videoProcessor, &QObject::deleteLater);
    videoProcessorThread->start();
    playbackManager = new PlaybackManager(this);
    recordingManager = new RecordingManager(this);
    networkController = new NetworkController(this);
    remoteAudioPlayer = new RemoteAudioPlayer(this);
    networkSettingsDebounceTimer = new QTimer(this);
    networkSettingsDebounceTimer->setSingleShot(true);
    connect(networkSettingsDebounceTimer, &QTimer::timeout, this, [this]() {
        sendRemoteControlCommand("settings");
    });
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
    digitalToggleButton = new QPushButton("Digital Audio", this);
    digitalToggleButton->setCheckable(true);
    digitalToggleButton->setMaximumWidth(120);
    digitalToggleButton->setToolTip("Show or hide the digital audio decoder panel");
    videoToggleButton = new QPushButton("Video", this);
    videoToggleButton->setCheckable(true);
    videoToggleButton->setMaximumWidth(80);
    videoToggleButton->setToolTip("Show or hide the video/image decoder panel");
    recordingModeCombo = new QComboBox(this);
    recordingModeCombo->addItem("Audio WAV", static_cast<int>(RecordingManager::Mode::AudioWav));
    recordingModeCombo->addItem("Channel IQ WAV", static_cast<int>(RecordingManager::Mode::ChannelIqWav));
    recordButton = new QPushButton("Record", this);
    recordButton->setCheckable(true);
    recordButton->setToolTip("Start/stop recording. Hold F9 for momentary recording.");
    recordingStatusLabel = new QLabel("Recording: idle", this);
    playbackStatusLabel = new QLabel("Playback: idle", this);
    playbackFileCombo = new QComboBox(this);
    playbackFileCombo->setMinimumContentsLength(24);
    playbackRefreshButton = new QPushButton("Refresh Playback", this);
    playbackButton = new QPushButton("Play", this);
    playbackButton->setCheckable(true);
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
    
    QLabel *centralFrequencyLabel = new QLabel("Central Frequency:", this);
    frequencyControl = new FrequencyControl(this);
    frequencyControl->setRangeHz(0.0, 6000000000.0);
    frequencyControl->setValueHz(100000000.0);
    
    QLabel *listeningFrequencyLabel = new QLabel("Listening Frequency:", this);
    listeningFrequencyControl = new FrequencyControl(this);
    listeningFrequencyControl->setRangeHz(0.0, 6000000000.0);
    listeningFrequencyControl->setValueHz(100000000.0);
    
    QLabel *fftLabel = new QLabel("FFT Length", this);
    
    QLabel *bandwidthLabel = new QLabel("Audio Bandwidth:", this);
    bandwidthControl = new FrequencyControl(this);
    bandwidthControl->setRangeHz(10.0, 5000000.0);
    bandwidthControl->setStepPresets({
        {"10 Hz", 10.0},
        {"100 Hz", 100.0},
        {"500 Hz", 500.0},
        {"1 kHz", 1000.0},
        {"2.5 kHz", 2500.0},
        {"5 kHz", 5000.0},
        {"10 kHz", 10000.0},
        {"25 kHz", 25000.0},
        {"100 kHz", 100000.0},
    });
    bandwidthControl->setValuePresets({
        {"CW 500 Hz", 500.0},
        {"SSB 2.7 kHz", 2700.0},
        {"FT8 3 kHz", 3000.0},
        {"AM 6 kHz", 6000.0},
        {"AM 10 kHz", 10000.0},
        {"NFM 12.5 kHz", 12500.0},
        {"DMR 12.5 kHz", 12500.0},
        {"WFM 200 kHz", 200000.0},
        {"SSTV 3 kHz", 3000.0},
        {"NOAA APT 40 kHz", 40000.0},
        {"WEFAX 3 kHz", 3000.0},
        {"LRPT 140 kHz", 140000.0},
        {"ATV 5 MHz", 5000000.0},
    });
    bandwidthControl->setValueHz(defaultBandwidthForModulation(MOD_AM));
   
    QStringList modulationNames = {"AM", "NFM", "SAM", "USB", "LSB", "DSB", "CW", "WFM"};
    QVector<int> modulationIds = {MOD_AM, MOD_NFM, MOD_SAM, MOD_USB, MOD_LSB, MOD_DSB,
                                  MOD_CW, MOD_WFM};
    
    QHBoxLayout* row1 = new QHBoxLayout();
    QHBoxLayout* row2 = new QHBoxLayout();
    
    scaleLayout->addLayout(contrastLayout);
    scaleLayout->addLayout(sensLayout);
    scaleLayout->addLayout(levelMinLayout);
    scaleLayout->addLayout(levelMaxLayout);
    
    graphToolLayout->addStretch();
    graphToolLayout->addWidget(digitalToggleButton);
    graphToolLayout->addWidget(videoToggleButton);

    graphLayout->addLayout(graphToolLayout);
    graphLayout->addWidget(graphWidget);
    graphLayout->addWidget(scaleWidget); 
    graphLayout->addWidget(waterfallWidget);
    graphLayout->addLayout(scaleLayout);
    
    for (int i = 0; i < modulationNames.size(); ++i) {
        QRadioButton* radioButton = addModulationRadioButton(controlsWidget,
                                                             i < 4 ? row1 : row2,
                                                             modulationNames[i],
                                                             modulationIds[i],
                                                             QString());
        if (i == 0) {
            radioButton->setChecked(true); 
        }
    }
    
    QHBoxLayout *deviceButtonLayout = new QHBoxLayout();
    deviceButtonLayout->addWidget(refreshButton);
    deviceButtonLayout->addWidget(fobosButton);

    QHBoxLayout *networkLnaLayout = new QHBoxLayout();
    networkLnaLayout->addWidget(networkButton);
    networkLnaLayout->addWidget(lnaGainLabel);
    networkLnaLayout->addWidget(lnaGainSlider);

    QHBoxLayout *recordingLayout = new QHBoxLayout();
    recordingLayout->addWidget(recordingModeCombo);
    recordingLayout->addWidget(recordButton);

    QHBoxLayout *playbackButtonLayout = new QHBoxLayout();
    playbackButtonLayout->addWidget(playbackRefreshButton);
    playbackButtonLayout->addWidget(playbackButton);

    layout->addLayout(deviceButtonLayout);
    layout->addWidget(comboBox);
    layout->addWidget(clkBox);
    layout->addWidget(modeBox);
    layout->addWidget(sampleBox);
    layout->addLayout(checkboxLayout);
    layout->addWidget(centralFrequencyLabel);
    layout->addWidget(frequencyControl);
    layout->addWidget(listeningFrequencyLabel);
    layout->addWidget(listeningFrequencyControl);
    layout->addWidget(fftLabel);
    layout->addWidget(fftComboBox);
    layout->addWidget(scaleLabel);
    layout->addWidget(scaleSlider);
    layout->addLayout(networkLnaLayout);
    layout->addWidget(startButton);
    layout->addWidget(stopButton);
    layout->addWidget(vgaGainLabel);
    layout->addWidget(vgaGainSlider);
    layout->addWidget(volumeLabel);
    layout->addWidget(volumeSlider);
    layout->addWidget(audioLowPassLabel);
    layout->addWidget(audioLowPassSlider);
    layout->addWidget(audioHighPassLabel);
    layout->addWidget(audioHighPassSlider);
    layout->addWidget(recordingStatusLabel);
    layout->addLayout(recordingLayout);
    layout->addWidget(playbackStatusLabel);
    layout->addWidget(playbackFileCombo);
    layout->addLayout(playbackButtonLayout);
    layout->addLayout(chckbox);
    layout->addWidget(audioDeviceComboBox);
    layout->addWidget(bandwidthLabel);
    layout->addWidget(bandwidthControl);
    layout->addLayout(row1);
    layout->addLayout(row2);
    
    controlsWidget->setLayout(layout);
    centralWidget->setLayout(graphLayout);
    graphLayout->setStretch(0, 2);
    graphLayout->setStretch(0, 0);
    graphLayout->setStretch(1, 2);
    graphLayout->setStretch(2, 0);
    graphLayout->setStretch(3, 5);
    graphLayout->setStretch(4, 0);
    
    scaleWidget->setTuning(listeningFrequency, globalFrequency, globalBandwidth, globalModulationType);
    scaleWidget->setMarkerPosition(0.5);
    scaleWidget->setRange(minFrequency, maxFrequency);

    updateTimer = new QTimer(this);
    updateSpectrumTimerInterval();
    stopPollTimer = new QTimer(this);
    stopPollTimer->setInterval(100);
    streamWatchdogTimer = new QTimer(this);
    streamWatchdogTimer->setInterval(250);
    videoSnapshotTimer = new QTimer(this);
    videoSnapshotTimer->setInterval(VIDEO_SNAPSHOT_INTERVAL_MS);
    
    connect(updateTimer, &QTimer::timeout, this, &YourClassName::updateSpectrum);
    connect(stopPollTimer, &QTimer::timeout, this, &YourClassName::pollStopCompletion);
    connect(streamWatchdogTimer, &QTimer::timeout, this, &YourClassName::checkStreamStartup);
    connect(videoSnapshotTimer, &QTimer::timeout, this, &YourClassName::processVideoSnapshotFrame);
    connect(graphCheckbox, &QCheckBox::toggled, this, &YourClassName::doubleGraphEnable);
    connect(colorCheckbox, &QCheckBox::toggled, this, &YourClassName::colorGraphEnable);
    connect(syncCheckbox, &QCheckBox::toggled, this, &YourClassName::syncEnable);
    connect(audioDeviceComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onAudioDeviceChanged(int)));
    connect(modulationButtonGroup, QOverload<int>::of(&QButtonGroup::idClicked), this, &YourClassName::onModulationChanged);
    connect(scaleSlider, &QSlider::valueChanged, this, &YourClassName::onScaleChanged);
    connect(frequencyControl, &FrequencyControl::valueCommitted, this, [this](double) {
        onFrequencyEntered();
    });
    connect(fftComboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(onfftLengthEntered()));
    connect(listeningFrequencyControl, &FrequencyControl::valueCommitted, this, [this](double) {
        onListeningFrequencyEntered();
    });
    connect(lnaGainSlider, &QSlider::valueChanged, this, &YourClassName::onLnaGainChanged);
    connect(vgaGainSlider, &QSlider::valueChanged, this, &YourClassName::onVgaGainChanged);
    connect(contrastSlider, &QSlider::valueChanged, this, &YourClassName::onContrastChanged);
    connect(sensitivitySlider, &QSlider::valueChanged, this, &YourClassName::onSensitivityChanged);
    connect(levelMinSlider, &QSlider::valueChanged, this, &YourClassName::onLevelMinChanged);
    connect(levelMaxSlider, &QSlider::valueChanged, this, &YourClassName::onLevelMaxChanged);
    connect(scaleSlider, &QSlider::sliderReleased, this, [this]() {
        if (isNetworkClientMode() && !isFullIqProcessingMode()) {
            scheduleRemoteSettingsCommand();
        }
    });
    connect(volumeSlider, &QSlider::valueChanged, this, [this](int value) {
        volumePercent = value;
        volumeLabel->setText(QString("Volume: %1%").arg(value));

        const float volume = value / 100.0f;

        if (audioProcessor) {
            audioProcessor->setVolume(volume);
        }

        if (remoteAudioPlayer) {
            remoteAudioPlayer->setVolume(volume);
        }
    });
    connect(audioLowPassSlider, &QSlider::valueChanged, this, [this](int value) {
        pendingSettings.audioLowPassHz = audioLowPassSliderValueToHz(value);
        updateAudioFilterLabels();
        publishSettingsToGlobals();
        if (isNetworkClientMode()) {
            scheduleRemoteSettingsCommand();
        }
    });
    connect(audioHighPassSlider, &QSlider::valueChanged, this, [this](int value) {
        pendingSettings.audioHighPassHz = audioHighPassSliderValueToHz(value);
        updateAudioFilterLabels();
        publishSettingsToGlobals();
        if (isNetworkClientMode()) {
            scheduleRemoteSettingsCommand();
        }
    });
    connect(startButton, &QPushButton::clicked, this, &YourClassName::startFobosProcessing);
    connect(stopButton, &QPushButton::clicked, this, &YourClassName::stopFobosProcessing);
    connect(modeBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &YourClassName::onDirectSamplingChanged);
    connect(clkBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &YourClassName::onClkChanged);
    connect(refreshButton, &QPushButton::clicked, [this]() {
        comboBox->blockSignals(true);
        comboBox->clear();
        comboBox->addItems(getFobosDevices());
        for (int i = 0; i < comboBox->count(); ++i) {
            comboBox->setItemData(i, i);
        }
        if (comboBox->count() > 0) {
            pendingSettings.deviceIndex = comboBox->currentData().toInt();
        }
        comboBox->blockSignals(false);
        if (sampleBox) {
            sampleBox->clear();
            populateSampleRates();
        }
    });
    connect(comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        if (index < 0 || isRunningOrTransitioning()) {
            return;
        }
        bool ok = false;
        const int selectedIndex = comboBox->currentData().toInt(&ok);
        pendingSettings.deviceIndex = ok ? selectedIndex : index;
        if (sampleBox) {
            sampleBox->clear();
            populateSampleRates();
        }
        qDebug() << "[FobosDevices] selected logical device" << pendingSettings.deviceIndex;
    });
    connect(fobosButton, &QPushButton::clicked, this, &YourClassName::listFobosDevices);
    connect(networkButton, &QPushButton::clicked, this, &YourClassName::openNetworkSettingsDialog);
    connect(networkController, &NetworkController::statusChanged, this, &YourClassName::onNetworkStatusChanged);
    connect(networkController, &NetworkController::channelReady, this, [this](const QString &status) {
        Q_UNUSED(status);
        if (isNetworkClientMode()) {
            networkClientReconnectPending = false;
            if (runState == RadioRunState::Running) {
                if (isClientIqProcessingMode()) {
                    startNetworkClientProcessing();
                } else {
                    stopNetworkClientProcessing();
                }
            }
            QTimer::singleShot(0, this, [this]() {
                sendRemoteControlCommand(runState == RadioRunState::Running ? "start" : "settings");
            });
        } else if (networkMode == NetworkMode::Server && runState == RadioRunState::Running) {
            applyServerLocalOutputPolicy();
        }
    });
    connect(networkController, &NetworkController::channelError, this, [this](const QString &message) {
        qDebug() << "[Network]" << message;
        if (networkMode == NetworkMode::Server && runState == RadioRunState::Running) {
            applyServerLocalOutputPolicy();
        }
        if (isNetworkClientMode() && runState == RadioRunState::Running) {
            if (isClientIqProcessingMode()) {
                stopNetworkClientProcessing();
            } else if (remoteAudioPlayer) {
                remoteAudioPlayer->stop();
            }
            updateUiForRunState();
        }
        if (!isNetworkClientMode() ||
            runState != RadioRunState::Running ||
            networkClientReconnectPending) {
            return;
        }

        networkClientReconnectPending = true;
        QTimer::singleShot(1500, this, [this]() {
            networkClientReconnectPending = false;
            if (isNetworkClientMode() &&
                runState == RadioRunState::Running &&
                networkController &&
                !networkController->isControlReady()) {
                qDebug() << "[Network] reconnecting client after channel loss";
                networkController->testClientConnection(networkServerAddress, networkControlPort);
            }
        });
    });
    connect(networkController, &NetworkController::controlCommandReceived, this, &YourClassName::onNetworkControlCommandReceived);
    connect(digitalClearButton, &QPushButton::clicked, this, [this]() {
        if (digitalTextEdit) {
            digitalTextEdit->clear();
        }
    });
    connect(digitalToggleButton, &QPushButton::toggled, this, [this](bool checked) {
        if (digitalDock) {
            digitalDock->setVisible(checked);
        }
    });
    connect(digitalDock, &QDockWidget::visibilityChanged, this, [this](bool visible) {
        if (digitalToggleButton && digitalToggleButton->isChecked() != visible) {
            QSignalBlocker blocker(digitalToggleButton);
            digitalToggleButton->setChecked(visible);
        }
    });
    connect(digitalDecodeCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
        digitalDecodeEnabled = checked;
        updateDigitalDecoderMode();
    });
    auto updateDmrLabControls = [this](bool enabled) {
        const QList<QWidget *> controls = {
            dmrLabColorCodeCombo,
            dmrLabSlotCombo,
            dmrLabCallTypeCombo,
            dmrLabSourceIdEdit,
            dmrLabTargetIdEdit,
            dmrLabRadioEdit,
            dmrLabNotesEdit,
        };
        for (QWidget *control : controls) {
            if (control) {
                control->setEnabled(enabled);
            }
        }
    };
    connect(dmrLabCaptureCheckbox, &QCheckBox::toggled, this, updateDmrLabControls);
    updateDmrLabControls(dmrLabCaptureCheckbox && dmrLabCaptureCheckbox->isChecked());
    connect(digitalDecoder, &DigitalDecoder::textDecoded, this, &YourClassName::onDigitalTextDecoded);
    connect(digitalDecoder, &DigitalDecoder::statusChanged, this, &YourClassName::onDigitalDecoderStatusChanged);
    connect(videoToggleButton, &QPushButton::toggled, this, [this](bool checked) {
        if (videoDock) {
            videoDock->setVisible(checked);
        }
    });
    connect(videoDock, &QDockWidget::visibilityChanged, this, [this](bool visible) {
        if (videoToggleButton && videoToggleButton->isChecked() != visible) {
            QSignalBlocker blocker(videoToggleButton);
            videoToggleButton->setChecked(visible);
        }
        updateVideoProcessorMode();
        updateIqFrameProducerSettings();
    });
    connect(videoDecodeCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
        videoDecodeEnabled = checked;
        updateVideoProcessorMode();
        updateIqFrameProducerSettings();
    });
    connect(videoDemodCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
        updateVideoProcessorMode();
    });
    connect(videoStandardCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
        updateVideoProcessorMode();
    });
    connect(videoInvertCheckbox, &QCheckBox::toggled, this, [this]() {
        updateVideoProcessorMode();
    });
    connect(videoHSyncCheckbox, &QCheckBox::toggled, this, [this]() {
        updateVideoProcessorMode();
    });
    connect(videoVSyncCheckbox, &QCheckBox::toggled, this, [this]() {
        updateVideoProcessorMode();
    });
    connect(videoTestPatternCheckbox, &QCheckBox::toggled, this, [this](bool checked) {
        const bool analogVideoTest = checked && pendingSettings.modulationType == MOD_ATV;
        const bool sstvTest = checked && pendingSettings.modulationType == MOD_SSTV;
        const bool aptTest = checked && pendingSettings.modulationType == MOD_APT;
        const bool wefaxTest = checked && pendingSettings.modulationType == MOD_WEFAX;
        if (videoProcessor && videoProcessorThread) {
            QMetaObject::invokeMethod(videoProcessor,
                                      [processor = videoProcessor, analogVideoTest]() {
                                          processor->setTestPatternEnabled(analogVideoTest);
                                      },
                                      Qt::QueuedConnection);
            QMetaObject::invokeMethod(videoProcessor,
                                      [processor = videoProcessor, sstvTest]() {
                                          processor->setSstvTestPatternEnabled(sstvTest);
                                      },
                                      Qt::QueuedConnection);
            QMetaObject::invokeMethod(videoProcessor,
                                      [processor = videoProcessor, aptTest]() {
                                          processor->setAptTestPatternEnabled(aptTest);
                                      },
                                      Qt::QueuedConnection);
            QMetaObject::invokeMethod(videoProcessor,
                                      [processor = videoProcessor, wefaxTest]() {
                                          processor->setWefaxTestPatternEnabled(wefaxTest);
                                      },
                                      Qt::QueuedConnection);
        }
        updateVideoProcessorMode();
        updateIqFrameProducerSettings();
    });
    connect(videoProcessor, &VideoProcessor::frameReady, videoWidget, &VideoWidget::setFrame);
    connect(videoProcessor, &VideoProcessor::statusChanged, this, &YourClassName::onVideoStatusChanged);
    connect(recordingManager, &RecordingManager::statusChanged, this, &YourClassName::updateRecordingStatus);
    connect(recordButton, &QPushButton::toggled, this, [this](bool checked) {
        if (checked) {
            startRecording(false);
        } else {
            stopRecording(false);
        }
    });
    connect(playbackRefreshButton, &QPushButton::clicked, this, &YourClassName::refreshPlaybackFiles);
    connect(playbackButton, &QPushButton::toggled, this, [this](bool checked) {
        if (checked) {
            startPlayback();
        } else {
            stopPlayback();
        }
    });
    connect(playbackManager, &PlaybackManager::audioFrameReady, this, &YourClassName::handlePlaybackAudioFrame);
    connect(playbackManager, &PlaybackManager::iqFrameReady, this, &YourClassName::handlePlaybackIqFrame);
    connect(playbackManager, &PlaybackManager::started, this, &YourClassName::onPlaybackStarted);
    connect(playbackManager, &PlaybackManager::stopped, this, &YourClassName::onPlaybackStopped);
    connect(playbackManager, &PlaybackManager::statusChanged, this, &YourClassName::onPlaybackStatusChanged);
    connect(audioProcessor,
            &AudioProcessor::audioFrameReady,
            this,
            [this](const QByteArray &pcmData) {
                if (recordingManager &&
                    recordingManager->isRecording() &&
                    recordingManager->mode() == RecordingManager::Mode::AudioWav) {
                    recordingManager->appendAudioFrame(pcmData);
                }
                processDigitalAudioFrame(pcmData);
                processSstvAudioFrame(pcmData);
                processAptAudioFrame(pcmData);
                processWefaxAudioFrame(pcmData);
                sendNetworkAudioFrame(pcmData);
            },
            Qt::QueuedConnection);
    connect(audioCheckbox, &QCheckBox::toggled,
            this, &YourClassName::onAudioEnabledChanged);
    connect(sampleBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &YourClassName::onSampleRateChanged);
    connect(bandwidthControl, &FrequencyControl::valueCommitted, this, [this](double) {
        onBandwidthChanged();
    });
    connect(scaleWidget, SIGNAL(frequencyChanged()), this, SLOT(updateFrequency()));
    connect(scaleWidget, SIGNAL(centralFrequencyChanged()), this, SLOT(updateCentralFrequency()));
    connect(scaleWidget, &ScaleWidget::tuningChanged, this, &YourClassName::updateTuningFromScale);
    //connect(scaleWidget, &ScaleWidget::frequencyChanged, this, &YourClassName::updateFrequency);
    //connect(scaleWidget, &ScaleWidget::centralFrequencyChanged, this, &YourClassName::updateCentralFrequency);
    //connect(waterfallWidget, SIGNAL(scaleChanged(int delta)), this, SLOT(onWaterfallScaleChanged(int delta)));
    //connect(graphWidget, SIGNAL(scaleChanged(int delta)), this, SLOT(onWaterfallScaleChanged(int delta)));
    connect(waterfallWidget, &MyWaterfallWidget::scaleChanged, this, &YourClassName::onWaterfallScaleChanged);
    connect(graphWidget, &MyGraphWidget::scaleChanged, this, &YourClassName::onWaterfallScaleChanged);
    connect(graphWidget, &MyGraphWidget::tuneContextRequested, this, &YourClassName::showTuneContextMenu);
    connect(waterfallWidget, &MyWaterfallWidget::tuneContextRequested, this, &YourClassName::showTuneContextMenu);
    onFrequencyEntered();
    onVgaGainChanged(3);
    onLnaGainChanged(1);
    populateSampleRates();
    populateAudioDevices();
    refreshSettingsFromUi();
    loadPersistentSettings();
    updateUiFromPendingSettings();
    publishSettingsToGlobals();
    updateDigitalDecoderMode();
    updateVideoProcessorMode();
    updateUiForRunState();
    refreshPlaybackFiles();
    qApp->installEventFilter(this);
}

YourClassName::~YourClassName() {
    qApp->removeEventFilter(this);
    refreshSettingsFromUi();
    savePersistentSettings();
    stopPlayback();
    stopRecording(false);

    if (stopPollTimer) {
        stopPollTimer->stop();
    }
    if (streamWatchdogTimer) {
        streamWatchdogTimer->stop();
    }
    if (videoSnapshotTimer) {
        videoSnapshotTimer->stop();
    }
    pendingAudioStartAfterStreamReady = false;
    pendingNetworkAudioStartAfterIqPrebuffer = false;
    pendingPlaybackAudioStartAfterIqPrebuffer = false;
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

    if (digitalDecoderThread) {
        digitalDecoderThread->quit();
        if (!digitalDecoderThread->wait(3000)) {
            qDebug() << "[Digital] decoder thread did not stop in time; terminating";
            digitalDecoderThread->terminate();
            digitalDecoderThread->wait();
        }
        digitalDecoder = nullptr;
        digitalDecoderThread = nullptr;
    }

    if (videoProcessorThread) {
        videoProcessorThread->quit();
        if (!videoProcessorThread->wait(3000)) {
            qDebug() << "[Video] processor thread did not stop in time; terminating";
            videoProcessorThread->terminate();
            videoProcessorThread->wait();
        }
        videoProcessor = nullptr;
        videoProcessorThread = nullptr;
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
    if (agileDevice) {
        closeFobosAgileDeviceSafely(agileDevice);
        agileDevice = nullptr;
    }
    if (iqData) {
        iqData = nullptr;
    }
}

bool YourClassName::eventFilter(QObject *watched, QEvent *event) {
    Q_UNUSED(watched);
    if (event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease) {
        auto *keyEvent = static_cast<QKeyEvent *>(event);
        if (keyEvent && keyEvent->key() == Qt::Key_F9 && !keyEvent->isAutoRepeat()) {
            if (event->type() == QEvent::KeyPress) {
                startRecording(true);
            } else {
                stopRecording(true);
            }
            return true;
        }
    }
    return QMainWindow::eventFilter(watched, event);
}

bool YourClassName::restartStreamForHardwareChange() {
    if (isChannelIqRecordingActive() &&
        networkMode != NetworkMode::Disabled &&
        isFullIqProcessingMode()) {
        stopRecording(false);
        updateRecordingStatus(QStringLiteral("Recording stopped: Channel IQ cannot run during Full IQ streaming"));
    }

    if (isIdle()) {
        if (!hasActiveFobosDevice()) {
            qDebug() << "[LiveHardware] settings changed while idle; no open Fobos session to restart";
            return true;
        }
        return applyFobosSettings();
    }

    if (!hasActiveFobosDevice()) {
        qDebug() << "[LiveHardware] cannot restart stream because Fobos session is missing";
        deviceOpened = false;
        runState = RadioRunState::Idle;
        updateUiForRunState();
        return false;
    }

    qDebug() << "[LiveHardware] restarting stream for hardware change";

    runState = RadioRunState::Stopping;
    updateUiForRunState();
    if (streamWatchdogTimer) streamWatchdogTimer->stop();
    if (stopPollTimer) stopPollTimer->stop();
    if (updateTimer) updateTimer->stop();
    pendingAudioStartAfterStreamReady = false;
    if (audioProcessor) audioProcessor->stopDemodulation();

    if (processor) {
        processor->requestStop();
        if (processor->isRunning() && !processor->wait(1500)) {
            const bool forced = processor->forceStop(1000);
            qDebug() << "[LiveHardware] forced DataProcessor stop during live restart" << forced;
            if (!forced || processor->isRunning()) {
                qDebug() << "[LiveHardware] DataProcessor is still running; live restart is deferred to stop recovery";
                stopCancelRetryCount = 0;
                stopElapsedTimer.restart();
                if (stopPollTimer) stopPollTimer->start();
                updateUiForRunState();
                return false;
            }
        }
        processor->finalizeStopped();
    }

    IqBuffer::clear();

    runState = RadioRunState::Starting;
    updateUiForRunState();
    if (!applyFobosSettings()) {
        qDebug() << "[LiveHardware] applyFobosSettings failed";
        closeFobosSession(false);
        deviceOpened = false;
        runState = RadioRunState::Idle;
        updateUiForRunState();
        return false;
    }

    fftResult = std::make_unique<FFTResult>();
    updateSpectrumTimerInterval();
    settingRange();

    const bool serverIqStreaming = networkMode == NetworkMode::Server && isClientIqProcessingMode();
    const bool serverFullIqStreaming = networkMode == NetworkMode::Server && isFullIqProcessingMode();
    const bool serverChannelIqStreaming = networkMode == NetworkMode::Server && isChannelIqProcessingMode();
    const bool channelIqRecording = isChannelIqRecordingActive();
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    const bool serverLocalAudioEnabled = pendingSettings.audioEnabled && !serverIqStreaming;
    const bool queueAudioBlocks = !serverIqStreaming;
    if (audioProcessor) {
        audioProcessor->setLocalPlaybackEnabled(!suppressServerLocalOutput);
    }

    if ((serverIqStreaming || channelIqRecording) && processor) {
        processor->configureNetworkIqStreaming(pendingSettings,
                                               true,
                                               serverChannelIqStreaming || channelIqRecording);
    }

    processor->startProcessing(activeFobosDevice(),
                               activeFobosApiKind,
                               pendingSettings.syncEnabled,
                               pendingSettings.sampleRate,
                               queueAudioBlocks,
                               serverIqStreaming || channelIqRecording);

    deviceOpened = true;

    if (updateTimer && !(serverFullIqStreaming && suppressServerLocalOutput)) {
        updateTimer->start();
    }

    pendingAudioStartAfterStreamReady = serverLocalAudioEnabled;
    streamStartCallbackCount = processor ? processor->callbackCount() : 0;
    streamStartElapsedTimer.restart();
    if (streamWatchdogTimer) streamWatchdogTimer->start();

    runState = RadioRunState::Running;
    updateUiForRunState();
    applyServerLocalOutputPolicy();
    return true;
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
        bool ok = false;
        const int selectedIndex = comboBox->currentData().toInt(&ok);
        pendingSettings.deviceIndex = ok ? selectedIndex : std::max(0, comboBox->currentIndex());
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
    if (frequencyControl) {
        double frequency = frequencyControl->valueHz();
        if (pendingSettings.inputMode == 0 && frequency < 50000000.0) {
            frequency = 50000000.0;
        }
        pendingSettings.centerFrequency = pendingSettings.inputMode == 0 ? frequency : 0.0;
    }
    if (listeningFrequencyControl) {
        pendingSettings.listeningFrequency = listeningFrequencyControl->valueHz();
    }
    if (bandwidthControl) {
        const double bandwidth = bandwidthControl->valueHz();
        if (bandwidth > 0.0) {
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
    if (audioLowPassSlider) {
        pendingSettings.audioLowPassHz = audioLowPassSliderValueToHz(audioLowPassSlider->value());
    }
    if (audioHighPassSlider) {
        pendingSettings.audioHighPassHz = audioHighPassSliderValueToHz(audioHighPassSlider->value());
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
        audioProcessor->configure(audioProcessorSettings());
    }
    const bool iqFrameProducerActive =
        isChannelIqRecordingActive() ||
        (networkMode == NetworkMode::Server && isClientIqProcessingMode());
    if (runState == RadioRunState::Running && iqFrameProducerActive) {
        updateIqFrameProducerSettings();
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

bool YourClassName::isClientIqProcessingMode(NetworkProcessingMode mode) const {
    return mode == NetworkProcessingMode::ChannelIqClientSide ||
           mode == NetworkProcessingMode::FullIqClientSide;
}

RecordingManager::Mode YourClassName::selectedRecordingMode() const {
    if (!recordingModeCombo) {
        return RecordingManager::Mode::AudioWav;
    }
    bool ok = false;
    const int value = recordingModeCombo->currentData().toInt(&ok);
    return ok ? static_cast<RecordingManager::Mode>(value) : RecordingManager::Mode::AudioWav;
}

QJsonObject YourClassName::recordingLabMetadata() const {
    QJsonObject lab;
    if (!dmrLabCaptureCheckbox || !dmrLabCaptureCheckbox->isChecked()) {
        return lab;
    }

    lab["schema"] = QStringLiteral("dmr-lab-capture");
    lab["schemaVersion"] = 1;
    lab["mode"] = QStringLiteral("DMR");
    lab["description"] = QStringLiteral("Expected values are user-supplied lab references for decoder verification.");

    if (dmrLabColorCodeCombo) {
        const int colorCode = dmrLabColorCodeCombo->currentData().toInt();
        if (colorCode >= 0) {
            lab["expectedColorCode"] = colorCode;
        }
    }
    if (dmrLabSlotCombo) {
        const int slot = dmrLabSlotCombo->currentData().toInt();
        if (slot == 1 || slot == 2) {
            lab["expectedTimeslot"] = slot;
        }
    }
    if (dmrLabCallTypeCombo) {
        const QString callType = dmrLabCallTypeCombo->currentData().toString();
        if (!callType.isEmpty() && callType != QStringLiteral("unknown")) {
            lab["expectedCallType"] = callType;
        }
    }

    auto addText = [&lab](const QString &key, const QLineEdit *edit) {
        if (!edit) {
            return;
        }
        const QString text = edit->text().trimmed();
        if (!text.isEmpty()) {
            lab[key] = text;
        }
    };
    addText(QStringLiteral("expectedSourceId"), dmrLabSourceIdEdit);
    addText(QStringLiteral("expectedTargetId"), dmrLabTargetIdEdit);
    addText(QStringLiteral("radio"), dmrLabRadioEdit);
    addText(QStringLiteral("notes"), dmrLabNotesEdit);
    return lab;
}

bool YourClassName::isChannelIqRecordingActive() const {
    return recordingManager &&
           recordingManager->isRecording() &&
           recordingManager->mode() == RecordingManager::Mode::ChannelIqWav;
}

void YourClassName::updateIqFrameProducerSettings() {
    if (!processor) {
        return;
    }

    const bool serverIqStreaming = networkMode == NetworkMode::Server && isClientIqProcessingMode();
    const bool serverFullIqStreaming = networkMode == NetworkMode::Server && isFullIqProcessingMode();
    const bool serverChannelIqStreaming = networkMode == NetworkMode::Server && isChannelIqProcessingMode();
    const bool channelIqRecording = isChannelIqRecordingActive();
    processor->configureNetworkIqStreaming(pendingSettings,
                                           serverIqStreaming || channelIqRecording,
                                           serverChannelIqStreaming || channelIqRecording);
}

void YourClassName::updateRecordingStatus(const QString &status) {
    if (recordingStatusLabel) {
        recordingStatusLabel->setText(status);
    }
}

QString YourClassName::selectedPlaybackFilePath() const {
    if (!playbackFileCombo) {
        return QString();
    }
    return playbackFileCombo->currentData().toString();
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
                if (recordingManager &&
                    recordingManager->isRecording() &&
                    recordingManager->mode() == RecordingManager::Mode::ChannelIqWav) {
                    recordingManager->appendIqFrame(iqData, sampleRate, sampleCount);
                }
                processVideoIqFrame(iqData, sampleRate, sampleCount);
                sendNetworkIqFrame(iqData, sampleRate, sampleCount);
            },
            Qt::QueuedConnection);
}

void YourClassName::applyServerLocalOutputPolicy() {
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();

    if (audioProcessor) {
        audioProcessor->setLocalPlaybackEnabled(!suppressServerLocalOutput);
    }
    updateVideoProcessorMode();
    updateIqFrameProducerSettings();

    if (!updateTimer || runState != RadioRunState::Running || networkMode != NetworkMode::Server) {
        return;
    }

    const bool spectrumNeededForNetwork =
        networkController &&
        networkController->isControlReady() &&
        !isFullIqProcessingMode();
    const bool localVisualNeeded = !suppressServerLocalOutput;

    if (spectrumNeededForNetwork || localVisualNeeded) {
        if (!updateTimer->isActive()) {
            updateTimer->start();
        }
    } else if (updateTimer->isActive()) {
        updateTimer->stop();
    }
}

bool YourClassName::applyCenterFrequencyToHardwareIfNeeded(const RadioSettings &previousSettings,
                                                           const char *reason) {
    if (pendingSettings.inputMode != 0 ||
        isIdle() ||
        !hasActiveFobosDevice() ||
        std::abs(previousSettings.centerFrequency - pendingSettings.centerFrequency) <= 0.5) {
        return true;
    }

    double tunedFrequency = pendingSettings.centerFrequency;
    const int result = setActiveFrequencySafely(pendingSettings.centerFrequency, &tunedFrequency);
    if (result == FOBOS_ERR_OK) {
        pendingSettings.actualFrequency = tunedFrequency;
        if (hardwareSettingsApplied) {
            appliedHardwareSettings.centerFrequency = pendingSettings.centerFrequency;
            appliedHardwareSettings.actualFrequency = tunedFrequency;
        }
        qDebug() << "[LiveTune]" << reason
                 << "center applied"
                 << "requested" << pendingSettings.centerFrequency
                 << "actual" << tunedFrequency;
        return true;
    }

    qDebug() << "[LiveTune]" << reason
             << "center apply failed"
             << "requested" << pendingSettings.centerFrequency
             << "error" << result;
    return false;
}

void YourClassName::resetNetworkIqReceptionState(bool clearGraph, bool clearWaterfall, bool restartAudioPrebuffer) {
    IqBuffer::clear();
    networkIqStreamMetadataValid = false;
    networkIqStreamWasChannelized = false;
    networkIqStreamSampleRate = 0.0;
    networkIqStreamCenterFrequency = 0.0;
    networkIqStreamListeningFrequency = 0.0;
    networkIqStreamInputMode = 0;
    networkSpectrumFrameMetadataValid = false;
    networkSpectrumFrameMinFrequency = 0.0;
    networkSpectrumFrameMaxFrequency = 0.0;
    networkSpectrumFrameFftLength = 0;

    if (audioProcessor) {
        audioProcessor->stopDemodulation();
    }
    pendingNetworkAudioStartAfterIqPrebuffer = restartAudioPrebuffer && pendingSettings.audioEnabled;

    if (clearGraph && graphWidget) {
        graphWidget->clearData();
    }
    if (clearWaterfall && waterfallWidget) {
        waterfallWidget->clearData();
    }

    fftResult = std::make_unique<FFTResult>();
    spectrumDebugFramesRemaining = fobosVerboseLoggingEnabled() ? 12 : 0;
    updateSpectrumTimerInterval();
}

void YourClassName::startNetworkClientProcessing() {
    if (!isNetworkClientMode() || !isClientIqProcessingMode()) {
        return;
    }

    if (networkClientIqProcessingActive &&
        activeNetworkClientProcessingMode == networkProcessingMode &&
        runState == RadioRunState::Running) {
        if (audioProcessor) {
            audioProcessor->setLocalPlaybackEnabled(true);
        }
        if (updateTimer && isFullIqProcessingMode() && !updateTimer->isActive()) {
            updateTimer->start();
        }
        return;
    }

    if (remoteAudioPlayer) {
        remoteAudioPlayer->stop();
    }
    if (updateTimer) {
        updateTimer->stop();
    }
    if (audioProcessor) {
        audioProcessor->setLocalPlaybackEnabled(true);
    }

    const bool switchingBetweenClientIqModes =
        networkClientIqProcessingActive &&
        activeNetworkClientProcessingMode != networkProcessingMode;
    resetNetworkIqReceptionState(false, switchingBetweenClientIqModes, pendingSettings.audioEnabled);
    IqBuffer::setSampleRateEstimate(pendingSettings.sampleRate);
    publishSettingsToGlobals();

    if (updateTimer && isFullIqProcessingMode()) {
        updateTimer->start();
    }
    networkClientIqProcessingActive = true;
    activeNetworkClientProcessingMode = networkProcessingMode;

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
    resetNetworkIqReceptionState(false, false, false);
    networkClientIqProcessingActive = false;
    activeNetworkClientProcessingMode = NetworkProcessingMode::ServerSide;
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
    settings["audioLowPassHz"] = pendingSettings.audioLowPassHz;
    settings["audioHighPassHz"] = pendingSettings.audioHighPassHz;
    settings["audioEnabled"] = pendingSettings.audioEnabled;
    settings["syncEnabled"] = false;
    settings["gpoValue"] = static_cast<int>(pendingSettings.gpoValue);
    settings["scalePercent"] = currentScale;
    return settings;
}

bool YourClassName::sendRemoteControlCommand(const QString &action, const QJsonObject &extra) {
    if (!networkController || networkMode != NetworkMode::Client) {
        return false;
    }

    if (action != QStringLiteral("settings")) {
        cancelPendingRemoteSettingsCommand();
    }

    const bool controlActionAllowed =
        action == QStringLiteral("requestPriority") ||
        action == QStringLiteral("priorityResponse");
    if (!networkController->clientHasControl() && !controlActionAllowed) {
        qDebug() << "[Network] remote command blocked because this client is observer" << action;
        return false;
    }

    refreshSettingsFromUi();

    QJsonObject command;
    command["type"] = "control";
    command["action"] = action;
    appendNetworkState(command);
    command["settings"] = settingsToJson();
    for (auto it = extra.constBegin(); it != extra.constEnd(); ++it) {
        command[it.key()] = it.value();
    }

    const bool sent = networkController->sendControlCommand(command);
    if (!sent) {
        qDebug() << "[Network] remote command could not be sent" << action;
    } else {
        qDebug() << "[Network] remote command sent" << action;
    }
    return sent;
}

void YourClassName::scheduleRemoteSettingsCommand(int delayMs) {
    if (!isNetworkClientMode() ||
        !networkController ||
        !networkController->clientHasControl()) {
        return;
    }

    if (!networkSettingsDebounceTimer) {
        sendRemoteControlCommand("settings");
        return;
    }

    const int clampedDelayMs = (std::clamp)(delayMs, 0, 1000);
    if (clampedDelayMs == 0) {
        networkSettingsDebounceTimer->stop();
        sendRemoteControlCommand("settings");
        return;
    }

    if (!networkSettingsDebounceTimer->isActive()) {
        networkSettingsDebounceTimer->start(clampedDelayMs);
    }
}

void YourClassName::cancelPendingRemoteSettingsCommand() {
    if (networkSettingsDebounceTimer) {
        networkSettingsDebounceTimer->stop();
    }
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
    pendingSettings.audioLowPassHz = clampAudioLowPassHz(readDouble("audioLowPassHz", pendingSettings.audioLowPassHz));
    pendingSettings.audioHighPassHz = clampAudioHighPassHz(readDouble("audioHighPassHz", pendingSettings.audioHighPassHz));
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
    if (frequencyControl) {
        QSignalBlocker blocker(frequencyControl);
        frequencyControl->setValueHz(pendingSettings.centerFrequency);
    }
    if (listeningFrequencyControl) {
        QSignalBlocker blocker(listeningFrequencyControl);
        if (pendingSettings.inputMode == 0) {
            listeningFrequencyControl->setRangeHz(RF_MIN_LISTENING_FREQUENCY, 6000000000.0);
        } else {
            listeningFrequencyControl->setRangeHz(directMinFrequencyForMode(pendingSettings.inputMode,
                                                                            pendingSettings.sampleRate),
                                                  directMaxFrequency(pendingSettings.sampleRate));
        }
        listeningFrequencyControl->setValueHz(pendingSettings.listeningFrequency);
    }
    if (bandwidthControl) {
        QSignalBlocker blocker(bandwidthControl);
        bandwidthControl->setValueHz(pendingSettings.bandwidth);
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
    if (audioDeviceComboBox) {
        audioDeviceComboBox->blockSignals(true);
        const int index = audioDeviceComboBox->findData(pendingSettings.audioDeviceId);
        if (index >= 0) {
            audioDeviceComboBox->setCurrentIndex(index);
        }
        audioDeviceComboBox->blockSignals(false);
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
    if (contrastSlider) {
        contrastSlider->blockSignals(true);
        contrastSlider->setValue(static_cast<int>(std::lround(contrast)));
        contrastSlider->blockSignals(false);
    }
    if (contrastLabel) {
        contrastLabel->setText(QString("Contrast: %1").arg(contrast));
    }
    if (sensitivitySlider) {
        sensitivitySlider->blockSignals(true);
        sensitivitySlider->setValue(static_cast<int>(std::lround(sensitivity)));
        sensitivitySlider->blockSignals(false);
    }
    if (sensitivityLabel) {
        sensitivityLabel->setText(QString("Sensitivity: %1").arg(sensitivity));
    }
    if (levelMinSlider) {
        levelMinSlider->blockSignals(true);
        levelMinSlider->setValue(levelToSliderValue(displayLevelMin));
        levelMinSlider->blockSignals(false);
    }
    if (levelMaxSlider) {
        levelMaxSlider->blockSignals(true);
        levelMaxSlider->setValue(levelToSliderValue(displayLevelMax));
        levelMaxSlider->blockSignals(false);
    }
    if (levelMinLabel) {
        levelMinLabel->setText(levelLabelText("Min", displayLevelMin));
    }
    if (levelMaxLabel) {
        levelMaxLabel->setText(levelLabelText("Max", displayLevelMax));
    }
    if (graphCheckbox) {
        graphCheckbox->blockSignals(true);
        graphCheckbox->setChecked(secondGraph);
        graphCheckbox->blockSignals(false);
    }
    if (colorCheckbox) {
        colorCheckbox->blockSignals(true);
        colorCheckbox->setChecked(colorf);
        colorCheckbox->blockSignals(false);
    }
    if (volumeSlider) {
        volumeSlider->blockSignals(true);
        volumeSlider->setValue(volumePercent);
        volumeSlider->blockSignals(false);
    }
    if (volumeLabel) {
        volumeLabel->setText(QString("Volume: %1%").arg(volumePercent));
    }
    if (audioLowPassSlider) {
        audioLowPassSlider->blockSignals(true);
        audioLowPassSlider->setValue(audioLowPassHzToSliderValue(pendingSettings.audioLowPassHz));
        audioLowPassSlider->blockSignals(false);
    }
    if (audioHighPassSlider) {
        audioHighPassSlider->blockSignals(true);
        audioHighPassSlider->setValue(audioHighPassHzToSliderValue(pendingSettings.audioHighPassHz));
        audioHighPassSlider->blockSignals(false);
    }
    updateAudioFilterLabels();
    if (digitalDecodeCheckbox) {
        digitalDecodeCheckbox->blockSignals(true);
        digitalDecodeCheckbox->setChecked(digitalDecodeEnabled);
        digitalDecodeCheckbox->blockSignals(false);
    }
    if (videoDecodeCheckbox) {
        videoDecodeCheckbox->blockSignals(true);
        videoDecodeCheckbox->setChecked(videoDecodeEnabled);
        videoDecodeCheckbox->blockSignals(false);
    }
    const float volume = volumePercent / 100.0f;
    if (audioProcessor) {
        audioProcessor->setVolume(volume);
    }
    if (remoteAudioPlayer) {
        remoteAudioPlayer->setVolume(volume);
    }
    if (graphWidget) {
        graphWidget->setLevelRange(displayLevelMin, displayLevelMax);
    }
    if (waterfallWidget) {
        waterfallWidget->setLevelRange(displayLevelMin, displayLevelMax);
    }
    updateSpectrumTimerInterval();
    settingRange();
    updateDigitalDecoderMode();
    updateVideoProcessorMode();
}

void YourClassName::loadPersistentSettings() {
    const QString settingsPath = persistentSettingsFilePath();
    const bool settingsFileExists = QFile::exists(settingsPath);
    QSettings settings(settingsPath, QSettings::IniFormat);
    if (settings.status() != QSettings::NoError) {
        qDebug() << "[Settings] unable to read settings file" << settingsPath
                 << "status" << settings.status();
        return;
    }

    pendingSettings.deviceIndex = (std::max)(0, settings.value("receiver/deviceIndex", pendingSettings.deviceIndex).toInt());
    pendingSettings.clockSource = settings.value("receiver/clockSource", pendingSettings.clockSource).toInt();
    pendingSettings.inputMode = settings.value("receiver/inputMode", pendingSettings.inputMode).toInt();
    pendingSettings.centerFrequency = settings.value("receiver/centerFrequency", pendingSettings.centerFrequency).toDouble();
    pendingSettings.actualFrequency = settings.value("receiver/actualFrequency", pendingSettings.actualFrequency).toDouble();
    pendingSettings.listeningFrequency = settings.value("receiver/listeningFrequency", pendingSettings.listeningFrequency).toDouble();
    pendingSettings.sampleRate = (std::max)(1000.0, settings.value("receiver/sampleRate", pendingSettings.sampleRate).toDouble());
    pendingSettings.bandwidth = (std::max)(1.0, settings.value("receiver/bandwidth", pendingSettings.bandwidth).toDouble());
    pendingSettings.modulationType = settings.value("receiver/modulationType", pendingSettings.modulationType).toInt();
    pendingSettings.fftLength = (std::max)(1024, settings.value("receiver/fftLength", pendingSettings.fftLength).toInt());
    pendingSettings.lnaGain = (std::clamp)(settings.value("receiver/lnaGain", pendingSettings.lnaGain).toInt(), 0, 1);
    pendingSettings.vgaGain = (std::clamp)(settings.value("receiver/vgaGain", pendingSettings.vgaGain).toInt(), 0, 31);
    pendingSettings.audioDeviceId = (std::max)(0, settings.value("receiver/audioDeviceId", pendingSettings.audioDeviceId).toInt());
    pendingSettings.audioLowPassHz = clampAudioLowPassHz(settings.value("audio/lowPassHz", pendingSettings.audioLowPassHz).toDouble());
    pendingSettings.audioHighPassHz = clampAudioHighPassHz(settings.value("audio/highPassHz", pendingSettings.audioHighPassHz).toDouble());
    pendingSettings.audioEnabled = settings.value("receiver/audioEnabled", pendingSettings.audioEnabled).toBool();
    pendingSettings.syncEnabled = false;
    pendingSettings.gpoValue = static_cast<std::uint8_t>((std::clamp)(settings.value("receiver/gpoValue", static_cast<int>(pendingSettings.gpoValue)).toInt(), 0, 255));

    currentScale = (std::clamp)(settings.value("display/scalePercent", currentScale).toDouble(),
                                MIN_SCALE_PERCENT,
                                MAX_SCALE_PERCENT);
    contrast = static_cast<float>((std::clamp)(settings.value("display/contrast", static_cast<double>(contrast)).toDouble(), 1.0, 15.0));
    sensitivity = static_cast<float>((std::clamp)(settings.value("display/sensitivity", static_cast<double>(sensitivity)).toDouble(), 1.0, 30.0));
    displayLevelMin = static_cast<float>((std::clamp)(settings.value("display/levelMin", static_cast<double>(displayLevelMin)).toDouble(), -160.0, 20.0));
    displayLevelMax = static_cast<float>((std::clamp)(settings.value("display/levelMax", static_cast<double>(displayLevelMax)).toDouble(), -160.0, 20.0));
    if (displayLevelMin >= displayLevelMax) {
        displayLevelMin = (std::max)(-160.0f, displayLevelMax - MIN_LEVEL_GAP);
    }
    secondGraph = settings.value("display/secondGraph", secondGraph).toBool();
    colorf = settings.value("display/color", colorf).toBool();
    volumePercent = (std::clamp)(settings.value("audio/volumePercent", volumePercent).toInt(), 0, 200);

    networkMode = NetworkMode::Disabled;
    const int processingModeValue = (std::clamp)(settings.value("network/processingMode", static_cast<int>(networkProcessingMode)).toInt(),
                                                 static_cast<int>(NetworkProcessingMode::ServerSide),
                                                 static_cast<int>(NetworkProcessingMode::FullIqClientSide));
    networkProcessingMode = static_cast<NetworkProcessingMode>(processingModeValue);
    networkServerAddress = settings.value("network/serverAddress", networkServerAddress).toString();
    networkBindAddress = settings.value("network/bindAddress", networkBindAddress).toString();
    networkControlPort = static_cast<quint16>((std::clamp)(settings.value("network/controlPort", static_cast<int>(networkControlPort)).toInt(), 1, 65535));
    serverDisableLocalVisualAudio = settings.value("network/serverDisableLocalVisualAudio", serverDisableLocalVisualAudio).toBool();
    digitalDecodeEnabled = settings.value("digital/decodeEnabled", digitalDecodeEnabled).toBool();
    auto setComboToData = [](QComboBox *combo, const QVariant &data) {
        if (!combo) {
            return;
        }
        const int index = combo->findData(data);
        if (index >= 0) {
            combo->setCurrentIndex(index);
        }
    };
    if (dmrLabCaptureCheckbox) {
        dmrLabCaptureCheckbox->setChecked(settings.value("digital/dmrLabCaptureEnabled", false).toBool());
    }
    setComboToData(dmrLabColorCodeCombo, settings.value("digital/dmrLabColorCode", -1));
    setComboToData(dmrLabSlotCombo, settings.value("digital/dmrLabTimeslot", 0));
    setComboToData(dmrLabCallTypeCombo, settings.value("digital/dmrLabCallType", QStringLiteral("unknown")));
    if (dmrLabSourceIdEdit) {
        dmrLabSourceIdEdit->setText(settings.value("digital/dmrLabSourceId").toString());
    }
    if (dmrLabTargetIdEdit) {
        dmrLabTargetIdEdit->setText(settings.value("digital/dmrLabTargetId").toString());
    }
    if (dmrLabRadioEdit) {
        dmrLabRadioEdit->setText(settings.value("digital/dmrLabRadio").toString());
    }
    if (dmrLabNotesEdit) {
        dmrLabNotesEdit->setText(settings.value("digital/dmrLabNotes").toString());
    }
    videoDecodeEnabled = settings.value("video/decodeEnabled", videoDecodeEnabled).toBool();
    if (videoDemodCombo) {
        const int demodMode = settings.value("video/demodMode", VideoProcessor::FmVideo).toInt();
        const int demodIndex = videoDemodCombo->findData(demodMode);
        if (demodIndex >= 0) {
            videoDemodCombo->setCurrentIndex(demodIndex);
        }
    }
    if (videoStandardCombo) {
        const int standardIndex = (std::clamp)(settings.value("video/standardIndex", 0).toInt(),
                                               0,
                                               (std::max)(0, videoStandardCombo->count() - 1));
        videoStandardCombo->setCurrentIndex(standardIndex);
    }
    if (videoInvertCheckbox) {
        videoInvertCheckbox->setChecked(settings.value("video/invert", false).toBool());
    }
    if (videoHSyncCheckbox) {
        videoHSyncCheckbox->setChecked(settings.value("video/hSync", true).toBool());
    }
    if (videoVSyncCheckbox) {
        videoVSyncCheckbox->setChecked(settings.value("video/vSync", true).toBool());
    }

    normalizeTuning(pendingSettings);
    qDebug() << (settingsFileExists ? "[Settings] loaded" : "[Settings] using defaults; settings file will be created on clean exit")
             << settingsPath
             << "sampleRate" << pendingSettings.sampleRate
             << "center" << pendingSettings.centerFrequency
             << "listening" << pendingSettings.listeningFrequency;
}

void YourClassName::savePersistentSettings() {
    QSettings settings(persistentSettingsFilePath(), QSettings::IniFormat);

    settings.setValue("receiver/deviceIndex", pendingSettings.deviceIndex);
    settings.setValue("receiver/clockSource", pendingSettings.clockSource);
    settings.setValue("receiver/inputMode", pendingSettings.inputMode);
    settings.setValue("receiver/centerFrequency", pendingSettings.centerFrequency);
    settings.setValue("receiver/actualFrequency", pendingSettings.actualFrequency);
    settings.setValue("receiver/listeningFrequency", pendingSettings.listeningFrequency);
    settings.setValue("receiver/sampleRate", pendingSettings.sampleRate);
    settings.setValue("receiver/bandwidth", pendingSettings.bandwidth);
    settings.setValue("receiver/modulationType", pendingSettings.modulationType);
    settings.setValue("receiver/fftLength", pendingSettings.fftLength);
    settings.setValue("receiver/lnaGain", pendingSettings.lnaGain);
    settings.setValue("receiver/vgaGain", pendingSettings.vgaGain);
    settings.setValue("receiver/audioDeviceId", pendingSettings.audioDeviceId);
    settings.setValue("receiver/audioEnabled", pendingSettings.audioEnabled);
    settings.setValue("receiver/gpoValue", static_cast<int>(pendingSettings.gpoValue));

    settings.setValue("display/scalePercent", currentScale);
    settings.setValue("display/contrast", contrast);
    settings.setValue("display/sensitivity", sensitivity);
    settings.setValue("display/levelMin", displayLevelMin);
    settings.setValue("display/levelMax", displayLevelMax);
    settings.setValue("display/secondGraph", secondGraph);
    settings.setValue("display/color", colorf);
    settings.setValue("audio/volumePercent", volumePercent);
    settings.setValue("audio/lowPassHz", pendingSettings.audioLowPassHz);
    settings.setValue("audio/highPassHz", pendingSettings.audioHighPassHz);

    settings.setValue("network/serverAddress", networkServerAddress);
    settings.setValue("network/bindAddress", networkBindAddress);
    settings.setValue("network/controlPort", static_cast<int>(networkControlPort));
    settings.setValue("network/processingMode", static_cast<int>(networkProcessingMode));
    settings.setValue("network/serverDisableLocalVisualAudio", serverDisableLocalVisualAudio);
    settings.setValue("digital/decodeEnabled", digitalDecodeEnabled);
    settings.setValue("digital/dmrLabCaptureEnabled", dmrLabCaptureCheckbox && dmrLabCaptureCheckbox->isChecked());
    settings.setValue("digital/dmrLabColorCode", dmrLabColorCodeCombo ? dmrLabColorCodeCombo->currentData().toInt() : -1);
    settings.setValue("digital/dmrLabTimeslot", dmrLabSlotCombo ? dmrLabSlotCombo->currentData().toInt() : 0);
    settings.setValue("digital/dmrLabCallType", dmrLabCallTypeCombo ? dmrLabCallTypeCombo->currentData().toString() : QStringLiteral("unknown"));
    settings.setValue("digital/dmrLabSourceId", dmrLabSourceIdEdit ? dmrLabSourceIdEdit->text().trimmed() : QString());
    settings.setValue("digital/dmrLabTargetId", dmrLabTargetIdEdit ? dmrLabTargetIdEdit->text().trimmed() : QString());
    settings.setValue("digital/dmrLabRadio", dmrLabRadioEdit ? dmrLabRadioEdit->text().trimmed() : QString());
    settings.setValue("digital/dmrLabNotes", dmrLabNotesEdit ? dmrLabNotesEdit->text().trimmed() : QString());
    settings.setValue("video/decodeEnabled", videoDecodeEnabled);
    settings.setValue("video/demodMode", videoDemodCombo ? videoDemodCombo->currentData().toInt() : VideoProcessor::FmVideo);
    settings.setValue("video/standardIndex", videoStandardCombo ? videoStandardCombo->currentIndex() : 0);
    settings.setValue("video/invert", videoInvertCheckbox && videoInvertCheckbox->isChecked());
    settings.setValue("video/hSync", !videoHSyncCheckbox || videoHSyncCheckbox->isChecked());
    settings.setValue("video/vSync", !videoVSyncCheckbox || videoVSyncCheckbox->isChecked());
    settings.sync();

    if (settings.status() == QSettings::NoError) {
        qDebug() << "[Settings] saved" << persistentSettingsFilePath();
    } else {
        qDebug() << "[Settings] save failed" << persistentSettingsFilePath()
                 << "status" << settings.status();
    }
}

void YourClassName::applyLiveRemoteSettings(const RadioSettings &previousSettings) {
    if (!hasActiveFobosDevice() || isIdle()) {
        return;
    }

    applyCenterFrequencyToHardwareIfNeeded(previousSettings, "remote settings");

    if (hardwareSettingsApplied && previousSettings.lnaGain != pendingSettings.lnaGain) {
        const int result = setActiveLnaGainSafely(static_cast<unsigned int>(pendingSettings.lnaGain));
        qDebug() << "[Network] remote LNA apply result" << result;
        if (result == FOBOS_ERR_OK) {
            appliedHardwareSettings.lnaGain = pendingSettings.lnaGain;
        }
    }
    if (hardwareSettingsApplied && previousSettings.vgaGain != pendingSettings.vgaGain) {
        const int result = setActiveVgaGainSafely(static_cast<unsigned int>(pendingSettings.vgaGain));
        qDebug() << "[Network] remote VGA apply result" << result;
        if (result == FOBOS_ERR_OK) {
            appliedHardwareSettings.vgaGain = pendingSettings.vgaGain;
        }
    }
    if (hardwareSettingsApplied && previousSettings.gpoValue != pendingSettings.gpoValue) {
        const int result = setActiveGpoSafely(pendingSettings.gpoValue);
        qDebug() << "[Network] remote GPO apply result" << result;
        if (result == FOBOS_ERR_OK) {
            appliedHardwareSettings.gpoValue = pendingSettings.gpoValue;
        }
    }

    if (std::abs(previousSettings.sampleRate - pendingSettings.sampleRate) > 0.5 ||
        previousSettings.inputMode != pendingSettings.inputMode ||
        previousSettings.clockSource != pendingSettings.clockSource) {
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

    if (networkMode == NetworkMode::Client && command.value("type").toString() == "control") {
        const QString action = command.value("action").toString();
        if (action == QStringLiteral("role")) {
            const bool canControl = command.value("canControl").toBool(true);
            const QString peerLabel = command.value("peerLabel").toString();
            qDebug() << "[Network] client role update"
                     << "canControl" << canControl
                     << "peer" << peerLabel
                     << "controllerPeerId" << command.value("controllerPeerId").toString();
            updateUiForRunState();
            updateNetworkButtonText();
            return;
        }

        if (action == QStringLiteral("priorityRequest")) {
            if (!networkController || !networkController->clientHasControl()) {
                return;
            }

            const QString requesterId = command.value("requesterId").toString();
            const QString requesterLabel = command.value("requesterLabel").toString(QStringLiteral("unknown client"));
            QMessageBox box(this);
            box.setWindowTitle("Control Request");
            box.setText(QString("Client %1 requests control of the receiver.").arg(requesterLabel));
            box.setInformativeText("If you allow it, this client will become observer.");
            box.setIcon(QMessageBox::Question);
            QCheckBox blockRequesterCheck("Block further requests from this client", &box);
            box.setCheckBox(&blockRequesterCheck);
            QPushButton *allowButton = box.addButton("Allow", QMessageBox::AcceptRole);
            box.addButton("Deny", QMessageBox::RejectRole);
            box.exec();

            QJsonObject response;
            response["requesterId"] = requesterId;
            response["accepted"] = box.clickedButton() == allowButton;
            response["blocked"] = blockRequesterCheck.isChecked();
            sendRemoteControlCommand("priorityResponse", response);
            return;
        }

        if (action == QStringLiteral("priorityDenied") ||
            action == QStringLiteral("controlRejected")) {
            const QString reason = command.value("reason").toString("Request denied");
            qDebug() << "[Network]" << reason;
            QMessageBox::information(this, "Network Control", reason);
            return;
        }

        return;
    }

    if (networkMode != NetworkMode::Server || command.value("type").toString() != "control") {
        return;
    }

    const QString action = command.value("action").toString();
    const QJsonObject settingsJson = command.value("settings").toObject();
    const QString peerId = command.value("_networkPeerId").toString();
    const QString peerLabel = command.value("_networkPeerLabel").toString(QStringLiteral("unknown client"));
    const bool peerIsController = command.value("_networkPeerIsController").toBool(true);

    if (action == QStringLiteral("requestPriority")) {
        if (!networkController || peerId.isEmpty()) {
            return;
        }

        if (peerIsController) {
            QJsonObject role;
            role["type"] = "control";
            role["action"] = "role";
            role["canControl"] = true;
            networkController->sendControlCommandToPeer(peerId, role);
            return;
        }

        if (networkController->isPriorityRequestBlocked(peerId)) {
            QJsonObject denied;
            denied["type"] = "control";
            denied["action"] = "priorityDenied";
            denied["reason"] = "Control request is blocked by current controller";
            networkController->sendControlCommandToPeer(peerId, denied);
            qDebug() << "[Network] blocked control request from" << peerLabel;
            return;
        }

        if (networkController->controllerPeerId().isEmpty()) {
            networkController->setControllerPeer(peerId);
            return;
        }

        QJsonObject request;
        request["type"] = "control";
        request["action"] = "priorityRequest";
        request["requesterId"] = peerId;
        request["requesterLabel"] = peerLabel;
        if (!networkController->sendControlCommandToController(request)) {
            qDebug() << "[Network] controller unavailable; granting control to requester" << peerLabel;
            networkController->setControllerPeer(peerId);
        }
        return;
    }

    if (action == QStringLiteral("priorityResponse")) {
        if (!networkController || !peerIsController) {
            return;
        }

        const QString requesterId = command.value("requesterId").toString();
        const bool accepted = command.value("accepted").toBool(false);
        const bool blocked = command.value("blocked").toBool(false);
        if (blocked && !accepted) {
            networkController->blockPriorityRequestsFromPeer(requesterId);
        }

        if (accepted && networkController->setControllerPeer(requesterId)) {
            qDebug() << "[Network] control transferred to" << requesterId;
        } else {
            QJsonObject denied;
            denied["type"] = "control";
            denied["action"] = "priorityDenied";
            denied["reason"] = blocked
                                    ? "Control request denied and further requests were blocked"
                                    : "Control request denied";
            networkController->sendControlCommandToPeer(requesterId, denied);
        }
        return;
    }

    if (!peerIsController) {
        QJsonObject rejected;
        rejected["type"] = "control";
        rejected["action"] = "controlRejected";
        rejected["reason"] = "This client is observer. Request control in Network Settings first.";
        if (networkController && !peerId.isEmpty()) {
            networkController->sendControlCommandToPeer(peerId, rejected);
        }
        qDebug() << "[Network] observer command rejected"
                 << "action" << action
                 << "peer" << peerLabel;
        return;
    }

    const NetworkProcessingMode previousProcessingMode = networkProcessingMode;
    const bool previousServerDisableLocalVisualAudio = serverDisableLocalVisualAudio;
    applyNetworkStateFromCommand(command);
    const bool serverLocalOutputChanged =
        previousServerDisableLocalVisualAudio != serverDisableLocalVisualAudio;
    qDebug() << "[Network] received remote control command" << action;

    if (action == "settings" || action == "start") {
        const RadioSettings previousSettings = pendingSettings;
        applySettingsFromJson(settingsJson);
        publishSettingsToGlobals();
        updateUiFromPendingSettings();
        applyLiveRemoteSettings(previousSettings);
        const bool audioChanged =
            previousSettings.audioEnabled != pendingSettings.audioEnabled;
        const bool fftChanged =
            previousSettings.fftLength != pendingSettings.fftLength;
        const bool streamModeChanged =
            previousProcessingMode != networkProcessingMode;

        if (fftChanged) {
            applyFftLengthChange(pendingSettings.fftLength, false);
        }

        if (audioChanged && !isIdle()) {
            if (pendingSettings.audioEnabled) {
                if (audioProcessor) {
                    audioProcessor->startDemodulation();
                }
            } else {
                if (audioProcessor) {
                    audioProcessor->stopDemodulation();
                }
                pendingAudioStartAfterStreamReady = false;
            }
        }
        const bool restartRequired =
            std::abs(previousSettings.sampleRate - pendingSettings.sampleRate) > 0.5 ||
            previousSettings.inputMode != pendingSettings.inputMode ||
            previousSettings.clockSource != pendingSettings.clockSource ||
            streamModeChanged;

        if (restartRequired && !isIdle()) {
            restartStreamForHardwareChange();
        } else if (serverLocalOutputChanged && !isIdle()) {
            applyServerLocalOutputPolicy();
        }

        if (processor && processor->isRunning()) {
            updateIqFrameProducerSettings();
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

    const int dataCount = std::min(static_cast<int>(frequencies.size()), static_cast<int>(magnitudes.size()));
    if (dataCount <= 0) {
        return;
    }
    if (isChannelIqProcessingMode() &&
        networkController->pendingBytes() > NETWORK_SPECTRUM_MAX_PENDING_BYTES) {
        return;
    }

    const int minIntervalMs = isChannelIqProcessingMode()
                                  ? NETWORK_CHANNEL_SPECTRUM_INTERVAL_MS
                                  : NETWORK_SPECTRUM_INTERVAL_MS;
    if (networkSpectrumFrameTimer.isValid() &&
        networkSpectrumFrameTimer.elapsed() < minIntervalMs) {
        return;
    }
    networkSpectrumFrameTimer.restart();

    const int maxBins = isChannelIqProcessingMode()
                            ? NETWORK_CHANNEL_SPECTRUM_MAX_BINS
                            : NETWORK_SPECTRUM_MAX_BINS;

    double frameMinFrequency = minFrequency;
    double frameMaxFrequency = maxFrequency;
    if (!std::isfinite(frameMinFrequency) ||
        !std::isfinite(frameMaxFrequency) ||
        frameMaxFrequency <= frameMinFrequency) {
        frameMinFrequency = frequencies.front();
        frameMaxFrequency = frequencies.back();
    }

    auto lower = std::lower_bound(frequencies.begin(), frequencies.begin() + dataCount, static_cast<float>(frameMinFrequency));
    auto upper = std::upper_bound(frequencies.begin(), frequencies.begin() + dataCount, static_cast<float>(frameMaxFrequency));
    int sourceStart = static_cast<int>(std::distance(frequencies.begin(), lower));
    int sourceEnd = static_cast<int>(std::distance(frequencies.begin(), upper));
    sourceStart = (std::max)(0, sourceStart - 1);
    sourceEnd = (std::min)(dataCount, sourceEnd + 1);
    if (sourceEnd <= sourceStart) {
        sourceStart = 0;
        sourceEnd = dataCount;
        frameMinFrequency = frequencies.front();
        frameMaxFrequency = frequencies.back();
    }

    const int sourceCount = sourceEnd - sourceStart;
    const int targetCount = (std::min)(maxBins, sourceCount);
    if (targetCount <= 0) {
        return;
    }
    const double step = static_cast<double>(sourceCount) / static_cast<double>(targetCount);
    QJsonArray frequencyArray;
    QJsonArray magnitudeArray;
    std::vector<float> resampledMagnitudes(static_cast<std::size_t>(targetCount),
                                           -160.0f);

    for (int i = 0; i < targetCount; ++i) {
        const int index = (std::min)(sourceEnd - 1,
                                     sourceStart + static_cast<int>(std::floor(i * step)));
        const int magnitudeIndex = (index + dataCount / 2) % dataCount;
        if (!std::isfinite(frequencies[index]) || !std::isfinite(magnitudes[magnitudeIndex])) {
            continue;
        }
        frequencyArray.append(frequencies[index]);
        resampledMagnitudes[static_cast<std::size_t>((i + targetCount / 2) % targetCount)] =
            magnitudes[magnitudeIndex];
    }

    for (const float value : resampledMagnitudes) {
        magnitudeArray.append(std::isfinite(value) ? value : -160.0f);
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
    frame["sourceFftLength"] = dataCount;
    frame["minFrequency"] = frameMinFrequency;
    frame["maxFrequency"] = frameMaxFrequency;
    frame["frequencies"] = frequencyArray;
    frame["magnitudes"] = magnitudeArray;

    networkController->sendControlCommand(frame);
}

void YourClassName::displayNetworkSpectrumFrame(const QJsonObject &frame) {
    if (networkMode == NetworkMode::Client &&
        runState == RadioRunState::Idle &&
        networkController &&
        !networkController->clientHasControl()) {
        runState = RadioRunState::Running;
        updateUiForRunState();
    }

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
    const bool spectrumShapeChanged =
        !networkSpectrumFrameMetadataValid ||
        std::abs(networkSpectrumFrameMinFrequency - frameMinFrequency) > 0.5 ||
        std::abs(networkSpectrumFrameMaxFrequency - frameMaxFrequency) > 0.5 ||
        networkSpectrumFrameFftLength != frameFftLength;
    if (spectrumShapeChanged) {
        qDebug() << "[NetworkSpectrum] frame range changed; preserving waterfall history"
                 << "min" << frameMinFrequency
                 << "max" << frameMaxFrequency
                 << "bins" << frameFftLength;
        networkSpectrumFrameMetadataValid = true;
        networkSpectrumFrameMinFrequency = frameMinFrequency;
        networkSpectrumFrameMaxFrequency = frameMaxFrequency;
        networkSpectrumFrameFftLength = frameFftLength;
    }

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
    if (runState != RadioRunState::Running) {
        return;
    }

    if (frame.value("sampleRate").toInt(48000) != 48000 ||
        frame.value("channels").toInt(1) != 1 ||
        frame.value("sampleFormat").toString() != QStringLiteral("pcm_s16le")) {
        qDebug() << "[NetworkAudio] unsupported remote audio frame format";
        return;
    }

    const QByteArray pcmData = QByteArray::fromBase64(frame.value("pcm").toString().toLatin1());
    processDigitalAudioFrame(pcmData);
    processSstvAudioFrame(pcmData);
    processAptAudioFrame(pcmData);
    processWefaxAudioFrame(pcmData);
    remoteAudioPlayer->playPcmFrame(pcmData);
}

void YourClassName::processDigitalAudioFrame(const QByteArray &pcmData) {
    if (!digitalDecoder ||
        !digitalDecoderThread ||
        !digitalDecodeEnabled ||
        !digitalDecodeCheckbox ||
        !digitalDecodeCheckbox->isChecked()) {
        return;
    }

    const RadioSettings settings = pendingSettings;
    QMetaObject::invokeMethod(digitalDecoder,
                              [decoder = digitalDecoder, pcmData, settings]() {
                                  decoder->processPcmFrame(pcmData, settings, 48000);
                              },
                              Qt::QueuedConnection);
}

void YourClassName::processSstvAudioFrame(const QByteArray &pcmData) {
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    if (!videoProcessor ||
        !videoProcessorThread ||
        !videoDecodeEnabled ||
        !videoDock ||
        !videoDock->isVisible() ||
        !videoDecodeCheckbox ||
        !videoDecodeCheckbox->isChecked() ||
        pendingSettings.modulationType != MOD_SSTV ||
        pcmData.isEmpty() ||
        suppressServerLocalOutput ||
        (videoTestPatternCheckbox && videoTestPatternCheckbox->isChecked())) {
        return;
    }

    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, pcmData]() {
                                  processor->processSstvPcmFrame(pcmData, 48000);
                              },
                              Qt::QueuedConnection);
}

void YourClassName::processAptAudioFrame(const QByteArray &pcmData) {
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    if (!videoProcessor ||
        !videoProcessorThread ||
        !videoDecodeEnabled ||
        !videoDock ||
        !videoDock->isVisible() ||
        !videoDecodeCheckbox ||
        !videoDecodeCheckbox->isChecked() ||
        pendingSettings.modulationType != MOD_APT ||
        pcmData.isEmpty() ||
        suppressServerLocalOutput ||
        (videoTestPatternCheckbox && videoTestPatternCheckbox->isChecked())) {
        return;
    }

    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, pcmData]() {
                                  processor->processAptPcmFrame(pcmData, 48000);
                              },
                              Qt::QueuedConnection);
}

void YourClassName::processWefaxAudioFrame(const QByteArray &pcmData) {
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    if (!videoProcessor ||
        !videoProcessorThread ||
        !videoDecodeEnabled ||
        !videoDock ||
        !videoDock->isVisible() ||
        !videoDecodeCheckbox ||
        !videoDecodeCheckbox->isChecked() ||
        pendingSettings.modulationType != MOD_WEFAX ||
        pcmData.isEmpty() ||
        suppressServerLocalOutput ||
        (videoTestPatternCheckbox && videoTestPatternCheckbox->isChecked())) {
        return;
    }

    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, pcmData]() {
                                  processor->processWefaxPcmFrame(pcmData, 48000);
                              },
                              Qt::QueuedConnection);
}

void YourClassName::updateDigitalDecoderMode() {
    if (!digitalDecoder || !digitalDecoderThread) {
        return;
    }
    const bool enabled = digitalDecodeEnabled;
    const RadioSettings settings = pendingSettings;
    QMetaObject::invokeMethod(digitalDecoder,
                              [decoder = digitalDecoder, enabled, settings]() {
                                  decoder->setEnabled(enabled);
                                  decoder->configure(settings, 48000);
                              },
                              Qt::QueuedConnection);
}

bool YourClassName::isVideoDecodeActive() const {
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    return videoDecodeEnabled &&
           videoDock &&
           videoDock->isVisible() &&
           videoDecodeCheckbox &&
           videoDecodeCheckbox->isChecked() &&
           (pendingSettings.modulationType == MOD_ATV ||
            pendingSettings.modulationType == MOD_LRPT) &&
           (!videoTestPatternCheckbox || !videoTestPatternCheckbox->isChecked()) &&
           !suppressServerLocalOutput;
}

void YourClassName::processVideoIqFrame(const QByteArray &iqData, double sampleRate, int sampleCount) {
    if (!isVideoDecodeActive() ||
        !videoProcessor ||
        !videoProcessorThread ||
        iqData.isEmpty() ||
        sampleRate <= 0.0 ||
        sampleCount <= 0) {
        return;
    }
    if (videoTestPatternCheckbox && videoTestPatternCheckbox->isChecked()) {
        return;
    }
    if (videoIqFramePending.exchange(true)) {
        return;
    }
    if (pendingSettings.modulationType == MOD_LRPT) {
        const int bytesPerIq = iqData.size() >= sampleCount * 4 ? 4 : 2;
        std::vector<float> floatSamples;
        floatSamples.reserve(static_cast<std::size_t>(sampleCount) * 2);
        const auto *src = reinterpret_cast<const uchar *>(iqData.constData());
        for (int i = 0; i < sampleCount; ++i) {
            float iSample = 0.0f;
            float qSample = 0.0f;
            if (bytesPerIq == 4) {
                const int offset = i * 4;
                if (offset + 3 >= iqData.size()) {
                    break;
                }
                const qint16 rawI = static_cast<qint16>(src[offset] | (src[offset + 1] << 8));
                const qint16 rawQ = static_cast<qint16>(src[offset + 2] | (src[offset + 3] << 8));
                iSample = rawI / 32768.0f;
                qSample = rawQ / 32768.0f;
            } else {
                const int offset = i * 2;
                if (offset + 1 >= iqData.size()) {
                    break;
                }
                iSample = (static_cast<int>(src[offset]) - 128) / 128.0f;
                qSample = (static_cast<int>(src[offset + 1]) - 128) / 128.0f;
            }
            floatSamples.push_back(iSample);
            floatSamples.push_back(qSample);
        }
        if (floatSamples.size() < 8) {
            videoIqFramePending.store(false);
            return;
        }
        RadioSettings settings = spectrumProcessingSettings();
        settings.sampleRate = sampleRate;
        QMetaObject::invokeMethod(videoProcessor,
                                  [this, processor = videoProcessor, samples = std::move(floatSamples), settings]() {
                                      processor->processFloatIqSnapshot(samples, settings);
                                      QMetaObject::invokeMethod(this,
                                                                [this]() {
                                                                    videoIqFramePending.store(false);
                                                                },
                                                                Qt::QueuedConnection);
                                  },
                                  Qt::QueuedConnection);
        return;
    }

    if (iqData.size() == sampleCount * 2 && sampleRate > 10000000.0) {
        if (videoStatusLabel) {
            videoStatusLabel->setText(QStringLiteral("Video: switch to ATV/channel IQ for wide signals"));
        }
        videoIqFramePending.store(false);
        return;
    }

    QMetaObject::invokeMethod(videoProcessor,
                              [this, processor = videoProcessor, iqData, sampleRate, sampleCount]() {
                                  processor->processIqFrame(iqData, sampleRate, sampleCount);
                                  QMetaObject::invokeMethod(this,
                                                            [this]() {
                                                                videoIqFramePending.store(false);
                                                            },
                                                            Qt::QueuedConnection);
                              },
                              Qt::QueuedConnection);
}

void YourClassName::processVideoSnapshotFrame() {
    const bool channelIqStreamMode =
        networkMode != NetworkMode::Disabled &&
        isChannelIqProcessingMode();
    if (!isVideoDecodeActive() ||
        channelIqStreamMode ||
        !videoProcessor ||
        !videoProcessorThread) {
        return;
    }
    if (videoIqFramePending.exchange(true)) {
        return;
    }

    std::vector<float> snapshot;
    std::uint64_t sequence = 0;
    if (!IqBuffer::snapshot(snapshot, &sequence) || snapshot.size() < 4) {
        videoIqFramePending.store(false);
        return;
    }

    if (snapshot.size() > VIDEO_SNAPSHOT_MAX_FLOATS) {
        std::vector<float> tail(snapshot.end() - static_cast<std::ptrdiff_t>(VIDEO_SNAPSHOT_MAX_FLOATS),
                                snapshot.end());
        snapshot.swap(tail);
    }

    RadioSettings settings = spectrumProcessingSettings();
    QMetaObject::invokeMethod(videoProcessor,
                              [this, processor = videoProcessor, samples = std::move(snapshot), settings]() {
                                  processor->processFloatIqSnapshot(samples, settings);
                                  QMetaObject::invokeMethod(this,
                                                            [this]() {
                                                                videoIqFramePending.store(false);
                                                            },
                                                            Qt::QueuedConnection);
                              },
                              Qt::QueuedConnection);
}

void YourClassName::updateVideoProcessorMode() {
    if (!videoProcessor || !videoProcessorThread) {
        return;
    }

    const bool iqVideoEnabled = isVideoDecodeActive();
    const bool analogVideoEnabled = iqVideoEnabled && pendingSettings.modulationType == MOD_ATV;
    videoIqFramePending.store(false);
    const bool testPatternEnabled = videoTestPatternCheckbox && videoTestPatternCheckbox->isChecked();
    const bool analogVideoTest = testPatternEnabled && pendingSettings.modulationType == MOD_ATV;
    const bool sstvTest = testPatternEnabled && pendingSettings.modulationType == MOD_SSTV;
    const bool aptTest = testPatternEnabled && pendingSettings.modulationType == MOD_APT;
    const bool wefaxTest = testPatternEnabled && pendingSettings.modulationType == MOD_WEFAX;
    const bool lrptTest = testPatternEnabled && pendingSettings.modulationType == MOD_LRPT;
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    const bool sstvEnabled =
        videoDock &&
        videoDock->isVisible() &&
        pendingSettings.modulationType == MOD_SSTV &&
        (sstvTest ||
         (videoDecodeEnabled &&
          videoDecodeCheckbox &&
          videoDecodeCheckbox->isChecked())) &&
        !suppressServerLocalOutput;
    const bool aptEnabled =
        videoDock &&
        videoDock->isVisible() &&
        pendingSettings.modulationType == MOD_APT &&
        (aptTest ||
         (videoDecodeEnabled &&
          videoDecodeCheckbox &&
          videoDecodeCheckbox->isChecked())) &&
        !suppressServerLocalOutput;
    const bool wefaxEnabled =
        videoDock &&
        videoDock->isVisible() &&
        pendingSettings.modulationType == MOD_WEFAX &&
        (wefaxTest ||
         (videoDecodeEnabled &&
          videoDecodeCheckbox &&
          videoDecodeCheckbox->isChecked())) &&
        !suppressServerLocalOutput;
    const bool lrptEnabled =
        videoDock &&
        videoDock->isVisible() &&
        pendingSettings.modulationType == MOD_LRPT &&
        (lrptTest ||
         (videoDecodeEnabled &&
          videoDecodeCheckbox &&
          videoDecodeCheckbox->isChecked())) &&
        !suppressServerLocalOutput;
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, analogVideoTest]() {
                                  processor->setTestPatternEnabled(analogVideoTest);
                              },
                              Qt::QueuedConnection);
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, sstvEnabled]() {
                                  processor->configureSstv(sstvEnabled);
                              },
                              Qt::QueuedConnection);
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, sstvTest]() {
                                  processor->setSstvTestPatternEnabled(sstvTest);
                              },
                              Qt::QueuedConnection);
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, aptEnabled]() {
                                  processor->configureApt(aptEnabled);
                              },
                              Qt::QueuedConnection);
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, aptTest]() {
                                  processor->setAptTestPatternEnabled(aptTest);
                              },
                              Qt::QueuedConnection);
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, wefaxEnabled]() {
                                  processor->configureWefax(wefaxEnabled);
                              },
                              Qt::QueuedConnection);
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, wefaxTest]() {
                                  processor->setWefaxTestPatternEnabled(wefaxTest);
                              },
                              Qt::QueuedConnection);
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor, lrptEnabled]() {
                                  processor->configureLrpt(lrptEnabled);
                              },
                              Qt::QueuedConnection);

    const bool channelIqStreamMode =
        networkMode != NetworkMode::Disabled &&
        isChannelIqProcessingMode();
    const bool snapshotVideoEnabled = iqVideoEnabled && !channelIqStreamMode;
    if (videoSnapshotTimer) {
        if (snapshotVideoEnabled && !videoSnapshotTimer->isActive()) {
            videoSnapshotTimer->start();
        } else if (!snapshotVideoEnabled && videoSnapshotTimer->isActive()) {
            videoSnapshotTimer->stop();
        }
    }
    if (sstvTest) {
        if (videoStatusLabel) {
            videoStatusLabel->setText(QStringLiteral("SSTV Robot36 test stream"));
        }
    }
    if (aptTest) {
        if (videoStatusLabel) {
            videoStatusLabel->setText(QStringLiteral("NOAA APT test stream"));
        }
    }
    if (wefaxTest) {
        if (videoStatusLabel) {
            videoStatusLabel->setText(QStringLiteral("WEFAX test stream"));
        }
    }
    if (lrptTest) {
        if (videoStatusLabel) {
            videoStatusLabel->setText(QStringLiteral("Meteor LRPT beta: QPSK monitor test"));
        }
    }
    if (!iqVideoEnabled && !sstvTest && !aptTest && !wefaxTest && !lrptEnabled && videoWidget) {
        videoWidget->clearFrame();
    }
    if (pendingSettings.modulationType == MOD_SSTV && !sstvTest && videoStatusLabel) {
        videoStatusLabel->setText(QStringLiteral("SSTV: image decoder setup ready"));
    }
    if (pendingSettings.modulationType == MOD_APT && !aptTest && videoStatusLabel) {
        videoStatusLabel->setText(QStringLiteral("NOAA APT: image decoder setup ready"));
    }
    if (pendingSettings.modulationType == MOD_WEFAX && !wefaxTest && videoStatusLabel) {
        videoStatusLabel->setText(QStringLiteral("WEFAX: image decoder setup ready"));
    }
    if (pendingSettings.modulationType == MOD_LRPT && !lrptTest && videoStatusLabel) {
        videoStatusLabel->setText(QStringLiteral("Meteor LRPT beta: QPSK IQ monitor ready"));
    }
    const int demodMode = videoDemodCombo ? videoDemodCombo->currentData().toInt()
                                          : VideoProcessor::FmVideo;
    const double lineRate = videoStandardCombo ? videoStandardCombo->currentData().toDouble()
                                               : 15625.0;
    const bool invertVideo = videoInvertCheckbox && videoInvertCheckbox->isChecked();
    const bool hSyncEnabled = !videoHSyncCheckbox || videoHSyncCheckbox->isChecked();
    const bool vSyncEnabled = !videoVSyncCheckbox || videoVSyncCheckbox->isChecked();
    QMetaObject::invokeMethod(videoProcessor,
                              [processor = videoProcessor,
                               analogVideoEnabled,
                               demodMode,
                               lineRate,
                               invertVideo,
                               hSyncEnabled,
                               vSyncEnabled]() {
                                  processor->configure(analogVideoEnabled,
                                                       demodMode,
                                                       lineRate,
                                                       384,
                                                       288,
                                                       invertVideo,
                                                       hSyncEnabled,
                                                       vSyncEnabled);
                              },
                              Qt::QueuedConnection);
}

RadioSettings YourClassName::audioProcessorSettings() const {
    RadioSettings settings = pendingSettings;
    if (offlineIqPlaybackActive && offlineIqPlaybackSampleRate > 0.0) {
        settings.sampleRate = offlineIqPlaybackSampleRate;
        settings.centerFrequency = pendingSettings.listeningFrequency;
        settings.actualFrequency = pendingSettings.listeningFrequency;
        settings.inputMode = 0;
    }
    return settings;
}

RadioSettings YourClassName::spectrumProcessingSettings() const {
    RadioSettings settings = pendingSettings;
    if (offlineIqPlaybackActive && offlineIqPlaybackSampleRate > 0.0) {
        settings.sampleRate = offlineIqPlaybackSampleRate;
        settings.centerFrequency = pendingSettings.listeningFrequency;
        settings.actualFrequency = pendingSettings.listeningFrequency;
        settings.inputMode = 0;
    }
    return settings;
}

void YourClassName::startRecording(bool momentary) {
    if (!recordingManager) {
        return;
    }
    if (recordingManager->isRecording()) {
        if (momentary) {
            momentaryRecordingActive = false;
        }
        return;
    }

    const RecordingManager::Mode mode = selectedRecordingMode();
    if (mode == RecordingManager::Mode::ChannelIqWav &&
        networkMode != NetworkMode::Disabled &&
        isFullIqProcessingMode() &&
        runState == RadioRunState::Running) {
        updateRecordingStatus(QStringLiteral("Recording blocked: Channel IQ cannot run during Full IQ streaming"));
        if (recordButton) {
            QSignalBlocker blocker(recordButton);
            recordButton->setChecked(false);
            recordButton->setText(QStringLiteral("Record"));
        }
        momentaryRecordingActive = false;
        return;
    }

    QString errorMessage;
    recordingManager->setDisplayScalePercent(currentScale);
    recordingManager->setLabMetadata(recordingLabMetadata());
    if (!recordingManager->start(mode, pendingSettings, &errorMessage)) {
        updateRecordingStatus(QStringLiteral("Recording failed: %1").arg(errorMessage));
        if (recordButton) {
            QSignalBlocker blocker(recordButton);
            recordButton->setChecked(false);
            recordButton->setText(QStringLiteral("Record"));
        }
        momentaryRecordingActive = false;
        return;
    }

    momentaryRecordingActive = momentary;
    if (recordButton) {
        QSignalBlocker blocker(recordButton);
        recordButton->setChecked(true);
        recordButton->setText(momentary ? QStringLiteral("Hold F9") : QStringLiteral("Stop Rec"));
    }
    if (recordingModeCombo) {
        recordingModeCombo->setEnabled(false);
    }
    if (mode == RecordingManager::Mode::ChannelIqWav) {
        updateIqFrameProducerSettings();
    }
}

void YourClassName::stopRecording(bool momentaryRelease) {
    if (!recordingManager) {
        return;
    }
    if (momentaryRelease && !momentaryRecordingActive) {
        return;
    }

    const bool wasChannelIqRecording = isChannelIqRecordingActive();
    momentaryRecordingActive = false;
    recordingManager->stop();
    if (recordButton) {
        QSignalBlocker blocker(recordButton);
        recordButton->setChecked(false);
        recordButton->setText(QStringLiteral("Record"));
    }
    if (recordingModeCombo) {
        recordingModeCombo->setEnabled(true);
    }
    if (wasChannelIqRecording) {
        updateIqFrameProducerSettings();
    }
}

void YourClassName::refreshPlaybackFiles() {
    if (!playbackFileCombo) {
        return;
    }

    const QString previousPath = selectedPlaybackFilePath();
    playbackFileCombo->blockSignals(true);
    playbackFileCombo->clear();

    QDir recordingsDir(QDir(QCoreApplication::applicationDirPath()).filePath(QStringLiteral("recordings")));
    const QFileInfoList files = recordingsDir.entryInfoList(QStringList() << QStringLiteral("*.wav"),
                                                            QDir::Files,
                                                            QDir::Time);
    for (const QFileInfo &fileInfo : files) {
        PlaybackManager::WavInfo info;
        if (!PlaybackManager::readWavInfo(fileInfo.absoluteFilePath(), info)) {
            continue;
        }

        const double seconds = info.dataSize / static_cast<double>(
            info.sampleRate * info.channels * (info.bitsPerSample / 8));
        const QString type = info.mode == PlaybackManager::Mode::AudioWav
                                 ? QStringLiteral("Audio")
                                 : QStringLiteral("Ch IQ");
        const QString label = QStringLiteral("%1  %2  %3 Hz  %4 s")
                                  .arg(fileInfo.fileName(),
                                       type,
                                       QString::number(info.sampleRate),
                                       QString::number(seconds, 'f', 1));
        playbackFileCombo->addItem(label, fileInfo.absoluteFilePath());
    }

    if (playbackFileCombo->count() == 0) {
        playbackFileCombo->addItem(QStringLiteral("No WAV recordings found"), QString());
        playbackFileCombo->setEnabled(false);
        if (playbackButton) {
            playbackButton->setEnabled(false);
        }
    } else {
        playbackFileCombo->setEnabled(true);
        if (playbackButton) {
            playbackButton->setEnabled(true);
        }
        const int previousIndex = playbackFileCombo->findData(previousPath);
        if (previousIndex >= 0) {
            playbackFileCombo->setCurrentIndex(previousIndex);
        }
    }

    playbackFileCombo->blockSignals(false);
}

void YourClassName::startPlayback() {
    if (!playbackManager) {
        return;
    }
    if (playbackManager->isPlaying()) {
        return;
    }
    if (runState != RadioRunState::Idle || deviceOpened || (processor && processor->isRunning())) {
        onPlaybackStatusChanged(QStringLiteral("Playback blocked: stop receiver first"));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
        return;
    }
    if (recordingManager && recordingManager->isRecording()) {
        onPlaybackStatusChanged(QStringLiteral("Playback blocked: stop recording first"));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
        return;
    }

    const QString path = selectedPlaybackFilePath();
    if (path.isEmpty()) {
        onPlaybackStatusChanged(QStringLiteral("Playback: no file selected"));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
        return;
    }

    PlaybackManager::WavInfo info;
    QString errorMessage;
    if (!PlaybackManager::readWavInfo(path, info, &errorMessage)) {
        onPlaybackStatusChanged(QStringLiteral("Playback failed: %1").arg(errorMessage));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
        return;
    }
    if (info.mode == PlaybackManager::Mode::AudioWav && (info.channels != 1 || info.sampleRate != 48000)) {
        onPlaybackStatusChanged(QStringLiteral("Playback failed: audio WAV must be mono 48 kHz"));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
        return;
    }

    if (!playbackManager->start(path, &errorMessage)) {
        onPlaybackStatusChanged(QStringLiteral("Playback failed: %1").arg(errorMessage));
        if (playbackButton) {
            QSignalBlocker blocker(playbackButton);
            playbackButton->setChecked(false);
        }
    }
}

void YourClassName::stopPlayback() {
    if (playbackManager && playbackManager->isPlaying()) {
        playbackManager->stop();
        return;
    }
    if (playbackButton && playbackButton->isChecked()) {
        QSignalBlocker blocker(playbackButton);
        playbackButton->setChecked(false);
        playbackButton->setText(QStringLiteral("Play"));
    }
}

void YourClassName::onPlaybackStarted(const QString &filePath, PlaybackManager::WavInfo info) {
    Q_UNUSED(filePath);
    if (playbackButton) {
        QSignalBlocker blocker(playbackButton);
        playbackButton->setChecked(true);
        playbackButton->setText(QStringLiteral("Stop Play"));
    }
    if (playbackFileCombo) {
        playbackFileCombo->setEnabled(false);
    }
    if (playbackRefreshButton) {
        playbackRefreshButton->setEnabled(false);
    }
    if (startButton) {
        startButton->setEnabled(false);
    }

    if (info.mode == PlaybackManager::Mode::ChannelIqWav) {
        if (!playbackSettingsSaved) {
            settingsBeforePlayback = pendingSettings;
            playbackSettingsSaved = true;
        }
        offlineIqPlaybackActive = true;
        offlineIqPlaybackHasMetadata = info.hasRadioSettings;
        offlineIqPlaybackSampleRate = info.sampleRate;
        IqBuffer::clear();
        IqBuffer::setSampleRateEstimate(info.sampleRate);
        const bool audioEnabledBeforePlayback = pendingSettings.audioEnabled;
        const int audioDeviceBeforePlayback = pendingSettings.audioDeviceId;
        if (info.hasRadioSettings) {
            pendingSettings.deviceIndex = info.radioSettings.deviceIndex;
            pendingSettings.clockSource = info.radioSettings.clockSource;
            pendingSettings.inputMode = info.radioSettings.inputMode;
            pendingSettings.centerFrequency = info.radioSettings.centerFrequency;
            pendingSettings.actualFrequency = info.radioSettings.actualFrequency;
            pendingSettings.listeningFrequency = info.radioSettings.listeningFrequency;
            pendingSettings.sampleRate = info.radioSettings.sampleRate;
            pendingSettings.bandwidth = info.radioSettings.bandwidth;
            pendingSettings.modulationType = info.radioSettings.modulationType;
            pendingSettings.fftLength = info.radioSettings.fftLength;
            if (info.radioSettings.lnaGain >= 0) {
                pendingSettings.lnaGain = info.radioSettings.lnaGain;
            }
            if (info.radioSettings.vgaGain >= 0) {
                pendingSettings.vgaGain = info.radioSettings.vgaGain;
            }
            const double playbackCenter = info.radioSettings.listeningFrequency > 0.0
                                              ? info.radioSettings.listeningFrequency
                                              : info.radioSettings.centerFrequency;
            if (playbackCenter > 0.0) {
                pendingSettings.listeningFrequency = playbackCenter;
            }
        } else {
            pendingSettings.sampleRate = info.sampleRate;
            pendingSettings.centerFrequency = pendingSettings.listeningFrequency;
            pendingSettings.actualFrequency = pendingSettings.listeningFrequency;
            pendingSettings.inputMode = 0;
        }
        pendingSettings.audioEnabled = audioEnabledBeforePlayback;
        pendingSettings.audioDeviceId = audioDeviceBeforePlayback;
        if (info.hasScalePercent) {
            currentScale = std::clamp(info.scalePercent,
                                      static_cast<double>(minScale) / 10.0,
                                      static_cast<double>(maxScale) / 10.0);
        } else if (info.hasRadioSettings &&
                   info.radioSettings.sampleRate > 0.0 &&
                   info.sampleRate > 0) {
            const double channelScalePercent =
                (static_cast<double>(info.sampleRate) / info.radioSettings.sampleRate) * 100.0;
            currentScale = std::clamp(channelScalePercent,
                                      static_cast<double>(minScale) / 10.0,
                                      static_cast<double>(maxScale) / 10.0);
        }
        publishSettingsToGlobals();
        updateUiFromPendingSettings();
        fftResult = std::make_unique<FFTResult>();
        updateSpectrumTimerInterval();
        if (audioProcessor) {
            audioProcessor->configure(audioProcessorSettings());
        }
        pendingPlaybackAudioStartAfterIqPrebuffer = pendingSettings.audioEnabled;
        if (updateTimer) {
            updateTimer->start();
        }
        settingRange();
    } else {
        offlineIqPlaybackActive = false;
        offlineIqPlaybackHasMetadata = false;
        offlineIqPlaybackSampleRate = 0.0;
    }
}

void YourClassName::onPlaybackStopped() {
    if (offlineIqPlaybackActive) {
        offlineIqPlaybackActive = false;
        offlineIqPlaybackHasMetadata = false;
        offlineIqPlaybackSampleRate = 0.0;
        pendingPlaybackAudioStartAfterIqPrebuffer = false;
        if (audioProcessor) {
            audioProcessor->stopDemodulation();
        }
        if (updateTimer && runState == RadioRunState::Idle) {
            updateTimer->stop();
        }
    }
    if (playbackSettingsSaved) {
        pendingSettings = settingsBeforePlayback;
        playbackSettingsSaved = false;
        publishSettingsToGlobals();
        updateUiFromPendingSettings();
    }
    if (remoteAudioPlayer && runState == RadioRunState::Idle) {
        remoteAudioPlayer->stop();
    }
    if (playbackButton) {
        QSignalBlocker blocker(playbackButton);
        playbackButton->setChecked(false);
        playbackButton->setText(QStringLiteral("Play"));
    }
    if (playbackFileCombo) {
        playbackFileCombo->setEnabled(playbackFileCombo->count() > 0 && !selectedPlaybackFilePath().isEmpty());
    }
    if (playbackRefreshButton) {
        playbackRefreshButton->setEnabled(true);
    }
    updateUiForRunState();
}

void YourClassName::onPlaybackStatusChanged(const QString &status) {
    if (playbackStatusLabel) {
        playbackStatusLabel->setText(status);
    }
}

void YourClassName::handlePlaybackAudioFrame(const QByteArray &pcmData) {
    processDigitalAudioFrame(pcmData);
    processSstvAudioFrame(pcmData);
    processAptAudioFrame(pcmData);
    processWefaxAudioFrame(pcmData);
    if (remoteAudioPlayer) {
        remoteAudioPlayer->playPcmFrame(pcmData);
    }
}

void YourClassName::handlePlaybackIqFrame(const QByteArray &iqData, double sampleRate, int sampleCount) {
    if (iqData.size() < static_cast<int>(2 * sizeof(qint16))) {
        return;
    }
    processVideoIqFrame(iqData,
                        sampleRate,
                        sampleCount > 0 ? sampleCount : iqData.size() / (2 * static_cast<int>(sizeof(qint16))));
    const int bytesPerIq = 2 * static_cast<int>(sizeof(qint16));
    std::vector<float> floatSamples(static_cast<std::size_t>(iqData.size() / sizeof(qint16)));
    const auto *src = reinterpret_cast<const uchar *>(iqData.constData());
    for (int i = 0, out = 0; i + bytesPerIq - 1 < iqData.size(); i += bytesPerIq) {
        const qint16 iSample = static_cast<qint16>(src[i] | (src[i + 1] << 8));
        const qint16 qSample = static_cast<qint16>(src[i + 2] | (src[i + 3] << 8));
        floatSamples[static_cast<std::size_t>(out++)] = iSample / 32768.0f;
        floatSamples[static_cast<std::size_t>(out++)] = qSample / 32768.0f;
    }
    IqBuffer::setSampleRateEstimate(sampleRate);
    IqBuffer::publish(floatSamples.data(), floatSamples.size(), pendingSettings.audioEnabled);
    if (pendingPlaybackAudioStartAfterIqPrebuffer && pendingSettings.audioEnabled && audioProcessor && sampleRate > 0.0) {
        const double queuedIqSamples = static_cast<double>(IqBuffer::queuedFloatCount()) / 2.0;
        const double queuedSeconds = queuedIqSamples / sampleRate;
        if (queuedSeconds >= NETWORK_AUDIO_PREBUFFER_SECONDS) {
            pendingPlaybackAudioStartAfterIqPrebuffer = false;
            qDebug() << "[PlaybackIQ] starting demodulator after IQ prebuffer"
                     << "queuedSeconds" << queuedSeconds
                     << "queuedBlocks" << IqBuffer::queuedBlocks()
                     << "sampleRate" << sampleRate;
            audioProcessor->startDemodulation();
        }
    }
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
    if (networkMode != NetworkMode::Client || !isClientIqProcessingMode()) {
        return;
    }
    if (runState == RadioRunState::Idle &&
        networkController &&
        !networkController->clientHasControl()) {
        runState = RadioRunState::Running;
        updateUiForRunState();
        startNetworkClientProcessing();
    }
    if (runState == RadioRunState::Idle) {
        return;
    }

    const bool channelizedFrame = frame.value("channelized").toBool(false);
    const QString sampleFormat = frame.value("sampleFormat").toString();
    if ((!channelizedFrame && sampleFormat != QStringLiteral("iq_s8_interleaved")) ||
        (channelizedFrame && sampleFormat != QStringLiteral("channel_iq_s16le"))) {
        qDebug() << "[NetworkIQ] unsupported IQ frame format";
        return;
    }
    if (channelizedFrame != isChannelIqProcessingMode()) {
        if (fobosVerboseLoggingEnabled()) {
            qDebug() << "[NetworkIQ] dropping IQ frame from previous processing mode"
                     << "frameChannelized" << channelizedFrame
                     << "currentProcessingMode" << static_cast<int>(networkProcessingMode);
        }
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
    const int frameSampleCount = frame.value("sampleCount").toInt(iqBytes.size() / bytesPerIqSample);
    processVideoIqFrame(iqBytes, frameSampleRate, frameSampleCount);
    if (channelizedFrame &&
        recordingManager &&
        recordingManager->isRecording() &&
        recordingManager->mode() == RecordingManager::Mode::ChannelIqWav) {
        recordingManager->appendIqFrame(iqBytes,
                                        frameSampleRate,
                                        frameSampleCount);
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

    const bool streamShapeChanged =
        !networkIqStreamMetadataValid ||
        networkIqStreamWasChannelized != channelizedFrame ||
        std::abs(networkIqStreamSampleRate - frameSampleRate) > 0.5 ||
        networkIqStreamInputMode != iqSettings.inputMode ||
        (!channelizedFrame &&
         std::abs(networkIqStreamCenterFrequency - iqSettings.centerFrequency) > 0.5) ||
        (channelizedFrame &&
         std::abs(networkIqStreamListeningFrequency - iqSettings.listeningFrequency) > 0.5);

    if (streamShapeChanged) {
        qDebug() << "[NetworkIQ] receiver stream shape changed; resetting client IQ buffers"
                 << "channelized" << channelizedFrame
                 << "sampleRate" << frameSampleRate
                 << "center" << iqSettings.centerFrequency
                 << "listening" << iqSettings.listeningFrequency
                 << "inputMode" << iqSettings.inputMode;
        resetNetworkIqReceptionState(false, false, pendingSettings.audioEnabled);
        networkIqStreamMetadataValid = true;
        networkIqStreamWasChannelized = channelizedFrame;
        networkIqStreamSampleRate = frameSampleRate;
        networkIqStreamCenterFrequency = iqSettings.centerFrequency;
        networkIqStreamListeningFrequency = iqSettings.listeningFrequency;
        networkIqStreamInputMode = iqSettings.inputMode;
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
    if (isFullIqProcessingMode() && updateTimer && !updateTimer->isActive()) {
        qDebug() << "[NetworkIQ] restarting client spectrum timer after IQ frame";
        updateTimer->start();
    }

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
    const bool clientCanControl =
        !isNetworkClientMode() ||
        !networkController ||
        networkController->clientHasControl();

    if (startButton) startButton->setEnabled(idle && clientCanControl);
    if (stopButton) {
        const bool canStop =
            runState == RadioRunState::Starting ||
            runState == RadioRunState::Running;
        stopButton->setEnabled(canStop && (clientCanControl || isNetworkClientMode()));
    }
    if (comboBox) comboBox->setEnabled(idle);
    if (refreshButton) refreshButton->setEnabled(idle);
    if (fobosButton) fobosButton->setEnabled(idle);
    if (modeBox) modeBox->setEnabled(true);
    if (sampleBox) sampleBox->setEnabled(true);
    if (clkBox) clkBox->setEnabled(idle);
    if (fftComboBox) fftComboBox->setEnabled(idle || runState == RadioRunState::Running);
    if (audioDeviceComboBox) audioDeviceComboBox->setEnabled(idle);
    const bool liveAudioControlsEnabled =
        idle || runState == RadioRunState::Running;

    if (audioCheckbox) audioCheckbox->setEnabled(liveAudioControlsEnabled);
    if (syncCheckbox) syncCheckbox->setEnabled(false);
    const bool liveDemodControlsEnabled =
        idle || runState == RadioRunState::Running;

    if (bandwidthControl) bandwidthControl->setEnabled(liveDemodControlsEnabled);
    const bool gainControlsEnabled =
        idle || runState == RadioRunState::Running;

    if (lnaGainSlider) lnaGainSlider->setEnabled(gainControlsEnabled);
    if (vgaGainSlider) vgaGainSlider->setEnabled(gainControlsEnabled);
    const bool gpioEnabled =
        idle || runState == RadioRunState::Running;
    for (int i = 0; i < 8; ++i) {
        if (checkBoxes[i]) {
            checkBoxes[i]->setEnabled(gpioEnabled);
        }
    }
}

void YourClassName::onAudioEnabledChanged(bool checked) {
    pendingSettings.audioEnabled = checked;
    publishSettingsToGlobals();

    qDebug() << "[Audio] checkbox changed" << checked
             << "networkMode" << static_cast<int>(networkMode)
             << "processingMode" << static_cast<int>(networkProcessingMode)
             << "runState" << runStateName(runState);

    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();

        if (isClientIqProcessingMode()) {
            if (checked) {
                IqBuffer::clear();
                pendingNetworkAudioStartAfterIqPrebuffer = true;
            } else {
                pendingNetworkAudioStartAfterIqPrebuffer = false;
                if (audioProcessor) {
                    audioProcessor->stopDemodulation();
                }
            }
        } else {
            if (!checked && remoteAudioPlayer) {
                remoteAudioPlayer->stop();
            }
        }

        return;
    }

    if (offlineIqPlaybackActive) {
        if (checked) {
            if (audioProcessor) {
                audioProcessor->configure(audioProcessorSettings());
            }
            pendingPlaybackAudioStartAfterIqPrebuffer = true;
        } else if (audioProcessor) {
            pendingPlaybackAudioStartAfterIqPrebuffer = false;
            audioProcessor->stopDemodulation();
        }
        return;
    }

    if (isIdle()) {
        return;
    }

    if (checked) {
        if (processor && processor->isRunning() && audioProcessor) {
            audioProcessor->startDemodulation();
        } else {
            pendingAudioStartAfterStreamReady = true;
        }
    } else {
        pendingAudioStartAfterStreamReady = false;
        if (audioProcessor) {
            audioProcessor->stopDemodulation();
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
    if (availableFobosDevices.isEmpty()) {
        refreshFobosDeviceList();
    }
    FobosDeviceInfo selectedInfo = selectedFobosDeviceInfo();
    selectedDevice = selectedInfo.nativeIndex;

    qDebug() << "[FobosLifecycle] openFobosSession enter"
             << "logicalDevice" << pendingSettings.deviceIndex
             << "nativeDevice" << selectedDevice
             << "apiKind" << fobosApiKindName(selectedInfo.apiKind)
             << "device" << activeFobosDevice()
             << "openedDeviceIndex" << openedDeviceIndex
             << "openedNativeDeviceIndex" << openedNativeDeviceIndex
             << "openedApiKind" << fobosApiKindName(openedDeviceApiKind)
             << "appliedSampleRate" << appliedSampleRate
             << "pendingSampleRate" << pendingSettings.sampleRate
             << "sampleRateReopenRequired" << sampleRateReopenRequired
             << "fobosCloseKnownUnsafe" << fobosCloseKnownUnsafe;

    if (hasActiveFobosDevice() &&
        openedDeviceIndex == pendingSettings.deviceIndex &&
        openedNativeDeviceIndex == selectedInfo.nativeIndex &&
        openedDeviceApiKind == selectedInfo.apiKind) {
        qDebug() << "[FobosLifecycle] reusing idle Fobos session; settings will be applied in place"
                 << activeFobosDevice();
        return true;
    }

    if (hasActiveFobosDevice()) {
        qDebug() << "[FobosLifecycle] closing mismatched existing Fobos session before open";
        if (!closeFobosSession(false)) {
            qDebug() << "[FobosLifecycle] existing Fobos session could not be closed; open aborted";
            return false;
        }
        qDebug() << "[FobosLifecycle] mismatched Fobos session closed before fresh open; waiting before reopen";
        QThread::msleep(350);
    }

    activeFobosApiKind = selectedInfo.apiKind;
    int ret = FOBOS_ERR_OK;
    if (selectedInfo.apiKind == FobosApiKind::Agile) {
        qDebug() << "[FobosLifecycle] fobos_sdr_open begin" << "selectedDevice" << selectedDevice;
        ret = openFobosAgileDeviceSafely(&agileDevice, static_cast<uint32_t>(selectedDevice));
        qDebug() << "[FobosLifecycle] fobos_sdr_open end" << "result" << ret << "device" << agileDevice;
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_open begin" << "selectedDevice" << selectedDevice;
        ret = openFobosDeviceSafely(&device, static_cast<uint32_t>(selectedDevice));
        qDebug() << "[FobosLifecycle] fobos_rx_open end" << "result" << ret << "device" << device;
    }

    if (ret != FOBOS_ERR_OK || !hasActiveFobosDevice()) {
        qDebug() << "Failed to open Fobos device, error code:" << ret;
        device = nullptr;
        agileDevice = nullptr;
        activeFobosApiKind = FobosApiKind::Standard;
        openedDeviceIndex = -1;
        openedNativeDeviceIndex = -1;
        return false;
    }

    openedDeviceIndex = pendingSettings.deviceIndex;
    openedNativeDeviceIndex = selectedInfo.nativeIndex;
    openedDeviceApiKind = selectedInfo.apiKind;
    appliedHardwareSettings = RadioSettings{};
    hardwareSettingsApplied = false;
    sampleRateReopenRequired = false;
    fobosCloseKnownUnsafe = false;
    return true;
}

bool YourClassName::closeFobosSession(bool clearIq) {
    qDebug() << "[FobosLifecycle] closeFobosSession enter"
             << "clearIq" << clearIq
             << "device" << activeFobosDevice()
             << "openedDeviceIndex" << openedDeviceIndex
             << "openedNativeDeviceIndex" << openedNativeDeviceIndex
             << "apiKind" << fobosApiKindName(openedDeviceApiKind);
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
    if (agileDevice) {
        qDebug() << "[FobosLifecycle] fobos_sdr_close begin" << agileDevice;
        const int closeResult = closeFobosAgileDeviceSafely(agileDevice);
        qDebug() << "[FobosLifecycle] fobos_sdr_close end" << "result" << closeResult;
        closeOk = closeOk && closeResult == FOBOS_ERR_OK;
        if (closeResult != FOBOS_ERR_OK) {
            qDebug() << "Fobos agile close returned error code:" << closeResult;
        }
        agileDevice = nullptr;
    }
    openedDeviceIndex = -1;
    openedNativeDeviceIndex = -1;
    appliedSampleRate = 0.0;
    appliedHardwareSettings = RadioSettings{};
    hardwareSettingsApplied = false;
    sampleRateReopenRequired = false;
    fobosCloseKnownUnsafe = false;
    activeFobosApiKind = FobosApiKind::Standard;
    qDebug() << "[FobosLifecycle] closeFobosSession exit";
    return closeOk;
}

bool YourClassName::applyFobosSettings() {
    if (!hasActiveFobosDevice()) {
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
             << "device" << activeFobosDevice()
             << "apiKind" << fobosApiKindName(activeFobosApiKind)
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
        qDebug() << "[FobosLifecycle] set clock source begin";
        result = setActiveClockSourceSafely(pendingSettings.clockSource);
        qDebug() << "[FobosLifecycle] set clock source end" << "result" << result;
        if (result != FOBOS_ERR_OK) {
            qDebug() << "Failed to set clock source, error code:" << result;
            return false;
        }
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_set_clk_source skipped unchanged";
    }

    const int libfobosMode = (pendingSettings.inputMode == 0) ? 0 : 1;
    if (firstApply || appliedHardwareSettings.inputMode != pendingSettings.inputMode) {
        qDebug() << "[FobosLifecycle] set direct sampling begin" << "libfobosMode" << libfobosMode;
        result = setActiveDirectSamplingSafely(static_cast<unsigned int>(libfobosMode));
        qDebug() << "[FobosLifecycle] set direct sampling end" << "result" << result;
        if (result != FOBOS_ERR_OK) {
            qDebug() << "Failed to set direct sampling mode, error code:" << result;
            return false;
        }
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_set_direct_sampling skipped unchanged";
    }

    if (pendingSettings.sampleRate > 0.0 &&
        (firstApply || changedDouble(appliedHardwareSettings.sampleRate, pendingSettings.sampleRate))) {
        qDebug() << "[FobosLifecycle] set sample rate begin" << "requested" << pendingSettings.sampleRate;
        result = setActiveSampleRateSafely(pendingSettings.sampleRate, &globalSampleRate);
        qDebug() << "[FobosLifecycle] set sample rate end"
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
            qDebug() << "[FobosLifecycle] set frequency begin" << "requested" << pendingSettings.centerFrequency;
            result = setActiveFrequencySafely(pendingSettings.centerFrequency, &actualFrequency);
            qDebug() << "[FobosLifecycle] set frequency end"
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
        qDebug() << "[FobosLifecycle] set LNA gain begin" << pendingSettings.lnaGain;
        result = setActiveLnaGainSafely(static_cast<unsigned int>(pendingSettings.lnaGain));
        qDebug() << "[FobosLifecycle] set LNA gain end" << "result" << result;
        if (result != FOBOS_ERR_OK) {
            qDebug() << "Failed to set LNA gain, error code:" << result;
        }
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_set_lna_gain skipped unchanged";
    }

    if (firstApply || appliedHardwareSettings.vgaGain != pendingSettings.vgaGain) {
        qDebug() << "[FobosLifecycle] set VGA gain begin" << pendingSettings.vgaGain;
        result = setActiveVgaGainSafely(static_cast<unsigned int>(pendingSettings.vgaGain));
        qDebug() << "[FobosLifecycle] set VGA gain end" << "result" << result;
        if (result != FOBOS_ERR_OK) {
            qDebug() << "Failed to set VGA gain, error code:" << result;
        }
    } else {
        qDebug() << "[FobosLifecycle] fobos_rx_set_vga_gain skipped unchanged";
    }

    if (firstApply || appliedHardwareSettings.gpoValue != pendingSettings.gpoValue) {
        qDebug() << "[FobosLifecycle] set user GPO begin" << pendingSettings.gpoValue;
        const int gpoResult = setActiveGpoSafely(pendingSettings.gpoValue);
        qDebug() << "[FobosLifecycle] set user GPO end" << "result" << gpoResult;
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
    if (frequencyControl) {
        QSignalBlocker blocker(frequencyControl);
        frequencyControl->setValueHz(pendingSettings.centerFrequency);
    }
    if (listeningFrequencyControl) {
        QSignalBlocker blocker(listeningFrequencyControl);
        listeningFrequencyControl->setValueHz(pendingSettings.listeningFrequency);
    }
    settingRange();
    qDebug() << "[FobosLifecycle] applyFobosSettings exit";
    return true;
}

void YourClassName::updateFrequency() {
   const double frequency = scaleWidget ? scaleWidget->currentListeningFrequency() : listeningFrequency;
   if (listeningFrequencyControl) {
       listeningFrequencyControl->setValueHz(frequency);
   }
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
   if (frequencyControl) {
       frequencyControl->setValueHz(frequency);
   }
   onFrequencyEntered();
}

void YourClassName::updateTuningFromScale(double tunedListeningFrequency, double tunedCenterFrequency) {
    const RadioSettings previousSettings = pendingSettings;
    pendingSettings.listeningFrequency = tunedListeningFrequency;
    pendingSettings.centerFrequency = tunedCenterFrequency;
    normalizeTuning(pendingSettings);

    applyCenterFrequencyToHardwareIfNeeded(previousSettings, "scale");

    publishSettingsToGlobals();
    if (frequencyControl) {
        QSignalBlocker blocker(frequencyControl);
        frequencyControl->setValueHz(pendingSettings.centerFrequency);
    }
    if (listeningFrequencyControl) {
        QSignalBlocker blocker(listeningFrequencyControl);
        listeningFrequencyControl->setValueHz(pendingSettings.listeningFrequency);
    }
    settingRange();
    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
    }
}

void YourClassName::showTuneContextMenu(double frequency, const QPoint &globalPos) {
    if (!std::isfinite(frequency)) {
        return;
    }

    const QString frequencyText = QString::number(frequency / 1e6, 'f', 6) + " MHz";
    QMenu menu(this);
    QAction *tuneCenterAction = menu.addAction("Tune signal center here (" + frequencyText + ")");
    QAction *usbEdgeAction = menu.addAction("Set USB lower edge here");
    QAction *lsbEdgeAction = menu.addAction("Set LSB upper edge here");
    menu.addSeparator();
    QAction *centerReceiverAction = menu.addAction("Center receiver here");

    QAction *selected = menu.exec(globalPos);
    if (!selected) {
        return;
    }

    if (selected == tuneCenterAction) {
        tuneSignalCenterAt(frequency);
    } else if (selected == usbEdgeAction) {
        tuneSidebandEdgeAt(frequency, MOD_USB);
    } else if (selected == lsbEdgeAction) {
        tuneSidebandEdgeAt(frequency, MOD_LSB);
    } else if (selected == centerReceiverAction) {
        centerReceiverAt(frequency);
    }
}

void YourClassName::tuneSignalCenterAt(double frequency) {
    double listeningTarget = frequency;
    if (isUpperSidebandMode(pendingSettings.modulationType)) {
        listeningTarget = frequency - pendingSettings.bandwidth * 0.5;
    } else if (isLowerSidebandMode(pendingSettings.modulationType)) {
        listeningTarget = frequency + pendingSettings.bandwidth * 0.5;
    }
    updateTuningFromScale(listeningTarget, pendingSettings.centerFrequency);
}

void YourClassName::tuneSidebandEdgeAt(double frequency, int modulationType) {
    if (pendingSettings.modulationType != modulationType) {
        if (modulationButtonGroup) {
            if (QAbstractButton *button = modulationButtonGroup->button(modulationType)) {
                modulationButtonGroup->blockSignals(true);
                button->setChecked(true);
                modulationButtonGroup->blockSignals(false);
            }
        }
        onModulationChanged(modulationType);
    }
    updateTuningFromScale(frequency, pendingSettings.centerFrequency);
}

void YourClassName::centerReceiverAt(double frequency) {
    updateTuningFromScale(pendingSettings.listeningFrequency, frequency);
}

void YourClassName::onModulationChanged(int id) {
    pendingSettings.modulationType = id;
    pendingSettings.bandwidth = defaultBandwidthForModulation(id);
    if (bandwidthControl) {
        QSignalBlocker blocker(bandwidthControl);
        bandwidthControl->setValueHz(pendingSettings.bandwidth);
    }
    publishSettingsToGlobals();
    if (scaleWidget) {
        scaleWidget->setModulationType(id);
    }
    updateDigitalDecoderMode();
    updateVideoProcessorMode();
    updateIqFrameProducerSettings();
    settingRange();
    qDebug() << "Modulation type changed to:" << id
             << "bandwidth preset" << pendingSettings.bandwidth;
    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
    }
}

void YourClassName::onDigitalTextDecoded(const QString &text) {
    if (!digitalTextEdit || text.isEmpty()) {
        return;
    }

    QTextCursor cursor = digitalTextEdit->textCursor();
    cursor.movePosition(QTextCursor::End);
    cursor.insertText(text);
    digitalTextEdit->setTextCursor(cursor);
}

void YourClassName::onDigitalDecoderStatusChanged(const QString &status) {
    if (digitalStatusLabel) {
        digitalStatusLabel->setText(status);
    }
}

void YourClassName::onVideoStatusChanged(const QString &status) {
    if (videoStatusLabel) {
        if (pendingSettings.modulationType == MOD_SSTV) {
            if (status.startsWith(QStringLiteral("SSTV"))) {
                videoStatusLabel->setText(status);
                return;
            }
            const bool sstvTest = videoTestPatternCheckbox && videoTestPatternCheckbox->isChecked();
            videoStatusLabel->setText(sstvTest
                                      ? QStringLiteral("SSTV: internal image test pattern")
                                      : QStringLiteral("SSTV: image decoder setup ready"));
            return;
        }
        if (pendingSettings.modulationType == MOD_APT) {
            if (status.startsWith(QStringLiteral("NOAA APT"))) {
                videoStatusLabel->setText(status);
                return;
            }
            const bool aptTest = videoTestPatternCheckbox && videoTestPatternCheckbox->isChecked();
            videoStatusLabel->setText(aptTest
                                      ? QStringLiteral("NOAA APT test stream")
                                      : QStringLiteral("NOAA APT: image decoder setup ready"));
            return;
        }
        if (pendingSettings.modulationType == MOD_WEFAX) {
            if (status.startsWith(QStringLiteral("WEFAX"))) {
                videoStatusLabel->setText(status);
                return;
            }
            const bool wefaxTest = videoTestPatternCheckbox && videoTestPatternCheckbox->isChecked();
            videoStatusLabel->setText(wefaxTest
                                      ? QStringLiteral("WEFAX test stream")
                                      : QStringLiteral("WEFAX: image decoder setup ready"));
            return;
        }
        if (pendingSettings.modulationType == MOD_LRPT) {
            if (status.startsWith(QStringLiteral("Meteor LRPT"))) {
                videoStatusLabel->setText(status);
                return;
            }
            videoStatusLabel->setText(QStringLiteral("Meteor LRPT beta: QPSK IQ monitor ready"));
            return;
        }
        videoStatusLabel->setText(status);
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
        scheduleRemoteSettingsCommand();
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
    if (!bandwidthControl) {
        return;
    }

    const double bandwidth = bandwidthControl->valueHz();
    if (bandwidth <= 0.0) {
        return;
    }

    pendingSettings.bandwidth = bandwidth;
    publishSettingsToGlobals();
    settingRange();

    if (scaleWidget) {
        scaleWidget->setTuning(pendingSettings.listeningFrequency,
                               pendingSettings.centerFrequency,
                               pendingSettings.bandwidth,
                               pendingSettings.modulationType);
    }

    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
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
    QPushButton *requestControlButton = new QPushButton("Request Control", &dialog);
    QPushButton *stopButton = new QPushButton("Stop Network", &dialog);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);

    QHBoxLayout *actionLayout = new QHBoxLayout();
    actionLayout->addWidget(testButton);
    actionLayout->addWidget(requestControlButton);
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
        requestControlButton->setEnabled(selectedMode == NetworkMode::Client);
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
        const NetworkMode previousNetworkMode = networkMode;
        const NetworkProcessingMode previousProcessingMode = networkProcessingMode;
        const bool previousServerDisableLocalVisualAudio = serverDisableLocalVisualAudio;

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
        if (isChannelIqRecordingActive() &&
            networkMode != NetworkMode::Disabled &&
            isFullIqProcessingMode()) {
            stopRecording(false);
            updateRecordingStatus(QStringLiteral("Recording stopped: Channel IQ cannot run during Full IQ streaming"));
        }

        const bool networkModeChanged = previousNetworkMode != networkMode;
        const bool processingModeChanged = previousProcessingMode != networkProcessingMode;
        const bool serverLocalOutputChanged =
            previousServerDisableLocalVisualAudio != serverDisableLocalVisualAudio;

        if (!networkController) {
            statusLabel->setText("Network controller unavailable");
            return;
        }

        if (networkMode == NetworkMode::Disabled) {
            if (previousNetworkMode == NetworkMode::Client) {
                stopNetworkClientProcessing();
            } else if (previousNetworkMode == NetworkMode::Server && runState == RadioRunState::Running) {
                if (audioProcessor) {
                    audioProcessor->setLocalPlaybackEnabled(true);
                }
                if (updateTimer) {
                    updateTimer->start();
                }
            }
            if (remoteAudioPlayer) {
                remoteAudioPlayer->stop();
            }
            networkController->stop();
            if (runState == RadioRunState::Running) {
                updateIqFrameProducerSettings();
            }
            savePersistentSettings();
            return;
        }

        if (previousNetworkMode == NetworkMode::Client && networkMode != NetworkMode::Client) {
            stopNetworkClientProcessing();
        }

        if (remoteAudioPlayer && networkMode != NetworkMode::Client) {
            remoteAudioPlayer->stop();
        }
        if (networkMode == NetworkMode::Server) {
            networkController->startServer(networkBindAddress, networkControlPort);
            const bool restartRequired =
                runState == RadioRunState::Running &&
                (networkModeChanged || processingModeChanged);
            if (restartRequired) {
                restartStreamForHardwareChange();
            } else if (runState == RadioRunState::Running && serverLocalOutputChanged) {
                applyServerLocalOutputPolicy();
            }
            savePersistentSettings();
            return;
        }

        if (runState == RadioRunState::Running &&
            (networkModeChanged || processingModeChanged)) {
            if (isClientIqProcessingMode()) {
                startNetworkClientProcessing();
            } else {
                stopNetworkClientProcessing();
            }
        }

        networkController->testClientConnection(networkServerAddress, networkControlPort);
        savePersistentSettings();
    });

    connect(stopButton, &QPushButton::clicked, &dialog, [=]() {
        const NetworkMode previousNetworkMode = networkMode;
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
        if (previousNetworkMode == NetworkMode::Client) {
            stopNetworkClientProcessing();
        } else if (previousNetworkMode == NetworkMode::Server && runState == RadioRunState::Running) {
            if (audioProcessor) {
                audioProcessor->setLocalPlaybackEnabled(true);
            }
            if (updateTimer) {
                updateTimer->start();
            }
        }
        if (runState == RadioRunState::Running) {
            updateIqFrameProducerSettings();
        }
        savePersistentSettings();
    });

    connect(requestControlButton, &QPushButton::clicked, &dialog, [=]() {
        if (networkMode != NetworkMode::Client) {
            statusLabel->setText("Switch to Client mode and connect first.");
            return;
        }
        if (!networkController || !networkController->isControlReady()) {
            statusLabel->setText("Control channel is not ready.");
            return;
        }
        if (networkController->clientHasControl()) {
            statusLabel->setText("This client already has control.");
            return;
        }
        if (sendRemoteControlCommand("requestPriority")) {
            statusLabel->setText("Control request sent.");
        } else {
            statusLabel->setText("Control request could not be sent.");
        }
    });

    connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    dialog.exec();
    if (networkController && statusConnection) {
        disconnect(statusConnection);
    }
}

void YourClassName::updateNetworkButtonText() {
    if (!networkButton) {
        return;
    }

    switch (networkMode) {
    case NetworkMode::Server:
        networkButton->setText(isChannelIqProcessingMode()
                                   ? "Network: Server ChIQ"
                                   : (isFullIqProcessingMode() ? "Network: Server IQ" : "Network: Server"));
        break;
    case NetworkMode::Client: {
        const QString roleSuffix =
            networkController && networkController->isControlReady()
                ? (networkController->clientHasControl() ? QStringLiteral(" Ctrl") : QStringLiteral(" Obs"))
                : QString();
        networkButton->setText((isChannelIqProcessingMode()
                                    ? "Network: Client ChIQ"
                                    : (isFullIqProcessingMode() ? "Network: Client IQ" : "Network: Client")) + roleSuffix);
        break;
    }
    case NetworkMode::Disabled:
    default:
        networkButton->setText("Network");
        break;
    }
}

void YourClassName::onNetworkStatusChanged(const QString &status) {
    qDebug() << "[Network]" << status;
    updateNetworkButtonText();
}

void YourClassName::updateAudioFilterLabels() {
    if (audioLowPassLabel) {
        const QString value = pendingSettings.audioLowPassHz > 0.0
                                  ? audioFilterFrequencyText(pendingSettings.audioLowPassHz)
                                  : QStringLiteral("Auto");
        audioLowPassLabel->setText(QStringLiteral("Audio LPF: %1").arg(value));
    }
    if (audioHighPassLabel) {
        const QString value = pendingSettings.audioHighPassHz > 0.0
                                  ? audioFilterFrequencyText(pendingSettings.audioHighPassHz)
                                  : QStringLiteral("Off");
        audioHighPassLabel->setText(QStringLiteral("Audio HPF: %1").arg(value));
    }
}

void YourClassName::onfftLengthEntered() {
    const int newFftLength = fftComboBox->currentText().toInt();
    applyFftLengthChange(newFftLength, true);
}

bool YourClassName::applyFftLengthChange(int newFftLength, bool notifyRemote) {
    if (newFftLength <= 0) {
        qDebug() << "Invalid FFT length selected.";
        return false;
    }

    if (pendingSettings.fftLength == newFftLength && fftResult) {
        return true;
    }

    qDebug() << "[FFT] applying FFT length"
             << "previous" << pendingSettings.fftLength
             << "new" << newFftLength
             << "state" << runStateName(runState)
             << "networkMode" << static_cast<int>(networkMode)
             << "processingMode" << static_cast<int>(networkProcessingMode);

    pendingSettings.fftLength = newFftLength;
    publishSettingsToGlobals();
    updateSpectrumTimerInterval();
    fftResult = std::make_unique<FFTResult>();

    if (fftComboBox) {
        fftComboBox->blockSignals(true);
        fftComboBox->setCurrentText(QString::number(pendingSettings.fftLength));
        fftComboBox->blockSignals(false);
    }

    if (notifyRemote && isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
    }

    return true;
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
    if (!sampleBox) {
        return;
    }

    QSignalBlocker sampleBoxBlocker(sampleBox);
    auto addDefaultSampleRates = [this]() {
        if (sampleBox->count() > 0) {
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
        qDebug() << "[FobosDevices] using fallback sample-rate list; no local receiver is required for client control";
    };

    void *sampleRateDevice = activeFobosDevice();
    FobosApiKind sampleRateApiKind = activeFobosApiKind;
    bool openedForSampleRates = false;

    int ret = FOBOS_ERR_OK;
    if (!sampleRateDevice) {
        const FobosDeviceInfo selectedInfo = selectedFobosDeviceInfo();
        sampleRateApiKind = selectedInfo.apiKind;
        if (selectedInfo.apiKind == FobosApiKind::Agile) {
            fobos_sdr_dev_t *openedDevice = nullptr;
            ret = openFobosAgileDeviceSafely(&openedDevice, static_cast<uint32_t>(selectedInfo.nativeIndex));
            sampleRateDevice = openedDevice;
        } else {
            fobos_dev_t *openedDevice = nullptr;
            ret = openFobosDeviceSafely(&openedDevice, static_cast<uint32_t>(selectedInfo.nativeIndex));
            sampleRateDevice = openedDevice;
        }
        openedForSampleRates = (ret == FOBOS_ERR_OK && sampleRateDevice);
    }
    if (ret != FOBOS_ERR_OK) {
        qDebug() << "[FobosDevices] sample-rate list unavailable; using fallback, open result:" << ret;
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
    int result = sampleRateApiKind == FobosApiKind::Agile
                     ? getFobosAgileSampleRatesSafely(static_cast<fobos_sdr_dev_t*>(sampleRateDevice), sampleRates, &count)
                     : getFobosSampleRatesSafely(static_cast<fobos_dev_t*>(sampleRateDevice), sampleRates, &count);
    if (result != FOBOS_ERR_OK) {
        qDebug() << "Failed to get sample rates, error code:" << result;
        if (openedForSampleRates) {
            if (sampleRateApiKind == FobosApiKind::Agile) {
                closeFobosAgileDeviceSafely(static_cast<fobos_sdr_dev_t*>(sampleRateDevice));
            } else {
                closeFobosDeviceSafely(static_cast<fobos_dev_t*>(sampleRateDevice));
            }
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
        if (sampleRateApiKind == FobosApiKind::Agile) {
            closeFobosAgileDeviceSafely(static_cast<fobos_sdr_dev_t*>(sampleRateDevice));
        } else {
            closeFobosDeviceSafely(static_cast<fobos_dev_t*>(sampleRateDevice));
        }
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
        const RadioSettings spectrumSettings = spectrumProcessingSettings();
        haveSpectrum = fftResult->storeFFTResults(spectrumSettings, spectrumFrequencies, spectrumMagnitudes);
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

    if (!sampleBox || index < 0) {
        return;
    }

    bool ok = false;
    const double selectedSampleRate = sampleBox->itemData(index).toDouble(&ok);

    if (!ok || selectedSampleRate <= 0.0) {
        qDebug() << "Invalid sample rate selected.";
        return;
    }

    const bool sampleRateChanged =
        std::abs(pendingSettings.sampleRate - selectedSampleRate) > 0.5;

    qDebug() << "[FobosLifecycle] sample rate selected"
             << "previous" << pendingSettings.sampleRate
             << "selected" << selectedSampleRate
             << "changed" << sampleRateChanged
             << "device" << activeFobosDevice()
             << "apiKind" << fobosApiKindName(activeFobosApiKind);

    if (!sampleRateChanged) {
        return;
    }

    pendingSettings.sampleRate = selectedSampleRate;
    normalizeTuning(pendingSettings);
    publishSettingsToGlobals();
    settingRange();

    if (isNetworkClientMode()) {
        if (isClientIqProcessingMode()) {
            resetNetworkIqReceptionState(false, false, pendingSettings.audioEnabled);
        }
        scheduleRemoteSettingsCommand();
        return;
    }

    if (!isIdle()) {
        restartStreamForHardwareChange();
        return;
    }

    if (hasActiveFobosDevice()) {
        const bool selectedRateMatchesOpenSession =
            appliedSampleRate > 0.0 &&
            std::abs(appliedSampleRate - selectedSampleRate) <= 0.5;

        sampleRateReopenRequired = !selectedRateMatchesOpenSession;
    } else {
        sampleRateReopenRequired = false;
    }

    qDebug() << "Sample rate will be applied on the next start.";
}

void YourClassName::onListeningFrequencyEntered() {
    if (!listeningFrequencyControl) {
        return;
    }

    const RadioSettings previousSettings = pendingSettings;
    pendingSettings.listeningFrequency = listeningFrequencyControl->valueHz();
    normalizeTuning(pendingSettings);
    applyCenterFrequencyToHardwareIfNeeded(previousSettings, "listening control");
    publishSettingsToGlobals();
    if (frequencyControl) {
        QSignalBlocker blocker(frequencyControl);
        frequencyControl->setValueHz(pendingSettings.centerFrequency);
    }
    {
        QSignalBlocker blocker(listeningFrequencyControl);
        listeningFrequencyControl->setValueHz(pendingSettings.listeningFrequency);
    }
    qDebug() << "Frequency set to" << listeningFrequency << "Hz";
    settingRange();
    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
    }
}
    
void YourClassName::onFrequencyEntered() {
    const RadioSettings previousSettings = pendingSettings;
    if (pendingSettings.inputMode == 0) {
        if (frequencyControl) {
            pendingSettings.centerFrequency = frequencyControl->valueHz();
            normalizeTuning(pendingSettings, true);
            applyCenterFrequencyToHardwareIfNeeded(previousSettings, "center control");
            publishSettingsToGlobals();
            QSignalBlocker frequencyBlocker(frequencyControl);
            frequencyControl->setValueHz(pendingSettings.centerFrequency);
            if (listeningFrequencyControl) {
                QSignalBlocker listeningBlocker(listeningFrequencyControl);
                listeningFrequencyControl->setValueHz(pendingSettings.listeningFrequency);
            }
            qDebug() << "Frequency set to" << globalFrequency << "Hz";
        }
    } else {
        pendingSettings.centerFrequency = 0;
        normalizeTuning(pendingSettings);
        publishSettingsToGlobals();
        if (frequencyControl) {
            QSignalBlocker frequencyBlocker(frequencyControl);
            frequencyControl->setValueHz(pendingSettings.centerFrequency);
        }
        if (listeningFrequencyControl) {
            QSignalBlocker listeningBlocker(listeningFrequencyControl);
            listeningFrequencyControl->setValueHz(pendingSettings.listeningFrequency);
        }
    }
    settingRange();
    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
    }
}

QString YourClassName::formatFobosDeviceLabel(const FobosDeviceInfo &info) const {
    const QString serial = info.serial.isEmpty() ? QStringLiteral("unknown SN") : info.serial;
    const QString hw = info.hardwareRevision.isEmpty() ? QStringLiteral("?") : info.hardwareRevision;
    const QString fw = info.firmwareVersion.isEmpty() ? QStringLiteral("?") : info.firmwareVersion;
    return QString("[%1] #%2  SN %3  HW %4  FW %5")
        .arg(fobosApiDisplayName(info.apiKind))
        .arg(info.nativeIndex)
        .arg(serial)
        .arg(hw)
        .arg(fw);
}

void YourClassName::refreshFobosDeviceList() {
    availableFobosDevices.clear();

    auto addDevice = [this](const FobosDeviceInfo &info) {
        if (!info.serial.isEmpty()) {
            for (int i = 0; i < availableFobosDevices.size(); ++i) {
                FobosDeviceInfo &existing = availableFobosDevices[i];
                if (existing.serial != info.serial) {
                    continue;
                }
                const bool preferAgile =
                    info.apiKind == FobosApiKind::Agile &&
                    (firmwareLooksAgile(info.firmwareVersion) ||
                     firmwareLooksAgile(existing.firmwareVersion));
                if (preferAgile) {
                    existing = info;
                }
                return;
            }
        }
        availableFobosDevices.append(info);
    };

    const int standardCount = getFobosStandardDeviceCountSafely();
    for (int i = 0; i < standardCount; ++i) {
        fobos_dev_t *infoDevice = nullptr;
        const int openResult = openFobosDeviceSafely(&infoDevice, static_cast<uint32_t>(i));
        if (openResult != FOBOS_ERR_OK || !infoDevice) {
            qDebug() << "[FobosDevices] standard open failed" << "index" << i << "result" << openResult;
            continue;
        }

        char hw[256] = {};
        char fw[256] = {};
        char manufacturer[256] = {};
        char product[256] = {};
        char serial[256] = {};
        const int infoResult = getFobosBoardInfoSafely(infoDevice, hw, fw, manufacturer, product, serial);
        closeFobosDeviceSafely(infoDevice);
        if (infoResult != FOBOS_ERR_OK) {
            qDebug() << "[FobosDevices] standard board info failed" << "index" << i << "result" << infoResult;
            continue;
        }

        FobosDeviceInfo info;
        info.apiKind = FobosApiKind::Standard;
        info.nativeIndex = i;
        info.hardwareRevision = QString::fromLocal8Bit(hw).trimmed();
        info.firmwareVersion = QString::fromLocal8Bit(fw).trimmed();
        info.manufacturer = QString::fromLocal8Bit(manufacturer).trimmed();
        info.product = QString::fromLocal8Bit(product).trimmed();
        info.serial = QString::fromLocal8Bit(serial).trimmed();
        info.label = formatFobosDeviceLabel(info);
        addDevice(info);
    }

    const int agileCount = getFobosAgileDeviceCountSafely();
    for (int i = 0; i < agileCount; ++i) {
        fobos_sdr_dev_t *infoDevice = nullptr;
        const int openResult = openFobosAgileDeviceSafely(&infoDevice, static_cast<uint32_t>(i));
        if (openResult != FOBOS_ERR_OK || !infoDevice) {
            qDebug() << "[FobosDevices] agile open failed" << "index" << i << "result" << openResult;
            continue;
        }

        char hw[256] = {};
        char fw[256] = {};
        char manufacturer[256] = {};
        char product[256] = {};
        char serial[256] = {};
        const int infoResult = getFobosAgileBoardInfoSafely(infoDevice, hw, fw, manufacturer, product, serial);
        closeFobosAgileDeviceSafely(infoDevice);
        if (infoResult != FOBOS_ERR_OK) {
            qDebug() << "[FobosDevices] agile board info failed" << "index" << i << "result" << infoResult;
            continue;
        }

        FobosDeviceInfo info;
        info.apiKind = FobosApiKind::Agile;
        info.nativeIndex = i;
        info.hardwareRevision = QString::fromLocal8Bit(hw).trimmed();
        info.firmwareVersion = QString::fromLocal8Bit(fw).trimmed();
        info.manufacturer = QString::fromLocal8Bit(manufacturer).trimmed();
        info.product = QString::fromLocal8Bit(product).trimmed();
        info.serial = QString::fromLocal8Bit(serial).trimmed();
        info.label = formatFobosDeviceLabel(info);
        addDevice(info);
    }

    qDebug() << "[FobosDevices] refreshed"
             << "standardCount" << standardCount
             << "agileCount" << agileCount
             << "usable" << availableFobosDevices.size();
}

QStringList YourClassName::getFobosDevices() {
    if (!deviceOpened && !(processor && processor->isRunning())) {
        refreshFobosDeviceList();
    }

    QStringList deviceList;
    for (const FobosDeviceInfo &info : std::as_const(availableFobosDevices)) {
        deviceList << info.label;
    }
    if (deviceList.isEmpty()) {
        deviceList << "No Fobos devices detected";
    }
    return deviceList;
}

YourClassName::FobosDeviceInfo YourClassName::selectedFobosDeviceInfo() const {
    int selected = pendingSettings.deviceIndex;
    if (comboBox) {
        bool ok = false;
        const int value = comboBox->currentData().toInt(&ok);
        if (ok) {
            selected = value;
        }
    }
    if (selected >= 0 && selected < availableFobosDevices.size()) {
        return availableFobosDevices[selected];
    }

    FobosDeviceInfo fallback;
    fallback.apiKind = FobosApiKind::Standard;
    fallback.nativeIndex = std::max(0, selected);
    fallback.label = QString("Standard device #%1").arg(fallback.nativeIndex);
    return fallback;
}

void YourClassName::listFobosDevices() {
    if (deviceOpened || (processor && processor->isRunning())) {
        QMessageBox::information(this, "Devices", "Stop processing before listing devices.");
        return;
    }

    refreshFobosDeviceList();
    char standardLib[256] = {};
    char standardDriver[256] = {};
    char agileLib[256] = {};
    char agileDriver[256] = {};
    getFobosStandardApiInfoSafely(standardLib, standardDriver);
    getFobosAgileApiInfoSafely(agileLib, agileDriver);

    QString deviceInfo = QString("Standard API: %1 (%2)\nAgile API: %3 (%4)\n\nDetected devices: %5\n")
                             .arg(standardLib)
                             .arg(standardDriver)
                             .arg(agileLib)
                             .arg(agileDriver)
                             .arg(availableFobosDevices.size());
    for (int i = 0; i < availableFobosDevices.size(); ++i) {
        const FobosDeviceInfo &info = availableFobosDevices[i];
        deviceInfo += QString("%1. %2\n    manufacturer: %3\n    product: %4\n")
                          .arg(i)
                          .arg(info.label)
                          .arg(info.manufacturer)
                          .arg(info.product);
    }
    QMessageBox::information(this, "Fobos Devices", deviceInfo);
}

void YourClassName::onDirectSamplingChanged(int index) {
    Q_UNUSED(index);

    if (!modeBox) {
        return;
    }

    const int value = modeBox->currentData().toInt();

    if (pendingSettings.inputMode == value) {
        return;
    }

    pendingSettings.inputMode = value;

    if (value != 0) {
        pendingSettings.centerFrequency = 0;
        pendingSettings.listeningFrequency = value == 1 ? 0 : 1250000;
    } else {
        pendingSettings.centerFrequency = 100000000;
        pendingSettings.listeningFrequency = 100000000;
    }

    normalizeTuning(pendingSettings);

    if (frequencyControl) {
        QSignalBlocker blocker(frequencyControl);
        frequencyControl->setValueHz(pendingSettings.centerFrequency);
    }

    if (listeningFrequencyControl) {
        QSignalBlocker blocker(listeningFrequencyControl);
        listeningFrequencyControl->setValueHz(pendingSettings.listeningFrequency);
    }

    publishSettingsToGlobals();
    settingRange();

    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
        return;
    }

    if (!isIdle()) {
        restartStreamForHardwareChange();
        return;
    }

    qDebug() << "Input mode will be applied on the next start.";
}
void YourClassName::settingRange() {
    if (!scaleWidget || globalSampleRate <= 0.0) {
        return;
    }

    double newRange = globalSampleRate * (currentScale / 100.0);
    double overallMin = directMinFrequencyForMode(globalMode, globalSampleRate);
    double overallMax = directMaxFrequency(globalSampleRate);

    if (offlineIqPlaybackActive && !offlineIqPlaybackHasMetadata) {
        overallMin = globalFrequency - globalSampleRate / 2.0;
        overallMax = globalFrequency + globalSampleRate / 2.0;
    } else if (globalMode == 0) {
        overallMin = (std::max)(RF_MIN_LISTENING_FREQUENCY,
                                globalFrequency - globalSampleRate / 2.0);
        overallMax = (std::max)(overallMin,
                                globalFrequency + globalSampleRate / 2.0);
    }

    if (listeningFrequencyControl) {
        const double controlMin = globalMode == 0 ? RF_MIN_LISTENING_FREQUENCY : overallMin;
        const double controlMax = globalMode == 0 ? 6000000000.0 : overallMax;
        QSignalBlocker blocker(listeningFrequencyControl);
        listeningFrequencyControl->setRangeHz(controlMin, controlMax);
        listeningFrequencyControl->setValueHz((std::clamp)(listeningFrequency, controlMin, controlMax));
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
        if (hasActiveFobosDevice() && !isIdle() && hardwareSettingsApplied) {
            const int result = setActiveGpoSafely(pendingSettings.gpoValue);
            qDebug() << "[Live] GPO apply result" << result;
            if (result == FOBOS_ERR_OK) {
                appliedHardwareSettings.gpoValue = pendingSettings.gpoValue;
            }
        } else {
            qDebug() << "GPO state will be applied on the next start.";
        }
        if (isNetworkClientMode()) {
            scheduleRemoteSettingsCommand();
        }
    }
}

void YourClassName::onLnaGainChanged(int value) {
    pendingSettings.lnaGain = value;
    lnaGainLabel->setText(QString("LNA Gain: %1").arg(value));

    if (hasActiveFobosDevice() && !isIdle() && hardwareSettingsApplied) {
        const int result = setActiveLnaGainSafely(static_cast<unsigned int>(value));
        qDebug() << "[Live] LNA gain apply result" << result;

        if (result == FOBOS_ERR_OK) {
            appliedHardwareSettings.lnaGain = value;
        }
    }

    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
    }
}

void YourClassName::onVgaGainChanged(int value) {
    pendingSettings.vgaGain = value;
    vgaGainLabel->setText(QString("VGA Gain: %1").arg(value));

    if (hasActiveFobosDevice() && !isIdle() && hardwareSettingsApplied) {
        const int result = setActiveVgaGainSafely(static_cast<unsigned int>(value));
        qDebug() << "[Live] VGA gain apply result" << result;

        if (result == FOBOS_ERR_OK) {
            appliedHardwareSettings.vgaGain = value;
        }
    }

    if (isNetworkClientMode()) {
        scheduleRemoteSettingsCommand();
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
        scheduleRemoteSettingsCommand();
    }
}

void YourClassName::startFobosProcessing() {
    if (playbackManager && playbackManager->isPlaying()) {
        stopPlayback();
    }

    if (isChannelIqRecordingActive() &&
        networkMode != NetworkMode::Disabled &&
        isFullIqProcessingMode()) {
        stopRecording(false);
        updateRecordingStatus(QStringLiteral("Recording stopped: Channel IQ cannot run during Full IQ streaming"));
    }

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
             << "device" << activeFobosDevice()
             << "apiKind" << fobosApiKindName(activeFobosApiKind)
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
        hasActiveFobosDevice() && appliedSampleRate > 0.0 &&
        std::abs(appliedSampleRate - pendingSettings.sampleRate) > 0.5;
    if (sampleRateDiffersFromOpenSession) {
        sampleRateReopenRequired = true;
        qDebug() << "[FobosLifecycle] sample rate differs from previous run; reopening Fobos session before applying"
                 << "appliedSampleRate" << appliedSampleRate
                 << "pendingSampleRate" << pendingSettings.sampleRate;
    }
    publishSettingsToGlobals();

    if (hasActiveFobosDevice() && sampleRateReopenRequired) {
        if (fobosCloseKnownUnsafe) {
            qDebug() << "[FobosLifecycle] previous Fobos close was unsafe; abandoning stale session pointer before reopen"
                     << activeFobosDevice();
            device = nullptr;
            agileDevice = nullptr;
            activeFobosApiKind = FobosApiKind::Standard;
            openedDeviceIndex = -1;
            openedNativeDeviceIndex = -1;
            appliedSampleRate = 0.0;
            appliedHardwareSettings = RadioSettings{};
            hardwareSettingsApplied = false;
            sampleRateReopenRequired = false;
            fobosCloseKnownUnsafe = false;
        }
    }

    if (hasActiveFobosDevice() && sampleRateReopenRequired) {
        if (processor && !processor->isRunning()) {
            processor->finalizeStopped();
        }
        qDebug() << "[FobosLifecycle] closing Fobos session before sample-rate change"
                 << "device" << activeFobosDevice()
                 << "apiKind" << fobosApiKindName(activeFobosApiKind)
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
    if (fobosVerboseLoggingEnabled()) {
        qDebug() << "[FobosLifecycle] clearing IQ buffer before reader start";
    }
    IqBuffer::clear();

    deviceOpened = true;
    const bool serverIqStreaming = networkMode == NetworkMode::Server && isClientIqProcessingMode();
    const bool serverFullIqStreaming = networkMode == NetworkMode::Server && isFullIqProcessingMode();
    const bool serverChannelIqStreaming = networkMode == NetworkMode::Server && isChannelIqProcessingMode();
    const bool channelIqRecording = isChannelIqRecordingActive();
    const bool suppressServerLocalOutput =
        networkMode == NetworkMode::Server &&
        serverDisableLocalVisualAudio &&
        networkController &&
        networkController->isControlReady();
    const bool serverLocalAudioEnabled = pendingSettings.audioEnabled && !serverIqStreaming;
    const bool queueAudioBlocks = !serverIqStreaming;
    if (audioProcessor) {
        audioProcessor->setLocalPlaybackEnabled(!suppressServerLocalOutput);
    }
    if ((serverIqStreaming || channelIqRecording) && processor) {
        processor->configureNetworkIqStreaming(pendingSettings,
                                               true,
                                               serverChannelIqStreaming || channelIqRecording);
    }
    qDebug() << "[FobosLifecycle] starting DataProcessor"
             << "device" << activeFobosDevice()
             << "apiKind" << fobosApiKindName(activeFobosApiKind)
             << "sampleRate" << pendingSettings.sampleRate
             << "syncEnabled" << pendingSettings.syncEnabled
             << "queueAudioBlocks" << queueAudioBlocks
             << "serverIqStreaming" << serverIqStreaming
             << "serverChannelIqStreaming" << serverChannelIqStreaming
             << "channelIqRecording" << channelIqRecording;
    processor->startProcessing(activeFobosDevice(),
                               activeFobosApiKind,
                               pendingSettings.syncEnabled,
                               pendingSettings.sampleRate,
                               queueAudioBlocks,
                               serverIqStreaming || channelIqRecording);
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
    applyServerLocalOutputPolicy();
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
             << "device" << activeFobosDevice()
             << "apiKind" << fobosApiKindName(activeFobosApiKind);

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
                 << "device" << activeFobosDevice()
                 << "apiKind" << fobosApiKindName(activeFobosApiKind);
        device = nullptr;
        agileDevice = nullptr;
        activeFobosApiKind = FobosApiKind::Standard;
        openedDeviceIndex = -1;
        openedNativeDeviceIndex = -1;
        appliedSampleRate = 0.0;
        appliedHardwareSettings = RadioSettings{};
        hardwareSettingsApplied = false;
        sampleRateReopenRequired = false;
        fobosCloseKnownUnsafe = false;
        if (processor && !processor->isRunning()) {
            recreateDataProcessor();
        }
    } else if (hasActiveFobosDevice()) {
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
             << "device" << activeFobosDevice()
             << "apiKind" << fobosApiKindName(activeFobosApiKind)
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
    if (playbackManager && playbackManager->isPlaying()) {
        stopPlayback();
        return;
    }

    if (recordingManager && recordingManager->isRecording()) {
        stopRecording(false);
    }

    if (isNetworkClientMode()) {
        const bool sent = sendRemoteControlCommand("stop");
        if (!sent) {
            qDebug() << "[Network] remote stop could not be sent; stopping local client state";
        }
        if (isClientIqProcessingMode()) {
            stopNetworkClientProcessing();
        } else if (remoteAudioPlayer) {
            remoteAudioPlayer->stop();
        }
        networkClientReconnectPending = false;
        runState = RadioRunState::Idle;
        updateUiForRunState();
        return;
    }

    qDebug() << "[FobosLifecycle] Stop requested"
             << "state" << runStateName(runState)
             << "deviceOpened" << deviceOpened
             << "processorRunning" << (processor && processor->isRunning())
             << "device" << activeFobosDevice()
             << "apiKind" << fobosApiKindName(activeFobosApiKind);
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
