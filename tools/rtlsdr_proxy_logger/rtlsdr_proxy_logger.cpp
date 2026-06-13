#include <Windows.h>

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cwchar>
#include <cstring>
#include <mutex>
#include <string>

struct rtlsdr_dev;
using rtlsdr_dev_t = rtlsdr_dev;
using rtlsdr_read_async_cb_t = void(*)(unsigned char *buf, uint32_t len, void *ctx);

namespace {

constexpr int PROXY_ERR_LOAD = -32000;
constexpr int PROXY_ERR_MISSING = -32001;

HMODULE g_module = nullptr;
HMODULE g_realDll = nullptr;
std::once_flag g_loadOnce;
std::mutex g_logMutex;

std::wstring moduleDirectory()
{
    wchar_t path[MAX_PATH] = {};
    DWORD size = GetModuleFileNameW(g_module, path, MAX_PATH);
    if (size == 0 || size >= MAX_PATH) {
        return L".";
    }
    wchar_t *slash = wcsrchr(path, L'\\');
    if (!slash) {
        return L".";
    }
    *slash = L'\0';
    return path;
}

std::wstring logPath()
{
    return moduleDirectory() + L"\\rtlsdr_proxy.log";
}

void logLine(const char *fmt, ...)
{
    std::lock_guard<std::mutex> lock(g_logMutex);
    FILE *file = nullptr;
    _wfopen_s(&file, logPath().c_str(), L"a");
    if (!file) {
        return;
    }

    SYSTEMTIME now = {};
    GetLocalTime(&now);
    std::fprintf(file,
                 "%04u-%02u-%02u %02u:%02u:%02u.%03u [tid %lu] ",
                 now.wYear,
                 now.wMonth,
                 now.wDay,
                 now.wHour,
                 now.wMinute,
                 now.wSecond,
                 now.wMilliseconds,
                 GetCurrentThreadId());

    va_list args;
    va_start(args, fmt);
    std::vfprintf(file, fmt, args);
    va_end(args);
    std::fputc('\n', file);
    std::fclose(file);
}

void loadRealDll()
{
    const std::wstring path = moduleDirectory() + L"\\rtlsdr_real.dll";
    g_realDll = LoadLibraryW(path.c_str());
    if (g_realDll) {
        logLine("loaded real dll: rtlsdr_real.dll");
    } else {
        logLine("failed to load rtlsdr_real.dll error=%lu", GetLastError());
    }
}

bool ensureLoaded()
{
    std::call_once(g_loadOnce, loadRealDll);
    return g_realDll != nullptr;
}

template <typename Fn>
Fn proc(const char *name)
{
    if (!ensureLoaded()) {
        return nullptr;
    }
    auto *p = reinterpret_cast<Fn>(GetProcAddress(g_realDll, name));
    if (!p) {
        logLine("missing export %s error=%lu", name, GetLastError());
    }
    return p;
}

template <typename Fn, typename... Args>
int callInt(const char *name, Args... args)
{
    Fn fn = proc<Fn>(name);
    if (!fn) {
        return ensureLoaded() ? PROXY_ERR_MISSING : PROXY_ERR_LOAD;
    }
    return fn(args...);
}

} // namespace

extern "C" __declspec(dllexport) uint32_t rtlsdr_get_device_count()
{
    using Fn = uint32_t(*)();
    Fn fn = proc<Fn>("rtlsdr_get_device_count");
    const uint32_t result = fn ? fn() : 0;
    logLine("rtlsdr_get_device_count -> %u", result);
    return result;
}

extern "C" __declspec(dllexport) const char *rtlsdr_get_device_name(uint32_t index)
{
    using Fn = const char*(*)(uint32_t);
    Fn fn = proc<Fn>("rtlsdr_get_device_name");
    const char *result = fn ? fn(index) : nullptr;
    logLine("rtlsdr_get_device_name index=%u -> %s", index, result ? result : "(null)");
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_get_device_usb_strings(uint32_t index, char *manufact, char *product, char *serial)
{
    using Fn = int(*)(uint32_t, char*, char*, char*);
    const int result = callInt<Fn>("rtlsdr_get_device_usb_strings", index, manufact, product, serial);
    logLine("rtlsdr_get_device_usb_strings index=%u -> %d", index, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_get_index_by_serial(const char *serial)
{
    using Fn = int(*)(const char*);
    const int result = callInt<Fn>("rtlsdr_get_index_by_serial", serial);
    logLine("rtlsdr_get_index_by_serial serial=%s -> %d", serial ? serial : "(null)", result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_open(rtlsdr_dev_t **dev, uint32_t index)
{
    using Fn = int(*)(rtlsdr_dev_t**, uint32_t);
    const int result = callInt<Fn>("rtlsdr_open", dev, index);
    logLine("rtlsdr_open index=%u -> %d dev=%p", index, result, dev ? *dev : nullptr);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_close(rtlsdr_dev_t *dev)
{
    using Fn = int(*)(rtlsdr_dev_t*);
    logLine("rtlsdr_close dev=%p begin", dev);
    const int result = callInt<Fn>("rtlsdr_close", dev);
    logLine("rtlsdr_close dev=%p -> %d", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_set_center_freq(rtlsdr_dev_t *dev, uint32_t freq)
{
    using Fn = int(*)(rtlsdr_dev_t*, uint32_t);
    const int result = callInt<Fn>("rtlsdr_set_center_freq", dev, freq);
    logLine("rtlsdr_set_center_freq dev=%p freq=%u -> %d", dev, freq, result);
    return result;
}

extern "C" __declspec(dllexport) uint32_t rtlsdr_get_center_freq(rtlsdr_dev_t *dev)
{
    using Fn = uint32_t(*)(rtlsdr_dev_t*);
    Fn fn = proc<Fn>("rtlsdr_get_center_freq");
    const uint32_t result = fn ? fn(dev) : 0;
    logLine("rtlsdr_get_center_freq dev=%p -> %u", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_set_sample_rate(rtlsdr_dev_t *dev, uint32_t rate)
{
    using Fn = int(*)(rtlsdr_dev_t*, uint32_t);
    const int result = callInt<Fn>("rtlsdr_set_sample_rate", dev, rate);
    logLine("rtlsdr_set_sample_rate dev=%p rate=%u -> %d", dev, rate, result);
    return result;
}

extern "C" __declspec(dllexport) uint32_t rtlsdr_get_sample_rate(rtlsdr_dev_t *dev)
{
    using Fn = uint32_t(*)(rtlsdr_dev_t*);
    Fn fn = proc<Fn>("rtlsdr_get_sample_rate");
    const uint32_t result = fn ? fn(dev) : 0;
    logLine("rtlsdr_get_sample_rate dev=%p -> %u", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_set_tuner_gain_mode(rtlsdr_dev_t *dev, int manual)
{
    using Fn = int(*)(rtlsdr_dev_t*, int);
    const int result = callInt<Fn>("rtlsdr_set_tuner_gain_mode", dev, manual);
    logLine("rtlsdr_set_tuner_gain_mode dev=%p manual=%d -> %d", dev, manual, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_set_tuner_gain(rtlsdr_dev_t *dev, int gain)
{
    using Fn = int(*)(rtlsdr_dev_t*, int);
    const int result = callInt<Fn>("rtlsdr_set_tuner_gain", dev, gain);
    logLine("rtlsdr_set_tuner_gain dev=%p gain=%d -> %d", dev, gain, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_get_tuner_gain(rtlsdr_dev_t *dev)
{
    using Fn = int(*)(rtlsdr_dev_t*);
    const int result = callInt<Fn>("rtlsdr_get_tuner_gain", dev);
    logLine("rtlsdr_get_tuner_gain dev=%p -> %d", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_get_tuner_gains(rtlsdr_dev_t *dev, int *gains)
{
    using Fn = int(*)(rtlsdr_dev_t*, int*);
    const int result = callInt<Fn>("rtlsdr_get_tuner_gains", dev, gains);
    logLine("rtlsdr_get_tuner_gains dev=%p gains=%p -> %d", dev, gains, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_get_tuner_type(rtlsdr_dev_t *dev)
{
    using Fn = int(*)(rtlsdr_dev_t*);
    const int result = callInt<Fn>("rtlsdr_get_tuner_type", dev);
    logLine("rtlsdr_get_tuner_type dev=%p -> %d", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_set_agc_mode(rtlsdr_dev_t *dev, int on)
{
    using Fn = int(*)(rtlsdr_dev_t*, int);
    const int result = callInt<Fn>("rtlsdr_set_agc_mode", dev, on);
    logLine("rtlsdr_set_agc_mode dev=%p on=%d -> %d", dev, on, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_reset_buffer(rtlsdr_dev_t *dev)
{
    using Fn = int(*)(rtlsdr_dev_t*);
    const int result = callInt<Fn>("rtlsdr_reset_buffer", dev);
    logLine("rtlsdr_reset_buffer dev=%p -> %d", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_read_async(rtlsdr_dev_t *dev,
                                                        rtlsdr_read_async_cb_t cb,
                                                        void *ctx,
                                                        uint32_t buf_num,
                                                        uint32_t buf_len)
{
    using Fn = int(*)(rtlsdr_dev_t*, rtlsdr_read_async_cb_t, void*, uint32_t, uint32_t);
    logLine("rtlsdr_read_async dev=%p cb=%p ctx=%p buf_num=%u buf_len=%u begin", dev, reinterpret_cast<void*>(cb), ctx, buf_num, buf_len);
    const int result = callInt<Fn>("rtlsdr_read_async", dev, cb, ctx, buf_num, buf_len);
    logLine("rtlsdr_read_async dev=%p -> %d end", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_wait_async(rtlsdr_dev_t *dev, rtlsdr_read_async_cb_t cb, void *ctx)
{
    using Fn = int(*)(rtlsdr_dev_t*, rtlsdr_read_async_cb_t, void*);
    logLine("rtlsdr_wait_async dev=%p begin", dev);
    const int result = callInt<Fn>("rtlsdr_wait_async", dev, cb, ctx);
    logLine("rtlsdr_wait_async dev=%p -> %d end", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_cancel_async(rtlsdr_dev_t *dev)
{
    using Fn = int(*)(rtlsdr_dev_t*);
    logLine("rtlsdr_cancel_async dev=%p begin", dev);
    const int result = callInt<Fn>("rtlsdr_cancel_async", dev);
    logLine("rtlsdr_cancel_async dev=%p -> %d", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_read_sync(rtlsdr_dev_t *dev, void *buf, int len, int *n_read)
{
    using Fn = int(*)(rtlsdr_dev_t*, void*, int, int*);
    const int result = callInt<Fn>("rtlsdr_read_sync", dev, buf, len, n_read);
    logLine("rtlsdr_read_sync dev=%p len=%d -> %d n_read=%d", dev, len, result, n_read ? *n_read : -1);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_set_direct_sampling(rtlsdr_dev_t *dev, int on)
{
    using Fn = int(*)(rtlsdr_dev_t*, int);
    const int result = callInt<Fn>("rtlsdr_set_direct_sampling", dev, on);
    logLine("rtlsdr_set_direct_sampling dev=%p on=%d -> %d", dev, on, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_get_direct_sampling(rtlsdr_dev_t *dev)
{
    using Fn = int(*)(rtlsdr_dev_t*);
    const int result = callInt<Fn>("rtlsdr_get_direct_sampling", dev);
    logLine("rtlsdr_get_direct_sampling dev=%p -> %d", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_set_offset_tuning(rtlsdr_dev_t *dev, int on)
{
    using Fn = int(*)(rtlsdr_dev_t*, int);
    const int result = callInt<Fn>("rtlsdr_set_offset_tuning", dev, on);
    logLine("rtlsdr_set_offset_tuning dev=%p on=%d -> %d", dev, on, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_get_offset_tuning(rtlsdr_dev_t *dev)
{
    using Fn = int(*)(rtlsdr_dev_t*);
    const int result = callInt<Fn>("rtlsdr_get_offset_tuning", dev);
    logLine("rtlsdr_get_offset_tuning dev=%p -> %d", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_set_freq_correction(rtlsdr_dev_t *dev, int ppm)
{
    using Fn = int(*)(rtlsdr_dev_t*, int);
    const int result = callInt<Fn>("rtlsdr_set_freq_correction", dev, ppm);
    logLine("rtlsdr_set_freq_correction dev=%p ppm=%d -> %d", dev, ppm, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_get_freq_correction(rtlsdr_dev_t *dev)
{
    using Fn = int(*)(rtlsdr_dev_t*);
    const int result = callInt<Fn>("rtlsdr_get_freq_correction", dev);
    logLine("rtlsdr_get_freq_correction dev=%p -> %d", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_set_tuner_bandwidth(rtlsdr_dev_t *dev, uint32_t bw)
{
    using Fn = int(*)(rtlsdr_dev_t*, uint32_t);
    const int result = callInt<Fn>("rtlsdr_set_tuner_bandwidth", dev, bw);
    logLine("rtlsdr_set_tuner_bandwidth dev=%p bw=%u -> %d", dev, bw, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_set_tuner_if_gain(rtlsdr_dev_t *dev, int stage, int gain)
{
    using Fn = int(*)(rtlsdr_dev_t*, int, int);
    const int result = callInt<Fn>("rtlsdr_set_tuner_if_gain", dev, stage, gain);
    logLine("rtlsdr_set_tuner_if_gain dev=%p stage=%d gain=%d -> %d", dev, stage, gain, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_set_testmode(rtlsdr_dev_t *dev, int on)
{
    using Fn = int(*)(rtlsdr_dev_t*, int);
    const int result = callInt<Fn>("rtlsdr_set_testmode", dev, on);
    logLine("rtlsdr_set_testmode dev=%p on=%d -> %d", dev, on, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_set_bias_tee(rtlsdr_dev_t *dev, int on)
{
    using Fn = int(*)(rtlsdr_dev_t*, int);
    const int result = callInt<Fn>("rtlsdr_set_bias_tee", dev, on);
    logLine("rtlsdr_set_bias_tee dev=%p on=%d -> %d", dev, on, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_set_bias_tee_gpio(rtlsdr_dev_t *dev, int gpio, int on)
{
    using Fn = int(*)(rtlsdr_dev_t*, int, int);
    const int result = callInt<Fn>("rtlsdr_set_bias_tee_gpio", dev, gpio, on);
    logLine("rtlsdr_set_bias_tee_gpio dev=%p gpio=%d on=%d -> %d", dev, gpio, on, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_set_xtal_freq(rtlsdr_dev_t *dev, uint32_t rtlFreq, uint32_t tunerFreq)
{
    using Fn = int(*)(rtlsdr_dev_t*, uint32_t, uint32_t);
    const int result = callInt<Fn>("rtlsdr_set_xtal_freq", dev, rtlFreq, tunerFreq);
    logLine("rtlsdr_set_xtal_freq dev=%p rtl=%u tuner=%u -> %d", dev, rtlFreq, tunerFreq, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_get_xtal_freq(rtlsdr_dev_t *dev, uint32_t *rtlFreq, uint32_t *tunerFreq)
{
    using Fn = int(*)(rtlsdr_dev_t*, uint32_t*, uint32_t*);
    const int result = callInt<Fn>("rtlsdr_get_xtal_freq", dev, rtlFreq, tunerFreq);
    logLine("rtlsdr_get_xtal_freq dev=%p -> %d rtl=%u tuner=%u",
            dev,
            result,
            rtlFreq ? *rtlFreq : 0,
            tunerFreq ? *tunerFreq : 0);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_get_usb_strings(rtlsdr_dev_t *dev, char *manufact, char *product, char *serial)
{
    using Fn = int(*)(rtlsdr_dev_t*, char*, char*, char*);
    const int result = callInt<Fn>("rtlsdr_get_usb_strings", dev, manufact, product, serial);
    logLine("rtlsdr_get_usb_strings dev=%p -> %d", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_read_eeprom(rtlsdr_dev_t *dev, uint8_t *data, uint8_t offset, uint16_t len)
{
    using Fn = int(*)(rtlsdr_dev_t*, uint8_t*, uint8_t, uint16_t);
    const int result = callInt<Fn>("rtlsdr_read_eeprom", dev, data, offset, len);
    logLine("rtlsdr_read_eeprom dev=%p offset=%u len=%u -> %d", dev, offset, len, result);
    return result;
}

extern "C" __declspec(dllexport) int rtlsdr_write_eeprom(rtlsdr_dev_t *dev, uint8_t *data, uint8_t offset, uint16_t len)
{
    using Fn = int(*)(rtlsdr_dev_t*, uint8_t*, uint8_t, uint16_t);
    const int result = callInt<Fn>("rtlsdr_write_eeprom", dev, data, offset, len);
    logLine("rtlsdr_write_eeprom dev=%p offset=%u len=%u -> %d", dev, offset, len, result);
    return result;
}

BOOL WINAPI DllMain(HINSTANCE instance, DWORD reason, LPVOID)
{
    if (reason == DLL_PROCESS_ATTACH) {
        g_module = instance;
        DisableThreadLibraryCalls(instance);
    }
    return TRUE;
}
