#include <Windows.h>

#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cwchar>
#include <mutex>
#include <string>

struct fobos_sdr_dev_t;
using fobos_sdr_cb_t = void(*)(float *buf, uint32_t buf_length, fobos_sdr_dev_t *sender, void *user);

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
    return moduleDirectory() + L"\\fobos_sdr_proxy.log";
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
    const std::wstring directory = moduleDirectory();
    const std::wstring path = directory + L"\\fobos_sdr_real.dll";
    SetDllDirectoryW(directory.c_str());
    g_realDll = LoadLibraryW(path.c_str());
    SetDllDirectoryW(nullptr);
    if (g_realDll) {
        logLine("loaded real dll: fobos_sdr_real.dll");
    } else {
        logLine("failed to load fobos_sdr_real.dll error=%lu", GetLastError());
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

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_get_api_info(char *libVersion, char *driverVersion)
{
    using Fn = int(__cdecl *)(char*, char*);
    const int result = callInt<Fn>("fobos_sdr_get_api_info", libVersion, driverVersion);
    logLine("fobos_sdr_get_api_info -> %d lib=%s driver=%s",
            result,
            libVersion ? libVersion : "(null)",
            driverVersion ? driverVersion : "(null)");
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_get_device_count()
{
    using Fn = int(__cdecl *)();
    const int result = callInt<Fn>("fobos_sdr_get_device_count");
    logLine("fobos_sdr_get_device_count -> %d", result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_list_devices(char *serials)
{
    using Fn = int(__cdecl *)(char*);
    const int result = callInt<Fn>("fobos_sdr_list_devices", serials);
    logLine("fobos_sdr_list_devices -> %d serials=%s", result, serials ? serials : "(null)");
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_open(fobos_sdr_dev_t **outDev, uint32_t index)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t**, uint32_t);
    const int result = callInt<Fn>("fobos_sdr_open", outDev, index);
    logLine("fobos_sdr_open index=%u -> %d dev=%p", index, result, outDev ? *outDev : nullptr);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_close(fobos_sdr_dev_t *dev)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*);
    logLine("fobos_sdr_close dev=%p begin", dev);
    const int result = callInt<Fn>("fobos_sdr_close", dev);
    logLine("fobos_sdr_close dev=%p -> %d", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_reset(fobos_sdr_dev_t *dev)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*);
    logLine("fobos_sdr_reset dev=%p begin", dev);
    const int result = callInt<Fn>("fobos_sdr_reset", dev);
    logLine("fobos_sdr_reset dev=%p -> %d", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_get_board_info(fobos_sdr_dev_t *dev,
                                                                       char *hwRevision,
                                                                       char *firmwareVersion,
                                                                       char *manufacturer,
                                                                       char *product,
                                                                       char *serial)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, char*, char*, char*, char*, char*);
    const int result = callInt<Fn>("fobos_sdr_get_board_info", dev, hwRevision, firmwareVersion, manufacturer, product, serial);
    logLine("fobos_sdr_get_board_info dev=%p -> %d hw=%s fw=%s manufacturer=%s product=%s serial=%s",
            dev,
            result,
            hwRevision ? hwRevision : "(null)",
            firmwareVersion ? firmwareVersion : "(null)",
            manufacturer ? manufacturer : "(null)",
            product ? product : "(null)",
            serial ? serial : "(null)");
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_set_frequency(fobos_sdr_dev_t *dev, double value)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, double);
    const int result = callInt<Fn>("fobos_sdr_set_frequency", dev, value);
    logLine("fobos_sdr_set_frequency dev=%p value=%.3f -> %d", dev, value, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_start_scan(fobos_sdr_dev_t *dev, double *frequencies, unsigned int count)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, double*, unsigned int);
    const int result = callInt<Fn>("fobos_sdr_start_scan", dev, frequencies, count);
    const double first = frequencies && count > 0 ? frequencies[0] : 0.0;
    const double last = frequencies && count > 0 ? frequencies[count - 1] : 0.0;
    logLine("fobos_sdr_start_scan dev=%p count=%u first=%.3f last=%.3f -> %d", dev, count, first, last, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_stop_scan(fobos_sdr_dev_t *dev)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*);
    logLine("fobos_sdr_stop_scan dev=%p begin", dev);
    const int result = callInt<Fn>("fobos_sdr_stop_scan", dev);
    logLine("fobos_sdr_stop_scan dev=%p -> %d", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_get_scan_index(fobos_sdr_dev_t *dev)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*);
    const int result = callInt<Fn>("fobos_sdr_get_scan_index", dev);
    logLine("fobos_sdr_get_scan_index dev=%p -> %d", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_is_scanning(fobos_sdr_dev_t *dev)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*);
    const int result = callInt<Fn>("fobos_sdr_is_scanning", dev);
    logLine("fobos_sdr_is_scanning dev=%p -> %d", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_set_direct_sampling(fobos_sdr_dev_t *dev, unsigned int enabled)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, unsigned int);
    const int result = callInt<Fn>("fobos_sdr_set_direct_sampling", dev, enabled);
    logLine("fobos_sdr_set_direct_sampling dev=%p enabled=%u -> %d", dev, enabled, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_set_lna_gain(fobos_sdr_dev_t *dev, unsigned int value)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, unsigned int);
    const int result = callInt<Fn>("fobos_sdr_set_lna_gain", dev, value);
    logLine("fobos_sdr_set_lna_gain dev=%p value=%u -> %d", dev, value, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_set_vga_gain(fobos_sdr_dev_t *dev, unsigned int value)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, unsigned int);
    const int result = callInt<Fn>("fobos_sdr_set_vga_gain", dev, value);
    logLine("fobos_sdr_set_vga_gain dev=%p value=%u -> %d", dev, value, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_get_samplerates(fobos_sdr_dev_t *dev, double *values, unsigned int *count)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, double*, unsigned int*);
    const unsigned int requestedCount = count ? *count : 0;
    const int result = callInt<Fn>("fobos_sdr_get_samplerates", dev, values, count);
    logLine("fobos_sdr_get_samplerates dev=%p requestedCount=%u -> %d count=%u",
            dev,
            requestedCount,
            result,
            count ? *count : 0);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_set_samplerate(fobos_sdr_dev_t *dev, double value)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, double);
    const int result = callInt<Fn>("fobos_sdr_set_samplerate", dev, value);
    logLine("fobos_sdr_set_samplerate dev=%p value=%.3f -> %d", dev, value, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_set_bandwidth(fobos_sdr_dev_t *dev, double value)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, double);
    const int result = callInt<Fn>("fobos_sdr_set_bandwidth", dev, value);
    logLine("fobos_sdr_set_bandwidth dev=%p value=%.3f -> %d", dev, value, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_set_auto_bandwidth(fobos_sdr_dev_t *dev, double value)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, double);
    const int result = callInt<Fn>("fobos_sdr_set_auto_bandwidth", dev, value);
    logLine("fobos_sdr_set_auto_bandwidth dev=%p value=%.6f -> %d", dev, value, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_read_async(fobos_sdr_dev_t *dev,
                                                                   fobos_sdr_cb_t cb,
                                                                   void *user,
                                                                   uint32_t bufCount,
                                                                   uint32_t bufLength)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, fobos_sdr_cb_t, void*, uint32_t, uint32_t);
    logLine("fobos_sdr_read_async dev=%p cb=%p user=%p buf_count=%u buf_length=%u begin",
            dev,
            reinterpret_cast<void*>(cb),
            user,
            bufCount,
            bufLength);
    const int result = callInt<Fn>("fobos_sdr_read_async", dev, cb, user, bufCount, bufLength);
    logLine("fobos_sdr_read_async dev=%p -> %d end", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_cancel_async(fobos_sdr_dev_t *dev)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*);
    logLine("fobos_sdr_cancel_async dev=%p begin", dev);
    const int result = callInt<Fn>("fobos_sdr_cancel_async", dev);
    logLine("fobos_sdr_cancel_async dev=%p -> %d", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_set_user_gpo(fobos_sdr_dev_t *dev, uint8_t value)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, uint8_t);
    const int result = callInt<Fn>("fobos_sdr_set_user_gpo", dev, value);
    logLine("fobos_sdr_set_user_gpo dev=%p value=%u -> %d", dev, static_cast<unsigned>(value), result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_set_clk_source(fobos_sdr_dev_t *dev, int value)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, int);
    const int result = callInt<Fn>("fobos_sdr_set_clk_source", dev, value);
    logLine("fobos_sdr_set_clk_source dev=%p value=%d -> %d", dev, value, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_start_sync(fobos_sdr_dev_t *dev, uint32_t bufLength)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, uint32_t);
    const int result = callInt<Fn>("fobos_sdr_start_sync", dev, bufLength);
    logLine("fobos_sdr_start_sync dev=%p buf_length=%u -> %d", dev, bufLength, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_read_sync(fobos_sdr_dev_t *dev, float *buf, uint32_t *actualBufLength)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, float*, uint32_t*);
    const int result = callInt<Fn>("fobos_sdr_read_sync", dev, buf, actualBufLength);
    logLine("fobos_sdr_read_sync dev=%p -> %d actual=%u", dev, result, actualBufLength ? *actualBufLength : 0);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_stop_sync(fobos_sdr_dev_t *dev)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*);
    logLine("fobos_sdr_stop_sync dev=%p begin", dev);
    const int result = callInt<Fn>("fobos_sdr_stop_sync", dev);
    logLine("fobos_sdr_stop_sync dev=%p -> %d", dev, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_read_firmware(fobos_sdr_dev_t *dev, const char *fileName, int verbose)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, const char*, int);
    const int result = callInt<Fn>("fobos_sdr_read_firmware", dev, fileName, verbose);
    logLine("fobos_sdr_read_firmware dev=%p file=%s verbose=%d -> %d", dev, fileName ? fileName : "(null)", verbose, result);
    return result;
}

extern "C" __declspec(dllexport) int __cdecl fobos_sdr_write_firmware(fobos_sdr_dev_t *dev, const char *fileName, int verbose)
{
    using Fn = int(__cdecl *)(fobos_sdr_dev_t*, const char*, int);
    const int result = callInt<Fn>("fobos_sdr_write_firmware", dev, fileName, verbose);
    logLine("fobos_sdr_write_firmware dev=%p file=%s verbose=%d -> %d", dev, fileName ? fileName : "(null)", verbose, result);
    return result;
}

extern "C" __declspec(dllexport) const char *__cdecl fobos_sdr_error_name(int error)
{
    using Fn = const char*(__cdecl *)(int);
    Fn fn = proc<Fn>("fobos_sdr_error_name");
    const char *result = fn ? fn(error) : "proxy error";
    logLine("fobos_sdr_error_name error=%d -> %s", error, result ? result : "(null)");
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
