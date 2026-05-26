#ifndef DIAGNOSTICLOGGING_H
#define DIAGNOSTICLOGGING_H

#include <QByteArray>
#include <QtGlobal>

inline bool fobosVerboseLoggingEnabled() {
    static const bool enabled = [] {
        const QByteArray value = qgetenv("FOBOS_VERBOSE_LOG").trimmed().toLower();
        return value == "1" || value == "true" || value == "yes" ||
               value == "on" || value == "debug" || value == "verbose";
    }();
    return enabled;
}

#endif // DIAGNOSTICLOGGING_H
