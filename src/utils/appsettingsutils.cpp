#include "appsettingsutils.h"

#include <QCoreApplication>

QString persistentSettingsFilePath() {
    return QCoreApplication::applicationDirPath() + QStringLiteral("/FobosAPP.ini");
}
