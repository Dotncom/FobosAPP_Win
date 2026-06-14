#include "main.h"

#include "presethelpers.h"
#include "qthlocator.h"

#include <QAbstractItemView>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QHeaderView>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTabWidget>
#include <QTableWidget>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
void YourClassName::openPresetManager() {
    ensureDefaultFrequencyPresets();
    ensureDefaultBandMarkers();

    QDialog dialog(this);
    dialog.setWindowTitle(uiText(QStringLiteral("preset_manager"), QStringLiteral("Preset Manager")));
    dialog.resize(720, 520);

    QVBoxLayout *rootLayout = new QVBoxLayout(&dialog);
    QTabWidget *tabs = new QTabWidget(&dialog);

    auto moveSelectedTableRow = [](QTableWidget *table, int direction) {
        if (!table || direction == 0) {
            return;
        }
        const int row = table->currentRow();
        const int targetRow = row + direction;
        if (row < 0 || targetRow < 0 || targetRow >= table->rowCount()) {
            return;
        }

        const int columnCount = table->columnCount();
        for (int column = 0; column < columnCount; ++column) {
            QTableWidgetItem *current = table->takeItem(row, column);
            QTableWidgetItem *target = table->takeItem(targetRow, column);
            table->setItem(row, column, target);
            table->setItem(targetRow, column, current);
        }
        table->setCurrentCell(targetRow, 0);
        table->selectRow(targetRow);
    };

    auto addOrderButtons = [this, moveSelectedTableRow](QHBoxLayout *layout,
                                                        QTableWidget *table,
                                                        QWidget *page) {
        QToolButton *moveUpButton = new QToolButton(page);
        moveUpButton->setArrowType(Qt::UpArrow);
        moveUpButton->setToolTip(uiText(QStringLiteral("move_up"), QStringLiteral("Move up")));
        QToolButton *moveDownButton = new QToolButton(page);
        moveDownButton->setArrowType(Qt::DownArrow);
        moveDownButton->setToolTip(uiText(QStringLiteral("move_down"), QStringLiteral("Move down")));
        layout->addWidget(moveUpButton);
        layout->addWidget(moveDownButton);
        QObject::connect(moveUpButton, &QToolButton::clicked, table, [table, moveSelectedTableRow]() {
            moveSelectedTableRow(table, -1);
        });
        QObject::connect(moveDownButton, &QToolButton::clicked, table, [table, moveSelectedTableRow]() {
            moveSelectedTableRow(table, 1);
        });
    };

    auto makeNumericTab = [this, &dialog, addOrderButtons](const QMap<QString, double> &presets,
                                                           const QStringList &order,
                                                           const QString &valueHeader,
                                                           double minimum,
                                                           double maximum) -> QTableWidget * {
        QWidget *page = new QWidget(&dialog);
        QVBoxLayout *pageLayout = new QVBoxLayout(page);
        QTableWidget *table = new QTableWidget(page);
        table->setColumnCount(2);
        table->setHorizontalHeaderLabels({uiText(QStringLiteral("name"), QStringLiteral("Name")), valueHeader});
        table->horizontalHeader()->setStretchLastSection(true);
        table->setSelectionBehavior(QAbstractItemView::SelectRows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);
        table->setRowCount(presets.size());
        int row = 0;
        for (const QString &name : normalizedPresetOrder(order, presets)) {
            auto it = presets.constFind(name);
            if (it == presets.constEnd()) {
                continue;
            }
            table->setItem(row, 0, new QTableWidgetItem(name));
            table->setItem(row, 1, new QTableWidgetItem(QString::number(it.value(), 'f', 3)));
            ++row;
        }
        table->setRowCount(row);

        QHBoxLayout *buttonLayout = new QHBoxLayout();
        QPushButton *addButton = new QPushButton(uiText(QStringLiteral("add"), QStringLiteral("Add")), page);
        QPushButton *removeButton = new QPushButton(uiText(QStringLiteral("remove"), QStringLiteral("Remove")), page);
        buttonLayout->addWidget(addButton);
        buttonLayout->addWidget(removeButton);
        addOrderButtons(buttonLayout, table, page);
        buttonLayout->addStretch();
        pageLayout->addWidget(table);
        pageLayout->addLayout(buttonLayout);

        QObject::connect(addButton, &QPushButton::clicked, table, [this, table, minimum]() {
            const int row = table->rowCount();
            table->insertRow(row);
            table->setItem(row, 0, new QTableWidgetItem(uiText(QStringLiteral("new_preset"), QStringLiteral("New preset"))));
            table->setItem(row, 1, new QTableWidgetItem(QString::number((std::max)(0.0, minimum), 'f', 3)));
            table->setCurrentCell(row, 0);
            table->editItem(table->item(row, 0));
        });
        QObject::connect(removeButton, &QPushButton::clicked, table, [this, table]() {
            const int row = table->currentRow();
            if (row >= 0) {
                table->removeRow(row);
            }
        });

        table->setProperty("minimumValue", minimum);
        table->setProperty("maximumValue", maximum);
        table->setProperty("pageWidget", QVariant::fromValue(static_cast<void*>(page)));
        return table;
    };

    auto makeAgileTab = [this, &dialog, addOrderButtons](const QMap<QString, QString> &presets,
                                                         const QStringList &order) -> QTableWidget * {
        QWidget *page = new QWidget(&dialog);
        QVBoxLayout *pageLayout = new QVBoxLayout(page);
        QTableWidget *table = new QTableWidget(page);
        table->setColumnCount(3);
        table->setHorizontalHeaderLabels({uiText(QStringLiteral("name"), QStringLiteral("Name")),
                                          uiText(QStringLiteral("ranges_mhz_plain"), QStringLiteral("Ranges MHz")),
                                          uiText(QStringLiteral("step_mhz"), QStringLiteral("Step MHz"))});
        table->horizontalHeader()->setStretchLastSection(true);
        table->setSelectionBehavior(QAbstractItemView::SelectRows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);
        table->setRowCount(presets.size());
        int row = 0;
        for (const QString &name : normalizedPresetOrder(order, presets)) {
            auto it = presets.constFind(name);
            if (it == presets.constEnd()) {
                continue;
            }
            table->setItem(row, 0, new QTableWidgetItem(name));
            table->setItem(row, 1, new QTableWidgetItem(agileScanPresetRanges(it.value())));
            table->setItem(row, 2, new QTableWidgetItem(QString::number(agileScanPresetStepMhz(it.value(), 0.0125), 'f', 6)));
            ++row;
        }
        table->setRowCount(row);

        QHBoxLayout *buttonLayout = new QHBoxLayout();
        QPushButton *addButton = new QPushButton(uiText(QStringLiteral("add"), QStringLiteral("Add")), page);
        QPushButton *removeButton = new QPushButton(uiText(QStringLiteral("remove"), QStringLiteral("Remove")), page);
        buttonLayout->addWidget(addButton);
        buttonLayout->addWidget(removeButton);
        addOrderButtons(buttonLayout, table, page);
        buttonLayout->addStretch();
        pageLayout->addWidget(table);
        pageLayout->addLayout(buttonLayout);

        QObject::connect(addButton, &QPushButton::clicked, table, [this, table]() {
            const int row = table->rowCount();
            table->insertRow(row);
            table->setItem(row, 0, new QTableWidgetItem(uiText(QStringLiteral("new_scan_preset"), QStringLiteral("New scan preset"))));
            table->setItem(row, 1, new QTableWidgetItem(QStringLiteral("430-432")));
            table->setItem(row, 2, new QTableWidgetItem(QStringLiteral("0.0125")));
            table->setCurrentCell(row, 0);
            table->editItem(table->item(row, 0));
        });
        QObject::connect(removeButton, &QPushButton::clicked, table, [this, table]() {
            const int row = table->currentRow();
            if (row >= 0) {
                table->removeRow(row);
            }
        });

        table->setProperty("pageWidget", QVariant::fromValue(static_cast<void*>(page)));
        return table;
    };

    auto makeStandardScanTab = [this, &dialog, addOrderButtons](const QMap<QString, QString> &presets,
                                                                const QStringList &order) -> QTableWidget * {
        QWidget *page = new QWidget(&dialog);
        QVBoxLayout *pageLayout = new QVBoxLayout(page);
        QTableWidget *table = new QTableWidget(page);
        table->setColumnCount(4);
        table->setHorizontalHeaderLabels({uiText(QStringLiteral("name"), QStringLiteral("Name")),
                                          uiText(QStringLiteral("centers_mhz_plain"), QStringLiteral("Centers MHz")),
                                          uiText(QStringLiteral("dwell_ms"), QStringLiteral("Dwell ms")),
                                          uiText(QStringLiteral("settle_ms"), QStringLiteral("Settle ms"))});
        table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
        table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
        table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
        table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
        table->setSelectionBehavior(QAbstractItemView::SelectRows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);
        table->setRowCount(presets.size());
        int row = 0;
        for (const QString &name : normalizedPresetOrder(order, presets)) {
            auto it = presets.constFind(name);
            if (it == presets.constEnd()) {
                continue;
            }
            table->setItem(row, 0, new QTableWidgetItem(name));
            table->setItem(row, 1, new QTableWidgetItem(standardScanPresetCenters(it.value())));
            table->setItem(row, 2, new QTableWidgetItem(QString::number(standardScanPresetDwellMs(it.value(), standardScanDwellMs))));
            table->setItem(row, 3, new QTableWidgetItem(QString::number(standardScanPresetSettleMs(it.value(), standardScanSettleMs))));
            ++row;
        }
        table->setRowCount(row);

        QHBoxLayout *buttonLayout = new QHBoxLayout();
        QPushButton *addButton = new QPushButton(uiText(QStringLiteral("add"), QStringLiteral("Add")), page);
        QPushButton *removeButton = new QPushButton(uiText(QStringLiteral("remove"), QStringLiteral("Remove")), page);
        buttonLayout->addWidget(addButton);
        buttonLayout->addWidget(removeButton);
        addOrderButtons(buttonLayout, table, page);
        buttonLayout->addStretch();
        pageLayout->addWidget(table);
        pageLayout->addLayout(buttonLayout);

        QObject::connect(addButton, &QPushButton::clicked, table, [this, table]() {
            const int row = table->rowCount();
            table->insertRow(row);
            table->setItem(row, 0, new QTableWidgetItem(uiText(QStringLiteral("new_standard_scan_preset"),
                                                               QStringLiteral("New standard scan preset"))));
            table->setItem(row, 1, new QTableWidgetItem(standardScanCentersMhz));
            table->setItem(row, 2, new QTableWidgetItem(QString::number(standardScanDwellMs)));
            table->setItem(row, 3, new QTableWidgetItem(QString::number(standardScanSettleMs)));
            table->setCurrentCell(row, 0);
            table->editItem(table->item(row, 0));
        });
        QObject::connect(removeButton, &QPushButton::clicked, table, [table]() {
            const int row = table->currentRow();
            if (row >= 0) {
                table->removeRow(row);
            }
        });

        table->setProperty("pageWidget", QVariant::fromValue(static_cast<void*>(page)));
        return table;
    };

    auto makeListeningScanTab = [this, &dialog, addOrderButtons](const QMap<QString, QString> &presets,
                                                                 const QStringList &order) -> QTableWidget * {
        QWidget *page = new QWidget(&dialog);
        QVBoxLayout *pageLayout = new QVBoxLayout(page);
        QTableWidget *table = new QTableWidget(page);
        table->setColumnCount(4);
        table->setHorizontalHeaderLabels({uiText(QStringLiteral("name"), QStringLiteral("Name")),
                                          uiText(QStringLiteral("targets_mhz_plain"), QStringLiteral("Targets MHz")),
                                          uiText(QStringLiteral("dwell_ms"), QStringLiteral("Dwell ms")),
                                          uiText(QStringLiteral("settle_ms"), QStringLiteral("Settle ms"))});
        table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
        table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
        table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
        table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
        table->setSelectionBehavior(QAbstractItemView::SelectRows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);
        table->setRowCount(presets.size());
        int row = 0;
        for (const QString &name : normalizedPresetOrder(order, presets)) {
            auto it = presets.constFind(name);
            if (it == presets.constEnd()) {
                continue;
            }
            table->setItem(row, 0, new QTableWidgetItem(name));
            table->setItem(row, 1, new QTableWidgetItem(listeningScanPresetTargets(it.value())));
            table->setItem(row, 2, new QTableWidgetItem(QString::number(listeningScanPresetDwellMs(it.value(), listeningScanDwellMs))));
            table->setItem(row, 3, new QTableWidgetItem(QString::number(listeningScanPresetSettleMs(it.value(), listeningScanSettleMs))));
            ++row;
        }
        table->setRowCount(row);

        QHBoxLayout *buttonLayout = new QHBoxLayout();
        QPushButton *addButton = new QPushButton(uiText(QStringLiteral("add"), QStringLiteral("Add")), page);
        QPushButton *removeButton = new QPushButton(uiText(QStringLiteral("remove"), QStringLiteral("Remove")), page);
        buttonLayout->addWidget(addButton);
        buttonLayout->addWidget(removeButton);
        addOrderButtons(buttonLayout, table, page);
        buttonLayout->addStretch();
        pageLayout->addWidget(table);
        pageLayout->addLayout(buttonLayout);

        QObject::connect(addButton, &QPushButton::clicked, table, [this, table]() {
            const int row = table->rowCount();
            table->insertRow(row);
            table->setItem(row, 0, new QTableWidgetItem(uiText(QStringLiteral("new_listening_scan_preset"),
                                                               QStringLiteral("New listening scan preset"))));
            table->setItem(row, 1, new QTableWidgetItem(listeningScanTargetsMhz));
            table->setItem(row, 2, new QTableWidgetItem(QString::number(listeningScanDwellMs)));
            table->setItem(row, 3, new QTableWidgetItem(QString::number(listeningScanSettleMs)));
            table->setCurrentCell(row, 0);
            table->editItem(table->item(row, 0));
        });
        QObject::connect(removeButton, &QPushButton::clicked, table, [table]() {
            const int row = table->currentRow();
            if (row >= 0) {
                table->removeRow(row);
            }
        });

        table->setProperty("pageWidget", QVariant::fromValue(static_cast<void*>(page)));
        return table;
    };

    auto makeBandMarkerTab = [this, &dialog](const QVector<GraphBandMarker> &markers) -> QTableWidget * {
        QWidget *page = new QWidget(&dialog);
        QVBoxLayout *pageLayout = new QVBoxLayout(page);
        QTableWidget *table = new QTableWidget(page);
        table->setColumnCount(4);
        table->setHorizontalHeaderLabels({uiText(QStringLiteral("layer"), QStringLiteral("Layer")),
                                          uiText(QStringLiteral("label"), QStringLiteral("Label")),
                                          uiText(QStringLiteral("start_mhz"), QStringLiteral("Start MHz")),
                                          uiText(QStringLiteral("end_mhz"), QStringLiteral("End MHz"))});
        table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
        table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);
        table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
        table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
        table->setSelectionBehavior(QAbstractItemView::SelectRows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);

        auto setLayerCell = [this, table](int row, bool amateur) {
            QComboBox *layerCombo = new QComboBox(table);
            layerCombo->addItem(uiText(QStringLiteral("common"), QStringLiteral("Common")), QStringLiteral("general"));
            layerCombo->addItem(QStringLiteral("HAM"), QStringLiteral("amateur"));
            layerCombo->setCurrentIndex(amateur ? 1 : 0);
            table->setCellWidget(row, 0, layerCombo);
        };
        auto setBandRow = [table, setLayerCell](int row, const GraphBandMarker &marker) {
            setLayerCell(row, marker.amateur);
            table->setItem(row, 1, new QTableWidgetItem(marker.label));
            table->setItem(row, 2, new QTableWidgetItem(QString::number(marker.startHz / 1000000.0, 'f', 6)));
            table->setItem(row, 3, new QTableWidgetItem(QString::number(marker.endHz / 1000000.0, 'f', 6)));
        };

        table->setRowCount(markers.size());
        for (int row = 0; row < markers.size(); ++row) {
            setBandRow(row, markers.at(row));
        }

        QHBoxLayout *buttonLayout = new QHBoxLayout();
        QPushButton *addCommonButton = new QPushButton(uiText(QStringLiteral("add_common"), QStringLiteral("Add common")), page);
        QPushButton *addHamButton = new QPushButton(uiText(QStringLiteral("add_ham"), QStringLiteral("Add HAM")), page);
        QPushButton *removeButton = new QPushButton(uiText(QStringLiteral("remove"), QStringLiteral("Remove")), page);
        buttonLayout->addWidget(addCommonButton);
        buttonLayout->addWidget(addHamButton);
        buttonLayout->addWidget(removeButton);
        buttonLayout->addStretch();
        pageLayout->addWidget(table);
        pageLayout->addLayout(buttonLayout);

        auto addBandRow = [this, table, setBandRow](bool amateur) {
            const int row = table->rowCount();
            table->insertRow(row);
            GraphBandMarker marker;
            marker.label = amateur ? uiText(QStringLiteral("new_ham_band"), QStringLiteral("New HAM band"))
                                   : uiText(QStringLiteral("new_band"), QStringLiteral("New band"));
            marker.startHz = amateur ? 144000000.0 : 118000000.0;
            marker.endHz = amateur ? 146000000.0 : 137000000.0;
            marker.amateur = amateur;
            setBandRow(row, marker);
            table->setCurrentCell(row, 1);
            table->editItem(table->item(row, 1));
        };
        QObject::connect(addCommonButton, &QPushButton::clicked, table, [addBandRow]() {
            addBandRow(false);
        });
        QObject::connect(addHamButton, &QPushButton::clicked, table, [addBandRow]() {
            addBandRow(true);
        });
        QObject::connect(removeButton, &QPushButton::clicked, table, [table]() {
            const int row = table->currentRow();
            if (row >= 0) {
                table->removeRow(row);
            }
        });

        table->setProperty("pageWidget", QVariant::fromValue(static_cast<void*>(page)));
        return table;
    };

    auto makeQthMarkerTab = [this, &dialog](const QVector<qth::UserMarker> &markers) -> QTableWidget * {
        QWidget *page = new QWidget(&dialog);
        QVBoxLayout *pageLayout = new QVBoxLayout(page);
        QTableWidget *table = new QTableWidget(page);
        table->setColumnCount(5);
        table->setHorizontalHeaderLabels({uiText(QStringLiteral("number"), QStringLiteral("Number")),
                                          uiText(QStringLiteral("name"), QStringLiteral("Name")),
                                          uiText(QStringLiteral("latitude"), QStringLiteral("Latitude")),
                                          uiText(QStringLiteral("longitude"), QStringLiteral("Longitude")),
                                          uiText(QStringLiteral("description"), QStringLiteral("Description"))});
        table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::ResizeToContents);
        table->horizontalHeader()->setSectionResizeMode(1, QHeaderView::ResizeToContents);
        table->horizontalHeader()->setSectionResizeMode(2, QHeaderView::ResizeToContents);
        table->horizontalHeader()->setSectionResizeMode(3, QHeaderView::ResizeToContents);
        table->horizontalHeader()->setSectionResizeMode(4, QHeaderView::Stretch);
        table->setSelectionBehavior(QAbstractItemView::SelectRows);
        table->setSelectionMode(QAbstractItemView::SingleSelection);

        auto setMarkerRow = [table](int row, const qth::UserMarker &marker) {
            table->setItem(row, 0, new QTableWidgetItem(QString::number(marker.number)));
            table->setItem(row, 1, new QTableWidgetItem(marker.name));
            table->setItem(row, 2, new QTableWidgetItem(QString::number(marker.latitude, 'f', 6)));
            table->setItem(row, 3, new QTableWidgetItem(QString::number(marker.longitude, 'f', 6)));
            table->setItem(row, 4, new QTableWidgetItem(marker.description));
        };

        table->setRowCount(markers.size());
        for (int row = 0; row < markers.size(); ++row) {
            setMarkerRow(row, markers.at(row));
        }

        QHBoxLayout *buttonLayout = new QHBoxLayout();
        QPushButton *addButton = new QPushButton(uiText(QStringLiteral("add_marker"), QStringLiteral("Add marker")), page);
        QPushButton *removeButton = new QPushButton(uiText(QStringLiteral("remove"), QStringLiteral("Remove")), page);
        buttonLayout->addWidget(addButton);
        buttonLayout->addWidget(removeButton);
        buttonLayout->addStretch();
        pageLayout->addWidget(table);
        pageLayout->addLayout(buttonLayout);

        QObject::connect(addButton, &QPushButton::clicked, table, [this, table, setMarkerRow]() {
            int nextNumber = 1;
            for (int row = 0; row < table->rowCount(); ++row) {
                bool ok = false;
                const int number = table->item(row, 0) ? table->item(row, 0)->text().toInt(&ok) : 0;
                if (ok) {
                    nextNumber = (std::max)(nextNumber, number + 1);
                }
            }
            qth::UserMarker marker;
            marker.number = nextNumber;
            marker.latitude = qthLatitude;
            marker.longitude = qthLongitude;
            marker.name = qth::maidenheadLocator(marker.latitude, marker.longitude, 6);
            const int row = table->rowCount();
            table->insertRow(row);
            setMarkerRow(row, marker);
            table->setCurrentCell(row, 1);
            table->editItem(table->item(row, 1));
        });
        QObject::connect(removeButton, &QPushButton::clicked, table, [table]() {
            const int row = table->currentRow();
            if (row >= 0) {
                table->removeRow(row);
            }
        });

        table->setProperty("pageWidget", QVariant::fromValue(static_cast<void*>(page)));
        return table;
    };

    QTableWidget *centerTable = makeNumericTab(centerFrequencyPresets,
                                              centerFrequencyPresetOrder,
                                              uiText(QStringLiteral("frequency_hz"), QStringLiteral("Frequency Hz")),
                                              0.0,
                                              PRESET_RF_EXPERIMENTAL_MAX_FREQUENCY);
    QTableWidget *listeningTable = makeNumericTab(listeningFrequencyPresets,
                                                 listeningFrequencyPresetOrder,
                                                 uiText(QStringLiteral("frequency_hz"), QStringLiteral("Frequency Hz")),
                                                 -PRESET_RF_EXPERIMENTAL_MAX_FREQUENCY,
                                                 PRESET_RF_EXPERIMENTAL_MAX_FREQUENCY);
    QTableWidget *bandwidthTable = makeNumericTab(bandwidthValuePresets,
                                                 bandwidthPresetOrder,
                                                 uiText(QStringLiteral("bandwidth_hz"), QStringLiteral("Bandwidth Hz")),
                                                 1.0,
                                                 PRESET_BANDWIDTH_MAX_HZ);
    QTableWidget *agileTable = makeAgileTab(agileScanPresets, agileScanPresetOrder);
    QTableWidget *standardScanTable = makeStandardScanTab(standardScanPresets, standardScanPresetOrder);
    QTableWidget *listeningScanTable = makeListeningScanTab(listeningScanPresets, listeningScanPresetOrder);
    QTableWidget *bandMarkerTable = makeBandMarkerTab(bandMarkers);
    QTableWidget *qthMarkerTable = makeQthMarkerTab(qthUserMarkers);

    tabs->addTab(static_cast<QWidget*>(centerTable->property("pageWidget").value<void*>()),
                 uiText(QStringLiteral("preset_tab_center"), QStringLiteral("Center")));
    tabs->addTab(static_cast<QWidget*>(listeningTable->property("pageWidget").value<void*>()),
                 uiText(QStringLiteral("preset_tab_listen"), QStringLiteral("Listen")));
    tabs->addTab(static_cast<QWidget*>(bandwidthTable->property("pageWidget").value<void*>()),
                 uiText(QStringLiteral("preset_tab_audio_bw"), QStringLiteral("Audio BW")));
    tabs->addTab(static_cast<QWidget*>(agileTable->property("pageWidget").value<void*>()),
                 uiText(QStringLiteral("agile_scan"), QStringLiteral("Agile scan")));
    tabs->addTab(static_cast<QWidget*>(standardScanTable->property("pageWidget").value<void*>()),
                 uiText(QStringLiteral("preset_tab_standard_scan"), QStringLiteral("Standard scan")));
    tabs->addTab(static_cast<QWidget*>(listeningScanTable->property("pageWidget").value<void*>()),
                 uiText(QStringLiteral("preset_tab_listening_scan"), QStringLiteral("Listening scan")));
    tabs->addTab(static_cast<QWidget*>(bandMarkerTable->property("pageWidget").value<void*>()),
                 uiText(QStringLiteral("general_band_markers"), QStringLiteral("Band markers")));
    tabs->addTab(static_cast<QWidget*>(qthMarkerTable->property("pageWidget").value<void*>()),
                 uiText(QStringLiteral("preset_tab_qth_markers"), QStringLiteral("QTH markers")));

    QLabel *hintLabel = new QLabel(uiText(QStringLiteral("presets_hint"),
                                          QStringLiteral("Values are stored in Hz for frequency/audio presets. Scan presets and band-marker ranges are edited in MHz. HAM defaults are Region-1-style hints; edit them for local rules.")),
                                   &dialog);
    hintLabel->setWordWrap(true);
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
    if (QPushButton *okButton = buttonBox->button(QDialogButtonBox::Ok)) {
        okButton->setText(uiText(QStringLiteral("ok"), QStringLiteral("OK")));
    }
    if (QPushButton *cancelButton = buttonBox->button(QDialogButtonBox::Cancel)) {
        cancelButton->setText(uiText(QStringLiteral("cancel"), QStringLiteral("Cancel")));
    }
    rootLayout->addWidget(tabs);
    rootLayout->addWidget(hintLabel);
    rootLayout->addWidget(buttonBox);

    auto tableNameOrder = [](QTableWidget *table) {
        QStringList order;
        if (!table) {
            return order;
        }
        for (int row = 0; row < table->rowCount(); ++row) {
            const QString name = table->item(row, 0) ? table->item(row, 0)->text().trimmed() : QString();
            if (!name.isEmpty() && !order.contains(name)) {
                order.append(name);
            }
        }
        return order;
    };

    auto readNumericTable = [](QTableWidget *table,
                               const QString &tabName,
                               QMap<QString, double> &target,
                               QString *error) {
        QMap<QString, double> next;
        const double minimum = table->property("minimumValue").toDouble();
        const double maximum = table->property("maximumValue").toDouble();
        for (int row = 0; row < table->rowCount(); ++row) {
            const QString name = table->item(row, 0) ? table->item(row, 0)->text().trimmed() : QString();
            const QString valueText = table->item(row, 1) ? table->item(row, 1)->text().trimmed() : QString();
            if (name.isEmpty() && valueText.isEmpty()) {
                continue;
            }
            bool ok = false;
            const double value = valueText.toDouble(&ok);
            if (name.isEmpty() || !ok || !std::isfinite(value) || value < minimum || value > maximum) {
                if (error) {
                    *error = QStringLiteral("%1: bad numeric preset at row %2 (%3 = %4, allowed %5..%6)")
                                 .arg(tabName,
                                      QString::number(row + 1),
                                      name.isEmpty() ? QStringLiteral("<empty>") : name,
                                      valueText,
                                      QString::number(minimum, 'f', 0),
                                      QString::number(maximum, 'f', 0));
                }
                return false;
            }
            next[name] = value;
        }
        target = next;
        return true;
    };

    auto readAgileTable = [](QTableWidget *table, QMap<QString, QString> &target, QString *error) {
        QMap<QString, QString> next;
        for (int row = 0; row < table->rowCount(); ++row) {
            const QString name = table->item(row, 0) ? table->item(row, 0)->text().trimmed() : QString();
            const QString ranges = table->item(row, 1) ? table->item(row, 1)->text().trimmed() : QString();
            const QString stepText = table->item(row, 2) ? table->item(row, 2)->text().trimmed() : QString();
            if (name.isEmpty() && ranges.isEmpty() && stepText.isEmpty()) {
                continue;
            }
            bool ok = false;
            const double step = stepText.toDouble(&ok);
            if (name.isEmpty() || ranges.isEmpty() || !ok ||
                !std::isfinite(step) ||
                step < PRESET_AGILE_SCAN_MIN_STEP_MHZ ||
                step > PRESET_AGILE_SCAN_MAX_STEP_MHZ) {
                if (error) {
                    *error = QStringLiteral("Bad Agile scan preset at row %1").arg(row + 1);
                }
                return false;
            }
            QString parseError;
            parseAgileScanFrequenciesMhz(ranges, step, &parseError);
            if (!parseError.isEmpty()) {
                if (error) {
                    *error = QStringLiteral("%1: %2").arg(name, parseError);
                }
                return false;
            }
            next[name] = agileScanPresetSpec(ranges, step);
        }
        target = next;
        return true;
    };

    auto readStandardScanTable = [this](QTableWidget *table, QMap<QString, QString> &target, QString *error) {
        QMap<QString, QString> next;
        for (int row = 0; row < table->rowCount(); ++row) {
            const QString name = table->item(row, 0) ? table->item(row, 0)->text().trimmed() : QString();
            const QString centers = table->item(row, 1) ? table->item(row, 1)->text().trimmed() : QString();
            const QString dwellText = table->item(row, 2) ? table->item(row, 2)->text().trimmed() : QString();
            const QString settleText = table->item(row, 3) ? table->item(row, 3)->text().trimmed() : QString();
            if (name.isEmpty() && centers.isEmpty() && dwellText.isEmpty() && settleText.isEmpty()) {
                continue;
            }
            bool dwellOk = false;
            bool settleOk = false;
            const int dwellMs = dwellText.toInt(&dwellOk);
            const int settleMs = settleText.toInt(&settleOk);
            if (name.isEmpty() ||
                centers.isEmpty() ||
                !dwellOk ||
                !settleOk ||
                dwellMs < PRESET_STANDARD_SCAN_MIN_DWELL_MS ||
                dwellMs > PRESET_STANDARD_SCAN_MAX_DWELL_MS ||
                settleMs < PRESET_STANDARD_SCAN_MIN_SETTLE_MS ||
                settleMs > PRESET_STANDARD_SCAN_MAX_SETTLE_MS) {
                if (error) {
                    *error = QStringLiteral("Bad standard scan preset at row %1").arg(row + 1);
                }
                return false;
            }
            QString parseError;
            parseStandardScanCentersMhz(centers,
                                        pendingSettings.sampleRate,
                                        PRESET_AGILE_SCAN_MIN_POINTS,
                                        &parseError,
                                        nullptr);
            if (!parseError.isEmpty()) {
                if (error) {
                    *error = QStringLiteral("%1: %2").arg(name, parseError);
                }
                return false;
            }
            next[name] = standardScanPresetSpec(centers, dwellMs, settleMs);
        }
        target = next;
        return true;
    };

    auto readListeningScanTable = [](QTableWidget *table, QMap<QString, QString> &target, QString *error) {
        QMap<QString, QString> next;
        for (int row = 0; row < table->rowCount(); ++row) {
            const QString name = table->item(row, 0) ? table->item(row, 0)->text().trimmed() : QString();
            const QString targets = table->item(row, 1) ? table->item(row, 1)->text().trimmed() : QString();
            const QString dwellText = table->item(row, 2) ? table->item(row, 2)->text().trimmed() : QString();
            const QString settleText = table->item(row, 3) ? table->item(row, 3)->text().trimmed() : QString();
            if (name.isEmpty() && targets.isEmpty() && dwellText.isEmpty() && settleText.isEmpty()) {
                continue;
            }
            bool dwellOk = false;
            bool settleOk = false;
            const int dwellMs = dwellText.toInt(&dwellOk);
            const int settleMs = settleText.toInt(&settleOk);
            if (name.isEmpty() ||
                targets.isEmpty() ||
                !dwellOk ||
                !settleOk ||
                dwellMs < PRESET_LISTENING_SCAN_MIN_DWELL_MS ||
                dwellMs > PRESET_LISTENING_SCAN_MAX_DWELL_MS ||
                settleMs < PRESET_LISTENING_SCAN_MIN_SETTLE_MS ||
                settleMs > PRESET_LISTENING_SCAN_MAX_SETTLE_MS) {
                if (error) {
                    *error = QStringLiteral("Bad listening scan preset at row %1").arg(row + 1);
                }
                return false;
            }
            QString parseError;
            parseListeningScanTargetsMhz(targets, 0.0, PRESET_RF_EXPERIMENTAL_MAX_FREQUENCY, 1, &parseError);
            if (!parseError.isEmpty()) {
                if (error) {
                    *error = QStringLiteral("%1: %2").arg(name, parseError);
                }
                return false;
            }
            next[name] = listeningScanPresetSpec(targets, dwellMs, settleMs);
        }
        target = next;
        return true;
    };

    auto readBandMarkerTable = [](QTableWidget *table, QVector<GraphBandMarker> &target, QString *error) {
        QVector<GraphBandMarker> next;
        for (int row = 0; row < table->rowCount(); ++row) {
            QComboBox *layerCombo = qobject_cast<QComboBox*>(table->cellWidget(row, 0));
            const QString layer = layerCombo ? layerCombo->currentData().toString() : QStringLiteral("general");
            const QString label = table->item(row, 1) ? table->item(row, 1)->text().trimmed() : QString();
            const QString startText = table->item(row, 2) ? table->item(row, 2)->text().trimmed() : QString();
            const QString endText = table->item(row, 3) ? table->item(row, 3)->text().trimmed() : QString();
            if (label.isEmpty() && startText.isEmpty() && endText.isEmpty()) {
                continue;
            }

            bool startOk = false;
            bool endOk = false;
            const double startMhz = startText.toDouble(&startOk);
            const double endMhz = endText.toDouble(&endOk);
            if (label.isEmpty() ||
                !startOk ||
                !endOk ||
                !std::isfinite(startMhz) ||
                !std::isfinite(endMhz) ||
                startMhz < 0.0 ||
                endMhz <= startMhz ||
                endMhz > 100000.0) {
                if (error) {
                    *error = QStringLiteral("Bad band marker at row %1").arg(row + 1);
                }
                return false;
            }

            GraphBandMarker marker;
            marker.startHz = startMhz * 1000000.0;
            marker.endHz = endMhz * 1000000.0;
            marker.label = label;
            marker.amateur = layer == QStringLiteral("amateur");
            next.append(marker);
        }
        target = next;
        return true;
    };

    auto readQthMarkerTable = [this](QTableWidget *table, QVector<qth::UserMarker> &target, QString *error) {
        QVector<qth::UserMarker> next;
        QVector<int> usedNumbers;
        for (int row = 0; row < table->rowCount(); ++row) {
            const QString numberText = table->item(row, 0) ? table->item(row, 0)->text().trimmed() : QString();
            QString name = table->item(row, 1) ? table->item(row, 1)->text().trimmed() : QString();
            const QString latText = table->item(row, 2) ? table->item(row, 2)->text().trimmed() : QString();
            const QString lonText = table->item(row, 3) ? table->item(row, 3)->text().trimmed() : QString();
            const QString description = table->item(row, 4) ? table->item(row, 4)->text().trimmed() : QString();
            if (numberText.isEmpty() && name.isEmpty() && latText.isEmpty() && lonText.isEmpty() && description.isEmpty()) {
                continue;
            }

            bool numberOk = false;
            bool latOk = false;
            bool lonOk = false;
            const int number = numberText.toInt(&numberOk);
            const double latitude = latText.toDouble(&latOk);
            const double longitude = lonText.toDouble(&lonOk);
            if (!numberOk ||
                number <= 0 ||
                usedNumbers.contains(number) ||
                !latOk ||
                !lonOk ||
                !qth::isValidLatitude(latitude) ||
                !qth::isValidLongitude(longitude)) {
                if (error) {
                    *error = uiText(QStringLiteral("bad_qth_marker_row"),
                                    QStringLiteral("Bad QTH marker at row %1"))
                                 .arg(row + 1);
                }
                return false;
            }

            usedNumbers.append(number);
            if (name.isEmpty()) {
                name = qth::maidenheadLocator(latitude, longitude, 6);
            }

            qth::UserMarker marker;
            marker.number = number;
            marker.name = name.left(80);
            marker.description = description.left(512);
            marker.latitude = latitude;
            marker.longitude = longitude;
            next.append(marker);
        }
        target = next;
        return true;
    };

    connect(buttonBox, &QDialogButtonBox::accepted, &dialog, [&]() {
        QString error;
        QMap<QString, double> nextCenter = centerFrequencyPresets;
        QMap<QString, double> nextListening = listeningFrequencyPresets;
        QMap<QString, double> nextBandwidth = bandwidthValuePresets;
        QMap<QString, QString> nextAgile = agileScanPresets;
        QMap<QString, QString> nextStandardScan = standardScanPresets;
        QMap<QString, QString> nextListeningScan = listeningScanPresets;
        QVector<GraphBandMarker> nextBandMarkers = bandMarkers;
        QVector<qth::UserMarker> nextQthMarkers = qthUserMarkers;
        if (!readNumericTable(centerTable,
                              uiText(QStringLiteral("preset_tab_center"), QStringLiteral("Center")),
                              nextCenter,
                              &error) ||
            !readNumericTable(listeningTable,
                              uiText(QStringLiteral("preset_tab_listen"), QStringLiteral("Listen")),
                              nextListening,
                              &error) ||
            !readNumericTable(bandwidthTable,
                              uiText(QStringLiteral("preset_tab_audio_bw"), QStringLiteral("Audio BW")),
                              nextBandwidth,
                              &error) ||
            !readAgileTable(agileTable, nextAgile, &error) ||
            !readStandardScanTable(standardScanTable, nextStandardScan, &error) ||
            !readListeningScanTable(listeningScanTable, nextListeningScan, &error) ||
            !readBandMarkerTable(bandMarkerTable, nextBandMarkers, &error) ||
            !readQthMarkerTable(qthMarkerTable, nextQthMarkers, &error)) {
            QMessageBox::warning(&dialog,
                                  uiText(QStringLiteral("preset_manager"), QStringLiteral("Preset Manager")),
                                  error);
            return;
        }
        centerFrequencyPresets = nextCenter;
        listeningFrequencyPresets = nextListening;
        bandwidthValuePresets = nextBandwidth;
        agileScanPresets = nextAgile;
        standardScanPresets = nextStandardScan;
        listeningScanPresets = nextListeningScan;
        centerFrequencyPresetOrder =
            normalizedPresetOrder(tableNameOrder(centerTable), centerFrequencyPresets);
        listeningFrequencyPresetOrder =
            normalizedPresetOrder(tableNameOrder(listeningTable), listeningFrequencyPresets);
        bandwidthPresetOrder =
            normalizedPresetOrder(tableNameOrder(bandwidthTable), bandwidthValuePresets);
        agileScanPresetOrder =
            normalizedPresetOrder(tableNameOrder(agileTable), agileScanPresets);
        standardScanPresetOrder =
            normalizedPresetOrder(tableNameOrder(standardScanTable), standardScanPresets);
        listeningScanPresetOrder =
            normalizedPresetOrder(tableNameOrder(listeningScanTable), listeningScanPresets);
        bandMarkers = nextBandMarkers;
        qthUserMarkers = nextQthMarkers;
        bandMarkersCustomized = true;
        updateFrequencyPresetControls();
        updateGraphBandMarkers();
        updateQthControls();
        savePersistentSettings();
        dialog.accept();
    });
    connect(buttonBox, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    dialog.exec();
}
