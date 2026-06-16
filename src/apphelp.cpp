#include "apphelp.h"

QString applicationHelpText(const QString &language) {
    if (language == QStringLiteral("uk")) {
        return QString::fromUtf8(R"HELP(FobosAPP: короткий практичний довідник

Призначення
FobosAPP - SDR-програма для Fobos SDR, Fobos Agile, RTL-SDR, rtl_tcp і експериментального SoapySDR backend. Вона поєднує прийом IQ, спектр, водоспад, аудіодемодуляцію, сканування, записи, пресети, GNSS/QTH-мапу, мережевий режим і дослідні цифрові декодери.

Основна логіка частот
- Central Frequency - центральна частота приймача, тобто центр видимої IQ-смуги.
- Listening Frequency - частота прослуховування/маркер демодулятора всередині поточної смуги.
- Bandwidth - аудіо/канальна смуга для демодулятора.
- У RF-режимі маркер має бути всередині центральна частота +/- половина sample rate.
- У прямому HF-режимі центральна RF-частота не використовується так само, як в RF-режимі.

Миша, жести і швидке налаштування
- Колесо миші над спектром або водоспадом змінює масштаб видимого діапазону.
- Середня кнопка миші/клік колесом по сигналу на спектрі або водоспаді автоматично ставить маркер на центр найближчого видимого сигналу.
- Подвійний лівий клік по спектру або водоспаду робить те саме автоцентрування.
- Правий клік по спектру або водоспаду відкриває меню: поставити центр сигналу, поставити край USB/LSB або перенести центральну частоту приймача.
- Ліве перетягування по спектру показує вимірювання ширини сигналу.
- Fine tune шкала або круглий регулятор рухає частоту прослуховування малими кроками.
- F9 працює як тангента запису: утримуйте для запису, відпустіть для зупинки.

Приймачі
- Fobos Standard - основний режим для стандартної прошивки.
- Fobos Agile - підтримує швидкий firmware scan і live-переналаштування.
- RTL-SDR native спершу використовує rtlsdr\\rtlsdr.dll і сумісний rtlsdr\\libusb-1.0.dll; DLL у корені є тільки запасним варіантом.
- rtl_tcp підключається до 127.0.0.1:1234.
- SoapySDR backend доданий як теоретична сумісність, якщо на системі є SoapySDR.dll і модулі приймача.
- Для Fobos типовий sample rate - 50 MHz. Для RTL типовий безпечний sample rate - 2.048 MHz.

Спектр і водоспад
- Spectrum/waterfall update у загальних налаштуваннях задає інтервал оновлення. Auto обирає безпечний режим.
- Waterfall speed додає кількість рядків на один FFT-кадр: це робить водоспад візуально швидшим без додаткового FFT-навантаження.
- FFT length впливає на деталізацію і навантаження. Великі FFT корисні для вузьких сигналів, але можуть вимагати повільнішого оновлення.
- Band markers показують діапазони: загальні радіодіапазони, аматорські діапазони або компактний шар.
- Spur suppression/Spur calibration допомагає позначати і приглушувати стабільні внутрішні спури.

Аудіо і демодуляція
- Audio вмикає локальне прослуховування.
- Modulation обирає AM, FM/NFM/WFM, SSB, CW, DMR та інші режими.
- Audio LPF/HPF у налаштуваннях фільтрує аудіо після демодуляції.
- Для SSB можна через правий клік поставити край USB/LSB по видимому сигналу.
- Якщо звук не відповідає сигналу після швидких переналаштувань, натисніть Stop/Start або змініть центральну частоту ще раз. Це має бути рідкісний аварійний сценарій.

Сканування
- Agile scan працює тільки з Fobos Agile firmware і сканує список діапазонів на рівні прошивки.
- Standard scan працює через послідовне live-переналаштування центральної частоти. Він доступний для Fobos, RTL, rtl_tcp і Soapy, якщо backend підтримує retune.
- У Standard scan користувач задає список центральних частот у MHz. Програма автоматично розсуває центри мінімум на sample rate, щоб смуги не накладалися.
- Кнопки +/- додають або прибирають сусідні центри відносно найменшого/найбільшого значення.
- Fill range створює список центрів з початкової і кінцевої частоти.
- Dwell ms - скільки часу слухати одну центральну частоту.
- Settle ms - пауза після переналаштування, щоб старі IQ-дані не змішувались з новими.
- Listening scan не рухає центральну частоту, а перебирає частоти прослуховування всередині видимої смуги. Це корисно для GNSS L1, FT8, маяків і будь-яких наборів частот у межах одного IQ-вікна.
- Lock listening frequency у скані фіксує частоту прослуховування, щоб маркер не стрибав між діапазонами.
- Measure у скані збирає current/peak/baseline/delta по бінових ділянках спектра.

Пресети
- Presets відкриває менеджер частот, аудіосмуг, Agile scan, Standard scan, Listening scan, band markers і QTH markers.
- Стрілки вгору/вниз у менеджері пресетів змінюють порядок показу.
- Перед оновленням програми бажано експортувати FobosAPP.ini, щоб не втратити власні пресети, маркери карти і списки скану.
- Import/Export settings у загальних налаштуваннях робить резервну копію або повертає збережені налаштування.

GNSS, GPS і QTH
- GPS/QTH блок зберігає своє місцеположення, показує Maidenhead/QTH locator і відкриває карту.
- QTH Map підтримує offline/grid view, online tile providers і QTH-сітку.
- Колесо миші масштабує карту навколо курсора, перетягування мишею рухає карту.
- Правий клік по карті може ставити користувацький маркер; правий клік по маркеру видаляє його.
- Маркери можна редагувати у Presets -> QTH markers.
- Tune GNSS L1 і GNSS scan виставляють частоти для GPS/Galileo/BeiDou/GLONASS L1.
- GPS C/A accumulate і GPS deep - дослідні корелятори для GPS L1 C/A. Для реального lock потрібен достатній сигнал, стабільна частота і хороша GNSS-антена.
- Save GNSS IQ записує поточний IQ-снепшот у WAV і додає контекст у лог.

Цифрові режими
- Digital Audio містить DMR-дослідний декодер. Він може визначати частину метаданих і працювати з зовнішнім voice backend, але DMR-голос поки експериментальний.
- DMR backend обирає FobosAPP+mbelib, FobosAPP+OpenDMR/OP25 або DSD-neo; DSD-neo передає DMR PCM дискримінатора через TCP і може приймати декодоване аудіо назад через UDP.
- Lock DMR фіксує обрані параметри DMR, корисно коли на частоті є різні color code/timeslot/contact.
- DMR hunter, FPV hunter і Digital Video hunter шукають характерні сигнали на спектрі за шириною/порогом.
- Video блок зараз дослідний; повноцінні відеодекодери можуть бути вимкнені у збірці.

Запис і відтворення
- Record пише Audio WAV або Channel IQ WAV залежно від вибраного режиму.
- Hold F9/F9 утримання - швидкий momentary recording.
- Playback дозволяє програти сумісні записи, коли приймач зупинений.
- Реальні IQ-записи можуть бути великими; перед релізом не додавайте особисті записи, логи і тестові ефіри у репозиторій.

Мережа
- Network відкриває налаштування server/client режиму.
- Server може передавати керування, спектр, аудіо або IQ залежно від режиму.
- Full-IQ/Channel-IQ режими важкі для мережі і CPU, тому їх краще тестувати поступово.
- Audio relay і Audio HTTP stream дозволяють передавати аудіо в інші програми або на інші пристрої.

Корисні правила
- Якщо щось виглядає дивно після зміни приймача або sample rate, натисніть Stop, перевірте backend/sample rate і запустіть знову.
- Для слабких сигналів спершу добийтеся стабільного спектра і правильного центру, а вже потім ускладнюйте демодуляцію.
- Для RTL не ставте занадто великий sample rate: 2.048 або 2.4 MHz зазвичай безпечніші.
- Для Fobos Agile дуже малі інтервали live retune можуть перевантажувати USB/UI на слабкому комп'ютері. Збільшуйте Agile live retune interval, якщо бачите зависання.
- Докладне логування вмикайте тільки на час тесту: воно корисне для діагностики, але може робити програму важчою.
)HELP");
    }

    return QString::fromUtf8(R"HELP(FobosAPP: practical user guide

Purpose
FobosAPP is an SDR application for Fobos SDR, Fobos Agile, RTL-SDR, rtl_tcp, experimental native bladeRF RX and an experimental SoapySDR backend. It combines IQ reception, spectrum, waterfall, audio demodulation, scanning, recordings, presets, GNSS/QTH mapping, network mode and experimental digital decoders.

Frequency model
- Central Frequency is the SDR receiver center, the middle of the visible IQ span.
- Listening Frequency is the demodulator marker inside the current span.
- Bandwidth is the demodulator/audio channel width.
- In RF mode the listening marker must stay inside center frequency +/- half of the sample rate.
- In direct HF modes the RF center is not used the same way as in RF mode.

Mouse actions and fast tuning
- Mouse wheel over the spectrum or waterfall changes visible span/zoom.
- Middle-click, usually clicking the mouse wheel, on a signal in the spectrum or waterfall snaps the listening marker to the nearest visible signal center.
- Double left-click on the spectrum or waterfall does the same auto-centering.
- Right-click on the spectrum or waterfall opens a tuning menu: tune signal center, set USB/LSB edge, or move receiver center here.
- Left-drag on the spectrum shows a bandwidth measurement.
- The fine-tune scale or round dial nudges the listening frequency in small steps.
- F9 works as a push-to-record key: hold to record, release to stop.

Receivers
- Fobos Standard is the main mode for the standard firmware.
- Fobos Agile supports firmware scan and live retuning.
- RTL-SDR native first uses rtlsdr\\rtlsdr.dll and the matching rtlsdr\\libusb-1.0.dll; root-folder DLLs are only a fallback.
- bladeRF native uses the bundled bladerf\\bladeRF.dll runtime in the Windows beta package, or a system libbladeRF installation. It is an RX-only experimental path without SoapySDR.
- rtl_tcp connects to 127.0.0.1:1234.
- SoapySDR is added as theoretical compatibility when SoapySDR.dll and device modules are installed.
- The default Fobos sample rate is 50 MHz. The safe RTL default is 2.048 MHz.

Spectrum and waterfall
- Spectrum/waterfall update in Settings controls redraw interval. Auto keeps a safe FFT-dependent default.
- Waterfall speed adds more rows per FFT frame. It makes the waterfall visually faster without increasing FFT load.
- FFT length controls frequency detail and CPU load. Larger FFT sizes help with narrow signals but may need slower updates.
- Band markers show general radio bands, amateur bands, or a compact combined layer.
- Spur suppression and calibration can mark and reduce stable internal spurs.

Audio and demodulation
- Audio enables local playback.
- Modulation selects AM, FM/NFM/WFM, SSB, CW, DMR and other modes.
- Audio LPF/HPF in settings filters demodulated audio.
- In SSB modes, right-click can align the USB or LSB edge to a visible signal.
- If audio does not match the visible signal after aggressive retuning, use Stop/Start or retune once more. This should be a rare recovery path.

Scanning
- Agile scan works only with Fobos Agile firmware and scans ranges inside the firmware.
- Standard scan works by live-retuning the receiver center through a list. It is available for Fobos, RTL, rtl_tcp, bladeRF and Soapy when the backend supports retune.
- In Standard scan, enter center frequencies in MHz. The app keeps centers at least one sample rate apart to avoid overlapping spans.
- +/- buttons add or remove neighboring centers from the low or high side.
- Fill range generates a center list between the start and end frequencies.
- Dwell ms is how long one center is observed.
- Settle ms is the pause after retune, used to keep old IQ blocks from mixing with new ones.
- Listening scan does not move the receiver center; it cycles listening frequencies inside the visible span. It is useful for GNSS L1, FT8, beacons and any fixed channel list inside one IQ window.
- Lock listening frequency keeps the listening marker fixed while scan centers move.
- Measure in scan mode collects current, peak, baseline and delta values for spectral coverage checks.

Presets
- Presets opens the manager for center frequencies, listening frequencies, audio bandwidths, Agile scan, Standard scan, Listening scan, band markers and QTH markers.
- Up/down arrows in the preset manager change display order.
- Before updating the app, export FobosAPP.ini if you want to keep custom presets, markers and scan lists.
- Import/Export settings in Settings creates or restores a settings backup.

GNSS, GPS and QTH
- GPS/QTH stores your position, shows Maidenhead/QTH locator and opens the map.
- QTH Map supports offline/grid view, online tile providers and a QTH grid overlay.
- Mouse wheel zooms the map around the cursor; dragging pans the map.
- Right-click on the map can place a user marker; right-click on a marker removes it.
- Markers can be edited in Presets -> QTH markers.
- Tune GNSS L1 and GNSS scan set receiver frequencies for GPS/Galileo/BeiDou/GLONASS L1.
- GPS C/A accumulate and GPS deep are experimental GPS L1 C/A correlators. A real lock needs enough signal, stable tuning and a proper GNSS antenna.
- Save GNSS IQ writes the current IQ snapshot to WAV and logs its tuning context.

Digital modes
- Digital Audio contains the experimental DMR decoder. It can detect some metadata and work with an external voice backend, but DMR voice is still experimental.
- DMR backend selects FobosAPP+mbelib, FobosAPP+OpenDMR/OP25 or DSD-neo; DSD-neo mirrors DMR discriminator PCM over TCP and can receive decoded audio back over UDP.
- Lock DMR fixes selected DMR parameters, useful when several color code/timeslot/contact combinations share a frequency.
- DMR hunter, FPV hunter and Digital Video hunter look for characteristic signals by width and threshold.
- Video is currently experimental; full video decoders may be disabled in the build.

Recording and playback
- Record writes Audio WAV or Channel IQ WAV depending on selected recording mode.
- Hold F9 is quick momentary recording.
- Playback can play compatible recordings while the receiver is stopped.
- Real IQ recordings can be huge; avoid adding personal recordings, logs and over-the-air tests to repository releases.

Network
- Network opens server/client settings.
- Server mode can share control, spectrum, audio or IQ depending on processing mode.
- Full-IQ and Channel-IQ modes are heavy for network and CPU, so test them gradually.
- Audio relay and Audio HTTP stream can send demodulated audio to other programs or devices.

Useful rules
- If something looks wrong after changing receiver or sample rate, press Stop, check backend/sample rate and start again.
- For weak signals, first get a stable spectrum and correct center, then tune the demodulator.
- For RTL, avoid excessive sample rates. 2.048 or 2.4 MHz are usually safer.
- On Fobos Agile, very small live-retune intervals can overload USB/UI on slower computers. Increase Agile live retune interval if you see stalls.
- Enable detailed logging only while testing. It is valuable for diagnosis but can make the app heavier.
)HELP");
}
