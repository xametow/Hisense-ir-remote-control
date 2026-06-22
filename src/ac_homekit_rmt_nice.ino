/*
 * HomeKit-кондиционер для ESP32-C3 + управление по ИК через аппаратный RMT
 *
 * ИК-сигнал генерируется аппаратным блоком RMT, без участия CPU по ходу
 * передачи — поэтому загрузка Wi-Fi-стеком не портит тайминги.
 *
 * Температура (16-30°C, целые градусы) и скорость вентилятора (1-5)
 * реализованы через таблицу из 75 реально записанных с пульта кодов
 * (kelon_temp_fan_data.h) — кондей шлёт пультом полное состояние одним
 * пакетом, поэтому проще и надёжнее проиграть нужный готовый пакет, чем
 * реконструировать байты протокола вручную.
 *
 * HomeKit-интерфейс (набор характеристик и логика update()) сделан таким
 * же, как в старом скетче на IRremoteESP8266:
 *   - добавлены CurrentTemperature и TargetFanState (только для вида,
 *     реального датчика температуры и авто-режима вентилятора нет);
 *   - режим всегда принудительно возвращается в Cool (через
 *     targetState->setVal(2) в update(), а не через setValidValues());
 *   - Active / CoolingThresholdTemperature / RotationSpeed обрабатываются
 *     по отдельности через updated(), как в старом скетче, а не разом
 *     через getNewVal() для всех трёх.
 *
 * Файлы в этом скетче (вкладки в Arduino IDE):
 *   - ac_homekit_rmt.ino       — основная логика
 *   - kelon_temp_fan_data.h    — таблица raw-кодов (генерируется один раз,
 *                                руками не редактируется)
 *
 * Библиотеки:
 *   - HomeSpan   https://github.com/HomeSpan/HomeSpan
 *   (IRremoteESP8266 для передачи теперь не нужна, можно не подключать)
 *
 * Установка:
 *  1. Library Manager -> установить "HomeSpan"
 *  2. Board: ESP32C3 Dev Module, USB CDC On Boot: Enabled
 *  3. ИК-светодиод подключи на PIN_IR_LED (через резистор/транзистор, как обычно)
 *
 * Примечание про совместимость: используется новый RMT-драйвер
 * driver/rmt_tx.h (esp-idf 5.x / Arduino-ESP32 core 3.x). HomeSpan сама
 * использует этот же новый драйвер внутри (для адресных светодиодов),
 * поэтому смешивать со старым driver/rmt.h в одном файле нельзя — конфликт
 * типов. Если у тебя core 2.x (esp-idf 4.4) и этот файл не компилируется —
 * напиши, дам версию под старый API.
 */

#include "HomeSpan.h"
#include "driver/rmt_tx.h"
#include "kelon_temp_fan_data.h"

#define PIN_STATUS_LED   8
#define PIN_IR_LED       4   // ИК-светодиод
#define IR_CARRIER_HZ    38000

// =====================================================================
//  RMT-плеер raw ИК-сигналов (новый RMT-драйвер, esp-idf 5.x / core 3.x)
// =====================================================================

rmt_channel_handle_t irTxChan    = NULL;
rmt_encoder_handle_t irCopyEnc   = NULL;

void irRmtInit() {
  rmt_tx_channel_config_t tx_chan_config = {};
  tx_chan_config.gpio_num = (gpio_num_t)PIN_IR_LED;
  tx_chan_config.clk_src = RMT_CLK_SRC_DEFAULT;
  tx_chan_config.resolution_hz = 1000000;     // 1 МГц -> 1 тик = 1 мкс
  tx_chan_config.mem_block_symbols = 64;
  tx_chan_config.trans_queue_depth = 4;
  rmt_new_tx_channel(&tx_chan_config, &irTxChan);

  rmt_carrier_config_t carrier_cfg = {};
  carrier_cfg.frequency_hz = IR_CARRIER_HZ;
  carrier_cfg.duty_cycle = 0.33;
  rmt_apply_carrier(irTxChan, &carrier_cfg);

  rmt_enable(irTxChan);

  rmt_copy_encoder_config_t copy_encoder_config = {};
  rmt_new_copy_encoder(&copy_encoder_config, &irCopyEnc);
}

// timings: чередующиеся mark/space в микросекундах, как из IRrecvDumpV2.
// Если длина массива нечётная (последний mark без закрывающего space —
// обычная история для таких дампов), последний space считаем нулевым.
void irRmtSendRaw(const uint16_t *timings, size_t len) {
  size_t itemCount = (len + 1) / 2;
  rmt_symbol_word_t *items = (rmt_symbol_word_t *)malloc(itemCount * sizeof(rmt_symbol_word_t));
  if (!items) return;

  for (size_t i = 0; i < itemCount; i++) {
    items[i].level0    = 1;
    items[i].duration0 = timings[2 * i];
    items[i].level1    = 0;
    items[i].duration1 = (2 * i + 1 < len) ? timings[2 * i + 1] : 0;
  }

  rmt_transmit_config_t tx_config = {};
  rmt_transmit(irTxChan, irCopyEnc, items, itemCount * sizeof(rmt_symbol_word_t), &tx_config);
  rmt_tx_wait_all_done(irTxChan, portMAX_DELAY);  // ждём завершения передачи
  free(items);
}

// =====================================================================
//  Записанные ИК-команды (raw, мкс), протокол KELON168, 168 бит
//  Получено через IRrecvDumpV2 (IRremoteESP8266 v2.9.0)
// =====================================================================

const uint16_t kelonPowerOnRaw[] = {
  8962, 4522,  540, 1732,  512, 1732,  512, 582,  538, 582,  538, 582,  538, 582,  538, 582,
  540, 1732,  510, 584,  536, 1732,  512, 1732,  512, 582,  538, 582,  538, 584,  538, 582,
  540, 582,  538, 582,  540, 1730,  512, 1732,  512, 582,  538, 582,  540, 582,  536, 586,
  538, 584,  538, 582,  538, 1732,  510, 584,  538, 584,  538, 584,  536, 584,  538, 582,
  538, 584,  538, 582,  538, 582,  536, 584,  538, 584,  536, 586,  536, 584,  536, 582,
  538, 586,  534, 586,  536, 584,  536, 586,  514, 604,  536, 586,  514, 606,  514, 606,
  534, 586,  516, 8044,  516, 604,  514, 606,  514, 606,  514, 608,  514, 604,  516, 606,
  516, 606,  516, 1754,  490, 606,  516, 606,  516, 604,  516, 604,  516, 604,  516, 606,
  516, 604,  516, 604,  516, 604,  516, 606,  516, 604,  516, 604,  518, 604,  516, 606,
  516, 604,  540, 582,  540, 582,  538, 582,  538, 582,  540, 582,  540, 582,  540, 582,
  538, 582,  538, 582,  540, 582,  540, 580,  540, 582,  538, 582,  540, 582,  540, 582,
  540, 582,  540, 580,  542, 580,  542, 578,  544, 578,  544, 578,  544, 576,  544, 578,
  544, 578,  542, 578,  544, 576,  542, 580,  540, 580,  542, 580,  540, 580,  540, 582,
  540, 580,  540, 580,  540, 582,  538, 584,  536, 1706,  536, 584,  538, 584,  536, 584,
  536, 584,  536, 1710,  534, 8024,  534, 586,  534, 588,  532, 588,  534, 588,  532, 590,
  530, 590,  530, 610,  512, 610,  510, 1734,  510, 612,  508, 612,  508, 612,  510, 612,
  508, 612,  508, 614,  506, 614,  506, 616,  504, 616,  504, 618,  502, 618,  502, 618,
  502, 620,  502, 618,  504, 618,  502, 618,  502, 618,  502, 620,  500, 620,  502, 620,
  502, 620,  500, 620,  502, 620,  502, 618,  502, 620,  500, 620,  502, 1742,  500, 1742,
  500, 1744,  500, 620,  500, 620,  502, 620,  500, 620,  500, 620,  500, 622,  500, 1766,
  478, 622,  500, 644,  476, 644,  476, 1766,  478, 644,  478, 644,  478, 1766,  478, 644,
  478, 1764,  478, 644,  476, 644,  478
};

const uint16_t kelonPowerOffRaw[] = {
  8962, 4522,  540, 1730,  512, 1730,  512, 584,  538, 582,  538, 582,  540, 582,  540, 582,
  538, 1732,  512, 582,  538, 1732,  512, 1730,  512, 582,  538, 582,  538, 582,  540, 582,
  538, 582,  538, 582,  538, 1732,  510, 1732,  512, 520,  598, 586,  538, 582,  536, 586,
  536, 584,  538, 582,  538, 1732,  490, 604,  534, 588,  536, 584,  516, 604,  516, 604,
  516, 606,  534, 586,  516, 606,  516, 606,  516, 604,  516, 606,  514, 606,  516, 606,
  516, 604,  516, 606,  516, 606,  516, 604,  516, 606,  516, 606,  516, 606,  516, 604,
  516, 606,  516, 8042,  516, 606,  516, 606,  514, 604,  516, 606,  516, 604,  538, 582,
  538, 582,  540, 1704,  540, 582,  540, 582,  540, 582,  540, 582,  540, 582,  538, 582,
  540, 582,  538, 582,  538, 582,  540, 582,  540, 580,  540, 580,  540, 582,  540, 580,
  542, 580,  542, 580,  542, 578,  542, 578,  544, 576,  568, 552,  544, 576,  544, 578,
  544, 578,  542, 580,  542, 580,  540, 580,  540, 580,  542, 580,  538, 582,  540, 580,
  540, 582,  538, 582,  538, 584,  538, 584,  538, 584,  538, 584,  536, 584,  536, 586,
  536, 584,  536, 584,  536, 586,  534, 588,  534, 588,  532, 590,  532, 588,  532, 590,
  532, 610,  512, 610,  510, 610,  510, 612,  510, 1732,  510, 612,  510, 612,  508, 614,
  508, 612,  508, 1738,  504, 8054,  504, 616,  504, 618,  502, 618,  502, 620,  502, 618,
  502, 620,  502, 618,  502, 620,  500, 1742,  502, 620,  502, 620,  502, 620,  502, 620,
  502, 620,  502, 620,  502, 620,  500, 620,  502, 618,  502, 620,  502, 620,  502, 620,
  502, 620,  500, 620,  502, 620,  500, 620,  500, 620,  502, 620,  502, 620,  500, 620,
  500, 620,  500, 620,  500, 620,  500, 644,  476, 644,  478, 644,  476, 1766,  478, 644,
  478, 1766,  478, 644,  478, 644,  478, 644,  478, 642,  478, 644,  476, 644,  476, 1766,
  476, 644,  476, 644,  476, 646,  476, 1766,  476, 644,  476, 644,  478, 1766,  476, 1768,
  476, 1766,  478, 644,  478, 644,  478
};

void sendPowerOn() {
  irRmtSendRaw(kelonPowerOnRaw, sizeof(kelonPowerOnRaw) / sizeof(kelonPowerOnRaw[0]));
}

void sendPowerOff() {
  irRmtSendRaw(kelonPowerOffRaw, sizeof(kelonPowerOffRaw) / sizeof(kelonPowerOffRaw[0]));
}

// Скорость вентилятора в HomeKit приходит как процент (0-100). У нас есть
// записанные коды только для дискретных скоростей 1-5, поэтому процент
// округляется до ближайшего уровня (20% = 1 уровень).
int fanPercentToIndex(int percent) {
  int idx = (percent + 10) / 20;
  if (idx < 1) idx = 1;
  if (idx > 5) idx = 5;
  return idx;
}

// Кондей шлёт пультом ПОЛНОЕ состояние (температура+вентилятор) одним пакетом,
// поэтому при изменении любого из двух параметров нужно посылать актуальную
// связку (temp, fan) целиком, а не "только температуру" отдельно.
void sendKelonState(int tempC, int fanIdx) {
  const uint16_t* data;
  uint16_t len;
  if (findKelonRaw(tempC, fanIdx, &data, &len)) {
    irRmtSendRaw(data, len);
  } else {
    Serial.printf("Нет записанного кода для %d°C / вентилятор %d\n", tempC, fanIdx);
  }
}

void myWifiBegin(const char* ssid, const char* pwd) {
  boolean status = WiFi.getAutoReconnect();
  WiFi.setAutoReconnect(false);
  WiFi.begin(ssid, pwd);
  WiFi.setBandMode(WIFI_BAND_MODE_2G_ONLY);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.setSleep(false);          // убираем power-save duty cycling радио
  WiFi.setAutoReconnect(status);
}

// ---------- Аксессуар "Кондиционер" ----------
// Набор характеристик и логика update() — как в старом скетче на
// IRremoteESP8266: отдельная обработка Active / CoolingThresholdTemperature /
// RotationSpeed через updated(), плюс CurrentTemperature и TargetFanState
// просто "для вида" (реального датчика и авто-режима вентилятора нет).
struct DEV_AirConditioner : Service::HeaterCooler {

  SpanCharacteristic *active;
  SpanCharacteristic *currentTemp;     // CurrentTemperature — нет датчика, держим статично
  SpanCharacteristic *currentState;
  SpanCharacteristic *targetState;
  SpanCharacteristic *coolThreshold;   // целевая температура
  SpanCharacteristic *fanMode;         // TargetFanState — нет авто-режима, просто для интерфейса
  SpanCharacteristic *rotationSpeed;   // скорость вентилятора, %

  // Что мы реально последний раз отправили по ИК — для дедупликации.
  // lastSentOn совпадает с дефолтным значением active(0) при старте,
  // lastSentTemp/-Fan заведомо невозможные значения, чтобы первая
  // настоящая команда после включения точно прошла.
  bool lastSentOn   = false;
  int  lastSentTemp = -1;
  int  lastSentFan  = -1;

  DEV_AirConditioner() : Service::HeaterCooler() {

    active        = new Characteristic::Active(0);
    currentTemp   = new Characteristic::CurrentTemperature(25.0);
    currentState  = new Characteristic::CurrentHeaterCoolerState(0);
    targetState   = new Characteristic::TargetHeaterCoolerState(2);   // 2 = Cool
    coolThreshold = new Characteristic::CoolingThresholdTemperature(24.0);
    fanMode       = new Characteristic::TargetFanState(0);
    rotationSpeed = new Characteristic::RotationSpeed(60);            // 60% = вентилятор 3

    coolThreshold->setRange(16, 30, 1);    // есть коды только на целые градусы 16-30
    rotationSpeed->setRange(0, 100, 20);   // 5 уровней: 20/40/60/80/100% = вентилятор 1-5

    pinMode(PIN_STATUS_LED, OUTPUT);
  }

  boolean update() override {

    // ====== РЕЖИМ ======
    // Физический кондей умеет только охлаждение — принудительно
    // возвращаем Cool, если приложение попыталось выставить что-то другое.
    if (targetState->updated()) {
      targetState->setVal(2);
    }

    int tempC  = (int)round(coolThreshold->getNewVal<float>());
    int fanIdx = fanPercentToIndex(rotationSpeed->getNewVal());

    // ====== ВКЛ / ВЫКЛ ======
    // Исключаем случай, когда Home вместе с Active присылает ещё и
    // temp/fan в том же запросе — это обычно повторная отправка текущих
    // настроек, а не осознанное нажатие "включить/выключить".
    if (active->updated() && !rotationSpeed->updated() && !coolThreshold->updated()) {

      bool isOn = active->getNewVal();

      if (isOn != lastSentOn) {
        isOn ? sendPowerOn() : sendPowerOff();
        currentState->setVal(isOn ? 3 : 0);   // 3=Cooling, 0=Inactive
        lastSentOn = isOn;
        if (isOn) delay(500);   // дать кондею время принять команду включения
      }

      // При включении сразу подтягиваем актуальные temp/fan, если они
      // отличаются от того, что было отправлено в прошлый раз.
      if (isOn && (tempC != lastSentTemp || fanIdx != lastSentFan)) {
        sendKelonState(tempC, fanIdx);
        lastSentTemp = tempC;
        lastSentFan  = fanIdx;
      }

      return true;
    }

    // ====== ТЕМПЕРАТУРА ======
    if (coolThreshold->updated() && !rotationSpeed->updated()) {

      // Изменение температуры включает кондей, если он был выключен
      // (как в старом скетче).
      if (!lastSentOn) {
        sendPowerOn();
        currentState->setVal(3);
        lastSentOn = true;
        delay(500);
      }

      if (tempC != lastSentTemp || fanIdx != lastSentFan) {
        sendKelonState(tempC, fanIdx);
        lastSentTemp = tempC;
        lastSentFan  = fanIdx;
      }
    }

    // ====== СКОРОСТЬ ВЕНТИЛЯТОРА ======
    if (rotationSpeed->updated()) {

      // Аналогично — изменение скорости вентилятора включает кондей,
      // если он был выключен.
      if (!lastSentOn) {
        sendPowerOn();
        currentState->setVal(3);
        lastSentOn = true;
        delay(500);
      }

      if (tempC != lastSentTemp || fanIdx != lastSentFan) {
        sendKelonState(tempC, fanIdx);
        lastSentTemp = tempC;
        lastSentFan  = fanIdx;
      }
    }

    return true;
  }
};

void setup() {
  Serial.begin(115200);
  irRmtInit();

  homeSpan.setStatusPin(PIN_STATUS_LED);
  homeSpan.begin(Category::AirConditioners, "Кондиционер");
  homeSpan.setWifiBegin(myWifiBegin);

  new SpanAccessory();
    new Service::AccessoryInformation();
      new Characteristic::Identify();
      new Characteristic::Name("Кондиционер");
      new Characteristic::Manufacturer("DIY");
      new Characteristic::SerialNumber("AC-0001");
      new Characteristic::Model("ESP32C3-AC");
      new Characteristic::FirmwareRevision("1.0");

    new DEV_AirConditioner();
}

void loop() {
  homeSpan.poll();
}
