#include <BluetoothSerial.h>
#include <ELMduino.h>
#include <EncButton.h>
#include <U8g2lib.h>
#include <WiFi.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/// --- Веб часть  --- ///
#define WIFI_SSID "Kit"
#define WIFI_PASS "QazXswCde"

#include <GyverDBFile.h>
#include <LittleFS.h>
GyverDBFile db(&LittleFS, "/db.db");

#include <SettingsGyver.h>
SettingsGyver sett("БК", &db);
/// --- Веб часть  --- ///

/// --- Датчик температуры  --- ///
#define ONE_WIRE_BUS 4 
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
unsigned long lastRequestTime = 0;  // Время последнего запроса
const unsigned long requestInterval = 5000;  // Интервал 5 секунд
/// --- Датчик температуры  --- ///

/// --- База данных --- ///
DB_KEYS(
    kk,
    odometrDB,         // Общий пробег
    fuelDB,            // Общее топливо
    odometr100DB,      // Пробег, после достижения 100км обнуляется сохраняя в базу данных расход топлива за этот километраж
    fuel100DB,         // Чтоб посчитать расход на 100, так же обнуляется
    last100lper100km,  // Чтоб посчитать расход на 100, так же обнуляется
    timeCorrect,       // Корректировка коэффициента времени arduino uno, у каждого свой.
    fuelRatio,         // Коэффициент коррекции топлива
    wifiMode,          //  Режим Wi-fi Точка доступа/Подключение к точке доступа
    wifiSSID,          // Название точки доступа для подключения
    wifiPass,          // Пароль точки доступа для подключения
    wifiSSIDAP,        // Название точки доступа для подключения
    wifiPassAP         // Пароль точки доступа для подключения
);
/// --- База данных --- ///

/// --- Bluetooth конфигурация --- ///
BluetoothSerial SerialBT;
#define ELM_PORT SerialBT
const uint8_t address[6] = {0x11, 0x33, 0x77, 0x54, 0x00, 0x34};
// const uint8_t address[6] = {0xE8, 0x6B, 0xEA, 0xF6, 0xB9, 0x76};
/// --- Bluetooth конфигурация --- ///

/// --- ELMDuino конфигурация --- ///
ELM327 myELM327;

typedef enum { ENG_RPM,
               ENG_TEMP,
               INTAKE_TEMP,
               SPEED,
               LONG,
               VOLT,
               SHORT,
               MAF,
               END_ITERATION } obd_pid_states;
obd_pid_states obd_state = ENG_RPM;
/// --- ELMDuino конфигурация --- ///

/// --- Data --- ///
unsigned long timeNew, timeOld, timeOldLog;
int engTemp, intakeTemp, rpm;
int32_t speed;
float outsideTemp;
float   odometrCurrent, voltage, LPH,  LP100, LP100z, LP100l, fuelAdd, odometr, fuelSent, odometrAdd, odometrAddLog, fuelAddLog, fuelCurrent;
double timer, longTerm, shortTerm, maf, FuelFlowGramsPerSecond, FuelFlowLitersPerSecond, correctFuel;
/// --- Data --- ///

/// --- Константы для формул --- ///
const float engineOnRpm = 400;
const float AirFuelRatio = 14.70;
const float FuelDensityGramsPerLiter = 750.0;
/// --- Константы для формул --- ///

/// --- Конфиг дисплея --- ///
#define RES_PIN 19
U8G2_SSD1309_128X64_NONAME0_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
/// --- Конфиг дисплея --- ///

/// --- Переменные кнопки --- ///
#define BUTTON_PIN 33
#define EB_HOLD_TIME 600
#define EB_NO_FOR
#define EB_NO_CALLBACK
#define EB_NO_COUNTER
#define EB_NO_BUFFER
Button btn(BUTTON_PIN);
int pressCount = 1;  // Счетчик нажатий
/// --- Переменные кнопки --- ///

void setupSerialAndDisplay();
void setupButton();

void connectToELM327();
void printStartInfo();
void displayIP();
void ObdWork();
void showMessageByPressCount();
void getOBDParams();
void updateDB();
void resetOdometerFuel();
void hardwareReset();
void displayTempVoltage();

void build(sets::Builder& b) {
    {
        sets::Group g(b, "Константы");
        b.Input(kk::fuelRatio, "Корекция топлива");
        b.Input(kk::timeCorrect, "Корекция времени");
    }
    {
        sets::Group g(b, "Сохранённая информация");
        b.Input(kk::odometrDB, "Весь одометр");
        b.Input(kk::fuelDB, "Всё топливо");
        b.Input(kk::odometr100DB, "Одометр последние 100");
        b.Input(kk::fuel100DB, "Топливо последние 100");
        b.Input(kk::last100lper100km, "Последний расход на 100");
    }
    {
        sets::Group g(b, "Параметры");
        b.Label("lbl1"_h, "Обороты");
        b.Label("lbl2"_h, "Температура двигателя");
        b.Label("lbl3"_h, "Скорость");
        b.Label("lbl4"_h, "Краткосрочная коррекция");
        b.Label("lbl5"_h, "Долгосрочная коррекция");
        b.Label("lbl6"_h, "Расход воздуха");
        b.Label("lbl7"_h, "Напряжение ELM");
        b.Label("lbl8"_h, "Температура впуска");
        b.Label("lbl9"_h, "Литры в час");
        b.Label("lbl10"_h, "Литры на 100");
    }
    {
        sets::Group g(b, "WI-FI настройки");
        if (b.Switch(kk::wifiMode, "Режим Точка доступа/Подключение к сети")) b.reload();
        if (db[kk::wifiMode]) {
            b.Input(kk::wifiSSID, "Имя WI-FI сети");
            b.Input(kk::wifiPass, "Пароль");
        } else {
            b.Input(kk::wifiSSIDAP, "Название точки доступа");
            b.Input(kk::wifiPassAP, "Пароль точки доступа");
        }
        if (b.Button("Перезагрузить ESP")) {
            db.update();
            ESP.restart();
        }
    }
}
void update(sets::Updater& upd) {
    upd.update("lbl1"_h, rpm);
    upd.update("lbl2"_h, engTemp);
    upd.update("lbl3"_h, speed);
    upd.update("lbl4"_h, longTerm);
    upd.update("lbl5"_h, shortTerm);
    upd.update("lbl6"_h, maf);
    upd.update("lbl7"_h, voltage);
    upd.update("lbl8"_h, intakeTemp);
    upd.update("lbl9"_h, LPH);
    upd.update("lbl10"_h, LP100);
}
void setup() {
    setupSerialAndDisplay();
    // ======== DATABASE ========
    LittleFS.begin(true);
    db.begin();
    db.init(kk::odometrDB, 0);
    db.init(kk::fuelDB, 0);
    db.init(kk::odometr100DB, 0);
    db.init(kk::fuel100DB, 0);
    db.init(kk::last100lper100km, 0);
    db.init(kk::timeCorrect, 1);
    db.init(kk::fuelRatio, 1);
    db.init(kk::wifiMode, false);
    db.init(kk::wifiSSID, "");
    db.init(kk::wifiPass, "");
    db.init(kk::wifiSSIDAP, "BK");
    db.init(kk::wifiPassAP, "12345678");
    // ======== DATABASE ========

    // ======== WIFI ========
    if (db[kk::wifiMode]) {
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        uint8_t tries = 20;
        while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
            if (!--tries) break;
        }
        if (WiFi.status() != WL_CONNECTED) {
            db[kk::wifiMode] = false;
            db.update();
            ESP.restart();
        }
    } else {
        WiFi.mode(WIFI_AP_STA);                               // Устанавливаем режим точки доступа
        WiFi.softAP(db[kk::wifiSSIDAP], db[kk::wifiPassAP]);  // Устанавливаем SSID и пароль точки доступа
        Serial.println("Access Point Started");
        Serial.print("IP Address: ");
        Serial.println(WiFi.softAPIP());  // Выводим IP-адрес точки доступа
    }
    // ======== WIFI ========

    // ======== SETTINGS ========
    sett.begin();
    sett.onBuild(build);
    sett.onUpdate(update);
    // ======== SETTINGS ========

    setupButton();
    ELM_PORT.begin("Bort", true);
    ELM_PORT.setPin("1234");

    connectToELM327();
    
    sensors.begin();
}

void loop() {
    sett.tick();
    btn.tick();
    if (btn.holding()) {  // если кнопка удерживается
        displayIP();      // выводим пока удерживается
    } else {
        showMessageByPressCount();
    }
    if (btn.click()) {
        pressCount++;
        if (pressCount > 7) pressCount = 1;
    }
    ObdWork();

    unsigned long outsideReqTime = millis();
    if (outsideReqTime - lastRequestTime >= requestInterval) {
    lastRequestTime = outsideReqTime;  // Обновляем время последнего запроса
    
    sensors.requestTemperatures();  // Запрос температуры с датчика
    outsideTemp = sensors.getTempCByIndex(0);  // Чтение температуры в градусах Цельсия
    Serial.print("Температура: ");
    Serial.print(outsideTemp);
    Serial.println(" °C");
  }

}
void calculateTime() {
    float timeCorrect = db[kk::timeCorrect];
    if (timeOld == 0) {
        timeOld = millis();
    }
    timeNew = millis();

    timer = (double(timeNew - timeOld) / 1000.0) * timeCorrect;
    if (timer > 10) {
        timer = 0;
    }
    timeOld = timeNew;
}

void ObdWork() {
    getOBDParams();  // Получение параметров OBD
    float fd = db[kk::fuelDB];
    float od = db[kk::odometrDB];
    LP100z = (db[kk::odometrDB] > 0) ? (fd / od) : 0;

    if (rpm > engineOnRpm) {
        // Проверка значений для корректного расчета
        if (maf > 0 && AirFuelRatio > 0 && FuelDensityGramsPerLiter > 0) {
            float fR = db[kk::fuelRatio];
            correctFuel = ((100.0 + (longTerm + shortTerm)) / 100.0) * fR;

            // Сглаживание MAF (массового расхода воздуха)
            static float mafSmoothed = 0;
            mafSmoothed = (mafSmoothed * 0.9) + (maf * 0.1);

            FuelFlowGramsPerSecond = (mafSmoothed / AirFuelRatio) * correctFuel;
            FuelFlowLitersPerSecond = FuelFlowGramsPerSecond / FuelDensityGramsPerLiter;
            LPH = FuelFlowLitersPerSecond * 3600.0;
        } else {
            // Установка нулевого расхода при некорректных данных
            FuelFlowLitersPerSecond = 0.0;
            LPH = 0.0;
        }

        calculateTime();

        // Обновление одометра
        if (speed > 0) {
            odometrAdd = (speed * 1000.0 / 3600.0) * timer / 1000.0;
            odometr += odometrAdd;
            odometrCurrent += odometrAdd;
        } else {
            odometrAdd = 0;
        }

        // Обновление потраченного топлива
        fuelAdd = FuelFlowLitersPerSecond * timer;
        fuelSent += fuelAdd;
        fuelCurrent += fuelAdd;

        // Обновление DB по условиям
        if ((speed > 1 && speed < 10 && (timeNew - timeOldLog) > 10000) || (speed == 0 && (timeNew - timeOldLog) > 10000)) {
            updateDB();

            odometrAddLog = 0;
            fuelAddLog = 0;

            timeOldLog = timeNew;
        } else {
            odometrAddLog += odometrAdd;
            fuelAddLog += fuelAdd;
        }

        // Сброс данных после достижения 100 км
        if (db[kk::odometr100DB] > 100) {
            resetOdometerFuel();
        }
    }

    // Расчет расхода топлива на 100 км
    if (odometr > 0) {
        LP100 = (fuelSent / odometr) * 100.0;
    } else {
        LP100 = 0.0;
    }
}

void getOBDParams() {
    switch (obd_state) {
        case ENG_RPM: {
            float tempRpm = myELM327.rpm();

            if (myELM327.nb_rx_state == ELM_SUCCESS) {
                rpm = tempRpm;
                // Serial.print("rpm: ");
                // Serial.println(rpm);
                obd_state = SPEED;
            } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
                myELM327.printError();
                obd_state = SPEED;
            }

            break;
        }

        case SPEED: {
            float tempSpeed = myELM327.kph();

            if (myELM327.nb_rx_state == ELM_SUCCESS) {
                speed = tempSpeed;
                // Serial.print("speed: ");
                // Serial.println(speed);
                obd_state = ENG_TEMP;
            } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
                myELM327.printError();
                obd_state = ENG_TEMP;
            }

            break;
        }

        case ENG_TEMP: {
            float tempEngTemp = myELM327.engineCoolantTemp();

            if (myELM327.nb_rx_state == ELM_SUCCESS) {
                engTemp = tempEngTemp;
                obd_state = VOLT;
            } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
                myELM327.printError();

                obd_state = VOLT;
            }

            break;
        }

        case VOLT: {
            float tempVoltage = myELM327.batteryVoltage();

            if (myELM327.nb_rx_state == ELM_SUCCESS) {
                voltage = tempVoltage;
                // Serial.print("voltage: ");
                // Serial.println(voltage);

                obd_state = LONG;
            } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
                myELM327.printError();
                obd_state = LONG;
            }

            break;
        }

        case LONG: {
            float tempLongTerm = myELM327.longTermFuelTrimBank_1();

            if (myELM327.nb_rx_state == ELM_SUCCESS) {
                longTerm = tempLongTerm;
                //  Serial.print("longTerm: ");
                //  Serial.println(longTerm);

                obd_state = INTAKE_TEMP;
            } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
                myELM327.printError();

                obd_state = INTAKE_TEMP;
            }

            break;
        }

        case INTAKE_TEMP: {
            float tempIntakeTemp = myELM327.longTermFuelTrimBank_1();

            if (myELM327.nb_rx_state == ELM_SUCCESS) {
                intakeTemp = tempIntakeTemp;
                // Serial.print("intakeTemp: ");
                // Serial.println(intakeTemp);

                obd_state = SHORT;
            } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
                myELM327.printError();
                obd_state = SHORT;
            }

            break;
        }

        case SHORT: {
            float tempShortTerm = myELM327.shortTermFuelTrimBank_1();

            if (myELM327.nb_rx_state == ELM_SUCCESS) {
                shortTerm = tempShortTerm;
                // Serial.print("shortTerm: ");
                // Serial.println(shortTerm);
                obd_state = MAF;
            } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
                myELM327.printError();
                obd_state = MAF;
            }

            break;
        }

        case MAF: {
            float tempMaf = myELM327.mafRate();

            if (myELM327.nb_rx_state == ELM_SUCCESS) {
                maf = tempMaf;
                // Serial.print("maf: ");
                // Serial.println(maf);
                obd_state = ENG_RPM;
            } else if (myELM327.nb_rx_state != ELM_GETTING_MSG) {
                myELM327.printError();
                obd_state = ENG_RPM;
            }

            break;
        }
    }
}

void setupButton() {
    btn.setBtnLevel(HIGH);
    btn.setTimeout(600);
}

void setupSerialAndDisplay() {
    Serial.begin(115200);
    hardwareReset();
    delay(100);
    u8g2.begin();
    u8g2.enableUTF8Print();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);
    u8g2.sendBuffer();
}

void connectToELM327() {
    Serial.println("Connecting to ELM...");
    u8g2.clearBuffer();
    u8g2.drawStr(30, 26, "Connect");
    u8g2.drawStr(30, 37, "to ELM");
    u8g2.sendBuffer();

    if (!ELM_PORT.connect((uint8_t*)address)) {
        Serial.println("Couldn't connect to OBD scanner - Phase 1");
        u8g2.clearBuffer();
        u8g2.drawStr(30, 28, "Fail to");
        u8g2.drawStr(30, 36, "connect ELM");
        u8g2.sendBuffer();
        delay(5000);
        ESP.restart();
    }
    if (!myELM327.begin(ELM_PORT, false, 2000)) {
        u8g2.clearBuffer();
        u8g2.drawStr(30, 28, "Fail to");
        u8g2.drawStr(30, 36, "connect ECU");
        u8g2.sendBuffer();
        while (1);
    }
}

void hardwareReset() {
    pinMode(RES_PIN, OUTPUT);
    digitalWrite(RES_PIN, LOW);   // Установить низкий уровень
    delay(10);                    // Подождать 10 мс
    digitalWrite(RES_PIN, HIGH);  // Установить высокий уровень
}
void printDataOnDisplay(String header, String fLabel, String fData, String sLabel = "", String sData = "") {
    u8g2.clearBuffer();
    u8g2.setCursor(24, 19);
    u8g2.setFont(u8g2_font_5x8_t_cyrillic);
    u8g2.print(header);
    u8g2.setFont(u8g2_font_haxrcorp4089_t_cyrillic);

    u8g2.setCursor(24, 28);
    u8g2.print(fLabel);
    u8g2.setCursor(78, 28);
    u8g2.print(fData);

    // Проверка на наличие второго параметра
    if (sLabel != "") {
        u8g2.setCursor(24, 38);
        u8g2.print(sLabel);
        u8g2.setCursor(78, 38);
        u8g2.print(sData);
    }

    u8g2.sendBuffer();
}
void showMessageByPressCount() {
    if (pressCount == 1) {
        displayTempVoltage();  // Уникальный экран обрабатывается отдельно
        return;
    }

    struct DisplayData {
        String header;
        String fLabel;
        String fData;
        String sLabel;
        String sData;
    };

    DisplayData messages[] = {
        {"Моментально:", "Л. в час:", String(LPH), "Л. на 100:", String(LP100)},
        {"За поездку:", "Проехал:", String(odometrCurrent), "Топливо:", String(fuelSent)},
        {"Литры на 100:", "В общем:", String(LP100z), "Последние:", db[kk::fuelDB]},
        {"За всё время:", "Проехал:", db[kk::odometrDB], "Топливо:", db[kk::odometrDB]},
        {"Прочее", "Т. снаружи:", String(outsideTemp), "Т. впуск:", String(intakeTemp)}};

    int index = pressCount - 2;  // Сдвиг на 2, так как pressCount=1 — это displayTempVoltage
    if (index >= 0 && index < (sizeof(messages) / sizeof(messages[0]))) {
        DisplayData msg = messages[index];
        printDataOnDisplay(msg.header, msg.fLabel, msg.fData, msg.sLabel, msg.sData);
    }
}
void printStartInfo() {
    if (db[kk::wifiMode]) {
        displayIP();
    } else {
        u8g2.clearBuffer();
        u8g2.setCursor(24, 19);
        u8g2.setFont(u8g2_font_5x8_t_cyrillic);
        u8g2.print("WIFI:");
        u8g2.setCursor(24, 27);
        String ssid = db[kk::wifiSSIDAP];
        u8g2.print("SSID " + ssid);
        u8g2.setCursor(24, 35);
        String pass = db[kk::wifiPassAP];
        u8g2.print("Pass " + pass);
        u8g2.setCursor(24, 43);
        u8g2.print(WiFi.softAPIP());
        u8g2.sendBuffer();
    }
}
void displayIP() {
    u8g2.clearBuffer();
    u8g2.setCursor(24, 23);
    if (db[kk::wifiMode]) {
        u8g2.print(WiFi.localIP());

    } else {
        printStartInfo();
    }
    u8g2.sendBuffer();
}

void displayTempVoltage() {
    u8g2.clearBuffer();
    u8g2.setCursor(24, 23);
    u8g2.print("Температура:");
    u8g2.setCursor(88, 23);
    u8g2.print(String(engTemp) + "с");
    u8g2.setCursor(24, 37);
    u8g2.print("Напряжение:");
    u8g2.setCursor(88, 37);
    u8g2.print(String(voltage));
    u8g2.sendBuffer();
}

void updateDB() {
    float odb = db[kk::odometrDB];
    float odbNew = odb + odometrAddLog;

    db[kk::odometrDB] = odbNew;

    float fdb = db[kk::fuelDB];
    float fdbNew = fdb + fuelAddLog;

    db[kk::fuelDB] = fdbNew;

    float odb100 = db[kk::odometr100DB];

    float odometr100DBNew = odb100 + odometrAddLog;

    db[kk::odometr100DB] = odometr100DBNew;
    float fdb100 = db[kk::fuel100DB];
    float fdb100New = fdb100 + fuelAddLog;

    db[kk::fuel100DB] = fdb100New;
}

void resetOdometerFuel() {
    // Подсчет нового значения расхода за последние 100км
    float fdb100 = db[kk::fuel100DB];
    float odb100 = db[kk::odometr100DB];
    float flast100lper100km = (fdb100 / odb100) * 100;
    db[kk::last100lper100km] = flast100lper100km;
    // Сброс значений одометра и топлива на 0
    db[kk::odometr100DB];
    db[kk::fuel100DB];
}
