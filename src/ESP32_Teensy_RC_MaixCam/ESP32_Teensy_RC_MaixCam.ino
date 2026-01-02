#include <EEPROM.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <AlfredoCRSF.h> 
#include <HardwareSerial.h>

#define CRSF_BAUDRATE 420000
#define EEP_Ident 0x5400
#define HYDRAULIC_ENABLED true
#define HYDRAULIC_DEBUG false

// Настройки Wi-Fi
const char* ssid = "AOG4";
const char* password = "12345678";
// UDP
WiFiUDP udp;
unsigned int localPort = 8888;  // Порт для приема данных от MaixCam

// Переменные для данных о препятствиях
bool obstacleDetected = false;
int obstacleCount = 0;
float steeringAngle = 0.0;
unsigned long lastObstacleTime = 0;
float actualSteerAngle = 0.0;  //  Используем actualSteerAngle из PGN 253
unsigned long lastAngleSend = 0;
const unsigned long angleSendInterval = 100; // Отправка каждые 100мс

// IP адрес MaixCam
IPAddress maixcamIP(192, 168, 4, 2); // Типичный IP для клиента в сети ESP32 AP
unsigned int maixcamPort = 8889;

HardwareSerial SerialTeensy(2); // Используем Serial2
byte SerialTeensyRX = 16;  // Пин RX ESP 16 
byte SerialTeensyTX = 17;  // Пин TX ESP 17
AlfredoCRSF Teensy;

//HardwareSerial SerialOut(0); // Используем Serial
//byte PIN_RX_OUT = 3;  // Пин RX ESP 3 
//byte PIN_TX_OUT = 1;  // Пин TX ESP 1
//AlfredoCRSF Out;

HardwareSerial SerialCRSF(1); // 1 - это UART1 на ESP32
byte CRSF_RX = 15;  // Пин RX для CRSF (15)
byte CRSF_TX = 4;   // Пин TX для CRSF (4)
AlfredoCRSF crsf; 

int channel2Value = 0; // Глобальная переменная для хранения значения канала 2 мощность
int channel4Value = 0; // Глобальная переменная для хранения значения канала 4 сцепление
int channel6Value = 0; // Глобальная переменная для хранения значения канала 6 геозона
int channel5Value = 0; // Глобальная переменная для хранения значения канала 5 подьем
int channel8Value = 0; // Глобальная переменная для хранения значения канала 8 опускание
int channel7Value = 0; // Глобальная переменная для хранения значения канала 7  сцепление
int channel9Value = 0; // Инициализация переменной включения руля фпв управление

struct Config {
        uint8_t raiseTime = 2;
        uint8_t lowerTime = 4;
        uint8_t enableToolLift = 0;
        uint8_t isRelayActiveHigh = 0; //if zero, active low (default)

        uint8_t user1 = 0; //user defined values set in machine tab
        uint8_t user2 = 0;
        uint8_t user3 = 0;
        uint8_t user4 = 0;

    };   Config hydConfig;   //8 bytes

int16_t temp, EEread = 0; // temp - временная переменная, EEread - значение из EEPROM

const uint8_t LOOP_TIME = 200; //5hz
uint32_t lastTime = LOOP_TIME;
uint32_t currentTime = LOOP_TIME;

//Switches
uint8_t  workSwitch = 0, steerSwitch = 1, switchByte = 0;

// Переменные для проверки связи
uint8_t watchdogTimer = 0; // Таймер для проверки связи с AOG
uint8_t serialResetTimer = 0; // Таймер для сброса буфера последовательного порта, если он переполнен

// Переменные для парсинга PGN
bool isPGNFound = false; // Флаг, указывающий, найден ли PGN
bool isHeaderFound = false; // Флаг, указывающий, найден ли заголовок
uint8_t pgn = 0; // Идентификатор PGN
uint8_t dataLength = 0; // Длина данных
uint8_t idx = 0; // Индекс для перебора данных
uint8_t byte2 = 0; //  ДОБАВИТЬ эту переменную
int16_t tempHeader = 0; // Временная переменная для хранения заголовка

//24 possible pins assigned to these functions
uint8_t pin[] = { 1,2,3,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
 
//The variables used for storage
uint8_t relayHi = 0;
uint8_t relayLo = 0;
uint8_t tram = 0; // переменная XTE
uint8_t tramline = 0; // Переменная для хранения состояния траектории
uint8_t uTurn = 0; // Переменная для хранения состояния разворота
uint8_t hydLift = 0; // Переменная для хранения состояния гидравлического подъема
uint8_t geoStop = 0; // Переменная для хранения состояния геозоны
float gpsSpeed; // Переменная для хранения скорости GPS
float recordedSpeed; // Переменная для хранения скорости в записанном пути
uint8_t hydLiftPrev = 3;
uint8_t uTurnPrev = 1; // Переменная для хранения предыдущего состояния uTurn
uint8_t geoStopPrev = 1; // Переменная для хранения предыдущего состояния uTurn
uint16_t channel7ValuePrev = 1600; // Переменная для хранения предыдущего состояния сцепления                                   
uint32_t raiseTimer = 0; // Таймер для подъема
uint32_t lowerTimer = 0; // Таймер для опускания
uint32_t powerupTimer = 0; // Таймер для мощности+
uint32_t powerdownTimer = 0; // Таймер для мощности-
uint32_t parkupTimer = 0; // Таймер для сцепления+
uint32_t parkdownTimer = 0; // Таймер для сцепления-
uint32_t StopupTimer = 0; // Таймер для стоп+
uint32_t StopdownTimer = 0; // Таймер для стоп-
int pidPowerValue = 0; // Глобальная переменная для значения ПИД-регулятора
    
// Определяем пины для управления
const int HYDRAULIC_GEOSTOP_UP =23 ; // Пин для гео-стопа +
const int HYDRAULIC_GEOSTOP_DOWN =22 ; // Пин для гео-стопа -
const int HYDRAULIC_POWER_UP = 32;   // Пин управления мощьностью +
const int HYDRAULIC_POWER_DOWN = 33;   // Пин управления мощьностью -
const int HYDRAULIC_LIFT_OR_UP = 25; // подьем
const int HYDRAULIC_LOWER_OR_DOWN = 26; // вниз
const int HYDRAULIC_PARK_UP = 21; // сцепление +
const int HYDRAULIC_PARK_DOWN = 19; // сцепление -

// Определяем пины для кнопок 
const int GAS_UP_PIN = 35;    // Газ +  поддянуть 10 ком к 3,3в
const int GAS_DOWN_PIN = 34;  // Газ -  поддянуть 10 ком к 3,3в
const int STOP_DOWN_PIN = 18; // Стоп вниз

//reset function
void(* resetFunc) (void) = 0;
  

void setup() {
     delay(250); // время для стабилизации питания
     Serial.begin(115200);
     // Создаем точку доступа
     WiFi.softAP(ssid, password);    
     IPAddress IP = WiFi.softAPIP();
     Serial.print("Точка доступа создана: ");
     Serial.println(ssid);
     Serial.print("IP адрес ESP32: ");
     Serial.println(IP);
     Serial.print("MAC адрес: ");
     Serial.println(WiFi.softAPmacAddress());    
     // Запускаем UDP сервер
     udp.begin(localPort);
     Serial.println("UDP сервер запущен на порту 8888"); 

     EEPROM.begin(512); // Инициализация EEPROM с размером 512 байт
     EEPROM.get(0, EEread); // read identifier
     if (EEread != EEP_Ident) { // check on first start and write EEPROM
         EEPROM.put(0, EEP_Ident);
         EEPROM.put(6, hydConfig);                    
     } else {     
             EEPROM.get(6, hydConfig);          
     }  
  
     // Настройка пинов как выходные
     pinMode(HYDRAULIC_GEOSTOP_UP, OUTPUT);
     pinMode(HYDRAULIC_GEOSTOP_DOWN, OUTPUT);
     pinMode(HYDRAULIC_POWER_UP, OUTPUT);
     pinMode(HYDRAULIC_POWER_DOWN, OUTPUT);
     pinMode(HYDRAULIC_LIFT_OR_UP, OUTPUT);
     pinMode(HYDRAULIC_LOWER_OR_DOWN, OUTPUT);  

     // пины кнопок с подтяжкой
     pinMode(GAS_UP_PIN, INPUT); // поддянуть 10 ком к 3,3в
     pinMode(GAS_DOWN_PIN, INPUT); // поддянуть 10 ком к 3,3в
     pinMode(STOP_DOWN_PIN, INPUT); //развязка оптопарой  масса-кнопка-катод-анод-220ом-5,5в; эмитер-масса; пин-коллектор:4,7ком-3,3в

     digitalWrite(HYDRAULIC_LIFT_OR_UP, hydConfig.isRelayActiveHigh);
     digitalWrite(HYDRAULIC_LOWER_OR_DOWN, hydConfig.isRelayActiveHigh);    
     digitalWrite(HYDRAULIC_GEOSTOP_UP, hydConfig.isRelayActiveHigh); // пин геозоны +
     digitalWrite(HYDRAULIC_GEOSTOP_DOWN, hydConfig.isRelayActiveHigh); // пин геозоны -
     digitalWrite(HYDRAULIC_POWER_UP, hydConfig.isRelayActiveHigh); // мощность
     digitalWrite(HYDRAULIC_POWER_DOWN, hydConfig.isRelayActiveHigh); // мощность  
  
     SerialCRSF.begin(CRSF_BAUDRATE, SERIAL_8N1, CRSF_RX, CRSF_TX);
     delay(1000); 
     SerialTeensy.begin(CRSF_BAUDRATE, SERIAL_8N1, SerialTeensyRX, SerialTeensyTX);// 115200
     //SerialOut.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX_OUT, PIN_TX_OUT);
     delay(1000);
     crsf.begin(SerialCRSF);
     Teensy.begin(SerialTeensy); // Инициализация последовательного порта CRSF
     //Out.begin(SerialOut);

     // Вывод значений из структуры hydConfig в монитор последовательного порта
     Serial.println("Values from EEPROM:");
     Serial.print("raiseTime: ");
     Serial.println(hydConfig.raiseTime);
     Serial.print("lowerTime: ");
     Serial.println(hydConfig.lowerTime);
     Serial.print("enableToolLift: ");
     Serial.println(hydConfig.enableToolLift);
     Serial.print("isRelayActiveHigh: ");
     Serial.println(hydConfig.isRelayActiveHigh);
     Serial.print("user1: ");
     Serial.println(hydConfig.user1);
     Serial.print("user2: ");
     Serial.println(hydConfig.user2);
     Serial.print("user3: ");
     Serial.println(hydConfig.user3);
     Serial.print("user4: ");
     Serial.println(hydConfig.user4);   
 }

 void loop() {

     static uint32_t lastGeoStopUpdate = millis(); // Инициализируем текущим временем
     currentTime = millis();
     if (currentTime - lastTime >= LOOP_TIME)    {
         lastTime = currentTime;    
         // Проверяем время с последнего обновления геостопа
         if (currentTime - lastGeoStopUpdate > 2000) { // 2 секунды
             geoStop = 1; // Аварийный стоп - нет данных от AgOpenGPS       
         }        
         if (serialResetTimer++ > 20) {
         while (SerialTeensy.available() > 0) SerialTeensy.read();
         serialResetTimer = 0;
         }
     }
     /*currentTime = millis();
     if (currentTime - lastTime >= LOOP_TIME) {      
         lastTime = currentTime;
         //If connection lost to AgOpenGPS, the watchdog will count up 
         if (watchdogTimer++ > 250) watchdogTimer = 12;           
           //clean out serial buffer to prevent buffer overflow
             if (serialResetTimer++ > 20) {          
                 while (SerialTeensy.available() > 0) SerialTeensy.read();
                 serialResetTimer = 0;
             }
             if (watchdogTimer > 10) geoStop = true; //значение в еденицах времени до срабатывания реле,  если связи с agopengps нет, то стоп
               //Serial.print("таймер: ");
               //Serial.println(geoStop);           
     }*/  

     if (SerialTeensy.available() > 4 && !isHeaderFound && !isPGNFound) {
         uint8_t temp = SerialTeensy.read();
         if (tempHeader == 0x80 && temp == 0x81) {
             isHeaderFound = true;
             tempHeader = 0;
             } else {
                 tempHeader = temp; // Сохраняем для следующей итерации
                 return;
                 }
         }
         if (SerialTeensy.available() > 2 && isHeaderFound && !isPGNFound) {
             byte2 = SerialTeensy.read(); // Byte 2 - Source (127 для PGN 238/239, 126 для PGN 253)    
             pgn = SerialTeensy.read();
             dataLength = SerialTeensy.read();
             isPGNFound = true;
             idx = 0;
             }
             if (SerialTeensy.available() >= dataLength && isHeaderFound && isPGNFound) {
                 if (pgn == 239 && byte2 == 127) { // Обработка PGN 239
                 uTurn = SerialTeensy.read();
                 //Serial.print("uTurn: ");
                 //Serial.println(uTurn);
                 gpsSpeed = (float)SerialTeensy.read();            
                 lastGeoStopUpdate = millis();// ОБНОВЛЯЕМ время последнего получения
                 //Serial.print(" gpsSpeed: ");
                 //Serial.println( gpsSpeed);
                 hydLift = SerialTeensy.read();
                 //Serial.print("hydLift: ");
                 //Serial.println(hydLift);
                 tramline = SerialTeensy.read();
                 geoStop = SerialTeensy.read();
                 //Serial.print("geoStop: ");
                 //Serial.println(geoStop);
                 recordedSpeed = (float)SerialTeensy.read(); // записанный путь
                 //Serial.print(" recordedSpeed: ");
                 //Serial.println( recordedSpeed);           
                 relayLo = SerialTeensy.read();
                 relayHi = SerialTeensy.read();
                     if (hydConfig.isRelayActiveHigh) {
                     tramline = 255 - tramline;
                     relayLo = 255 - relayLo;
                     }     
                 SerialTeensy.read(); // Пропускаем CRC            
             } else if (pgn == 238 && byte2 == 127 && channel9Value <= 1500) { // Обработка PGN 238 только если channel9Value <= 1500
                 hydConfig.raiseTime = SerialTeensy.read();
                 hydConfig.lowerTime = SerialTeensy.read();
                 hydConfig.enableToolLift = SerialTeensy.read();
                 uint8_t sett = SerialTeensy.read();
                 hydConfig.isRelayActiveHigh = bitRead(sett, 0);
                 hydConfig.enableToolLift = bitRead(sett, 1);
                 hydConfig.user1 = SerialTeensy.read();
                 hydConfig.user2 = SerialTeensy.read();
                 hydConfig.user3 = SerialTeensy.read();
                 hydConfig.user4 = SerialTeensy.read();
                 SerialTeensy.read(); // Пропускаем CRC
                 EEPROM.put(6, hydConfig);
                 EEPROM.commit();
                 resetFunc(); // Перезагрузка

             } else if (pgn == 254 && byte2 == 127) { // ОБРАБОТКА PGN 254 (0xFE с byte2 = 127)        
                 SerialTeensy.read(); //пропускаем
                 SerialTeensy.read(); //пропускаем          
                 SerialTeensy.read(); //пропускаем
                 SerialTeensy.read(); //пропускаем
                 SerialTeensy.read(); //пропускаем
                 tram = SerialTeensy.read(); //читаем XTE
                 Serial.print(" tram: ");
                 Serial.println( tram);
                 SerialTeensy.read(); //пропускаем               
                 SerialTeensy.read(); //пропускаем        
                 SerialTeensy.read(); // Пропускаем CRC (байт 13)

             } else if (pgn == 0xFD && byte2 == 126) { // ОБРАБОТКА PGN 253 (0xFD с byte2 = 126)
                 // PGN 253 - From AutoSteer (8 байт данных)
                 // Структура: ActualSteerAngle*100 (2 байта), Heading (2 байта), Roll (2 байта), SwitchByte, pwmDisplay        
                 // Читаем ActualSteerAngle * 100 (байты 5-6)
                 uint8_t angleLow = SerialTeensy.read();
                 uint8_t angleHigh = SerialTeensy.read();
                 int16_t steerAngleRaw = (angleHigh << 8) | angleLow;
                 actualSteerAngle = steerAngleRaw * 0.01f; // Преобразуем в градусы        
                 SerialTeensy.read(); //пропускаем
                 SerialTeensy.read(); //пропускаем
                 SerialTeensy.read(); //пропускаем
                 SerialTeensy.read(); //пропускаем
                 switchByte = SerialTeensy.read(); //значение кнопок автопилота
                 // Распаковка битов 
                 workSwitch = switchByte & 1; // Бит 0
                 steerSwitch = (switchByte >> 1) & 1; // Бит 1               
                 SerialTeensy.read(); //пропускаем        
                 SerialTeensy.read(); // Пропускаем CRC (байт 13)              
                 //Serial.print(actualSteerAngle);
                 //Serial.println();
                 }    
             watchdogTimer = 0; // Сброс watchdog
             serialResetTimer = 0; // Сброс таймера
             // Сброс состояния для следующего пакета
             isHeaderFound = isPGNFound = false;
             pgn = dataLength = byte2 = 0;
             }
     // Прием UDP пакетов от MaixCam
     int packetSize = udp.parsePacket();
     if (packetSize) {
         char packetBuffer[255];
         int len = udp.read(packetBuffer, 255);
         if (len > 0) {
             packetBuffer[len] = 0;
             processObstacleData(packetBuffer);
             }
         }    
     // Отправка actualSteerAngle на MaixCam
     if (millis() - lastAngleSend > angleSendInterval) {
         sendSteeringAngle();
         lastAngleSend = millis();
         }       
     channel2Value = crsf.getChannel(2); // мощность
     channel4Value = crsf.getChannel(4); // сцепление
     channel6Value = crsf.getChannel(6); // геостоп
     channel5Value = crsf.getChannel(5); // обновление значения на канале 5 вверх
     channel7Value = crsf.getChannel(7); // сцепление фикс
     channel8Value = crsf.getChannel(8); // обновление значения на канале 8 вниз    
     channel9Value = crsf.getChannel(9); // обновление значения на канале 9 фпв управление             
     // Обработка ВСЕХ функций управления
     handleButtons();                   // Кнопки + мощность
     handlePowerActuatorBySpeedPID();   // ПИД регулятор
     handleUTurn();                     // Разворот
     handlePark();                      // Сцепление
     handleStop();                      // Стоп
     handleHydraulicRC();               // RC управление
     handleHydraulic();                 // Гидравлика (AOG команды)
     hydraulicTimedPins();              // Таймеры гидравлики
     crsf.update(); // Обновление данных CRSF
     //Check if RX1 is connected
     if (crsf.isLinkUp() && channel9Value > 1500) {      
         sendChannels(crsf);
         }/* else {
                 // No RX connected, send fallback channels
                 sendFallbackChannels();       
                 }*/
     }

 void setGasPin(int pin, bool state) { // Для пинов, управляемых через ШИМ (газ)
     if (state) {
         analogWrite(pin, 255); 
         } else {
             analogWrite(pin, 0);   
             }
     }

 void hydraulicTimedPins() {
     if (raiseTimer  && millis() > raiseTimer )  raiseTimer  = triggerPin(HYDRAULIC_LIFT_OR_UP, hydConfig.isRelayActiveHigh, 0);
     if (lowerTimer  && millis() > lowerTimer )  lowerTimer  = triggerPin(HYDRAULIC_LOWER_OR_DOWN, hydConfig.isRelayActiveHigh, 0);
     }

 int triggerPin(int pin, bool state, int timer) {    
     digitalWrite(pin, state);
     if(timer) return millis() + (1000 * timer);
     return 0;
     }    

 void handleHydraulic() {  // функция управления  гидравликой
     if(hydLift == 0 ) {
        triggerPin(HYDRAULIC_LIFT_OR_UP, hydConfig.isRelayActiveHigh ,0);
        triggerPin(HYDRAULIC_LOWER_OR_DOWN, hydConfig.isRelayActiveHigh ,0);    
        hydLiftPrev = 0;
        return; //Disabled
     } 
     if (hydLift == hydLiftPrev) return; //nothing changed nothing to do
         hydLiftPrev = hydLift;
         if (hydLift == 2 && hydConfig.enableToolLift) { //raise    
             if ( raiseTimer == 0 && lowerTimer == 0) { //now we care about timing      
                  raiseTimer = triggerPin(HYDRAULIC_LIFT_OR_UP, !hydConfig.isRelayActiveHigh, hydConfig.raiseTime);
                 }
         } else if(hydLift == 1 && hydConfig.enableToolLift ) { //lower    
                if ( raiseTimer == 0 && lowerTimer == 0) { //now we care about timing    
                     lowerTimer = triggerPin(HYDRAULIC_LOWER_OR_DOWN, !hydConfig.isRelayActiveHigh, hydConfig.lowerTime);
                     }
                 } 
     }

 void handleHydraulicRC() {    
     if (raiseTimer == 0 && lowerTimer == 0) {
         if (channel5Value > 1500) {
             triggerPin(HYDRAULIC_LIFT_OR_UP, !hydConfig.isRelayActiveHigh, 0);
             } else {
             triggerPin(HYDRAULIC_LIFT_OR_UP, hydConfig.isRelayActiveHigh, 0);
             }
         }        
     if (lowerTimer == 0 && raiseTimer == 0) {
         if (channel8Value > 1500) {
             triggerPin(HYDRAULIC_LOWER_OR_DOWN, !hydConfig.isRelayActiveHigh, 0);
             } else {
                 triggerPin(HYDRAULIC_LOWER_OR_DOWN, hydConfig.isRelayActiveHigh, 0);
                 }
             }
     }

 void handleUTurn() {  // функция управления таймерами  мощности   
     if (powerupTimer && millis() >  powerupTimer) { // Обработка таймера POWER_UP
         setGasPin(HYDRAULIC_POWER_UP, false);
         powerupTimer = 0;
     }
     if (powerdownTimer && millis() > powerdownTimer) { // Обработка таймера POWER_DOWN
         setGasPin(HYDRAULIC_POWER_DOWN, false);
         powerdownTimer = 0;
     }
     if (uTurn == uTurnPrev) return; // если состояние не изменилось, выходим
         uTurnPrev = uTurn; // обновляем предыдущее значение
         if (uTurn == 1) { // если uTurn активирован
             setGasPin(HYDRAULIC_POWER_DOWN, true); // сбавить газ
             powerdownTimer = millis() + (hydConfig.user1 * 100); // Установить таймер отключения
             } else {
                 setGasPin(HYDRAULIC_POWER_UP, true); // Активировать газ
                 powerdownTimer = millis() + (hydConfig.user1 * 200); // Установить таймер отключения
             }    
 }
 void handlePark() {  // функция управления  сцеплением  
     if (channel4Value > 1600) {
         // Плавное выжимание сцепления (пропорционально значению channel4Value)
         int pwmValue = map(channel4Value, 1600, 2100, 0, 255); // Преобразуем значение в диапазон 0–255
         analogWrite(HYDRAULIC_PARK_UP, pwmValue); // Плавное выжимание
         analogWrite(HYDRAULIC_PARK_DOWN, 0); // Отключаем отпускание
         }
         else if (crsf.isLinkUp() && channel4Value < 1400) { // Проверяем, есть ли связь и значение канала меньше 1400 
             // Плавное отпускание сцепления (пропорционально значению channel4Value)
             int pwmValue = map(channel4Value, 1400, 1000, 0, 255); // Преобразуем значение в диапазон 0–255
             analogWrite(HYDRAULIC_PARK_DOWN, pwmValue); // Плавное отпускание
             analogWrite(HYDRAULIC_PARK_UP, 0); // Отключаем выжимание
             }
             else {
             // Оставляем сцепление в текущем состоянии
              analogWrite(HYDRAULIC_PARK_UP, 0);
              analogWrite(HYDRAULIC_PARK_DOWN, 0);
             } 
     }
 // Управление геостопом
 void handleStop() { // функция управления стопом  
     if (StopupTimer  && millis() > StopupTimer )  StopupTimer  = triggerPin(HYDRAULIC_GEOSTOP_UP, hydConfig.isRelayActiveHigh, 0);
     if (StopdownTimer  && millis() > StopdownTimer )  StopdownTimer  = triggerPin(HYDRAULIC_GEOSTOP_DOWN, hydConfig.isRelayActiveHigh, 0);

     if (StopdownTimer == 0 && StopupTimer == 0)
         if (channel6Value > 1500) { // проверяем    канал 6 
         powerdownTimer= triggerPin(HYDRAULIC_POWER_DOWN, !hydConfig.isRelayActiveHigh, 3); // Активировать газ              
         StopdownTimer= triggerPin(HYDRAULIC_GEOSTOP_DOWN, !hydConfig.isRelayActiveHigh, 3); // Активировать стоп        
         } 
  
     if (geoStop == geoStopPrev) return; // если состояние не изменилось, выходим
         geoStopPrev = geoStop; // обновляем предыдущее значение 

     if (geoStop == 1  && StopdownTimer == 0 && StopupTimer == 0) { // если геостоп активирован
         setGasPin(HYDRAULIC_POWER_DOWN, true); // сбавить газ
         powerdownTimer = millis() + (hydConfig.user1 * 200); // Установить таймер отключения
         StopdownTimer= triggerPin(HYDRAULIC_GEOSTOP_DOWN, !hydConfig.isRelayActiveHigh, 3); // Активировать стоп
         } else {
             if (StopdownTimer == 0) {
             StopupTimer= triggerPin(HYDRAULIC_GEOSTOP_UP, !hydConfig.isRelayActiveHigh, 3); // Деактивировать стоп
             }
         }     
     }
 void handleButtons() {  
     bool gasUpPressed = (digitalRead(GAS_UP_PIN) == LOW);
     bool gasDownPressed = (digitalRead(GAS_DOWN_PIN) == LOW);
     bool stopDownPressed = (digitalRead(STOP_DOWN_PIN) == LOW);
     static bool lastStopDownState = false; // Добавляем переменную для хранения предыдущего состояния    
     if (gasUpPressed || channel2Value > 1600) { 
         setGasPin(HYDRAULIC_POWER_UP, true);
         setGasPin(HYDRAULIC_POWER_DOWN, false);                                                        
         } else if (gasDownPressed || (crsf.isLinkUp() && channel2Value < 1400)) {
             setGasPin(HYDRAULIC_POWER_DOWN, true);
             setGasPin(HYDRAULIC_POWER_UP, false);      
             } else {        
                     if (!powerupTimer && !powerdownTimer && recordedSpeed < 1 ) {
                         setGasPin(HYDRAULIC_POWER_UP, false);
                         setGasPin(HYDRAULIC_POWER_DOWN, false);
                         } 
                     }      
     // Обработка стоп-кнопки
     if (stopDownPressed != lastStopDownState)  { // Проверяем изменение состояния
         lastStopDownState = stopDownPressed; // Обновляем состояние    
         if (stopDownPressed  && StopupTimer == 0) {
             StopdownTimer = triggerPin(HYDRAULIC_GEOSTOP_DOWN, !hydConfig.isRelayActiveHigh, 3); // Активировать стоп              
         } else if (!stopDownPressed  && StopdownTimer == 0) {               
                 StopupTimer = triggerPin(HYDRAULIC_GEOSTOP_UP, !hydConfig.isRelayActiveHigh, 3);
                 }
         }
     }
 void handlePowerActuatorBySpeedPID() {   
     static float integral = 0;  // Накопленная сумма ошибок    
     float targetSpeed = recordedSpeed;
     float currentSpeed = gpsSpeed * 0.1;

     if (targetSpeed < 1 || currentSpeed < 1) {
         return;   
         }   
     float speedError = targetSpeed - currentSpeed;    
     integral += speedError;    
     // Анти-windup (ограничение интеграла)
     float maxIntegral = 50.0;
     if (integral > maxIntegral) integral = maxIntegral;
     if (integral < -maxIntegral) integral = -maxIntegral;    
    
     float Kp = (hydConfig.user2 > 0) ? (hydConfig.user2 / 2 ) : 30.0;    
     float Ki = (hydConfig.user3 > 0) ? (hydConfig.user3 / 10.0) : 0.5;  
     float pValue = Kp * speedError;
     float iValue = Ki * integral;
     float output = pValue + iValue;
    
     int16_t pwmDrive = constrain((int16_t)output, -255, 255);    
    
     if (pwmDrive > 0) {        
         analogWrite(HYDRAULIC_POWER_DOWN, 0);
         analogWrite(HYDRAULIC_POWER_UP, pwmDrive);
         } 
         else if (pwmDrive < 0) {        
             int brakePower = -pwmDrive;
              analogWrite(HYDRAULIC_POWER_UP, 0);
              analogWrite(HYDRAULIC_POWER_DOWN, brakePower);
             }
             else {        
                 analogWrite(HYDRAULIC_POWER_UP, 0);
                 analogWrite(HYDRAULIC_POWER_DOWN, 0);
                 }
     }
 void processObstacleData(char* data) {
     // Формат: "OBSTACLE:1:COUNT:2:ANGLE:12.5"
     String message = String(data);
    
     if (message.startsWith("OBSTACLE:")) {
         int obstacleValue = 0;
         int countValue = 0;
         float angleValue = 0.0;
        
         sscanf(data, "OBSTACLE:%d:COUNT:%d:ANGLE:%f", &obstacleValue, &countValue, &angleValue);
        
         obstacleDetected = (obstacleValue == 1);
         obstacleCount = countValue;
         lastObstacleTime = millis();
         if (obstacleDetected && StopupTimer == 0) { 
             setGasPin(HYDRAULIC_POWER_DOWN, true); // сбавить газ
             powerdownTimer = millis() + (hydConfig.user1 * 200); // Установить таймер отключения          
             StopdownTimer = triggerPin(HYDRAULIC_GEOSTOP_DOWN, !hydConfig.isRelayActiveHigh, 3);        
             }
         }
     }
 void sendSteeringAngle() {
     // Формат: "ANGLE:12.5" - отправляем actualSteerAngle
     char angleBuffer[20];
     snprintf(angleBuffer, sizeof(angleBuffer), "ANGLE:%.1f", actualSteerAngle);    
     udp.beginPacket(maixcamIP, maixcamPort);
     udp.write((uint8_t*)angleBuffer, strlen(angleBuffer));
     udp.endPacket();   
     }
 // Method to send channels based on CRSF instance
 void sendChannels(AlfredoCRSF& crsf) {
     const crsf_channels_t* channels_ptr = crsf.getChannelsPacked();
     Teensy.writePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_RC_CHANNELS_PACKED, channels_ptr, sizeof(*channels_ptr));
     }

 