#include <SPI.h>

const uint8_t PIN_CC_GDO0{ 3 };
const uint8_t PIN_CC_GDO2{ 2 };

void SetupPin()
{
    // --------------------------------------------------
    // Ќа плате arduino
    // SPI
    //      PIN_SPI_SS (10) - выбор ведомого устройства
    pinMode(PIN_SPI_SS, OUTPUT);
    //      Ёти пины инициализирует библиотека SPI
    //      PIN_SPI_MOSI (11) - лини€ передачи данных от ведущего к ведомым устройствам
    //      PIN_SPI_MISO (12) - лини€ передачи от ведомого к ведущему устройству
    //      PIN_SPI_SCK (13) - тактовые импульсы синхронизации, генерируемые ведущим устройством 
    // ¬заимодействие с cc1101
    //      PIN_CC_GDO0 - программируемый вывод сс1101
    pinMode(PIN_CC_GDO0, INPUT);
    //      PIN_CC_GDO2 - программируемый вывод сс1101
    pinMode(PIN_CC_GDO2, INPUT);

    // --------------------------------------------------
    // Ќа плате cc1101
    //      SI (Slave Input)
    //      SCK (Serial ClocK) - тактовые импульсы синхронизации
    //      SO (Slave Output)
    //      GD2 - программируемый вывод чипа
    //      GD0 - программируемый вывод чипа, например, индикаци€ конца передачи пакета
    //      CSN - выбор устройства
    // –асположение (маркировка со стороны деталей)
    //      VCC VCC
    //      SI  SCK
    //      SO  GD2
    //      CSN GD0
    //      GND GND
    
    // --------------------------------------------------
    // ѕодключение arduino - cc1101
    //      VCC (3.3v) -   VCC (3.3v)
    //      GND -          GND
    //      PIN_SPI_SS -   CSN
    //      PIN_SPI_MOSI - SI
    //      PIN_SPI_MISO - SO
    //      PIN_SPI_SS -   SCK
    //      PIN_CC_GDO0 -  GD0
    //      PIN_CC_GDO2 -  GD2
}

void setup() 
{
    SetupPin();

    Serial.begin(9600);

}

void loop() {}
