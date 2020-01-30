#include <SPI.h>

const uint8_t PIN_CC_GDO0{ 3 };
const uint8_t PIN_CC_GDO2{ 2 };

// cc1101 Command Strobes
const byte SRES{ 0x30 };    // Reset chip.
const byte SFSTXON{ 0x31 }; // Enableand calibrate frequency synthesizer(if MCSM0.FS_AUTOCAL = 1).If in RX(with CCA) :
                            // Go to a wait state where only the synthesizer is running(for quick RX / TX turnaround).
const byte SXOFF{ 0x32 };   // Turn off crystal oscillator.
const byte SCAL{ 0x33 };    // Calibrate frequency synthesizerand turn it off.SCAL can be strobed from IDLE mode without
                            // setting manual calibration mode(MCSM0.FS_AUTOCAL = 0)
const byte SRX{ 0x34 };     // Enable RX.Perform calibration first if coming from IDLEand MCSM0.FS_AUTOCAL = 1.
const byte STX{ 0x35 };     // In IDLE state : Enable TX.Perform calibration first if MCSM0.FS_AUTOCAL = 1.
                            // If in RX state and CCA is enabled : Only go to TX if channel is clear.
const byte SIDLE{ 0x36 };   // Exit RX / TX, turn off frequency synthesizerand exit Wake - On - Radio mode if applicable.
const byte SWOR{ 0x38 };    // Start automatic RX polling sequence(Wake - on - Radio) as described in Section 19.5 if
                            // WORCTRL.RC_PD = 0.
const byte SPWD{ 0x39 };    // Enter power down mode when CSn goes high.
const byte SFRX{ 0x3A };    // Flush the RX FIFO buffer.Only issue SFRX in IDLE or RXFIFO_OVERFLOW states.
const byte SFTX{ 0x3B };    //  Flush the TX FIFO buffer.Only issue SFTX in IDLE or TXFIFO_UNDERFLOW states.
const byte SWORRST{ 0x3C }; // Reset real time clock to Event1 value.
const byte SNOP{ 0x3D };    // No operation.May be used to get access to the chip status byte.// cc1101 Configuration Registersconst byte IOCFG2{ 0x00 };const byte IOCFG1{ 0x01 };const byte IOCFG0{ 0x02 };const byte FIFOTHR{ 0x03 };const byte SYNC1{ 0x04 };const byte SYNC0{ 0x05 };const byte PKTLEN{ 0x06 };const byte PKTCRTL1{ 0x07 };const byte PKTCTRL0{ 0x08 };const byte ADDR{ 0x09 };const byte CHANNR{ 0x0A };const byte FSCTRL1{ 0x0B };const byte FSCTRL0{ 0x0C };const byte FREQ2{ 0x0D };const byte FREQ1{ 0x0E };const byte FREQ0{ 0x0F };const byte MDMCFG4{ 0x10 };const byte MDMCFG3{ 0x11 };const byte MDMCFG2{ 0x12 };const byte MDMCFG1{ 0x13 };const byte MDMCFG0{ 0x14 };const byte DEVIATN{ 0x15 };const byte MCSM2{ 0x16 };const byte MCSM1{ 0x17 };const byte MCSM0{ 0x18 };const byte FOCCFG{ 0x19 };const byte BSCFG{ 0x1A };const byte AGCCTRL2{ 0x1B };const byte AGCCTRL1{ 0x1C };const byte AGCCTRL0{ 0x1D };const byte WOREVT1{ 0x1E };const byte WOREVT0{ 0x1F };const byte WORCTRL{ 0x20 };const byte FREND1{ 0x21 };const byte FREND0{ 0x22 };const byte FSCAL3{ 0x23 };const byte FSCAL2{ 0x24 };const byte FSCAL1{ 0x25 };const byte FSCAL0{ 0x26 };const byte RCCTRL1{ 0x27 };const byte RCCTRL0{ 0x28 };const byte FSTEST{ 0x29 };const byte PTEST{ 0x2A };const byte AGCTEST{ 0x2B };const byte TEST2{ 0x2C };const byte TEST1{ 0x2D };const byte TEST0{ 0x2E };

const byte CC1101_CONFIG[] =
{
	IOCFG2, 0x29,	IOCFG1, 0x2E,	IOCFG0, 0x06,	FIFOTHR, 0x07,	SYNC1, 0xD3,	SYNC0, 0x91,	PKTLEN, 0x22, //  переменная длина пакета  - 64	PKTCRTL1, 0x04,	PKTCTRL0, 0x01,	ADDR, 0x00,	CHANNR, 0x00,	FSCTRL1, 0x0C,	FSCTRL0, 0x00,	FREQ2, 0x10,	FREQ1, 0xA7,	FREQ0, 0x62,	MDMCFG4, 0x25,	MDMCFG3, 0x83,	MDMCFG2, 0x13,	MDMCFG1, 0x22,	MDMCFG0, 0xF8,	DEVIATN, 0x62,	MCSM2, 0x07,	MCSM1, 0x30,	MCSM0, 0x18,	FOCCFG, 0x1D,	BSCFG, 0x1C,	AGCCTRL2, 0xC7,	AGCCTRL1, 0x00,	AGCCTRL0, 0xB0,	WOREVT1, 0x87,	WOREVT0, 0x6B,	WORCTRL, 0xFB,	FREND1, 0xB6,	FREND0, 0x10,	FSCAL3, 0xE9,	FSCAL2, 0x2A,	FSCAL1, 0x00,	FSCAL0, 0x1F,	RCCTRL1, 0x41,	RCCTRL0, 0x00,	FSTEST, 0x59,	PTEST, 0x7F,	AGCTEST, 0x3F,	TEST2, 0x88,	TEST1, 0x31,	TEST0, 0x09};

void PinSetup()
{
    // --------------------------------------------------
    // На плате arduino
    // SPI
    //      PIN_SPI_SS (10) - выбор ведомого устройства
    pinMode(PIN_SPI_SS, OUTPUT);
    digitalWrite(PIN_SPI_SS, HIGH);
    //      Эти пины инициализирует библиотека SPI
    //      PIN_SPI_MOSI (11) - линия передачи данных от ведущего к ведомым устройствам
    //      PIN_SPI_MISO (12) - линия передачи от ведомого к ведущему устройству
    //      PIN_SPI_SCK (13) - тактовые импульсы синхронизации, генерируемые ведущим устройством 
    // Взаимодействие с cc1101
    //      PIN_CC_GDO0 - программируемый вывод сс1101
    pinMode(PIN_CC_GDO0, INPUT);
    //      PIN_CC_GDO2 - программируемый вывод сс1101
    pinMode(PIN_CC_GDO2, INPUT);

    // --------------------------------------------------
    // На плате cc1101
    //      SI (Slave Input)
    //      SCK (Serial ClocK) - тактовые импульсы синхронизации
    //      SO (Slave Output)
    //      GD2 - программируемый вывод чипа
    //      GD0 - программируемый вывод чипа, например, индикация конца передачи пакета
    //      CSN - выбор устройства
    // Расположение (маркировка со стороны деталей)
    //      VCC VCC
    //      SI  SCK
    //      SO  GD2
    //      CSN GD0
    //      GND GND
    
    // --------------------------------------------------
    // Подключение arduino - cc1101
    //      VCC (3.3v) -   VCC (3.3v)
    //      GND -          GND
    //      PIN_SPI_SS -   CSN
    //      PIN_SPI_MOSI - SI
    //      PIN_SPI_MISO - SO
    //      PIN_SPI_SS -   SCK
    //      PIN_CC_GDO0 -  GD0
    //      PIN_CC_GDO2 -  GD2
}

byte CC1101CommandStrobe(const byte& command)
{
    SPI.beginTransaction(SPISettings());
    digitalWrite(PIN_SPI_SS, LOW);
    while (digitalRead(PIN_SPI_MISO) == HIGH); // Ждем отклик
	delay(1);
    byte result{ SPI.transfer(command) };
    digitalWrite(PIN_SPI_SS, HIGH);
    SPI.endTransaction();
    return result;
}

byte CC1101Read(const byte& address)
{
    SPI.beginTransaction(SPISettings());
    digitalWrite(PIN_SPI_SS, LOW);
    while (digitalRead(PIN_SPI_MISO) == HIGH);
    delay(1);
    SPI.transfer(address | 0xC0);
    byte result{ SPI.transfer(0) };
    digitalWrite(PIN_SPI_SS, HIGH);
    SPI.endTransaction();
    return result;
}

//Тут расшифровываем статус байтvoid C1101Status(byte _ST) {    byte _Val = 0;    _Val = _ST & 240;    _Val = _Val >> 4;    _ST = _ST & 14;    _ST = _ST >> 1;    Serial.println("============================");    if (_ST == 0) { Serial.println("IDLE"); }    if (_ST == 1) { Serial.println("RX"); }    if (_ST == 2) { Serial.println("TX"); }    if (_ST == 3) { Serial.println("Fast TX ready"); }    if (_ST == 4) { Serial.println("Frequency synthesizer calibration is running"); }    if (_ST == 5) { Serial.println("PLL is settling"); }    if (_ST == 6) { Serial.println("RX FIFO has overflowed. Read out any useful data, then flush the FIFO with SFRX"); }    if (_ST == 7) { Serial.println("TX FIFO has underflowed. Acknowledge with SFTX"); }    Serial.print("bytes available ");    Serial.println(_Val, DEC);    Serial.println("============================");}

void CC1101SetupTransmitter()
{
    SPI.beginTransaction(SPISettings());
    digitalWrite(PIN_SPI_SS, LOW);
    SPI.transfer(SRES);
    delay(100);
    for (int i{ 0 }; i < sizeof(CC1101_CONFIG); i++)
        SPI.transfer(CC1101_CONFIG[i]);
    SPI.transfer(SRX);
    digitalWrite(PIN_SPI_SS, HIGH);
    SPI.endTransaction();

    CC1101CommandStrobe(SIDLE); // Ждущий режим
    CC1101CommandStrobe(SFRX); // Очищаем буфер
    CC1101CommandStrobe(SFSTXON); // Запускаем синтезатор частоты
    delay(100);
}

void setup() 
{
    PinSetup();
    Serial.begin(9600);
    CC1101SetupTransmitter();
}

void loop() 
{
    SPI.beginTransaction(SPISettings());
    digitalWrite(PIN_SPI_SS, LOW);
    while (digitalRead(PIN_SPI_MISO) == HIGH); // Ждем ответа
    delay(1);
    SPI.transfer(0x3F | 0x40); // Команда на запись в TX FIFO
    SPI.transfer(4); // Данные    SPI.transfer(2); // Данные    SPI.transfer(3); // Данные    SPI.transfer(5); // Данные    SPI.transfer(6); // Данные    digitalWrite(PIN_SPI_SS, HIGH);    SPI.endTransaction();
    Serial.println(CC1101Read(0x3A), DEC); //Выводим кол-во байт в ТХ Фифо    C1101Status(CC1101CommandStrobe(STX)); //Запускаем передачу строблом STX и выводим статус байт. всегда пишет андерфловед    Serial.println(CC1101Read(0x3A), DEC); //Опять выводим кол-во байт в ТХ фифо и убеждаемся, что число уменьшилось на число байт в пакете, или 0 - если длина пакета переменная    delay(1);    CC1101CommandStrobe(SFTX); //Очищаем ТХ фифо. Выкидываем все лишнее    delay(1);    Serial.println("-----------------------"); //ХЗ.    delay(10);}
