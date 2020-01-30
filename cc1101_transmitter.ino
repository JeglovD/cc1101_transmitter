#include <SPI.h>

const uint8_t PIN_CC_GDO0{ 3 };
const uint8_t PIN_CC_GDO2{ 2 };

void SetupPin()
{
    // --------------------------------------------------
    // �� ����� arduino
    // SPI
    //      PIN_SPI_SS (10) - ����� �������� ����������
    pinMode(PIN_SPI_SS, OUTPUT);
    //      ��� ���� �������������� ���������� SPI
    //      PIN_SPI_MOSI (11) - ����� �������� ������ �� �������� � ������� �����������
    //      PIN_SPI_MISO (12) - ����� �������� �� �������� � �������� ����������
    //      PIN_SPI_SCK (13) - �������� �������� �������������, ������������ ������� ����������� 
    // �������������� � cc1101
    //      PIN_CC_GDO0 - ��������������� ����� ��1101
    pinMode(PIN_CC_GDO0, INPUT);
    //      PIN_CC_GDO2 - ��������������� ����� ��1101
    pinMode(PIN_CC_GDO2, INPUT);

    // --------------------------------------------------
    // �� ����� cc1101
    //      SI (Slave Input)
    //      SCK (Serial ClocK) - �������� �������� �������������
    //      SO (Slave Output)
    //      GD2 - ��������������� ����� ����
    //      GD0 - ��������������� ����� ����, ��������, ��������� ����� �������� ������
    //      CSN - ����� ����������
    // ������������ (���������� �� ������� �������)
    //      VCC VCC
    //      SI  SCK
    //      SO  GD2
    //      CSN GD0
    //      GND GND
    
    // --------------------------------------------------
    // ����������� arduino - cc1101
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
