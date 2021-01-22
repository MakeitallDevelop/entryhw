/**********************************************************************************
   The following software may be included in this software : orion_firmware.ino
   from http://www.makeblock.cc/
   This software contains the following license and notice below:
   CC-BY-SA 3.0 (https://creativecommons.org/licenses/by-sa/3.0/)
   Author : Ander, Mark Yan
   Updated : Ander, Mark Yan
   Date : 01/09/2016
   Description : Firmware for Makeblock Electronic modules with Scratch.
   Copyright (C) 2013 - 2016 Maker Works Technology Co., Ltd. All right reserved.
 **********************************************************************************/

#include <dht.h>
#include <Servo.h> //헤더 호출
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <Adafruit_NeoPixel.h>
#include <LedControl.h>
#include <DFRobotDFPlayerMini.h>

// Module Constant //핀설정
#define ALIVE 0
#define DIGITAL 1
#define ANALOG 2
#define PWM 3
#define SERVO 4
#define TONE 5
#define PULSEIN 6
#define ULTRASONIC 7
#define TIMER 8
#define READ_BLUETOOTH 9
#define WRITE_BLUETOOTH 10
#define LCD 11
#define LCDCLEAR 12
//#define RGBLED 13
#define DCMOTOR 14
#define OLED 15
#define PIR 16
#define LCDINIT 17
#define DHTHUMI 18
#define DHTTEMP 19
//#define NEOPIXEL 20
#define NEOPIXELINIT 20
#define NEOPIXELBRIGHT 21
#define NEOPIXEL 22
#define NEOPIXELALL 23
#define NEOPIXELCLEAR 24
#define DOTMATRIXINIT 25
#define DOTMATRIXBRIGHT 26
#define DOTMATRIX 27
#define DOTMATRIXEMOJI 28
#define DOTMATRIXCLEAR 29
#define MP3INIT 30
#define MP3PLAY1 31
#define MP3PLAY2 32
#define MP3VOL 33
#define RESET_ 34

// State Constant
#define GET 1
#define SET 2
#define MODULE 3
#define RESET 4

Servo servos[8];
Servo sv;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(4, 7, NEO_GRB + NEO_KHZ800);

int tx;
int rx;
int vol;

unsigned long previousMillis = 0;

//LedControl lcjikko = LedControl(12, 11, 10, 1);

LiquidCrystal_I2C lcd(0x27, 16, 2);
dht myDHT11;
SoftwareSerial softSerial(2, 3);

//U8GLIB_SSD1306_128X64 oled(U8G_I2C_OPT_NONE);

// val Union        //??
union
{
    byte byteVal[4];
    float floatVal;
    long longVal;
} val;

// valShort Union       //??
union
{
    byte byteVal[2];
    short shortVal;
} valShort;

int analogs[6] = {0, 0, 0, 0, 0, 0}; // 아날로그 디지털 핀 값저장
int digitals[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int servo_pins[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Ultrasonic             //초음파 센서
float lastUltrasonic = 0;
int trigPin = 13;
int echoPin = 12;
int dinPin = 12;
int clkPin = 11;
int csPin = 10;
int dotBright = 8;

bool dotFlag = false;

int dhtPin = 0;
int dhtMode = 0;
// bluetooth                //블루투스
String makeBtString;
int softSerialRX = 2;
int softSerialTX = 3;
unsigned long prev_time_BT = 0;

// LCD

// Buffer
char buffer[52];
unsigned char prevc = 0;
byte index = 0;
byte dataLen;

double lastTime = 0.0;
double currentTime = 0.0;

uint8_t command_index = 0;

boolean isStart = false;
boolean isUltrasonic = false;
boolean isBluetooth = false;
boolean isDHThumi = false;
boolean isDHTtemp = false;

// End Public Value

void _delay(float seconds)
{
    long endTime = millis() + seconds * 1000;
    while (millis() < endTime)
        ;
}

void setup()
{                           //초기화
    Serial.begin(115200);   //시리얼 115200
    softSerial.begin(9600); //블루투스 9600
    //MP3Module.begin(9600);
    initPorts();
    initNeo();
    initLCD();
    delay(200);
}

void initPorts()
{ //디지털 포트 초기화(4~14)
    for (int pinNumber = 4; pinNumber < 14; pinNumber++)
    {
        pinMode(pinNumber, OUTPUT);
        digitalWrite(pinNumber, LOW);
    }
}

void initNeo()
{ // 네오픽셀 초기화
    strip.begin();
    strip.show();
}

void initLCD()
{ //lcd 초기화
    lcd.init();
    lcd.backlight();
    lcd.clear();
    // lcd.setCursor(0, 0);
    // lcd.print("Blacksmith Board");
    // lcd.setCursor(6, 1);
    // lcd.print("with Entry");
}

void loop()
{ //반복 시리얼 값 , 블루투스 값 받기
    while (Serial.available())
    {
        if (Serial.available() > 0)
        {
            char serialRead = Serial.read();
            setPinValue(serialRead & 0xff);
        }
    }
    while (softSerial.available())
    {
        if (softSerial.available() > 0)
        {
            char softSerialRead = softSerial.read();
            makeBtString += softSerialRead;
        }
    }
    delay(15);
    sendPinValues(); //핀 값보내기
    delay(10);
}

void setPinValue(unsigned char c)
{
    if (c == 0x55 && isStart == false)
    {
        if (prevc == 0xff)
        {
            index = 1;
            isStart = true;
        }
    }
    else
    {
        prevc = c;
        if (isStart)
        {
            if (index == 2)
            {
                dataLen = c;
            }
            else if (index > 2)
            {
                dataLen--;
            }
            writeBuffer(index, c);
        }
    }

    index++;

    if (index > 51)
    {
        index = 0;
        isStart = false;
    }

    if (isStart && dataLen == 0 && index > 3)
    {
        isStart = false;
        parseData();
        index = 0;
    }
}

unsigned char readBuffer(int index)
{
    return buffer[index];
}

void parseData()
{
    isStart = false;
    int idx = readBuffer(3);
    command_index = (uint8_t)idx;
    int action = readBuffer(4);
    int device = readBuffer(5);
    int port = readBuffer(6);

    switch (action)
    {
    case GET:
    {
        if (device == DIGITAL)
        {
            digitals[port] = readBuffer(7);
        }
        else if (device == ULTRASONIC)
        {
            if (!isUltrasonic)
            {
                setUltrasonicMode(true);
                trigPin = readBuffer(6);
                echoPin = readBuffer(7);
                digitals[trigPin] = 1;
                digitals[echoPin] = 1;
                pinMode(trigPin, OUTPUT);
                pinMode(echoPin, INPUT);
                delay(50);
            }
            else
            {
                int trig = readBuffer(6);
                int echo = readBuffer(7);
                if (trig != trigPin || echo != echoPin)
                {
                    trigPin = trig;
                    echoPin = echo;
                    digitals[trigPin] = 1;
                    digitals[echoPin] = 1;
                    pinMode(trigPin, OUTPUT);
                    pinMode(echoPin, INPUT);
                    delay(50);
                }
            }
        }
        else if (device == DHTHUMI)
        {
            isDHThumi = true;
            //setDHTTEMPMode(true);
            dhtPin = readBuffer(6);
            //myDHT11.read11(dhtPin);
            digitals[port] = 1;
        }
        else if (device == DHTTEMP)
        {
            isDHTtemp = true;

            //setDHTHUMIMode(true);
            dhtPin = readBuffer(6);
            //  myDHT11.read11(dhtPin);
            digitals[port] = 1;
        }
        else if (device == READ_BLUETOOTH)
        {
            softSerial.begin(9600);
            pinMode(softSerialRX, INPUT);
            if (!isBluetooth)
            {
                setBluetoothMode(true);
            }
        }
        else if (device == WRITE_BLUETOOTH)
        {
            softSerial.begin(9600);
            pinMode(softSerialTX, OUTPUT);
            if (!isBluetooth)
            {
                setBluetoothMode(true);
            }
        }
        else if (port == trigPin || port == echoPin)
        {
            setUltrasonicMode(false);
            digitals[port] = 0;
        }
        else if (device != READ_BLUETOOTH && port == softSerialRX)
        {
            softSerial.end();
            setBluetoothMode(false);
            digitals[port] = 0;
        }
        else if (device != WRITE_BLUETOOTH && port == softSerialTX)
        {
            softSerial.end();
            setBluetoothMode(false);
            digitals[port] = 0;
        }
        else
        {
            digitals[port] = 0;
        }
    }
    break;
    case SET:
    {
        runSet(device);
        callOK();
    }
    break;
    case MODULE:
    {
        runModule(device);
        callOK();
    }
    case RESET:
    {
        callOK();
    }
    break;
    }
}

void runSet(int device)
{
    //0xff 0x55 0x6 0x0 0x1 0xa 0x9 0x0 0x0 0xa

    int port = readBuffer(6);
    unsigned char pin = port;
    if (pin == trigPin || pin == echoPin)
    {
        setUltrasonicMode(false);
    }

    switch (device)
    {
    case RESET_:
    {
        lcd.init();
        lcd.clear();

        LedControl *lcjikko = new LedControl(dinPin, clkPin, csPin, 1);
        lcjikko->clearDisplay(0);
        delete lcjikko;

        strip.setPixelColor(0, 0, 0, 0);
        strip.setPixelColor(1, 0, 0, 0);
        strip.setPixelColor(2, 0, 0, 0);
        strip.setPixelColor(3, 0, 0, 0);
        strip.show();
    }
    break;
    case DIGITAL:
    {
        setPortWritable(pin);
        int v = readBuffer(7);
        digitalWrite(pin, v);
    }
    break;
    case PWM:
    {
        setPortWritable(pin);
        int v = readBuffer(7);
        analogWrite(pin, v);
    }
    break;
    case TONE:
    {
        setPortWritable(pin);
        int hz = readShort(7);
        int ms = readShort(9);
        if (ms > 0)
        {
            tone(pin, hz, ms);
        }
        else
        {
            noTone(pin);
        }
    }
    break;
        // case DHTINIT:
        // {
        //     dhtPin = readBuffer(6);
        //     digitals[dhtPin] = 1;
        // }
        // break;
    case NEOPIXELINIT:
    {
        setPortWritable(pin);
        strip = Adafruit_NeoPixel(readBuffer(7), readBuffer(6), NEO_GRB + NEO_KHZ800);
        strip.begin();
        strip.setPixelColor(0, 0, 0, 0);
        strip.setPixelColor(1, 0, 0, 0);
        strip.setPixelColor(2, 0, 0, 0);
        strip.setPixelColor(3, 0, 0, 0);
        strip.show();
        //delay(1);
    }
    break;
    case NEOPIXELBRIGHT:
    {
        //setPortWritable(pin);
        int bright = readBuffer(7);

        strip.setBrightness(bright);
        //delay(10);
    }
    break;
    case NEOPIXEL:
    {
        //setPortWritable(pin);
        //strip.begin();

        int num = readBuffer(7);
        int r = readBuffer(9);
        int g = readBuffer(11);
        int b = readBuffer(13);

        if (num == 4)
        {
            setPortWritable(pin);
            //strip.begin();
            strip.setPixelColor(0, 0, 0, 0);
            strip.setPixelColor(1, 0, 0, 0);
            strip.setPixelColor(2, 0, 0, 0);
            strip.setPixelColor(3, 0, 0, 0);
            strip.show();
            delay(50);
            break;
        }
        else
        {
            strip.setPixelColor(num, r, g, b);
            strip.show();
        }
    }
    break;
    case NEOPIXELALL:
    {
        //setPortWritable(pin);
        //strip.begin();

        int r = readBuffer(7);
        int g = readBuffer(9);
        int b = readBuffer(11);

        strip.setPixelColor(0, r, g, b);
        strip.setPixelColor(1, r, g, b);
        strip.setPixelColor(2, r, g, b);
        strip.setPixelColor(3, r, g, b);

        strip.show();
        //delay(50);
    }
    break;
    case NEOPIXELCLEAR:
    {
        setPortWritable(pin);
        //strip.begin();
        strip.setPixelColor(0, 0, 0, 0);
        strip.setPixelColor(1, 0, 0, 0);
        strip.setPixelColor(2, 0, 0, 0);
        strip.setPixelColor(3, 0, 0, 0);
        strip.show();
        //delay(1);
    }
    break;
    /*
    case PULLUP:
    {
        pinMode(pin, INPUT_PULLUP);
        if(digitalRead(pin) == LOW){
            digitalWrite(pin, 1);
        }
        else if(digitalRead(pin) == HIGH){
            digitalWrite(pin, 0);
        }
    }
    break;
    */
    case DOTMATRIXINIT:
    {
        dinPin = readBuffer(7);
        clkPin = readBuffer(9);
        csPin = readBuffer(11);
    }
    break;
    case DOTMATRIXCLEAR:
    {
        LedControl *lcjikko = new LedControl(dinPin, clkPin, csPin, 1);
        lcjikko->clearDisplay(0);
        delete lcjikko;
    }
    break;
    case DOTMATRIXBRIGHT:
    {
        dotBright = readBuffer(7);
    }
    break;
    case DOTMATRIX:
    {
        int len = readBuffer(7);
        String txt = readString(len, 9);
        String row;

        ////////////////////////////////////////////////////////////////

        LedControl *lcjikko = new LedControl(dinPin, clkPin, csPin, 1);
        lcjikko->shutdown(0, false);
        lcjikko->setIntensity(0, dotBright);

        for (int temp = 15; temp > 0; temp -= 2)
        {
            switch (txt.charAt(temp - 1))
            {
            case '0':
                row = "0000";
                break;
            case '1':
                row = "0001";
                break;
            case '2':
                row = "0010";
                break;
            case '3':
                row = "0011";
                break;
            case '4':
                row = "0100";
                break;
            case '5':
                row = "0101";
                break;
            case '6':
                row = "0110";
                break;
            case '7':
                row = "0111";
                break;
            case '8':
                row = "1000";
                break;
            case '9':
                row = "1001";
                break;
            case 'a':
                row = "1010";
                break;
            case 'b':
                row = "1011";
                break;
            case 'c':
                row = "1100";
                break;
            case 'd':
                row = "1101";
                break;
            case 'e':
                row = "1111";
                break;
            }
            for (int col = 0; col < 4; col++)
            {
                if (row.charAt(col) == '1')
                {
                    lcjikko->setLed(0, 7 - (temp - 1) / 2, col, true);
                }
                else
                {
                    lcjikko->setLed(0, 7 - (temp - 1) / 2, col, false);
                }
            }
            switch (txt.charAt(temp))
            {
            case '0':
                row = "0000";
                break;
            case '1':
                row = "0001";
                break;
            case '2':
                row = "0010";
                break;
            case '3':
                row = "0011";
                break;
            case '4':
                row = "0100";
                break;
            case '5':
                row = "0101";
                break;
            case '6':
                row = "0110";
                break;
            case '7':
                row = "0111";
                break;
            case '8':
                row = "1000";
                break;
            case '9':
                row = "1001";
                break;
            case 'a':
                row = "1010";
                break;
            case 'b':
                row = "1011";
                break;
            case 'c':
                row = "1100";
                break;
            case 'd':
                row = "1101";
                break;
            case 'e':
                row = "1111";
                break;
            }
            for (int col = 0; col < 4; col++)
            {
                if (row.charAt(col) == '1')
                {
                    lcjikko->setLed(0, 7 - (temp - 1) / 2, col + 4, true);
                }
                else
                {
                    lcjikko->setLed(0, 7 - (temp - 1) / 2, col + 4, false);
                }
            }
        }

        delete lcjikko;
    }
    break;
    case DOTMATRIXEMOJI:
    {
        int list = readBuffer(7);
        
        LedControl *lcjikko = new LedControl(dinPin, clkPin, csPin, 1);
        lcjikko->shutdown(0, false);
        lcjikko->setIntensity(0, dotBright);
        
        switch(list){
            case 1:
                lcjikko->setRow(0, 0, B01100110);
                lcjikko->setRow(0, 1, B11111111);
                lcjikko->setRow(0, 2, B11111111);
                lcjikko->setRow(0, 3, B11111111);
                lcjikko->setRow(0, 4, B01111110);
                lcjikko->setRow(0, 5, B00111100);
                lcjikko->setRow(0, 6, B00011000);
                lcjikko->setRow(0, 7, B01100110);
                break;
            case 2:
                lcjikko->setRow(0, 0, B01100110);
                lcjikko->setRow(0, 1, B10011001);
                lcjikko->setRow(0, 2, B10000001);
                lcjikko->setRow(0, 3, B10000001);
                lcjikko->setRow(0, 4, B01000010);
                lcjikko->setRow(0, 5, B00100100);
                lcjikko->setRow(0, 6, B00011000);
                lcjikko->setRow(0, 7, B00000000);
                break;    
            case 3:
                lcjikko->setRow(0, 0, B00000000);
                lcjikko->setRow(0, 1, B00010000);
                lcjikko->setRow(0, 2, B00111000);
                lcjikko->setRow(0, 3, B01010100);
                lcjikko->setRow(0, 4, B00010000);
                lcjikko->setRow(0, 5, B00010000);
                lcjikko->setRow(0, 6, B00010000);
                lcjikko->setRow(0, 7, B00000000);
                break;    
            case 4:
                lcjikko->setRow(0, 0, B00000000);
                lcjikko->setRow(0, 1, B00010000);
                lcjikko->setRow(0, 2, B00010000);
                lcjikko->setRow(0, 3, B00010000);
                lcjikko->setRow(0, 4, B01010100);
                lcjikko->setRow(0, 5, B00111000);
                lcjikko->setRow(0, 6, B00010000);
                lcjikko->setRow(0, 7, B00000000);
                break;
            case 5:
                lcjikko->setRow(0, 0, B00000000);
                lcjikko->setRow(0, 1, B00010000);
                lcjikko->setRow(0, 2, B00100000);
                lcjikko->setRow(0, 3, B01111110);
                lcjikko->setRow(0, 4, B00100000);
                lcjikko->setRow(0, 5, B00010000);
                lcjikko->setRow(0, 6, B00000000);
                lcjikko->setRow(0, 7, B00000000);
                break;
            case 6:
                lcjikko->setRow(0, 0, B00000000);
                lcjikko->setRow(0, 1, B00001000);
                lcjikko->setRow(0, 2, B00000100);
                lcjikko->setRow(0, 3, B01111110);
                lcjikko->setRow(0, 4, B00000100);
                lcjikko->setRow(0, 5, B00001000);
                lcjikko->setRow(0, 6, B00000000);
                lcjikko->setRow(0, 7, B00000000);
                break;
                
        }
        delete lcjikko;
    }
    break;

    case MP3INIT:
    {
        tx = readBuffer(7);
        rx = readBuffer(9);
        vol = 15;
    }
    break;
    case MP3PLAY1:
    {
        SoftwareSerial MP3Module = SoftwareSerial(tx, rx); // tx rx
        DFRobotDFPlayerMini MP3Player;
        MP3Module.begin(9600);
        MP3Player.begin(MP3Module);

        MP3Player.volume(vol);
        delay(10);

        int num = readBuffer(9);
        MP3Player.play(num);
    }
    break;

    case MP3PLAY2:
    {
        int num = readBuffer(9);
        int time_value = readBuffer(11);
        SoftwareSerial MP3Module = SoftwareSerial(tx, rx); // tx rx
        DFRobotDFPlayerMini MP3Player;
        MP3Module.begin(9600);
        MP3Player.begin(MP3Module);

        MP3Player.volume(vol);
        delay(10);
        MP3Player.play(num);
    }
    break;

    case MP3VOL:
    {
        vol = readBuffer(9);
    }
    break;
    case SERVO:
    {
        setPortWritable(pin);
        int v = readBuffer(7);
        if (v >= 0 && v <= 180)
        {
            sv = servos[searchServoPin(pin)];
            sv.attach(pin);
            sv.write(v);
        }
    }
    break;
    case TIMER:
    {
        lastTime = millis() / 1000.0;
    }
    break;
    case DCMOTOR:
    {
        int directionPort = readBuffer(7);
        int speedPort = readBuffer(9);
        int directionValue = readBuffer(11);
        int speedValue = readBuffer(13);
        setPortWritable(directionPort);
        setPortWritable(speedPort);
        digitalWrite(directionPort, directionValue);
        analogWrite(speedPort, speedValue);
    }
    break;
    default:
        break;
    }
}

void runModule(int device)
{
    //0xff 0x55 0x6 0x0 0x1 0xa 0x9 0x0 0x0 0xa
    //head head                        pinNUM
    //                                      A/D

    int port = readBuffer(6);
    unsigned char pin = port;
    switch (device)
    {
    case LCDINIT:
    {
        //주소, column, line 순서
        if (readBuffer(7) == 0) //0x27
        {
            lcd = LiquidCrystal_I2C(0x27, readBuffer(9), readBuffer(11));
        }
        else
        {
            lcd = LiquidCrystal_I2C(0x3f, readBuffer(9), readBuffer(11));
        }
        // lcd.init();
        // lcd.backlight();
        // lcd.clear();
        initLCD();
    }
    break;

    case LCDCLEAR:
    {
        lcd.clear();
    }
    break;
    case LCD:
    {
        int row = readBuffer(7);
        if (row == 3)
        {
            lcd.init();
            lcd.clear();
            break;
        }
        int column = readBuffer(9);
        int len = readBuffer(11);
        String txt = readString(len, 13);

        lcd.setCursor(column, row);
        lcd.print(txt);
    }
    break;
    case WRITE_BLUETOOTH:
    {
        char softSerialTemp[32];
        int arrayNum = 7;
        for (int i = 0; i < 17; i++)
        {
            softSerialTemp[i] = readBuffer(arrayNum);
            arrayNum += 2;
        }
        softSerial.write(softSerialTemp);
    }
    break;
    default:
        break;
    }
}

void sendPinValues()
{ //핀 값 보내기
    int pinNumber = 0;
    for (pinNumber = 4; pinNumber < 14; pinNumber++)
    {
        if (digitals[pinNumber] == 0 || digitals[pinNumber] == 2)
        {
            sendDigitalValue(pinNumber);
            callOK();
        }
    }
    for (pinNumber = 0; pinNumber < 6; pinNumber++)
    {
        if (analogs[pinNumber] == 0)
        {
            sendAnalogValue(pinNumber);
            callOK();
        }
    }

    if (isUltrasonic)
    {
        sendUltrasonic();
        callOK();
    }

    if (isDHThumi)
    {
        sendDHT();
        callOK();
    }
    if (isDHTtemp)
    {
        sendDHT();
        callOK();
    }
    if (isBluetooth && millis() - prev_time_BT < 300)
    {
        sendBluetooth();
        callOK();
    }
    else
    {
        makeBtString = "";
        prev_time_BT = millis();
    }
}

void setUltrasonicMode(boolean mode)
{
    isUltrasonic = mode;
    if (!mode)
    {
        lastUltrasonic = 0;
    }
}

void setBluetoothMode(boolean mode)
{
    isBluetooth = mode;
    if (!mode)
    {
        makeBtString = "";
    }
}

void sendDHT()
{
    myDHT11.read11(dhtPin);
    float fTempC = myDHT11.temperature;
    float fHumid = myDHT11.humidity;

    delay(50);
    if (isDHTtemp)
    {
        writeHead();
        sendFloat(fTempC);
        writeSerial(DHTTEMP);
        writeEnd();
    }

    if (isDHThumi)

    {
        writeHead();
        sendFloat(fHumid);
        writeSerial(DHTHUMI);
        writeEnd();
    }
}

void sendUltrasonic()
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    float value;

    if(digitalRead(echoPin) == LOW){
        value = (float)pulseIn(echoPin, HIGH)/29/2;
    }
    else{
        pinMode(echoPin, OUTPUT);
        digitalWrite(echoPin, LOW);
        pinMode(echoPin, INPUT);
    }
    //float value = pulseIn(echoPin, HIGH, 30000) / 29.0 / 2.0;

    
    if (value == 0)
    {
        value = lastUltrasonic;
    }
    else
    {
        lastUltrasonic = value;
    }
    
    writeHead();
    sendFloat(value);
    writeSerial(trigPin);
    writeSerial(echoPin);
    writeSerial(ULTRASONIC);
    writeEnd();
}

void sendBluetooth()
{
    writeHead();
    sendString(makeBtString);
    writeSerial(softSerialRX);
    writeSerial(READ_BLUETOOTH);
    writeEnd();
}

void sendDigitalValue(int pinNumber)
{
    if (digitals[pinNumber] == 0)
    {
        pinMode(pinNumber, INPUT);
    }
    else if (digitals[pinNumber] == 2)
    {
        pinMode(pinNumber, INPUT_PULLUP);
    }
    // pinMode(pinNumber, INPUT);
    writeHead();
    sendFloat(digitalRead(pinNumber));
    writeSerial(pinNumber);
    writeSerial(DIGITAL);
    writeEnd();
}

void sendAnalogValue(int pinNumber)
{
    float prevData, lpfData, measurement;
    float alpha = 0.1;
    bool firstRun = true;

    for (int i = 0; i < 20; i++)
    {
        measurement = analogRead(pinNumber);
        if (firstRun == true)
        {
            prevData = measurement;
            firstRun = false;
        }
        lpfData = alpha * prevData + (1 - alpha) * measurement;
        prevData = lpfData;
    }

    writeHead();
    sendFloat((int)lpfData);
    writeSerial(pinNumber);
    writeSerial(ANALOG);
    writeEnd();
}

void writeBuffer(int index, unsigned char c)
{
    buffer[index] = c;
}

void writeHead()
{
    writeSerial(0xff);
    writeSerial(0x55);
}

void writeEnd()
{
    Serial.println();
}

void writeSerial(unsigned char c)
{
    Serial.write(c);
}

void sendString(String s)
{
    int l = s.length();
    writeSerial(4);
    writeSerial(l);
    for (int i = 0; i < l; i++)
    {
        writeSerial(s.charAt(i));
    }
}

void sendFloat(float value)
{
    writeSerial(2);
    val.floatVal = value;
    writeSerial(val.byteVal[0]);
    writeSerial(val.byteVal[1]);
    writeSerial(val.byteVal[2]);
    writeSerial(val.byteVal[3]);
}

void sendShort(double value)
{
    writeSerial(3);
    valShort.shortVal = value;
    writeSerial(valShort.byteVal[0]);
    writeSerial(valShort.byteVal[1]);
}

short readShort(int idx)
{
    valShort.byteVal[0] = readBuffer(idx);
    valShort.byteVal[1] = readBuffer(idx + 1);
    return valShort.shortVal;
}

float readFloat(int idx)
{
    val.byteVal[0] = readBuffer(idx);
    val.byteVal[1] = readBuffer(idx + 1);
    val.byteVal[2] = readBuffer(idx + 2);
    val.byteVal[3] = readBuffer(idx + 3);
    return val.floatVal;
}

long readLong(int idx)
{
    val.byteVal[0] = readBuffer(idx);
    val.byteVal[1] = readBuffer(idx + 1);
    val.byteVal[2] = readBuffer(idx + 2);
    val.byteVal[3] = readBuffer(idx + 3);
    return val.longVal;
}
String readString(int len, int startIdx)
{
    String str = "";

    for (int i = startIdx; i < (startIdx + len); i++)
    {
        str += (char)readBuffer(i);
    }

    return str;
}

int searchServoPin(int pin)
{
    for (int i = 0; i < 8; i++)
    {
        if (servo_pins[i] == pin)
        {
            return i;
        }
        if (servo_pins[i] == 0)
        {
            servo_pins[i] = pin;
            return i;
        }
    }
    return 0;
}

void setPortWritable(int pin)
{
    if (digitals[pin] == 0)
    {
        digitals[pin] = 1;
        pinMode(pin, OUTPUT);
    }
}

void callOK()
{                      //상태 확인용
    writeSerial(0xff); //테일
    writeSerial(0x55); //테일2
    writeEnd();        //다음줄로 넘기기
}

void callDebug(char c)
{
    writeSerial(0xff);
    writeSerial(0x55);
    writeSerial(c);
    writeEnd();
}