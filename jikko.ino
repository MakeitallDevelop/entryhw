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

//dht 센서 라이브러리
#include <dht.h>
//서보 라이브러리
#include <Servo.h>
//I2C LCD 라이브러리
#include <LiquidCrystal_I2C.h>
//MP3 라이브러리
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
//네오픽셀 라이브러리
#include <Adafruit_NeoPixel.h>
//도트 매트릭스 라이브러리
#include <LedControl.h>
//로드셀 라이브러리
#include <HX711.h>
//스텝모터 라이브러리
#include <Stepper.h>
// RFID 라이브러리
#include <SPI.h>
#include <MFRC522.h>

// 핀 설정
#define ALIVE 0
#define DIGITAL 1
#define ANALOG 2
#define PWM 3
#define SERVO 4
#define TONE 5
#define PULSEIN 6
#define ULTRASONIC 7
#define TIMER 8
#define LCD 11
#define LCDCLEAR 12
#define DCMOTOR 14
#define PIR 16
#define LCDINIT 17
#define DHTHUMI 18
#define DHTTEMP 19
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
#define MP3VOL 33
#define LOADINIT 35
#define LOADSCALE 36
#define LOADVALUE 37
#define DUST 38
#define JOYINIT 39
#define JOYX 40
#define JOYY 41
#define JOYZ 42
#define JOYMOVE 43
#define RFIDINIT 44
#define RFIDTAP 45
#define RFIDVALUE 46
#define STEPINIT 47
#define STEPSPEED 48
#define STEPROTATE 49
#define STEPROTATE2 50
#define STEPROTATE3 51


// State Constant
#define GET 1
#define SET 2
#define MODULE 3
#define RESET 4

// val Union
union
{
    byte byteVal[4];
    float floatVal;
    long longVal;
} val;

// valShort Union
union
{
    byte byteVal[2];
    short shortVal;
} valShort;

int analogs[6] = {0, 0, 0, 0, 0, 0}; // 아날로그 디지털 핀 값저장
// 0:INPUT 1:OUTPUT 2: INPUT_PULLUP
int digitals[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int servo_pins[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// 초음파 센서 포트
float lastUltrasonic = 0;
int trigPin = 13;
int echoPin = 12;

//도트 매트릭스 포트, 밝기
int dinPin = 12;
int clkPin = 11;
int csPin = 10;
int dotBright = 8;

//dht 포트
int dhtPin = 0;

//mp3 포트
int tx = 10;
int rx = 11;
int vol = 15;

//로드셀 포트
int dout = 6;
int sck = 7;

//미세먼지 포트
int dustDPin = 7;
int dustAPin = 0;
float preVal;

//스텝모터 포트
int in1 = 8;
int in2 = 9;
int in3 = 10;
int in4 = 11;
int stepSpeed = 14;
int stepsPerRevolution = 2048;

//RFID 포트
int SS_PIN = 10;
int RST_PIN = 9;

//서보
Servo servos[8];
Servo sv;
//네오픽셀, 기본 핀: D7
Adafruit_NeoPixel strip = Adafruit_NeoPixel(4, 7, NEO_GRB + NEO_KHZ800);
//I2C LCD, 기본 주소: 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);
//dht
dht myDHT11;
//mp3 포트 & 볼륨
SoftwareSerial MP3Module = SoftwareSerial(tx, rx);
DFRobotDFPlayerMini MP3Player;
//도트매트릭스
LedControl dotMatrix = LedControl(dinPin, clkPin, csPin, 1);
//로드셀
HX711 scale(dout, sck);
int calibration_factor = 20000;
//스텝모터
Stepper myStepper = Stepper(stepsPerRevolution, in4, in2, in3, in1);

//RFID
MFRC522 rfid(SS_PIN, RST_PIN); // Instance of the class
String lastCard = "";
// Buffer
char buffer[52];
unsigned char prevc = 0;
byte index = 0;
byte dataLen;

double lastTime = 0.0;
double currentTime = 0.0;

uint8_t command_index = 0;

boolean isStart = false;

//GET&SET을 동시에 하는 초음파, DHT 등의 센서를 위한 플래그
boolean isUltrasonic = false;
boolean isDust = false;
boolean isDHThumi = false;
boolean isDHTtemp = false;
boolean isLoad = false;
boolean isRFID = false;
boolean isRFIDtap = false;

// End Public Value

void setup()
{                         //초기화
    Serial.begin(115200); //시리얼 115200
    initPorts();          //포트 초기화
    initNeo();            //네오픽셀 초기화
    initLCD();            //LCD 초기화
    delay(200);
}

void initPorts()
{ //디지털 포트 초기화(2~14)
    for (int pinNumber = 2; pinNumber < 14; pinNumber++)
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
}

void loop()
{ //반복 시리얼 값 받기
    while (Serial.available())
    {
        if (Serial.available() > 0)
        {
            char serialRead = Serial.read(); //시리얼 읽어옴
            setPinValue(serialRead & 0xff);  //읽어온 값을 버퍼에 저장
        }
    }
    delay(15);
    sendPinValues(); //핀 값 전송
    delay(10);
}

// 시리얼에서 읽어온 값을 버퍼에 읽어오고 파싱
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
            writeBuffer(index, c); //버퍼에 읽어옴
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
        parseData(); //센서 케이스 별로 데이터 파싱
        index = 0;
    }
}

/**
 * 버퍼의 index에 위치한 값 반환
 * @param int index: 읽을 버퍼 인덱스
**/
unsigned char readBuffer(int index)
{
    return buffer[index];
}

/**
 * GET/SET/MODULE/RESET 분류
 * GET인 경우 해당 포트 셋팅
 * SET/MODULE인 경우 runSet/runMoudle 호출
 * 
 * [ Buffer ]
 * 0xff 0x55 bufLen sensorIdx actionType device port  data   data  ....
 *  0    1     2        3          4       5      6    7      9
 * sensorIdx: 센서 인덱스 => 사용안함.
 * actionType: get/set/moudule/reset
 * device: 센서 종류
 * port: 포트 번호
 * **/
void parseData()
{
    isStart = false;
    int idx = readBuffer(3);
    command_index = (uint8_t)idx;
    int action = readBuffer(4);
    int device = readBuffer(5);
    int port = readBuffer(6);

    switch (action) //actionType
    {
    case GET:
    { /*
    데이터를 받아와 값을 전송해야함 
    => 해당 함수에선 플래그 설정 및 setup()과 같이 핀모드 설정만 하고
    센서 작동 및 시리얼 전송은 sendPinValues()에서 실행
    */
        if (device == DIGITAL)
        {
            //[readBuffer(7)의 값] pullup: 2, normal:0
            digitals[port] = readBuffer(7);
        }
        else if (device == ULTRASONIC)
        {
            if (!isUltrasonic)
            { //초음파 센서 초기 셋팅
                setUltrasonicMode(true);
                trigPin = readBuffer(6);
                echoPin = readBuffer(7);
                //1로 설정하여 sendPinValues()에서 INPUT으로 설정되는 것 방지
                digitals[trigPin] = 1;
                digitals[echoPin] = 1;
                pinMode(trigPin, OUTPUT);
                pinMode(echoPin, INPUT);
                delay(50);
            }
            else //초음파모드가 셋팅되어있는 경우
            {
                int trig = readBuffer(6);
                int echo = readBuffer(7);
                if (trig != trigPin || echo != echoPin)
                { //trig, echo 포트가 변경된 경우 포트 재셋팅
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
            dhtPin = readBuffer(6);
            digitals[dhtPin] = 1;
        }
        else if (device == DHTTEMP)
        {
            isDHTtemp = true;
            dhtPin = readBuffer(6);
            digitals[dhtPin] = 1;
        }
        else if (device == LOADVALUE)
        {
            isLoad = true;
            digitals[dout] = 1;
            digitals[sck] = 1;
        }
        else if (device == DUST)
        {
            isDust = true;
            dustDPin = readBuffer(6);
            dustAPin = readBuffer(7);
            digitals[dustDPin] = 1;
            analogs[dustAPin] = 1;
            //digitals[dustAPin] = 1;
            pinMode(dustDPin, OUTPUT);
            pinMode(A0 + dustAPin, INPUT);
        }
        else if (device == RFIDTAP)
        {
            isRFIDtap = true;
            digitals[SS_PIN] = 1;
            digitals[RST_PIN] = 1;
            digitals[11] = 1;
            digitals[12] = 1;
            digitals[13] = 1;
        }
        else if (device == RFIDVALUE)
        {
            isRFID = true;
            digitals[SS_PIN] = 1;
            digitals[RST_PIN] = 1;
            digitals[11] = 1;
            digitals[12] = 1;
            digitals[13] = 1;
        }
        else if (port == trigPin || port == echoPin)
        { //12,13번 포트를 사용하지만 초음파 센서가 아닌 경우
            setUltrasonicMode(false);
            digitals[port] = 0;
        }
        else
        {
            setUltrasonicMode(false);
            digitals[port] = 0;
        }
    }
    break;
    case SET: //센서에 출력해야하는 경우
    {
        runSet(device);
        callOK();
    }
    break;
    case MODULE: //LCD 작동
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

/**
 * action == set인 경우 해당 포트 셋팅
 * setPortWritable(pin)으로 digitals[pin]으로 1로 셋팅해야 함
 * 센서에 값만 셋팅하면 되므로 아두이노에서 센서를 동작시킨 것처럼 하면 됨 
 * */
void runSet(int device)
{
    //0xff 0x55 0x6 0x0 0x1 0xa 0x9 0x0 0x0 0xa

    int port = readBuffer(6);
    unsigned char pin = port;
    if (pin == trigPin || pin == echoPin)
    { //12,13번 포트를 사용하지만 초음파 센서가 아닌 경우 모드 비활성화
        setUltrasonicMode(false);
    }

    switch (device)
    {

    case DIGITAL: //센서 셋팅
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
    case TONE: //피에조 부저
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
    case NEOPIXELINIT:
    {
        setPortWritable(pin);
        //받아온 값으로 포트 재설정
        strip = Adafruit_NeoPixel(readBuffer(7), readBuffer(6), NEO_GRB + NEO_KHZ800);
        strip.begin();
        strip.setPixelColor(0, 0, 0, 0);
        strip.setPixelColor(1, 0, 0, 0);
        strip.setPixelColor(2, 0, 0, 0);
        strip.setPixelColor(3, 0, 0, 0);
        strip.show();
    }
    break;
    case NEOPIXELBRIGHT:
    {
        int bright = readBuffer(7);
        //밝기 설정
        strip.setBrightness(bright);
    }
    break;
    case NEOPIXEL:
    {
        int num = readBuffer(7);
        int r = readBuffer(9);
        int g = readBuffer(11);
        int b = readBuffer(13);

        //data=0인 경우 초기화
        if (num == 0 && r == 0 && g == 0 && b == 0)
        {
            setPortWritable(pin);
            strip.setPixelColor(0, 0, 0, 0);
            strip.setPixelColor(1, 0, 0, 0);
            strip.setPixelColor(2, 0, 0, 0);
            strip.setPixelColor(3, 0, 0, 0);
            strip.show();
            delay(50);
            break;
        }

        strip.setPixelColor(num, r, g, b);
        strip.show();
    }
    break;
    case NEOPIXELALL:
    {
        int r = readBuffer(7);
        int g = readBuffer(9);
        int b = readBuffer(11);

        strip.setPixelColor(0, r, g, b);
        strip.setPixelColor(1, r, g, b);
        strip.setPixelColor(2, r, g, b);
        strip.setPixelColor(3, r, g, b);

        strip.show();
    }
    break;
    case NEOPIXELCLEAR:
    {
        setPortWritable(pin);
        strip.setPixelColor(0, 0, 0, 0);
        strip.setPixelColor(1, 0, 0, 0);
        strip.setPixelColor(2, 0, 0, 0);
        strip.setPixelColor(3, 0, 0, 0);
        strip.show();
    }
    break;
    case STEPINIT:
    {
        in1 = readBuffer(7);
        in2 = readBuffer(9);
        in3 = readBuffer(11);
        in4 = readBuffer(13);

        setPortWritable(in1);
        setPortWritable(in2);
        setPortWritable(in3);
        setPortWritable(in4);

        //받아온 값으로 포트 재설정
        myStepper = Stepper(stepsPerRevolution, in4, in2, in3, in1);
    }
    break;
    case STEPSPEED:
    {
        stepSpeed = readBuffer(7);
    }
    break;
    case STEPROTATE:
    {
        int dir = readBuffer(7);
        float num = readFloat(9);

        if (dir == 0 && num == 0)
        {
            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);
            digitalWrite(in3, LOW);
            digitalWrite(in4, LOW);

            break;
        }

        if(dir == 1){
            myStepper.setSpeed(stepSpeed);
            myStepper.step(num);
        }
        else if(dir == 2){
            myStepper.setSpeed(stepSpeed);
            myStepper.step((-1) * num);
        }
    }
    break;
    case STEPROTATE2:
    {
        int dir = readBuffer(7);
        float num = readFloat(9);

        if (dir == 0 && num == 0)
        {
            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);
            digitalWrite(in3, LOW);
            digitalWrite(in4, LOW);

            break;
        }

        if(dir == 1){
            myStepper.setSpeed(stepSpeed);
            myStepper.step(num);
        }
        else if(dir == 2){
            myStepper.setSpeed(stepSpeed);
            myStepper.step((-1) * num);
        }
    }
    break;
    case STEPROTATE3:
    {
        int dir = readBuffer(7);
        int sec = readBuffer(9);
        float st = 0;

        if (dir == 0 && sec == 0)
        {
            digitalWrite(in1, LOW);
            digitalWrite(in2, LOW);
            digitalWrite(in3, LOW);
            digitalWrite(in4, LOW);

            break;
        }
  
        st = float(float(sec) * float(stepSpeed) / 60) * 2048;

        if(dir == 1){
            myStepper.setSpeed(stepSpeed);
            myStepper.step(st);
        }
        else if(dir == 2){
            myStepper.setSpeed(stepSpeed);
            myStepper.step((-1) * st);
        }
    }
    break;
    case DOTMATRIXINIT:
    {
        dinPin = readBuffer(7);
        clkPin = readBuffer(9);
        csPin = readBuffer(11);
        setPortWritable(dinPin);
        setPortWritable(clkPin);
        setPortWritable(csPin);
        //받아온 값으로 포트 재설정
        dotMatrix = LedControl(dinPin, clkPin, csPin, 1);
        dotMatrix.shutdown(0, false);
        dotMatrix.setIntensity(0, dotBright);
    }
    break;
    case DOTMATRIXCLEAR:
    {
        dotMatrix.clearDisplay(0);
    }
    break;
    case DOTMATRIXBRIGHT:
    {
        dotBright = readBuffer(7);
        //밝기 재설정
        dotMatrix.setIntensity(0, dotBright);
    }
    break;
    case DOTMATRIX:
    {
        int len = readBuffer(7);
        String txt = readString(len, 9); //입력받은 Hex 저장
        String row;
        //입력받은 Hex 값에 따라 도트 매트릭스 on/off
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
                row = "1110";
                break;
            case 'f':
                row = "1111";
                break;
            }
            for (int col = 0; col < 4; col++)
            {
                if (row.charAt(col) == '1') //1인 경우 킴
                {
                    dotMatrix.setLed(0, 7 - (temp - 1) / 2, col, true);
                }
                else
                {
                    dotMatrix.setLed(0, 7 - (temp - 1) / 2, col, false);
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
                row = "1110";
                break;
            case 'f':
                row = "1111";
                break;
            }
            for (int col = 0; col < 4; col++)
            {
                if (row.charAt(col) == '1')
                {
                    dotMatrix.setLed(0, 7 - (temp - 1) / 2, col + 4, true);
                }
                else
                {
                    dotMatrix.setLed(0, 7 - (temp - 1) / 2, col + 4, false);
                }
            }
        }
    }
    break;
    case DOTMATRIXEMOJI:
    {
        int list = readBuffer(7);

        switch (list)
        {
        case 0:
            dotMatrix.clearDisplay(0);
            break;
        case 1:
            dotMatrix.setRow(0, 0, B01100110);
            dotMatrix.setRow(0, 1, B11111111);
            dotMatrix.setRow(0, 2, B11111111);
            dotMatrix.setRow(0, 3, B11111111);
            dotMatrix.setRow(0, 4, B01111110);
            dotMatrix.setRow(0, 5, B00111100);
            dotMatrix.setRow(0, 6, B00011000);
            dotMatrix.setRow(0, 7, B00000000);
            break;
        case 2:
            dotMatrix.setRow(0, 0, B01100110);
            dotMatrix.setRow(0, 1, B10011001);
            dotMatrix.setRow(0, 2, B10000001);
            dotMatrix.setRow(0, 3, B10000001);
            dotMatrix.setRow(0, 4, B01000010);
            dotMatrix.setRow(0, 5, B00100100);
            dotMatrix.setRow(0, 6, B00011000);
            dotMatrix.setRow(0, 7, B00000000);
            break;
        case 3:
            dotMatrix.setRow(0, 0, B00000000);
            dotMatrix.setRow(0, 1, B00010000);
            dotMatrix.setRow(0, 2, B00111000);
            dotMatrix.setRow(0, 3, B01010100);
            dotMatrix.setRow(0, 4, B00010000);
            dotMatrix.setRow(0, 5, B00010000);
            dotMatrix.setRow(0, 6, B00010000);
            dotMatrix.setRow(0, 7, B00000000);
            break;
        case 4:
            dotMatrix.setRow(0, 0, B00000000);
            dotMatrix.setRow(0, 1, B00010000);
            dotMatrix.setRow(0, 2, B00010000);
            dotMatrix.setRow(0, 3, B00010000);
            dotMatrix.setRow(0, 4, B01010100);
            dotMatrix.setRow(0, 5, B00111000);
            dotMatrix.setRow(0, 6, B00010000);
            dotMatrix.setRow(0, 7, B00000000);
            break;
        case 5:
            dotMatrix.setRow(0, 0, B00000000);
            dotMatrix.setRow(0, 1, B00010000);
            dotMatrix.setRow(0, 2, B00100000);
            dotMatrix.setRow(0, 3, B01111110);
            dotMatrix.setRow(0, 4, B00100000);
            dotMatrix.setRow(0, 5, B00010000);
            dotMatrix.setRow(0, 6, B00000000);
            dotMatrix.setRow(0, 7, B00000000);
            break;
        case 6:
            dotMatrix.setRow(0, 0, B00000000);
            dotMatrix.setRow(0, 1, B00001000);
            dotMatrix.setRow(0, 2, B00000100);
            dotMatrix.setRow(0, 3, B01111110);
            dotMatrix.setRow(0, 4, B00000100);
            dotMatrix.setRow(0, 5, B00001000);
            dotMatrix.setRow(0, 6, B00000000);
            dotMatrix.setRow(0, 7, B00000000);
            break;
        case 7:
            dotMatrix.setRow(0, 0, B00000000);
            dotMatrix.setRow(0, 1, B01000010);
            dotMatrix.setRow(0, 2, B10100101);
            dotMatrix.setRow(0, 3, B00000000);
            dotMatrix.setRow(0, 4, B00000000);
            dotMatrix.setRow(0, 5, B01000010);
            dotMatrix.setRow(0, 6, B00111100);
            dotMatrix.setRow(0, 7, B00000000);
            break;
        case 8:
            dotMatrix.setRow(0, 0, B00000000);
            dotMatrix.setRow(0, 1, B00000000);
            dotMatrix.setRow(0, 2, B11100111);
            dotMatrix.setRow(0, 3, B01000010);
            dotMatrix.setRow(0, 4, B01000010);
            dotMatrix.setRow(0, 5, B00011000);
            dotMatrix.setRow(0, 6, B00100100);
            dotMatrix.setRow(0, 7, B00000000);
            break;
        case 9:
            dotMatrix.setRow(0, 0, B10000001);
            dotMatrix.setRow(0, 1, B01000010);
            dotMatrix.setRow(0, 2, B00100100);
            dotMatrix.setRow(0, 3, B00000000);
            dotMatrix.setRow(0, 4, B00000000);
            dotMatrix.setRow(0, 5, B00111100);
            dotMatrix.setRow(0, 6, B01111110);
            dotMatrix.setRow(0, 7, B01111110);
            break;
        case 10:
            dotMatrix.setRow(0, 0, B00000000);
            dotMatrix.setRow(0, 1, B01000010);
            dotMatrix.setRow(0, 2, B00100100);
            dotMatrix.setRow(0, 3, B01000010);
            dotMatrix.setRow(0, 4, B00000000);
            dotMatrix.setRow(0, 5, B00111100);
            dotMatrix.setRow(0, 6, B00100100);
            dotMatrix.setRow(0, 7, B00011000);
            break;
        }
    }
    break;

    case MP3INIT:
    {
        tx = readBuffer(7);
        rx = readBuffer(9);
        setPortWritable(tx);
        setPortWritable(rx);
        //입력받은 값으로 포트 재설정
        MP3Module = SoftwareSerial(tx, rx);
        MP3Module.begin(9600);
        MP3Player.begin(MP3Module);

        vol = 15; //vol 초기 값
        MP3Player.volume(vol);
        delay(10); //없으면 작동 X
    }
    break;
    case MP3PLAY1:
    {
        MP3Player.volume(vol);
        delay(10); //없으면 작동 X
        int num = readBuffer(9);
        MP3Player.play(num);
    }
    break;
    case MP3VOL:
    {
        vol = readBuffer(9);
    }
    break;
    case LOADINIT:
    {
        dout = readBuffer(7);
        sck = readBuffer(9);
        setPortWritable(dout);
        setPortWritable(sck);
        //받아온 값으로 포트 재설정
        scale = HX711(dout, sck);
    }
    break;
    case LOADSCALE:
    {
        calibration_factor = readBuffer(7);
        scale.set_scale(calibration_factor);
        scale.tare();
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
    case RFIDINIT:
    {
        SS_PIN = readBuffer(7);
        RST_PIN = readBuffer(9);

        digitals[SS_PIN] = 1;
        digitals[RST_PIN] = 1;
        digitals[11] = 1;
        digitals[12] = 1;
        digitals[13] = 1;
        //받아온 값으로 포트 재설정
        rfid = MFRC522(SS_PIN, RST_PIN);
        // SPI.begin();     // Init SPI bus
        // rfid.PCD_Init(); // Init MFRC522
        // for (byte i = 0; i < 6; i++)
        // {
        //     key.keyByte[i] = 0xFF;
        // }
    }
    case TIMER:
    {
        lastTime = millis() / 1000.0;
    }
    break;
    default:
        break;
    }
}

//I2CLCD 작동
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
    {                           //주소, column, line 순서
        if (readBuffer(7) == 0) //주소: 0x27
        {
            lcd = LiquidCrystal_I2C(0x27, readBuffer(9), readBuffer(11));
        }
        else //주소: 0x3f
        {
            lcd = LiquidCrystal_I2C(0x3f, readBuffer(9), readBuffer(11));
        }
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
        int column = readBuffer(9);
        int len = readBuffer(11);
        String txt = readString(len, 13);
        if (len == 0) //data=0인 경우
        {
            lcd.init();
            lcd.clear();
            break;
        }

        lcd.setCursor(column, row);
        lcd.print(txt);
    }
    break;
        break;
    default:
        break;
    }
}

void sendPinValues()
{ //핀 값 보내기
    int pinNumber = 0;
    for (pinNumber = 2; pinNumber < 14; pinNumber++)
    {
        if (digitals[pinNumber] == 0 || digitals[pinNumber] == 2)
        { //0:INPUT, 1:OUTPUT, 2:PULLUP
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

    //초음파 센서 값 전송
    if (isUltrasonic)
    {
        sendUltrasonic();
        callOK();
    }
    //DHT 센서 값 전송
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
    //로드셀 센서 값 전송
    if (isLoad)
    {
        sendLoad();
        callOK();
    }
    //미세먼지 센서 값 전송
    if (isDust)
    {
        sendDust();
        callOK();
    }

    if (isRFID)
    {
        sendRFID();
        callOK();
    }

    if (isRFIDtap)
    {
        sendRFIDtap();
        callOK();
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

void sendDHT()
{
    myDHT11.read11(dhtPin);
    float fTempC = myDHT11.temperature;
    float fHumid = myDHT11.humidity;

    delay(50);
    if (isDHTtemp)
    {
        writeHead();       //읽어온 값 시리얼 전송 시작
        sendFloat(fTempC); //float 시리얼 전송
        writeSerial(DHTTEMP);
        writeEnd(); //줄바꿈
    }

    if (isDHThumi)

    {
        writeHead();
        sendFloat(fHumid);
        writeSerial(DHTHUMI);
        writeEnd();
    }
}
void sendLoad()
{
    float load_value = scale.get_units() / 2.205;

    delay(50);
    if (isLoad)
    {
        writeHead();           //읽어온 값 시리얼 전송 시작
        sendFloat(load_value); //float 시리얼 전송
        writeSerial(dout);
        writeSerial(sck);
        writeSerial(LOADVALUE);
        writeEnd(); //줄바꿈
    }
}

void sendDust()
{
    float dust = 0;
    int SampleTime = 30;
    int samplingTime = 280;
    int deltaTime = 40;
    int sleepTime = 9680;
    float voMeasured = 0;
    float calcVoltage = 0;
    float dustDensity = 0;

    for (int i = 0; i < SampleTime; i++)
    {

        digitalWrite(dustDPin, LOW); // power on the LED 0.028초부터 led가 가장 밝음으로 최소샘플링 타임으로 결정
        delayMicroseconds(samplingTime);
        voMeasured = analogRead(dustAPin); // read the dust value
        delayMicroseconds(deltaTime);
        digitalWrite(dustDPin, HIGH); // turn the LED off
        delayMicroseconds(sleepTime);
        //////////////////////////////////////////// dust vlaue 받아오기
        calcVoltage = voMeasured * (5.0 / 1024.0);
        dustDensity = calcVoltage / 0.005; //(0.17 * calcVoltage - 0.1) * 1000;
        /////////////////////value 의 미세한 변화를 수식으로 먼지량으로 변환
        if (dustDensity < 0)
        { // value 의 값이 마이너스일경우 오류 임으로 0을 대입
            dustDensity = 0;
        }
        dust += dustDensity;
    }

    dust = dust / SampleTime; // 센싱한값의 평균을 return
    writeHead();
    sendFloat(dust);
    writeSerial(dustDPin);
    writeSerial(dustAPin);
    writeSerial(DUST);
    writeEnd();
}

void sendRFID()
{
    boolean flag1 = false, flag2 = false;
    String hexstring = "";

    SPI.begin();                    // Init SPI bus
    rfid.PCD_Init(SS_PIN, RST_PIN); // Init MFRC522

    if (rfid.PICC_IsNewCardPresent()) //두 조건문없으면 실행 X
        flag1 = true;
    if (rfid.PICC_ReadCardSerial())
        flag2 = true;

    if (flag1 && flag2)
    {
        for (byte i = 0; i < 4; i++)
        {
            hexstring += String(rfid.uid.uidByte[i], HEX);
        }
        lastCard = hexstring;
    }
    else
    {
        hexstring = "";
    }

    writeHead();
    sendString(hexstring);
    writeSerial(SS_PIN);
    writeSerial(RST_PIN);
    writeSerial(RFIDVALUE);
    writeEnd();
}

void sendRFIDtap()
{
    SPI.begin();                             // Init SPI bus
    rfid.PCD_Init(SS_PIN, RST_PIN);          // Init MFRC522
    writeHead();                             //읽어온 값 시리얼 전송 시작
    sendFloat(rfid.PICC_IsNewCardPresent()); //float 시리얼 전송

    writeSerial(SS_PIN);
    writeSerial(RST_PIN);
    writeSerial(RFIDTAP);
    writeEnd(); //줄바꿈

    rfid.PCD_Reset();
}

void sendUltrasonic()
{
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    float value = pulseIn(echoPin, HIGH, 30000) / 29.0 / 2.0;

    if (value == 0)
    {
        value = lastUltrasonic;
    }
    else
    {
        lastUltrasonic = value;
    }

    writeHead(); //읽어온 값 시리얼 전송 시작
    sendFloat(value);
    writeSerial(trigPin);
    writeSerial(echoPin);
    writeSerial(ULTRASONIC);
    writeEnd();
}

void sendDigitalValue(int pinNumber)
{
    if (digitals[pinNumber] == 0)
    { //INPUT
        pinMode(pinNumber, INPUT);
    }
    else if (digitals[pinNumber] == 2)
    { //INPUT PULLUP
        pinMode(pinNumber, INPUT_PULLUP);
    }
    writeHead(); //읽어온 값 시리얼 전송 시작
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

//LCD String을 버퍼에 읽어들임
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

//pinMode OUTPUT 설정
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