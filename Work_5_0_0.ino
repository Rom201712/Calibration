char VER[] = "Ver: 5_0_0           "; //Передача даных в трекер по CAN

#include <TimerOne.h>
#include <Keypad.h>
#include <EEPROM2.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN0(53);     // Set CS to pin 53


#define OUT_PUMP          40        // вывод управления насосом
#define SPEAKEROUT        39        // спикер 36
#define IN_KCOUNT         2         // вход счетчика топлива
#define IN_BUTTON         0         // вход кнопок управления
#define DATA_LENGTH       8         // длина протокола передачи
#define LCD_SCREEN        38        // выход управления подсветкой дисплея
#define SerialTxControl   17        // RS485 управляющий контакт на arduino pin 17
#define RS485Transmit     HIGH
#define RS485Receive      LOW
#define UP                '2'
#define DOWN              '8'
#define SELECT            '#'
#define RESET             '*'
#define AUTO              'A'
#define MANUAL            'M'
// режимы работы
#define CALIBR        B010   // Калибровка счетчика
#define SETTING       B001   // Настройка
#define TARRING       B100   // Тарировка
#define PAUSE         B101   // Пауза между проливами
#define MENU          B011   // Меню
#define COUNTER       B110   // Счетчик 
//#define             B111   // РЕЗЕРВ!
#define PERELIV       0    // объем топлива для заполнения горловины бака (в 0,1 литра)
// TONES  ==========================================
// Start by defining the relationship between
//       note, period, &  frequency.
#define  c0    3830        // 261 Hz 
#define  d     3400         // 294 Hz 
#define  e     3038         // 329 Hz 
#define  f     2864         // 349 Hz 
#define  g     2550         // 392 Hz 
#define  a     2272         // 440 Hz 
#define  b     2028         // 493 Hz 
#define  C     1912         // 523 Hz 
#define  R     0

const byte ROWS = 4;       //four rows
const byte COLS = 3;       //three columns
enum { SYMBOL_HEIGHT = 8 };
char keys[ROWS][COLS] = {
  {'1', '2', '3'},
  {'4', '5', '6'},
  {'7', '8', '9'},
  {'*', '0', '#'}
};
char key;
byte rowPins[ROWS] = {22, 24, 26, 28}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {30, 32, 34};     //connect to the column pinouts of the keypad

boolean communicationError = true;

// MELODY and TIMING  =======================================
//  melody[] is an array of notes, accompanied by beats[],
//  which sets each note's relative length (higher #, longer note)
int melody[] = {  C,  b,  g,  C,  b,   e,  R,  C,  c0,  g, a, C };
int beats[]  = { 16, 16, 16,  8,  8,  16, 32, 16, 16, 16, 8, 8 };
int MAX_COUNT = sizeof(melody) / 2; // Melody length, for looping.
// Set overall tempo
long tempo = 10000;
// Loop variable to increase Rest length
int rest_count = 100; //<-BLETCHEROUS HACK; See NOTES
// Initialize core variables
int tone_ = 0;
int beat = 0;
long duration  = 0;

static const uint8_t PROGMEM dscrc_table[] = {
  0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
  157, 195, 33, 127, 252, 162, 64, 30, 95,  1, 227, 189, 62, 96, 130, 220,
  35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93,  3, 128, 222, 60, 98,
  190, 224,  2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
  70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89,  7,
  219, 133, 103, 57, 186, 228,  6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
  101, 59, 217, 135,  4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
  248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91,  5, 231, 185,
  140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
  17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
  175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
  50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
  202, 148, 118, 40, 171, 245, 23, 73,  8, 86, 180, 234, 105, 55, 213, 139,
  87,  9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
  233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
  116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
};
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );
LiquidCrystal lcd(47, 45, 43, 46, 44, 42);                // пины для подключения экрана  белый корпус с CAN

//LiquidCrystal lcd(52, 50, 48, 46, 44, 42);                // пины для подключения экрана синий корпус
byte DataToSendRS232[] = {62, 2, 7, 0, 0, 0, 0, 0, 0 };   //объем топлива пролитого через счетчик, режимы работы, номер пролива
int ArrayDUT[] = {0, 0, 0};                               // массив данных от тарируемого ДУТа
byte DataToSendRS485[] = {62, 3, 7, 0, 0, 0, 0, 0, 0 };   //данные от ДУТ (чило N)
byte DataToSendCAN[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; //(1-2 байт)объем топлива пролитого через счетчик, (3 байт) режимы работы, (4 байт) номер пролива, (5-6 байт)данные от тарируемого ДУТ (чило N), (7-8 байт) номер автомобиля (цифры)

byte DataToSendCANVNC[8] = {0x2A, 0x07, 0x08, 0x08, 0x2A, 0x2A, 0x00, 0x00};


int indexArrayDUT = 0;
volatile long Kcount = 0;                                 // счетчик  импульсов ДАРТа
int kRefill = 15;                                         //количество проливов по умолчанию
float k_in_Litr;                                          // количество импульсов на 10 литров
float Vtank = 1000;                                       // объем бака автомобиля в 0,1 литра
float Vrefill = Vtank / kRefill;                          // объем проливов 0,1 литра
const int serialSpeed = 19200;
char typeTarring = AUTO;
byte mode = MENU;
int resultN = 0;                                          // данные с ДУТа
unsigned long timeWork = 0;
volatile long timeSendRS485 = 0;
volatile long timeconnectDUT = 0;
int kRefillNum = 1;         // счетчик проливов;
float VinKinLitr = 1;  // объем топлива на 1 импульс счетчика в 0,1 литра
long kcountTemp = 0;
byte dutNumber = 1;                                       //сетефой адрес тарируемого ДУТа (0,1,2)
float VrefillFfull = 0;     // объем залитого топлива всего в 0,1 литра
float VrefillNum = 0;       // объем залитого топлива в проливе в 0,1 литра
int resultNProliv = 0;

//--------------------------------------------------------------------------------------------------------------------------------------
// мои символы
byte enter[8] = {  B00000, B00001, B00001,  B00101,  B01001,  B11111,  B01000,  B00100,};
byte circ[8] = {  B00000,  B00100,  B01110,  B11111,  B01110,  B00100,  B00000,  B00000,};
byte znac[8] = {  B00100,  B01010,  B10001,  B00000,  B10001,  B01010,  B00100,  B00000,};
byte pause[8] = {  B11111,  B10001,  B10101,  B10001,  B10111,  B10111,  B10111,  B11111,};
//--------------------------------------------------------------------------------------------------------------------------------------

void setup()
{
  CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ);
  CAN0.setMode(MCP_NORMAL);


  pinMode(IN_KCOUNT, INPUT);                            // инициализация входа импульсов ДАРТ
  digitalWrite(IN_KCOUNT, HIGH);                        // подтяжка к +5В.
  pinMode (OUT_PUMP, OUTPUT);                           // инициализация выхода управления насосом
  digitalWrite(OUT_PUMP, HIGH);                         // установка состояния ВЫКЛ насоса
  pinMode(LCD_SCREEN, OUTPUT);                          // инициализация выхода управления подсветкой дисплея
  digitalWrite(LCD_SCREEN, 220);                        //включение подсветки
  pinMode(SerialTxControl, OUTPUT);
  digitalWrite(SerialTxControl, RS485Receive);          // читаем данные с порта RS485
  Serial1.begin(serialSpeed);                           // открываем последовательное соединение 1
  Serial.begin(serialSpeed);                            // открываем последовательное соединение 0
  lcd.begin(20, 4);                                     // Инициализация дисплея
  lcd.createChar(1, enter);
  lcd.createChar(2, circ);
  lcd.createChar(3, znac);
  lcd.createChar(4, pause);

  attachInterrupt(0, rpm_fun, FALLING);                 // функция прерывания значении  0
  Timer1.initialize(3000000);                           // функция прерывания по таймеру для передачи данных по RS232 в трекер о проливах топлива
  Timer1.attachInterrupt(Timer1_action);

  lcd.clear();
  for (int i = 0; i < 19; i++) lcd.print(VER[i]);
  delay(2000);
  lcd.clear();
  pinMode(SPEAKEROUT, OUTPUT);
  tone(SPEAKEROUT, 3000, 200);
  delay(300);
  tone(SPEAKEROUT, 3000, 200);
}
void loop ()
{
  DataToSendRS232[3] = MENU << 5 ;
  DataToSendCAN[2] = MENU;
  lcd.setCursor(0, 0);
  lcd.print ("*   Calibration");
  lcd.setCursor(0, 1);
  lcd.print ("#   Setting");
  lcd.setCursor(0, 2);
  lcd.print ("2   Adjustment");
  lcd.setCursor(0, 3);
  lcd.print ("5   Counter");
  key = keypad.getKey();
  if (key == '2')
  {
    calibr ();
    lcd.clear();
  }
  if (key == SELECT)
  {
    setting ();
    lcd.clear();
  }
  if (key == RESET)
  {
    taring ();
    lcd.clear();
  }
  if (key == '5')
  {
    counter ();
    lcd.clear();
  }
  if (key == '3' || key == '4' || key == '6' || key == '7' || key == '8' || key == '9' || key == '0') timeWork = 0;
  timeWork++;
  if (timeWork > 20000) digitalWrite(LCD_SCREEN, LOW); else digitalWrite(LCD_SCREEN, HIGH);
}
//==========================================================================================================
//функция (основная) тарировки бака
void taring() {
  timeWork = 0;
  digitalWrite(LCD_SCREEN, HIGH);
  unsigned long Pause = 300000;
  tone(SPEAKEROUT, 2000, 200);
  lcd.clear();
  lcd.print("    CALIBRATION");
  lcd.setCursor(0, 1);
  lcd.print("Enter the VRN WIN ID");
  char VRNSymbol = '0';
  char VRNArray[6] = {0, 0, 0, 0, 0, 0};
  key = keypad.getKey();
  lcd.setCursor(0, 2);
  lcd.blink();
  lcd.print(VRNSymbol);
  lcd.setCursor(0, 2);
  int i = 0 ;
  while (key != SELECT)
  {
    key = keypad.getKey();
    if (key == '2')
    {
      if (VRNSymbol == 90) VRNSymbol = 47;
      if (VRNSymbol == 57) VRNSymbol = 64;
      VRNSymbol++;
      lcd.setCursor(i, 2);
      lcd.print(VRNSymbol);
      lcd.setCursor(i, 2);
    }
    if (key == '8')
    {
      if (VRNSymbol == 48) VRNSymbol = 91;
      if (VRNSymbol == 65) VRNSymbol = 58;
      VRNSymbol--;
      lcd.setCursor(i, 2);
      lcd.print(VRNSymbol);
      lcd.setCursor(i, 2);
    }
    if (key == '6')
    {
      VRNArray[i] = VRNSymbol;
      VRNSymbol = '0';
      i++;
      lcd.setCursor(i, 2);
      lcd.print(VRNSymbol);
      lcd.setCursor(i, 2);
    }
    if (key == '4')
    {
      i--;
      if (i < 0) i = 0;
      VRNSymbol = VRNArray[i];
      lcd.setCursor(i, 2);
      lcd.print(VRNSymbol);
      lcd.print(" ");
      lcd.setCursor(i, 2);
    }
  }
  lcd.clear();
//  lcd.print("    CALIBRATION");
  if (typeTarring == AUTO)
  {
    lcd.print("Enter the DUT number");
    lcd.setCursor(0, 2);
    lcd.print("0 to 7 ");
    key = keypad.getKey();
    while (key != '0' && key != '1' && key != '2' && key != '4' && key != '5' && key != '6' && key != '7') key = keypad.getKey(); // ждем нажатия 0 - 7 для выбора сетевого адреса тарируемого ДУТа (кроме 3 - адрес для станции)
    dutNumber = key - 48;
    lcd.noBlink();
    lcd.setCursor(0, 2);
    // проверка связи с ДУТ
    lcd.print("Check connection DUT");
    lcd.setCursor(0, 3);
    connectDUT();
    if (communicationError == false)
    {
      if (resultN < 50)
      {
        lcd.print("Communication - OK");
        resultNProliv = resultN;
      }
      else {
        tone(SPEAKEROUT, 4000, 400);
        delay(500);
        tone(SPEAKEROUT, 4000, 400);
        lcd.setCursor(0, 1);
        lcd.print("Error N > 50        ");
        lcd.setCursor(0, 2);
        lcd.print("Check fuel level    ");
        communicationError = true;
        lcd.setCursor(0, 3);
        lcd.print("Press #            ");
        lcd.setCursor(19, 3);
        lcd.blink(); // заставляем мигать
        key = keypad.getKey();
        while (key != SELECT) key = keypad.getKey(); // ждем нажатия # для продолжения
        delay(1000);
        lcd.noBlink();
      }
    }
    else
    {
      tone(SPEAKEROUT, 4000, 400);
      delay(500);
      tone(SPEAKEROUT, 4000, 400);
      lcd.clear();
      lcd.print("Communication Error");
      lcd.setCursor(0, 1);
      lcd.print("Check cabel connect");
      lcd.setCursor(0, 3);
      lcd.print("Press #");
      lcd.setCursor(19, 3);
      lcd.blink(); // заставляем мигать
      key = keypad.getKey();
      while (key != SELECT) key = keypad.getKey(); // ждем нажатия # для продолжения
      lcd.noBlink();
    }
  }
  if (typeTarring == MANUAL)
  {
    lcd.print("Select PAUSE(1-6)min");
    lcd.setCursor(19, 3);
    lcd.blink(); // заставляем его мигать
    key = keypad.getKey();
    while (key != '1' && key != '2' && key != '3' && key != '4' && key != '5' && key != '6') key = keypad.getKey(); // ждем нажатия 1, 2, 3, 4, 5 или 6 для выбора длительности паузы между проливами
    lcd.noBlink(); // больше мигать не нужно
    Pause = 60000 * (key - 48);
    tone(SPEAKEROUT, 2000, 200);
  }
  if (communicationError == false || typeTarring == MANUAL)
  {
    lcd.clear();
    lcd.print("Vtank = ");
    lcd.print(Vtank / 10.0, 1);
    lcd.print(" L");
    lcd.setCursor(0, 1);
    lcd.print("Vref  = ");
    lcd.print(Vrefill / 10.0, 1);
    lcd.print(" L");
    lcd.setCursor(0, 2);
    if (typeTarring == AUTO)
    {
      lcd.print("PAUSE = Auto");
    }
    else {
      lcd.print("PAUSE = ");
      lcd.print(key);
      lcd.print(" min");
    }
    lcd.setCursor(0, 3);
    lcd.print("# - OK    * - Reset");
    key = keypad.getKey();
    while (key != SELECT && key != RESET) key = keypad.getKey();
    if (key == SELECT)
    {
      lcd.clear();
      EEPROM_read(0, k_in_Litr);  //чтение значения К, полученного при последней калибровке
      kRefillNum = 1;         // счетчик проливов;
      int  iKey = 0;
      kcountTemp = 0;
      VrefillFfull = 0;     // объем залитого топлива всего в 0,1 литра
      VrefillNum = 0;       // объем залитого топлива в проливе в 0,1 литра
      VinKinLitr = 100.00 / k_in_Litr;  // объем топлива на 1 импульс счетчика в 0,1 литра
      Kcount = 0;
      lcd.clear();
      DataToSendRS232[3] = TARRING << 5 ;
      DataToSendCAN[2] = TARRING;
      lcd.print("Vt=");
      lcd.print(Vtank / 10.00 , 0);
      lcd.print(" L");
      lcd.print(" Vrf=");
      lcd.print(Vrefill / 10.00, 1);
      lcd.print(" L");
      DataToSendRS232[4] =  0;
      DataToSendCAN[0] =  0;
      DataToSendRS232[5] =  0;
      DataToSendCAN[1] =  0;
      tone(SPEAKEROUT, 2000, 200);
      delay(350);
      tone(SPEAKEROUT, 2000, 200);
      timeSendRS485 = millis();
      timeconnectDUT = millis();

      //--------------------------------------------------------------------------------------------------------------------------------------
      while (VrefillFfull - Vtank  < PERELIV)
      { // запас на долив топлива в горловину бака - PERELIV
        digitalWrite(OUT_PUMP, LOW);                      // ВКЛ насоса
        VrefillNum = VinKinLitr * (Kcount - kcountTemp);  // считаем объем залитого в проливе топлива (в 0,1 литрах)
        VrefillFfull = VinKinLitr * Kcount;               // считаем общий объем залитого топлива (в 0,1 литрах)
        if (Vrefill * kRefillNum - VrefillFfull > 0.3 )
        { //условие продолжение очередного пролива
          if (typeTarring == AUTO)
          {
            lcd.setCursor(0, 1);
            lcd.print("N=    ");
            lcd.print("    ");
            lcd.setCursor(5, 1);
            lcd.print(resultN);
            if (millis() - timeSendRS485 > 5000 ) connectRS485();
          }
          lcd.setCursor(0, 2);
          lcd.print("Vrf =");
          lcd.print(VrefillNum / 10 , 2);
          lcd.print(" L");
          lcd.setCursor(14, 2);
          lcd.print("  K-");
          lcd.print(kRefillNum);
          lcd.setCursor(0, 3);
          lcd.print("Vful=");
          lcd.print((VrefillFfull) / 10 , 2);
          lcd.print(" L      ");
          key = keypad.getKey();
          // ручная остановка во время пролива------------------------------------------------------------------------------------------------------
          if ( key == SELECT)
          {
            tone(SPEAKEROUT, 4000, 400);
            digitalWrite(OUT_PUMP, HIGH); // ВЫКЛ насоса
            lcd.setCursor(19, 3);
            lcd.print ("\4");
            lcd.setCursor(19, 3);
            lcd.blink();
            key = keypad.getKey();
            while ( key != SELECT && key != RESET) key = keypad.getKey(); // ждем нажатия # для выхода из режима Пауза
            lcd.noBlink();
            //--------------------------------------------------------------------------------------------------------------------------------------
            //  досрочное прекращение ТАРИРОВКИ по клавише * в режиме Пауза
            if ( key == RESET )
            {
              delay(500);
              break;                    // выход из цикла тарировки
            }
            tone(SPEAKEROUT, 2000, 200);
            delay(350);
            tone(SPEAKEROUT, 2000, 200);
            //--------------------------------------------------------------------------------------------------------------------------------------
            lcd.clear();
            lcd.print("Vt=");
            lcd.print(Vtank / 10.00 , 0);
            lcd.print(" L");
            lcd.print(" Vrf=");
            lcd.print(Vrefill / 10.00, 1);
            lcd.print(" L");
          }
        }
        // окончание очередного пролива --------------------------------------------------------------------------------------------------------------------------------------
        else
        {
          tone(SPEAKEROUT, 2000, 200);
          digitalWrite(OUT_PUMP, HIGH);     // ВЫКЛ насоса
          delay(2000);                      // пауза 1 сек для остановки движения топлива
          unsigned long kcountPause = Kcount;
          delay(1000);                      // пауза 1 сек для подтверждения остановки насоса
          while (kcountPause != Kcount)
          {
            kcountPause = Kcount;
            lcd.setCursor(13, 3);
            lcd.print(" PUMP ");
            tone(SPEAKEROUT, 4000, 400);
            delay(1000);
            lcd.setCursor(13, 3);
            lcd.print("     ");
            tone(SPEAKEROUT, 2000, 200);
            delay(1000);
          }
          VrefillNum = VinKinLitr * (Kcount - kcountTemp);
          VrefillFfull = VinKinLitr * Kcount;  // уточняем объем залитого топлива (в 0,1 литрах)
          lcd.clear();
          lcd.print("Vt=");
          lcd.print(Vtank / 10.00 , 0);
          lcd.print(" L");
          lcd.print(" Vrf=");
          lcd.print(Vrefill / 10.00, 1);
          lcd.print(" L");
          lcd.setCursor(0, 2);
          lcd.print("Vrf =");
          lcd.print(VrefillNum / 10.00 , 2);
          lcd.print(" L");
          lcd.setCursor(14, 2);
          lcd.print("  K-");
          lcd.print(kRefillNum);
          lcd.setCursor(0, 3);
          lcd.print("Vful=");
          lcd.print(VrefillFfull / 10.00 , 2);
          lcd.print(" L");
          // запись данных о величине пролива в массив для передачи в трекер
          int V = (round(VrefillFfull));
          byte *c = (byte*) &V;
          DataToSendRS232[4] = (byte)c[0];
          DataToSendCAN[0] = (byte)c[0];
          DataToSendRS232[5] = (byte)c[1];
          DataToSendCAN[1] = (byte)c[1];
          DataToSendRS232[3] = TARRING << 5 | kRefillNum;
          DataToSendCAN[3] =  kRefillNum;
          // пауза между проливами
          unsigned long i = millis();
          int j = 0;
          while ( millis() < (i + Pause))
          {
            if (typeTarring == MANUAL)
            {
              lcd.setCursor(16, 3);
              lcd.print (Pause / 1000 - j);
              lcd.print(' ');
              lcd.setCursor(19, 3);
              lcd.print ("\4");
              delay(1000);
            }
            else
            {
              connectDUT();
              if (communicationError != true)
              {
                lcd.setCursor(19, 3);
                lcd.print ("\4");
                if (abs(resultNProliv - resultN) < 4)
                {
                  lcd.setCursor(0, 1);
                  lcd.print("N havent changed");
                  lcd.setCursor(19, 3);
                  lcd.print ("\4");
                  tone(SPEAKEROUT, 4000, 400);
                  delay(350);
                  tone(SPEAKEROUT, 4000, 400);
                  delay(350);
                  tone(SPEAKEROUT, 4000, 400);
                  delay(5000);
                  lcd.setCursor(0, 1);
                  lcd.print("                 ");
                }
                else
                {
                  delay(3000);
                  connectDUT();
                  delay(3000);
                  connectDUT();
                  if (abs(ArrayDUT[0] - ArrayDUT[1]) < 4 && abs(ArrayDUT[1] - ArrayDUT[2]) < 4 && abs(ArrayDUT[0] - ArrayDUT[2]) < 2)
                  {
                    resultN = (ArrayDUT[0] + ArrayDUT[1] + ArrayDUT[2]) / 3;
                    resultNProliv = resultN;
                    break;
                  }
                }
                lcd.setCursor(0, 1);
                lcd.print("N=   ");
                lcd.setCursor(5, 1);
                lcd.print(resultN);
                delay(3000);
              }
              else {
                lcd.setCursor(0, 1);
                //                lcd.print("  Connection error  ");
                lcd.setCursor(19, 3);
                lcd.print ("\4");
                tone(SPEAKEROUT, 4000, 40);
                //                delay(350);
                //                tone(SPEAKEROUT, 4000, 400);
                //                delay(350);
                //                tone(SPEAKEROUT, 4000, 400);
                //                resultN = 5555;
                //                delay(1000);
              }
            }
            j++;
            if (key == SELECT) break;        // ручное продолжение тарировки
            else  key = keypad.getKey();
          }
          tone(SPEAKEROUT, 2000, 200);
          delay(350);
          tone(SPEAKEROUT, 2000, 200);
          kRefillNum++;
          kcountTemp = Kcount;
          VrefillNum = 0;
          lcd.clear();
          lcd.print("Vt=");
          lcd.print(Vtank / 10.00 , 0);
          lcd.print(" L");
          lcd.print(" Vrf=");
          lcd.print(Vrefill / 10.00, 1);
          lcd.print(" L");
        }
      }
      digitalWrite(OUT_PUMP, HIGH);         // ВЫКЛ насоса
      lcd.clear();
      lcd.print("CALIBRATION FINISHED");
      lcd.setCursor(0, 3);
      lcd.print("Vt   =");
      lcd.print(Vtank / 10.00 , 1);
      lcd.print(" L");
      delay(1000);
      unsigned long kcountPause = Kcount;
      delay(1000);
      while (kcountPause != Kcount)
      {
        kcountPause = Kcount;
        lcd.setCursor(13, 3);
        lcd.print(" PUMP ");
        tone(SPEAKEROUT, 4000, 400);
        delay(1000);
        lcd.setCursor(13, 3);
        lcd.print("     ");
        tone(SPEAKEROUT, 4000, 400);
        delay(1000);
      }
      VrefillFfull = VinKinLitr * Kcount;   // объем залитого топлива (в 0,1 литрах)
      lcd.setCursor(0, 2);
      lcd.print("Vful =");
      lcd.print(VrefillFfull / 10.00 , 2);
      lcd.print(" L ");
      lcd.setCursor(0, 1);
      lcd.print("K    =");
      lcd.print(kRefillNum);
      lcd.setCursor(19, 3);
      lcd.blink();
      int V = (round(VrefillFfull));
      byte *c = (byte*) &V;

      DataToSendRS232[4] = (byte)c[0];
      DataToSendCAN[0] = (byte)c[0];
      DataToSendRS232[5] = (byte)c[1];
      DataToSendCAN[1] = (byte)c[1];
      DataToSendRS232[3] = TARRING << 5 | kRefillNum;
      DataToSendCAN[3] =  kRefillNum;

      if (typeTarring == AUTO)
      {
        connectDUT();
        while (abs(ArrayDUT[0] - ArrayDUT[1]) > 4 || abs(ArrayDUT[1] - ArrayDUT[2]) > 4 || abs(ArrayDUT[0] - ArrayDUT[2]) > 2)
        {
          delay(3000);
          tone(SPEAKEROUT, 4000, 40);
          connectDUT();
        }
        resultNProliv = (ArrayDUT[0] + ArrayDUT[1] + ArrayDUT[2]) / 3;
        connectRS485();
      }
      //      key = keypad.getKey();
      //      while ( key != SELECT ) key = keypad.getKey();  // ждем нажатия # для выхода из режима Тарировки
      lcd.noBlink();
      // Проигрывание финальной мелодии
      for (int j = 0; j < 2;) {
        for (int i = 0; i < MAX_COUNT; i++)
        {
          tone_ = melody[i];
          beat = beats[i];
          duration = beat * tempo; // Set up timing
          playTone();
        }
        j++;
      }
      for (int i = 0; i < 5; i++)
      {
        connectRS485();
        delay(100);
      }
    }
    else {};
  }
  key = '0';
}
//==========================================================================================================
//функция настроек параметров бака, количества проливов, выбор типа работы
//(ручной выбор времени паузы или автоматическое продолжение проливов в зависитмости от показаний тарируемого ДУТа в баке)
void setting ()
{
  timeWork = 0;
  digitalWrite(LCD_SCREEN, HIGH);
  tone(SPEAKEROUT, 2000, 200);
  Vtank = 0;
  DataToSendRS232[3] = SETTING << 5;
  DataToSendCAN[2] = SETTING;
  lcd.clear();
  lcd.print("      SETTING");
  //  ввод объема бака
  int Key[5];
  int i = 0;
  int j = 0;
  lcd.setCursor(0, 2);
  lcd.print ("Enter Vt =  L");
  lcd.setCursor(10, 2);
  lcd.blink();
  key = keypad.getKey();
  while (key != SELECT )
  {
    if (key) {
      if (key != 42)
      {
        Key[i] = key - 48;
        i++;
        lcd.print(key);
        lcd.print (" L");
        lcd.setCursor(10 + i, 2);
        if (i > 5)
        {
          delay(2000);
          break;
        };
      }
    }
    if (key == RESET && i > 0)
    {
      j = j - key + 48;
      lcd.setCursor(10 + i - 1, 2);
      lcd.print(" L ");
      lcd.setCursor(10 + i - 1, 2);
      i--;
    }
    key = keypad.getKey();
  }
  for (int j = 1 ;  j <  i + 1; )
  {
    Vtank = Vtank + Key[i - j] * pow(10, j);
    j++;
  }
  Serial.println (Key[i - j]);
  lcd.noBlink();
  //--------------------------------------------------------------------------------------------------------------------------------------
  // ввод количество проливов, регулируя объем каждого пролива
  tone(SPEAKEROUT, 2000, 200);
  lcd.clear();
  lcd.print("      SETTING");
  lcd.setCursor(0, 3);
  lcd.print("\3"" N =");
  lcd.setCursor(0, 2);
  lcd.print("Vrf =");
  key = keypad.getKey();
  while (key != SELECT )
  {
    Vrefill = Vtank / kRefill;
    lcd.setCursor(5, 3);
    lcd.print(kRefill);
    lcd.print(" ");
    lcd.setCursor(5, 2);
    lcd.print(Vrefill / 10.0, 1);
    lcd.print(" L  ");
    lcd.setCursor(19, 3);
    key = keypad.getKey();
    if ( key == DOWN)
    {
      delay(100);
      kRefill = kRefill - 1;
      if (kRefill < 10) kRefill = 10; //минимальное число проливов - 10
    }
    if (key == UP) {
      delay(100);
      kRefill = kRefill + 1;
      if (kRefill > 30) kRefill = 30; //максимальное число проливов - 30
    }
  }
  tone(SPEAKEROUT, 2000, 200);
  //--------------------------------------------------------------------------------------------------------------------------------------
  // выбор ручного или автоматического продолжения проливов
  typeTarring = AUTO;
  lcd.clear();
  lcd.print("SELECT TYPE DUT");
  lcd.setCursor(0, 2);
  lcd.print("1 - Auto");
  lcd.setCursor(0, 3);
  lcd.print("2 - Manual");
  key = keypad.getKey();
  while (key != '1' && key != '2') key = keypad.getKey(); // ждем нажатия 1, 2,
  if (key == '2') typeTarring = MANUAL;
  lcd.clear();
  lcd.print("  SETTING FINISHED");
  lcd.setCursor(0, 2);
  lcd.print("Vt  =");
  lcd.print(Vtank / 10.0, 1);
  lcd.print(" L");
  lcd.setCursor(0, 3);
  lcd.print("Vrf =");
  lcd.print(Vrefill / 10.0 , 1);
  lcd.print(" L");
  DataToSendRS232[4] =  0;
  DataToSendCAN[0] = 0;
  DataToSendRS232[5] =  0;
  DataToSendCAN[1] = 0;
  tone(SPEAKEROUT, 2000, 50);
  delay(300);
  tone(SPEAKEROUT, 2000, 50);
  delay(1000);
  taring ();
}
//==========================================================================================================
//функция калибровки счетчика
void calibr ()
{
  timeWork = 0;
  DataToSendRS232[3] = CALIBR << 5;
  DataToSendCAN[2] = CALIBR;
  digitalWrite(LCD_SCREEN, HIGH);
  tone(SPEAKEROUT, 2000, 200);
  EEPROM_read(0, k_in_Litr);      //чтение значения К, полученного при прошлой калибровке
  lcd.clear();
  lcd.print("    ADJUSTMENT");
  lcd.setCursor(0, 2);
  lcd.print("\3"" K = ");
  lcd.print(k_in_Litr / 10, 2);
  lcd.setCursor(0, 3);
  lcd.print("* - default K=100");
  key = keypad.getKey();
  while (key != SELECT || key != RESET)
  {
    if ( key == RESET )
    {
      k_in_Litr = 1000;
      lcd.setCursor(5, 2);
      lcd.print("      ");
      lcd.setCursor(6, 2);
      lcd.print(k_in_Litr / 10, 2);
    }
    key = keypad.getKey();
    if (key == SELECT) break;
    if ( key == DOWN)
    {
      k_in_Litr = k_in_Litr - 0.1;
      if (k_in_Litr < 500) k_in_Litr = 500;
      lcd.setCursor(6, 2);
      lcd.print(k_in_Litr / 10, 2);
      lcd.print(" ");
      delay(100);
    }
    if ( key == '7')
    {
      k_in_Litr = k_in_Litr - 1;
      if (k_in_Litr < 500) k_in_Litr = 500;
      lcd.setCursor(6, 2);
      lcd.print(k_in_Litr / 10, 2);
      lcd.print(" ");
      delay(100);
    }
    if ( key == UP)
    {
      k_in_Litr = k_in_Litr + 0.1;
      if (k_in_Litr > 2000) k_in_Litr = 2000;
      lcd.setCursor(6, 2);
      lcd.print(k_in_Litr / 10 , 2);
      delay(100);
    }
    if ( key == '1')
    {
      k_in_Litr = k_in_Litr + 1;
      if (k_in_Litr > 2000) k_in_Litr = 2000;
      lcd.setCursor(6, 2);
      lcd.print(k_in_Litr / 10 , 2);
      delay(100);
    }
  }
  EEPROM_write(0, k_in_Litr);   //запись значения К в энергонезависимую память
  delay(500);
  Kcount = 0;
  tone(SPEAKEROUT, 2000, 200);
  lcd.setCursor(0, 3);
  lcd.print("* Save     # Onwards");
  //  lcd.setCursor(0, 2);
  //  lcd.print ("# - Onwards");
  key = keypad.getKey();
  while (key != SELECT && key != RESET) key = keypad.getKey();
  //--------------------------------------------------------------------------------------------------------------------------------------
  // посчет k_in_Litr проливом топлива в мерную емкость
  if (key == SELECT)
  {
    lcd.clear();
    lcd.print("    ADJUSTMENT");
    lcd.setCursor(0, 2);
    lcd.print ("Vfill = 0");
    key = keypad.getKey();
    lcd.blink();
    int i = 0;
    EEPROM_read(0, k_in_Litr);
    //  if (Kcount > 500) k_in_Litr = 100.0;
    // проверка нажатия кнопки select - остановка калибровки
    while (key != SELECT)
    {
      if (i != Kcount)
      {
        lcd.setCursor(8, 2);
        lcd.print (10.00 / k_in_Litr * Kcount, 2);         //выводим результат работы счетчика на дисплей
        i = Kcount;
      }
      key = keypad.getKey();
    }
    tone(SPEAKEROUT, 2000, 200);
    lcd.clear();
    if (Kcount > 500)
    {
      lcd.print("    ADJUSTMENT");
      //    lcd.setCursor(0, 1);
      //    lcd.print ("Adjustment finished");
      lcd.setCursor(0, 2);
      lcd.print ("Enter Vref = ");
      Vrefill = 100.00 / k_in_Litr * Kcount;
      lcd.setCursor(13, 2);
      lcd.print(Vrefill / 10, 2);
      key = keypad.getKey();
      while (key != SELECT)
      {
        key = keypad.getKey();
        if ( key == DOWN)
        {
          lcd.setCursor(13, 2);
          Vrefill = Vrefill - 0.1;
          lcd.print(Vrefill / 10, 2);
          lcd.print(" ");
          delay(100);
        }
        if ( key == '7')
        {
          lcd.setCursor(13, 2);
          Vrefill = Vrefill - 1;
          lcd.print(Vrefill / 10, 2);
          lcd.print(" ");
          delay(100);
        }
        if ( key == UP)
        {
          lcd.setCursor(13, 2);
          Vrefill = Vrefill + 0.1;
          lcd.print(Vrefill / 10, 2);
          delay(100);
        }
        if ( key == '1')
        {
          lcd.setCursor(13, 2);
          Vrefill = Vrefill + 1;
          lcd.print(Vrefill / 10, 2);
          delay(100);
        }
      }
      k_in_Litr = (100.0 * Kcount) / Vrefill ;
    }
    else ;
    lcd.setCursor(0, 3);
    lcd.print("New K = ");
    lcd.print(k_in_Litr / 10 , 2);
    EEPROM_write(0, k_in_Litr);   //запись значения К в энергонезависимую память
    delay(2000);
    key = keypad.getKey();
    while (key != SELECT)  key = keypad.getKey(); // проверка нажатия кнопки select - остановка калибровки
    key = keypad.getKey();
    lcd.noBlink();
    tone(SPEAKEROUT, 2000, 200);
    delay(300);
    tone(SPEAKEROUT, 2000, 200);
    delay(2000);
  }
  else ;
  key = keypad.getKey();
}
//--------------------------------------------------------------------------------------------------------------------------------------
// режим счетчика
void counter()
{
  timeWork = 0;
  digitalWrite(LCD_SCREEN, HIGH);
  tone(SPEAKEROUT, 2000, 200);
  lcd.clear();
  EEPROM_read(0, k_in_Litr);                    //чтение значения К, полученного при последней калибровке
  float VrefillFfull = 0;                       // объем залитого топлива всего в 0,1 литра
  float VrefillNum = 0;                         // объем залитого топлива между нажатиями RESET в 0,1 литра
  VinKinLitr = 100.00 / k_in_Litr;  // объем топлива на 1 импульс счетчика в 0,1 литра
  Kcount = 0;
  DataToSendRS232[3] = COUNTER << 5 ;
  DataToSendCAN[2] = COUNTER;
  long kcountTemp = 0;
  lcd.setCursor(0, 1);
  lcd.print("Vref = ");
  lcd.setCursor(0, 3);
  lcd.print("Vfull= ");

  while (key != SELECT)
  { // проверка нажатия кнопки select - остановка работы счетчика
    key = keypad.getKey();
    VrefillNum = VinKinLitr * (Kcount - kcountTemp); // считаем объем залитого в проливе топлива (в 0,1 литрах)
    VrefillFfull = VinKinLitr * Kcount;              // считаем общий объем залитого топлива (в 0,1 литрах)
    int V = (round(VrefillFfull));
    byte *c = (byte*) &V;
    DataToSendRS232[4] = (byte)c[0];
    DataToSendCAN[0] = (byte)c[0];
    DataToSendRS232[5] = (byte)c[1];
    DataToSendCAN[1] = (byte)c[1];
    lcd.setCursor(7, 1);
    lcd.print(VrefillNum / 10.00 , 2);
    lcd.print(" L   ");
    lcd.setCursor(7, 3);
    lcd.print(VrefillFfull / 10.00, 2);
    lcd.print(" L");
    if (key == RESET)
    { // обнуление Vref
      tone(SPEAKEROUT, 2000, 200);
      kcountTemp = Kcount;
    }
    if (typeTarring == AUTO)
    {
      lcd.setCursor(14, 2);
      if (millis() - timeconnectDUT > 10000) connectDUT();
      if (!communicationError)
      {
        lcd.print ("N=    ");
        lcd.setCursor(16, 2);
        lcd.print(resultN);
      }
      else
      {
        lcd.setCursor(16, 2);
        lcd.print("----");
      }
    }
  }
  lcd.setCursor(19, 3);
  lcd.blink(); // заставляем его мигать
  key = keypad.getKey();
  while (key != SELECT) key = keypad.getKey();    // проверка нажатия кнопки select - выход в основное меню
  lcd.noBlink();
  tone(SPEAKEROUT, 2000, 200);
  delay(300);
  tone(SPEAKEROUT, 2000, 200);
}
//==========================================================================================================
// функция подсчета импульсов с ДАРТ
void rpm_fun()
{
  //обновляем счетчик
  if (digitalRead (IN_KCOUNT) == LOW) Kcount++; // увеличиваем на  единицу
}
//==========================================================================================================
// функция вычисления контрольной суммы
uint8_t crc8( uint8_t *addr, uint8_t len)
{
  uint8_t crc = 0;
  while (len--) {
    crc = pgm_read_byte(dscrc_table + (crc ^ *addr++));
  }
  return crc;
}
//==========================================================================================================
// переодическая оправка данных в трекер по RS232 и CAN
void Timer1_action()
{
  DataToSendRS232[8] =  crc8(DataToSendRS232, DATA_LENGTH);
  Serial.write(DataToSendRS232, 9);
  connectCAN();
}
//  оправка данных в трекер по CAN
void connectCAN()
{
  byte *c = (byte*) &resultNProliv;             // передача N тарируемого ДУТа
  DataToSendCAN[4] = (byte)c[0];
  DataToSendCAN[5] = (byte)c[1];
  CAN0.sendMsgBuf(0x0CFE6CEE, 1, 8, DataToSendCAN);
//  CAN0.sendMsgBuf(0x0CFE6C00, 1, 8, DataToSendCANVNC);
}
//==========================================================================================================
//  оправка данных в трекер по RS485 сетевой адрес 3
void connectRS485()
{
  DataToSendRS485[3] = DataToSendRS232[3];
  byte *c = (byte*) &resultNProliv;             // передача N, вычисленного станцией как среднее между 3 последними уровнями
  DataToSendRS485[4] = (byte)c[0];
  DataToSendRS485[5] = (byte)c[1];
  DataToSendRS485[8] =  crc8(DataToSendRS485, DATA_LENGTH);
  for (int i = 0; i < 60; i++)
  {
    delay(100);
    while (Serial1.available())
    {
      if (Vrefill * kRefillNum - VinKinLitr * Kcount < 0.1 ) digitalWrite(OUT_PUMP, HIGH);         // ВЫКЛ насоса
      if (Serial1.read() == 0x31)
        if (Serial1.read() == 0x3)
        { // проверяем, что запрос к нам
          delay(5);
          digitalWrite(SerialTxControl, RS485Transmit);  // разрешаем передачу данных в порт
          Serial1.write(DataToSendRS485, 9);
          delay(5);
          digitalWrite(SerialTxControl, RS485Receive);  // разрешаем чтение данных из порта
          goto RS485out;
        }
    }
  }
RS485out:;
  timeSendRS485 = millis();
}
//==========================================================================================================
// запрос на выдачу данных и чтeние данных от тарируемого ДУТа
void connectDUT()
{
  byte bufferRead485[9] = {62, 0, 0, 255, 0, 0, 0, 0, 0}; // обнуление массива данных от ДУТа
  communicationError = true;
  unsigned long tiMe = millis();
  byte RS485TransmitArray[] = {0x31, dutNumber, 0x6, 0xa8};
  while (millis() - tiMe < 3000)
  {
    digitalWrite(SerialTxControl, RS485Transmit);               // разрешаем передачу данных в порт
    Serial1.write(RS485TransmitArray, 4);
    delay(5);
    digitalWrite(SerialTxControl, RS485Receive);                // разрешаем чтение данных из порта
    while (Serial1.available())
    {
      byte buf = Serial1.read();
      if (buf == 0x3E) {                                       // заголовок пакета с данными об уровне топлива
        for (int i = 0; i < 9; i++)
        { //запись данных в массив
          bufferRead485[i] = buf;
          buf = Serial1.read();
        }
      }
    }
    if (bufferRead485[8] == crc8(bufferRead485, DATA_LENGTH))
    { //проверка контрольной суммы
      if (bufferRead485[5] << 8 | bufferRead485[4] != 4096) resultN = bufferRead485[5] << 8 | bufferRead485[4];
      ArrayDUT[indexArrayDUT] = resultN;
      indexArrayDUT++;
      if (indexArrayDUT == 3) indexArrayDUT = 0;
      communicationError = false;
      digitalWrite(SerialTxControl, RS485Receive);             // разрешаем чтение данных из порта
      break;
    }
    delay(250);
    if (Vrefill * kRefillNum - VinKinLitr * Kcount < 0.1 ) digitalWrite(OUT_PUMP, HIGH);         // ВЫКЛ насоса
  }
  timeconnectDUT = millis();
}
//==========================================================================================================
// проигрывание финальной мелодии
void playTone()
{
  long elapsed_time = 0;
  if (tone_ > 0)
  { // if this isn't a Rest beat, while the tone has
    //  played less long than 'duration', pulse speaker HIGH and LOW
    while (elapsed_time < duration)
    {
      digitalWrite(SPEAKEROUT, HIGH);
      delayMicroseconds(tone_ / 2);
      digitalWrite(SPEAKEROUT, LOW);
      delayMicroseconds(tone_ / 2);
      // Keep track of how long we pulsed
      elapsed_time += (tone_);
    }
  }
  else
  { // Rest beat; loop times delay
    for (int j = 0; j < rest_count; j++)
    { // See NOTE on rest_count
      delayMicroseconds(duration);
    }
  }
}
