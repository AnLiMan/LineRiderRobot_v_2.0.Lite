//-------------------Все постоянные-----------------

//---Постоянные автомобиля---
#define L 0.02 //Расстояние между передним и задним колесом (м)
#define dr 0.01 // Расстояние между задними колесами (м)
#define lr 0.01 // Расстояние между задним колесом и центром тяжести (м)
#define r 0.003 // Радиус колеса (м)
#define K 0.01 // Расстояние между правым и левым kingpin (м)

#define MinAngleSensetivity -5 // Минимальный угол нечувствительности руля в градусах
#define MaxAngleSensetivity 5 // Максимальный угол нечувствительности руля в градусах

#define BlinkPeriodForTurnSignal 1000 //Период моргания поворотников
#define MaxModes 3 //Количество режимов

//-----Выводы------

#define Button1 2 // Кнопка 1 (перключение режимов)
#define Button2 7 // Кнопка 2 (разрешение работы)

#define LeftTurnSignal 0 // Левый поворотник
#define RightTurnSignal 8 // Правый поворотник

#define LeftMotor 10 // Пин для движения левого двигателя 
#define RightMotor 9 // Пин для движения правого двигателя 

#define RightMotorAdj 1 // Подстройка скорости правого двигателя
#define LeftMotorAdj 1 // Подстройка скорости левого двигателя

#define BlueChannelRGB 11 // Синий канал RGB-светодиода
#define GrennChannelRGB 12 // Зелёный канал RGB-светодиода
#define RedChannelRGB 13 // Красный канал RGB-светодиода

#define StearingValue A0 // Сигнал с потенциометра руля
#define Accelerator A1 // Установка скорости (педаль газа)

#define InfraredSensorLeft A2 // Инфракрасный датчик слева 
#define InfraredSensorRight A3 // Инфракрасный датчик справа
#define InfraredSensorStraight A4 // Инфракрасный датчик посередине

#define DeltaT 1000 // Время отработки пары значений в массиве заготовленного пути

//---------------Все переменные------------

bool Buttflag1 = false; //Флаг нажатия кнопки 1
bool Buttflag2 = false; //Флаг нажатия кнопки 2
volatile int counter = 0;  // Переменная-счётчик для переключения режимов
volatile bool OnWork = false;  // Переменная разрешения на работу

//Для электронного дифференциала
float Omega1; //Скорость вращения 1
float Omega2; //Скорость вращения 2
float LinearIdleSpeed = 0.75; //Скорость машинки, линейная (м/c) по дефолту указана максимальная
int idleSpeed; //Скорость движения машинки

uint32_t myTimer1; // Переменная хранения времени 1
uint32_t myTimer2; // Переменная хранения времени 2

// Массив пути для демострации, каждое первое число значение ШИМ 
//(от 0 до 255) для левого двигателя, каждое второе для правого
byte ArrayOfTheWay[] = {110, 110, 110, 180, 180, 110, 255, 110, 255, 110, 110, 255, 110, 255, 255, 255, 110, 130, 110, 130, 110, 130, 130, 110, 130, 110, 110, 255, 255, 110, 255, 110, 110, 255};

//----------------------//
//--------Setup-------
//---------------------//

void setup() {

  Serial.begin(9600); // Серийный порт для отладки

  //Законфигурируем порты
  pinMode(Button1, INPUT_PULLUP);
  pinMode(Button2, INPUT_PULLUP);

  pinMode(LeftTurnSignal, OUTPUT);
  pinMode(RightTurnSignal, OUTPUT);

  pinMode(LeftMotor, OUTPUT);
  pinMode(RightMotor, OUTPUT);

  pinMode(BlueChannelRGB, OUTPUT);
  pinMode(GrennChannelRGB, OUTPUT);
  pinMode(RedChannelRGB, OUTPUT);

  //Выключаем двигатели
  digitalWrite(LeftMotor, LOW);
  digitalWrite(RightMotor, LOW);

  //Подключаем прерывания
  attachInterrupt(digitalPinToInterrupt(Button1), Button1Tick, FALLING); //Подключаем прерывание для кнопки 1
  attachInterrupt(digitalPinToInterrupt(Button2), Button2Tick, FALLING); //Подключаем прерывание для кнопки 2

  //Разгоняем ШИМ на пинах D9 и D10 до 7.8 кГц
  TCCR1A = 0b00000001;  // 8bit
  TCCR1B = 0b00001010;  // x8 fast pwm
}

//---------------------------//
//-------Основной цикл--------
//---------------------------//

void loop() {
  Buttflag1 = false; //Сброс флага кнопки 1
  Buttflag2 = false; //Сброс флага кнопки 2
  CurrentMode(counter); //Отображение текущего режима

  //Работа разрешена
  if (OnWork) {
    //Включение разных режимов работы, в зависимости от значения текущей переменной
    switch (counter) {
      //1.1. Ручное управление. Индикатор - красный цвет светодиода
      case 0:
        ManualControl();
        break;
      // 1.2. Движение по линии. Индикатор - зелёный цвет светодиода
      case 1:
        idleSpeed = map(analogRead(Accelerator), 0, 1019, 0, 255); // Преобразуем входное значение сопротивления к новому диапазону
        LineRider();
        break;
      //1.3. Движение по памяти. Индикатор - синий цвет светодиода
      case 2:
        idleSpeed = map(analogRead(Accelerator), 0, 1019, 0, 255); // Преобразуем входное значение сопротивления к новому диапазону
        RideByMemory();
        break;
    }
  }
  //Если работа не разрешена, переходим в режим ожидания
  else {
    StandBy();
  }
}

//--------------------------------------------------------------//
//-----------------1. Основные режимы работы-------------
//--------------------------------------------------------------//

//------------1.1. Ручное управление-----------------
void ManualControl() {
  idleSpeed = map(analogRead(Accelerator), 0, 1019, 0, 255); // Преобразуем входное значение сопротивления к новому диапазону
  int SteeringAngle = map(analogRead(StearingValue), 1023, 0, -60, 60); // Преобразуем входное значение сопротивления к новому диапазону

  //Если едем прямо
  if (MinAngleSensetivity < SteeringAngle && SteeringAngle < MaxAngleSensetivity) {
    analogWrite(RightMotor, int(idleSpeed * RightMotorAdj));
    analogWrite(LeftMotor, int (idleSpeed * LeftMotorAdj));
    //Выключаем поворотники
    digitalWrite(LeftTurnSignal, LOW);
    digitalWrite(RightTurnSignal, LOW);
  }

  //Поворот вправо
  else if (SteeringAngle > MaxAngleSensetivity) {
    analogWrite(LeftMotor, int (idleSpeed * LeftMotorAdj));
    ElectronicDiff(SteeringAngle); //Расчитаем скорости
    int RightMotorSpeed = idleSpeed * (Omega2 / Omega1); //Корректируем скорость правого двигателя на Omega2/Omega1
    analogWrite(RightMotor, int (RightMotorSpeed * RightMotorAdj));
    BlinkTurnLight(LeftTurnSignal, RightTurnSignal); //Добавим индикации
  }

  //Поворот налево
  else if (SteeringAngle < MinAngleSensetivity) {
    analogWrite(RightMotor, int(idleSpeed * RightMotorAdj));
    ElectronicDiff(SteeringAngle); //Расчитаем скорости
    int LeftMotorSpeed = idleSpeed *  (Omega2 / Omega1); //Корректируем скорость левого двигателя на Omega2/Omega1 (0.8 для гипертрофированного поворота)
    analogWrite(LeftMotor, int (LeftMotorSpeed * LeftMotorAdj));
    BlinkTurnLight(RightTurnSignal, LeftTurnSignal); //Добавим индикации
  }
}

//------------1.2. Движение по линии-----------------
void LineRider() {

  //Считаем значения с датчиков
  byte InfraredSensorRightState = digitalRead(InfraredSensorRight); // 1 - белый, чёрный - 0
  byte InfraredSensorLeftState = digitalRead(InfraredSensorLeft); // 1 - белый, чёрный - 0
  byte InfraredSensorStraightState = digitalRead(InfraredSensorStraight); // 1 - белый, чёрный - 0


  // Если датчик посередине на черной линии
  if (InfraredSensorStraightState == 0) {
    analogWrite(LeftMotor, int (idleSpeed * LeftMotorAdj));
    analogWrite(RightMotor, int(idleSpeed * RightMotorAdj));
  }

  // Иначе
  else {
    //Поворот налево
    if (InfraredSensorRightState == 1 && InfraredSensorLeftState == 0) {
      digitalWrite(LeftMotor, LOW);
    }

    //Поворот направо
    else if (InfraredSensorRightState == 0 && InfraredSensorLeftState == 1) {
      digitalWrite(RightMotor, LOW);
    }
  }
}

//------------1.3. Движение по памяти-----------------
void RideByMemory () {

  for (int i = 0; i < (sizeof(ArrayOfTheWay));) {

    analogWrite(LeftMotor, int (ArrayOfTheWay[i] * LeftMotorAdj));
    analogWrite(RightMotor, int (ArrayOfTheWay[i + 1] * RightMotorAdj));

    if (millis() - myTimer2 >= DeltaT) {
      myTimer2 += DeltaT;
      i += 2;
    }
  }
}


//--------------------------------------------------------------//
//----------------2 Функции отработки прерываний--------------
//--------------------------------------------------------------//

// --2.1. Отработка нажатия первой кнопки (для переключения режимов)---
void Button1Tick () {
  if (!Buttflag1) {
    Buttflag1 = true;
    counter++;
    if (counter >= MaxModes) {
      counter = 0;
    }
  }
}

//--2.2. Отработка нажатия второй кнопки (включение/отключение текущего режима)---
void Button2Tick () {
  if (!Buttflag2) {
    Buttflag2 = true;
    OnWork = !OnWork; //Инвертируем переменную разрешения работы
  }
}

//--------------------------------------------------------------//
//-----------------3. Иные вспомогательные функциии--------------
//--------------------------------------------------------------//

//-----3.1. Функция отображения текущего режима----
void CurrentMode(int currentMode) {
  switch (counter) {
    case 0:
      digitalWrite(RedChannelRGB, HIGH);
      digitalWrite(GrennChannelRGB, LOW);
      digitalWrite(BlueChannelRGB, LOW);
      break;
    case 1:
      digitalWrite(RedChannelRGB, LOW);
      digitalWrite(GrennChannelRGB, HIGH);
      digitalWrite(BlueChannelRGB, LOW);
      break;
    case 2:
      digitalWrite(RedChannelRGB, LOW);
      digitalWrite(GrennChannelRGB, LOW);
      digitalWrite(BlueChannelRGB, HIGH);
      break;
  }
}

//------3.2. Функция электронного дифференциала----
void ElectronicDiff(int Angle) {
  float AngleRad = Angle / 57.32; //Переведём градусы в радианы
  float tanAngle = tan(fabs(AngleRad));
  float delta1 = atan((L / (L / tanAngle - K / 2)));
  float delta2 = atan((L / (L / tanAngle + K / 2)));
  float R1 = L / sin(delta1);
  float R2 = L / sin(delta2);
  float R3 = L / tanAngle - dr / 2;
  float Rcg = sqrtf(R3 + dr * dr / 4 + lr * lr);
  Omega1 = (LinearIdleSpeed * R1) / (Rcg * r);
  Omega2 = (LinearIdleSpeed * R2) / (Rcg * r);
}

//------3.3. Функция моргания поворотниками-----
void BlinkTurnLight (int LightOff, int LightOn) {
  digitalWrite(LightOff, LOW);
  if (millis() - myTimer1 >= BlinkPeriodForTurnSignal) {
    myTimer1 += BlinkPeriodForTurnSignal;
    digitalWrite(LightOn, !digitalRead(LightOn));
  }
}

//------3.4. Функция отключения текущего режима и выход в ожидание-----
void StandBy() {
  //Выключаем свет
  digitalWrite(LeftTurnSignal, LOW);
  digitalWrite(RightTurnSignal, LOW);

  //Выключаем двигатели
  digitalWrite(LeftMotor, LOW);
  digitalWrite(RightMotor, LOW);
}
