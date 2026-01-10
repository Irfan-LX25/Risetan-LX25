// Pemanggilan pustaka Wire untuk komunikasi I2C
#include <Wire.h>

// Definisi pin-pin yang digunakan 
//motor steer
#define pin1 PA3
#define pin2 PA1
#define ENAM1 PB8
#define ENBM1 PB9

// Motor Drive
#define pin3 PA6
#define pin4 PA7
#define ENAM2 PB7
#define ENBM2 PB6

#define pinHall PB5 // Pin sensor hall effect
#define pinLed PC13 // Pin LED indikator

// Alamat I2C Slave dan Ukuran data
#define I2C_SDA PB11
#define I2C_SCL PB10
#define SLAVE_ADDR 11
#define DATA_SIZE 20

//TwoWire Wire2(PB11, PB10); //SDA, SCL
//Konstanta PD Steer
const double kp_steer = 1.175; // konstanta proporsional untuk steering
const double kd_steer = 0.472; // konstanta derivatif untuk steering

//konstanta PID Drive
const double kp_drive = 0.09; // konstanta proporsional untuk drive
const double kd_drive = 0.00059; // konstanta derivatif untuk drive
const double ki_drive = 0.0905; // konstanta integral untuk drive

//variabel Steer
volatile long encoderCount1 = 0;
double lastError_steer = 0;
short int degree = 0;

//variabel Drive
short int speed = 0;
double lastRPM = 0;
double eIntegral_drive = 0;
volatile long encoderCount2 = 0;

//konstanta umum
const double PPR_steer = 330; //pulse per revolution motor steer
const double GEAR_RATIO_steer = 2.2285714; //gear ratio motor steer
const double PPR_drive = 209; //pulse per revolution motor drive
const double GEAR_RATIO_drive = 0.432; //gear ratio motor drive
const double SAMPLING_TIME = 100; //waktu sampling dalam ms


// Buffer untuk menyimpan data yang diterima dan dikirim, ditambah 1 untuk null terminator
char receiveData[DATA_SIZE + 1];

//waktu pengaturan interval
unsigned long lastTimePID_steer = 0;
unsigned long lastTimePID_drive = 0;
unsigned long lastTimeI2C = 0;

void setup() {
    Serial.begin(115200);
  // Inisialisasi komunikasi I2C sebagai Slave
//    Wire2.begin(SLAVE_ADDR);
//    Wire2.onReceive(receiveEvent);
// Setup I2C dengan pin custom
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin(SLAVE_ADDR);
  Wire.onReceive(receiveEvent);

  // Inisialisasi pin-pin motor sebagai output
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(ENAM1, OUTPUT);
    pinMode(ENBM1, OUTPUT);

    pinMode(pin3, OUTPUT);
    pinMode(pin4, OUTPUT);
    pinMode(ENAM2, OUTPUT);
    pinMode(ENBM2, OUTPUT);

    pinMode(pinHall, INPUT);
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, HIGH); // LED mati

  // Inisialisasi interrupt untuk encoder motor steer
    attachInterrupt(digitalPinToInterrupt(PA0), EN_steer, RISING);
  // Inisialisasi interrupt untuk encoder motor drive
    attachInterrupt(digitalPinToInterrupt(PA5), EN_drive, RISING);

  //program untuk melakukan homing zero position pada roda steer
    while (digitalRead(pinHall) != LOW) {
        short int dirA = (degree >= 0)? 30 : 0;
        short int dirB = (degree >= 0)? 0 : 30;
        analogWrite(pin1, dirA);
        analogWrite(pin2, dirB);
    }
    degree = 0;
    digitalWrite(PC13, LOW); // LED menyala menandakan homing selesai
    analogWrite(pin1, 0);
    analogWrite(pin2, 0);
    delay(1000);
    encoderCount1 = 0;
    encoderCount2 = 0;
}

void loop() {
    unsigned long currentTime = millis();
  // I2C Communication
    if(currentTime - lastTimeI2C >= 100) {
        lastTimeI2C = currentTime;
        Serial.print("Current Speed: ");
        Serial.print(speed);
        Serial.print(" | Current Degree: ");
        Serial.println(degree);
    }
  // PID Steer
    if (currentTime - lastTimePID_steer >= 100) {
        TaskControlAngle((double)degree);
        lastTimePID_steer = currentTime;
    }

  // PID Drive
    if (currentTime - lastTimePID_drive >= 100) {
        TaskControlSpeed(speed);
        lastTimePID_drive = currentTime;
    }
}

/*
    Data setpoint sudut akan dikonversikan ke dalam derajat
    per pulsa. Nilai error kontrol PD didapat dari nilai 
    derajat per pulsa dikurangi nilai pulsa encoder. Hasil
    dari output PD akan dijadikan input PWM pada motor steer
*/
void TaskControlAngle(float setpoint) {
    static double PWM_L = 0;
    static double PWM_R = 0;
    int degreesPerPulse = (setpoint/360.0) * (PPR_steer * GEAR_RATIO_steer);
    double error = degreesPerPulse - encoderCount1;
    double eDerivative_steer = error - lastError_steer;
    double pdoutput_steer = (kp_steer * error) + (kd_steer * eDerivative_steer);
    pdoutput_steer = constrain(pdoutput_steer, -255, 255);
    PWM_L = (pdoutput_steer >= 0) ? 0 : abs (pdoutput_steer);
    PWM_R = (pdoutput_steer >= 0) ? abs (pdoutput_steer) : 0;
    analogWrite(pin1, PWM_L);
    analogWrite(pin2, PWM_R);
    lastError_steer = error;
}

/*
    Nilai pulsa dari pembacaan encoder akan dikonversikan ke dalam RPM.
    Nilai error kontrol PID didapat dari nilai setpoint dikurangi RPM yg dibaca encoder.
    Hasil dari output PID akan dijadikan input PWM pada motor drive
*/
void TaskControlSpeed(short int setpoint) {
    static double PWM_L = 0;
    static double PWM_R = 0;
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    double rev = (double)encoderCount2 / (PPR_drive * GEAR_RATIO_drive);
    double currentRPM = (rev / (SAMPLING_TIME / 1000)) / 60.0;
    encoderCount2 = 0; // Reset encoder count for next calculation
    double error = setpoint - currentRPM;
    eIntegral_drive += error;
    double eDerivative_drive = currentRPM - lastRPM;
    double pidoutput_drive = (kp_drive * error) + (ki_drive * eIntegral_drive) - (kd_drive * eDerivative_drive);
    pidoutput_drive = constrain(pidoutput_drive, -255, 255);
    if (setpoint !=0) {
        PWM_L =(pidoutput_drive >= 0) ? abs (pidoutput_drive): 0;
        PWM_R =(pidoutput_drive >= 0) ? 0 : abs (pidoutput_drive);
    } else {
        PWM_L = 0;
        PWM_R = 0;
        pidoutput_drive = 0;
        eIntegral_drive = 0;
        lastRPM = 0;
    }
    analogWrite(pin3, PWM_L);
    analogWrite(pin4, PWM_R);
    lastRPM = currentRPM;
}


//Program untuk akumulasi pulsa dari encoder motor steer
void EN_steer() {
    if (digitalRead(ENBM1) == LOW) {
        encoderCount1++;
    } else {
        encoderCount1--;
    }
}
// program untuk akumulasi pulsa dari encoder motor drive
void EN_drive() {
    if (digitalRead(ENBM2) == LOW) {
        encoderCount2++;
    } else {
        encoderCount2--;
    }
}

/*
    Program untuk menerima data setpoint dari I2C Master. Data yang baru diterima
    kemudian akan dilakukan parsing untuk memisahkan nilai setpoint sudut dan kecepatan
    Format: Speed#Degree (contoh: 100#45)
*/
void receiveEvent(int DataSize) {
    // Clear buffer
    memset(receiveData, 0, DATA_SIZE + 1);
    
    int index = 0;
    while (Wire.available() && index < DATA_SIZE) {
        receiveData[index++] = Wire.read();
    }
    receiveData[index] = '\0'; // Null terminator
    
    // Debug: print received data
    Serial.print("Received: ");
    Serial.println(receiveData);
    
    // Parse data format: Speed#Degree
    char* token = strtok(receiveData, "#");
    if (token != NULL) {
        speed = atoi(token);
        Serial.print("Speed: ");
        Serial.println(speed);
        
        token = strtok(NULL, "#");
        if (token != NULL) {
            degree = atoi(token);
            Serial.print("Degree: ");
            Serial.println(degree);
        }
    } else {
        Serial.println("Parse error: delimiter not found");
    }
}
