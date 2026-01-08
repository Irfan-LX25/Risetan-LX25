/*---------------------------------------------------------------------------------------------------------*/
/*-------------------------STM32 BLUEPILL SLAVE MOTOR CONTROLLER WITH HARDWARE ENCODER--------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*--------------------------------------Source Code by LEXARGA-25 TEAM-------------------------------------*/
/*-----------------------------------Modified & Adapted by LEXARGA-25 TEAM---------------------------------*/
/*----------------------------------------------------V1.0-------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
/*------------------------------------LAST UPDATE AT 20:20:00, 08 JAN 26-----------------------------------*/

#include <Wire.h>

// ========================================== I2C SLAVE ADDRESS ============================================
const uint8_t I2C_ADDRESS = 0x08;  // Ganti ke 0x09 untuk Slave 2

// ========================================== PIN DEFINITIONS (STM32 BLUEPILL) =============================
// Motor 1 (FL di Slave 1, RL di Slave 2)
#define MOTOR1_PWM PA_8
#define MOTOR1_IN1 PA_9
#define MOTOR1_IN2 PA_10
#define ENC1_A PA_0   // Timer 2 Channel 1
#define ENC1_B PA_1   // Timer 2 Channel 2

// Motor 2 (FR di Slave 1, RR di Slave 2)
#define MOTOR2_PWM PB_0
#define MOTOR2_IN1 PB_1
#define MOTOR2_IN2 PB_10
#define ENC2_A PA_6   // Timer 3 Channel 1
#define ENC2_B PA_7   // Timer 3 Channel 2

// ========================================== INDIVIDUAL PID CONSTANTS =====================================
struct MotorConfig {
    float Kp;
    float Ki;
    float Kd;
    int PPR;
    int maxRPM;
    float integralLimit;
    int minPWM;      // Minimum PWM untuk mengatasi friksi
    int deadband;    // Deadband untuk mencegah motor buzzing
};

// Konfigurasi individual untuk setiap motor
MotorConfig motor1Config = {0.8, 1.2, 0.05, 326, 200, 100.0, 30, 5};
MotorConfig motor2Config = {0.8, 1.2, 0.05, 326, 200, 100.0, 30, 5};

// ========================================== PID VARIABLES ================================================
struct PIDData {
    float targetRPM;
    float currentRPM;
    float error;
    float lastError;
    float integral;
    float derivative;
    uint8_t outputPWM;
    bool direction;  // true = forward, false = reverse
    unsigned long lastPulseTime;
    volatile long encoderCount;
    long lastEncoderCount;
    unsigned long lastRPMTime;
    int timerCount;
};

PIDData motor1, motor2;

// ========================================== I2C COMMUNICATION ============================================
uint8_t receivedData[4];  // [RPM1, DIR1, RPM2, DIR2]
unsigned long lastI2CTime = 0;
const unsigned long I2C_TIMEOUT_MS = 500;  // ms - Nama diubah untuk hindari konflik

// ========================================== ENCODER VARIABLES ============================================
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
void readEncoder1A();
void readEncoder1B();
void readEncoder2A();
void readEncoder2B();

// ========================================== ENCODER INTERRUPT FUNCTIONS ==================================
void readEncoder1A() {
    if (digitalRead(ENC1_A) == digitalRead(ENC1_B)) {
        encoder1Count++;
    } else {
        encoder1Count--;
    }
}

void readEncoder1B() {
    if (digitalRead(ENC1_A) != digitalRead(ENC1_B)) {
        encoder1Count++;
    } else {
        encoder1Count--;
    }
}

void readEncoder2A() {
    if (digitalRead(ENC2_A) == digitalRead(ENC2_B)) {
        encoder2Count++;
    } else {
        encoder2Count--;
    }
}

void readEncoder2B() {
    if (digitalRead(ENC2_A) != digitalRead(ENC2_B)) {
        encoder2Count++;
    } else {
        encoder2Count--;
    }
}

// ========================================== ENCODER READING FUNCTIONS ====================================
void updateEncoderCounts() {
    unsigned long currentTime = millis();
    
    // Baca encoder setiap 10ms untuk RPM calculation
    if (currentTime - motor1.lastRPMTime >= 10) {
        motor1.lastRPMTime = currentTime;
        
        // Hitung delta encoder count
        long currentCount1 = encoder1Count;
        long delta1 = currentCount1 - motor1.lastEncoderCount;
        motor1.lastEncoderCount = currentCount1;
        
        // Hitung RPM: RPM = (delta_pulses / PPR) * (60000 / sample_ms)
        if (motor1.targetRPM != 0) {  // Hanya hitung RPM jika motor aktif
            motor1.currentRPM = (delta1 * 60000.0) / (motor1Config.PPR * 10.0);
        } else {
            motor1.currentRPM = 0;
        }
    }
    
    if (currentTime - motor2.lastRPMTime >= 10) {
        motor2.lastRPMTime = currentTime;
        
        long currentCount2 = encoder2Count;
        long delta2 = currentCount2 - motor2.lastEncoderCount;
        motor2.lastEncoderCount = currentCount2;
        
        if (motor2.targetRPM != 0) {
            motor2.currentRPM = (delta2 * 60000.0) / (motor2Config.PPR * 10.0);
        } else {
            motor2.currentRPM = 0;
        }
    }
}

// ========================================== INDIVIDUAL PID COMPUTATION ===================================
void computeIndividualPID(PIDData* motor, MotorConfig* config) {
    unsigned long currentTime = millis();
    
    if (currentTime - motor->lastPulseTime >= 10) {  // PID loop 100Hz
        motor->lastPulseTime = currentTime;
        
        // Jika target RPM = 0, reset PID dan matikan motor
        if (motor->targetRPM == 0) {
            motor->integral = 0;
            motor->lastError = 0;
            motor->outputPWM = 0;
            applyMotorControl(motor);
            return;
        }
        
        // Hitung error dengan deadband
        motor->error = motor->targetRPM - motor->currentRPM;
        
        // Apply deadband untuk mengurangi buzzing
        if (fabs(motor->error) < config->deadband) {
            motor->error = 0;
        }
        
        // Integral term dengan anti-windup
        motor->integral += motor->error * 0.01;  // 10ms = 0.01s
        motor->integral = constrain(motor->integral, -config->integralLimit, config->integralLimit);
        
        // Derivative term
        float derivative = (motor->error - motor->lastError) / 0.01;
        motor->lastError = motor->error;
        
        // Filter derivative untuk mengurangi noise
        static float lastDerivative1 = 0, lastDerivative2 = 0;
        if (motor == &motor1) {
            derivative = 0.7 * derivative + 0.3 * lastDerivative1;
            lastDerivative1 = derivative;
        } else {
            derivative = 0.7 * derivative + 0.3 * lastDerivative2;
            lastDerivative2 = derivative;
        }
        
        // Hitung output PID
        float output = (config->Kp * motor->error) + 
                      (config->Ki * motor->integral) + 
                      (config->Kd * derivative);
        
        // Konversi ke PWM dengan batasan dan minimum PWM
        int pwmOutput;
        if (motor->targetRPM > 0) {
            pwmOutput = map(fabs(output), 0, config->maxRPM, config->minPWM, 255);
        } else {
            pwmOutput = 0;
        }
        
        pwmOutput = constrain(pwmOutput, config->minPWM, 255);
        motor->outputPWM = pwmOutput;
        
        // Terapkan ke motor
        applyMotorControl(motor);
    }
}

// ========================================== MOTOR CONTROL ================================================
void applyMotorControl(PIDData* motor) {
    // Tentukan pin berdasarkan motor
    uint8_t pwmPin, in1Pin, in2Pin;
    
    if (motor == &motor1) {
        pwmPin = MOTOR1_PWM;
        in1Pin = MOTOR1_IN1;
        in2Pin = MOTOR1_IN2;
    } else {
        pwmPin = MOTOR2_PWM;
        in1Pin = MOTOR2_IN1;
        in2Pin = MOTOR2_IN2;
    }
    
    if (motor->targetRPM == 0) {
        // Motor stop (brake)
        analogWrite(pwmPin, 0);
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
    } else if (motor->direction) {
        // Forward
        analogWrite(pwmPin, motor->outputPWM);
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
    } else {
        // Reverse
        analogWrite(pwmPin, motor->outputPWM);
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
    }
}

// ========================================== I2C EVENT HANDLER ============================================
void receiveData(int byteCount) {
    if (byteCount == 4) {
        for (int i = 0; i < 4; i++) {
            receivedData[i] = Wire.read();
        }
        
        // Update target RPM dan direction
        motor1.targetRPM = receivedData[0];
        motor1.direction = (receivedData[1] == 1);
        motor2.targetRPM = receivedData[2];
        motor2.direction = (receivedData[3] == 1);
        
        lastI2CTime = millis();  // Reset timeout timer
        
        // Debug serial
        Serial.print("Rcv: M1:");
        Serial.print(motor1.targetRPM);
        Serial.print(motor1.direction ? "F" : "R");
        Serial.print(" M2:");
        Serial.print(motor2.targetRPM);
        Serial.print(motor2.direction ? "F" : "R");
        Serial.println();
    }
}

// ========================================== SAFETY FAILSAFE ==============================================
void checkFailsafe() {
    if (millis() - lastI2CTime > I2C_TIMEOUT_MS) {
        // Komunikasi terputus, hentikan motor
        motor1.targetRPM = 0;
        motor2.targetRPM = 0;
        
        // Emergency stop
        analogWrite(MOTOR1_PWM, 0);
        analogWrite(MOTOR2_PWM, 0);
        digitalWrite(MOTOR1_IN1, LOW);
        digitalWrite(MOTOR1_IN2, LOW);
        digitalWrite(MOTOR2_IN1, LOW);
        digitalWrite(MOTOR2_IN2, LOW);
        
        Serial.println("Failsafe: I2C Timeout!");
    }
}

// ========================================== SETUP ========================================================
void setup() {
    // Inisialisasi Serial untuk debugging
    Serial.begin(115200);
    
    // Setup pin motor
    pinMode(MOTOR1_PWM, OUTPUT);
    pinMode(MOTOR1_IN1, OUTPUT);
    pinMode(MOTOR1_IN2, OUTPUT);
    pinMode(MOTOR2_PWM, OUTPUT);
    pinMode(MOTOR2_IN1, OUTPUT);
    pinMode(MOTOR2_IN2, OUTPUT);
    
    // Setup encoder pins dengan interrupt
    pinMode(ENC1_A, INPUT_PULLUP);
    pinMode(ENC1_B, INPUT_PULLUP);
    pinMode(ENC2_A, INPUT_PULLUP);
    pinMode(ENC2_B, INPUT_PULLUP);
    
    // Attach interrupts untuk encoder (4x mode dengan kedua channel)
    attachInterrupt(digitalPinToInterrupt(ENC1_A), readEncoder1A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC1_B), readEncoder1B, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_A), readEncoder2A, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENC2_B), readEncoder2B, CHANGE);
    
    // Setup I2C sebagai slave
    Wire.begin(I2C_ADDRESS);
    Wire.onReceive(receiveData);
    
    // Inisialisasi variabel PID
    motor1.lastPulseTime = millis();
    motor2.lastPulseTime = millis();
    motor1.lastEncoderCount = encoder1Count;
    motor2.lastEncoderCount = encoder2Count;
    motor1.lastRPMTime = millis();
    motor2.lastRPMTime = millis();
    
    Serial.println("STM32 Slave PID Controller Ready");
    Serial.print("I2C Address: 0x");
    Serial.println(I2C_ADDRESS, HEX);
}

// ========================================== MAIN LOOP ====================================================
void loop() {
    // 1. Baca encoder
    updateEncoderCounts();
    
    // 2. Hitung PID untuk setiap motor
    computeIndividualPID(&motor1, &motor1Config);
    computeIndividualPID(&motor2, &motor2Config);
    
    // 3. Cek failsafe
    checkFailsafe();
    
    // 4. Debug output (opsional)
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 500) {
        Serial.print("M1 RPM:");
        Serial.print(motor1.currentRPM);
        Serial.print("/");
        Serial.print(motor1.targetRPM);
        Serial.print(" PWM:");
        Serial.print(motor1.outputPWM);
        Serial.print(" M2 RPM:");
        Serial.print(motor2.currentRPM);
        Serial.print("/");
        Serial.print(motor2.targetRPM);
        Serial.print(" PWM:");
        Serial.print(motor2.outputPWM);
        Serial.println();
        
        lastDebugTime = millis();
    }
    
    delay(1);  // Memberi waktu untuk proses lain
}