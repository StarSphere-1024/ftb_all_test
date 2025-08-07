#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <FastLED.h>
#include <PS2X_lib.h>
#include <SoftwareSerial.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include "LIS3DHTR.h"
#include "Grove_Temperature_And_Humidity_Sensor.h"
#include "Ultrasonic.h"
#include <WonderK210.h>

#define max(a, b) ((a) > (b) ? (a) : (b))
#define min(a, b) ((a) < (b) ? (a) : (b))

#define USER_BUTTON_A_PIN 21
#define USER_BUTTON_B_PIN 0

#define RGB_PIN 33
#define NUM_RGB_LEDS 9
#define RGB_BRIGHTNESS 150
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_RGB_LEDS];

#define I2C_SDA 39
#define I2C_SCL 40

#define LOG_RX_PIN 44
#define LOG_TX_PIN 43
#define K210_RX_PIN 37
#define K210_TX_PIN 36
#define ASR_RX_PIN 35
#define ASR_TX_PIN 34
#define SOFT_SERIAL_RX_PIN 20
#define SOFT_SERIAL_TX_PIN 19
EspSoftwareSerial::UART softSerial;

#define GROVE6_PIN_A 1
#define GROVE6_PIN_B 2
#define GROVE3_PIN_A 3
#define GROVE3_PIN_B 4
#define GROVE5_PIN_A 5
#define GROVE5_PIN_B 6
#define GROVE2_PIN_A 7
#define GROVE2_PIN_B 8
#define GROVE4_PIN_A 26
#define GROVE4_PIN_B 38

#define ANALOG1_PIN_A GROVE3_PIN_A
#define ANALOG1_PIN_B GROVE3_PIN_B
#define ANALOG2_PIN_A GROVE6_PIN_A
#define ANALOG2_PIN_B GROVE6_PIN_B

#define PS2_CMD_PIN 9
#define PS2_DATA_PIN 10
#define PS2_CLK_PIN 41
#define PS2_CS_PIN 42

#define LIGHT_PIN ANALOG2_PIN_A
#define DHT_PIN GROVE5_PIN_A
#define ULTRASONIC_PIN GROVE2_PIN_A

#define LF_MOTOR_FWD_PWM 11
#define LF_MOTOR_REV_PWM 12
#define RF_MOTOR_FWD_PWM 14
#define RF_MOTOR_REV_PWM 13
#define LR_MOTOR_FWD_PWM 15
#define LR_MOTOR_REV_PWM 16
#define RR_MOTOR_FWD_PWM 18
#define RR_MOTOR_REV_PWM 17
const uint8_t MIN_SPEED = 100;
enum CarState
{
    STOP,
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    CLOCKWISE,
    COUNTER_CLOCKWISE,
    BRAKING
} CurrentState = STOP;

#define SERVO1_PIN 48
#define SERVO2_PIN 47
#define SERVO_MIN_PULSE 500
#define SERVO_MAX_PULSE 2500
#define SERVO_PERIOD 20000

PS2X ps2x;
int ps2_error = 0;
byte ps2_type = 0;

const char *wifi_ssid = "MI4A";
const char *wifi_password = "star123!";

#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

LIS3DHTR<TwoWire> LIS;
#define DHTTYPE DHT11
DHT dht(DHT_PIN, DHTTYPE);
Ultrasonic ultrasonic(ULTRASONIC_PIN);

WonderK210 *wk;
Find_Box_st *result;

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
        Serial.println("BLE客户端已连接");
    }

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
        Serial.println("BLE客户端已断开");
        pServer->getAdvertising()->start();
        Serial.println("已重新开始广播");
    }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        String value = pCharacteristic->getValue();
        if (value.length() > 0)
        {
            Serial.print("收到BLE写入数据: ");
            Serial.println(value);
            String response = "ECHO: " + value;
            pCharacteristic->setValue(response);
            pCharacteristic->notify();
            Serial.print("已发送回显: ");
            Serial.println(response);
        }
    }
};

hw_timer_t *g_servo_timer = NULL;
portMUX_TYPE g_servo_timer_mux = portMUX_INITIALIZER_UNLOCKED;
#define SERVO_TIMER_FREQUENCY 1000000
#define SERVO_TIMER_TICK_US 50
#define SERVO_PWM_PERIOD_MS 20
#define SERVO_PWM_PERIOD_TICKS (SERVO_PWM_PERIOD_MS * 1000 / SERVO_TIMER_TICK_US)
#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
volatile uint16_t g_servo1_pulse_ticks, g_servo2_pulse_ticks;

void ARDUINO_ISR_ATTR on_servo_timer()
{
    portENTER_CRITICAL_ISR(&g_servo_timer_mux);
    static uint16_t counter = 0;
    if (counter == 0)
    {
        digitalWrite(SERVO1_PIN, HIGH);
        digitalWrite(SERVO2_PIN, HIGH);
    }
    if (counter == g_servo1_pulse_ticks)
    {
        digitalWrite(SERVO1_PIN, LOW);
    }
    if (counter == g_servo2_pulse_ticks)
    {
        digitalWrite(SERVO2_PIN, LOW);
    }
    counter++;
    if (counter >= SERVO_PWM_PERIOD_TICKS)
    {
        counter = 0;
    }
    portEXIT_CRITICAL_ISR(&g_servo_timer_mux);
}

void Servos_SetAngle(uint8_t servo_num, uint8_t angle)
{
    angle = constrain(angle, 0, 180);
    uint32_t pulse_us = map(angle, 0, 180, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
    uint16_t pulse_ticks = pulse_us / SERVO_TIMER_TICK_US;
    portENTER_CRITICAL(&g_servo_timer_mux);
    if (servo_num == 1)
        g_servo1_pulse_ticks = pulse_ticks;
    else if (servo_num == 2)
        g_servo2_pulse_ticks = pulse_ticks;
    portEXIT_CRITICAL(&g_servo_timer_mux);
}

void controlWheel(uint8_t fwdPin, uint8_t revPin, int speed)
{
    if (speed > 0)
    {
        analogWrite(fwdPin, speed);
        analogWrite(revPin, 0);
    }
    else if (speed < 0)
    {
        analogWrite(fwdPin, 0);
        analogWrite(revPin, -speed);
    }
    else
    {
        analogWrite(fwdPin, 0);
        analogWrite(revPin, 0);
    }
}

void setMecanumWheels(int lfSpeed, int rfSpeed, int lrSpeed, int rrSpeed)
{
    controlWheel(LF_MOTOR_FWD_PWM, LF_MOTOR_REV_PWM, lfSpeed);
    controlWheel(RF_MOTOR_FWD_PWM, RF_MOTOR_REV_PWM, rfSpeed);
    controlWheel(LR_MOTOR_FWD_PWM, LR_MOTOR_REV_PWM, lrSpeed);
    controlWheel(RR_MOTOR_FWD_PWM, RR_MOTOR_REV_PWM, rrSpeed);
}

void moveForward(uint8_t speed)
{
    CurrentState = FORWARD;
    setMecanumWheels(speed, speed, speed, speed);
}
void moveBackward(uint8_t speed)
{
    CurrentState = BACKWARD;
    setMecanumWheels(-speed, -speed, -speed, -speed);
}
void strafeLeft(uint8_t speed)
{
    CurrentState = LEFT;
    setMecanumWheels(-speed, speed, speed, -speed);
}
void strafeRight(uint8_t speed)
{
    CurrentState = RIGHT;
    setMecanumWheels(speed, -speed, -speed, speed);
}
void rotateClockwise(uint8_t speed)
{
    CurrentState = CLOCKWISE;
    setMecanumWheels(speed, -speed, speed, -speed);
}
void rotateCounterClockwise(uint8_t speed)
{
    CurrentState = COUNTER_CLOCKWISE;
    setMecanumWheels(-speed, speed, -speed, speed);
}
void release()
{
    CurrentState = STOP;
    setMecanumWheels(0, 0, 0, 0);
}

void rampMovement(void (*moveFunction)(uint8_t), uint8_t targetSpeed, unsigned long rampUpTime, unsigned long holdTime, unsigned long rampDownTime)
{
    const int interval = 20;
    if (targetSpeed < MIN_SPEED)
        targetSpeed = MIN_SPEED;
    float stepsUp = rampUpTime > 0 ? (float)(targetSpeed - MIN_SPEED) / (rampUpTime / interval) : targetSpeed;
    float stepsDown = rampDownTime > 0 ? (float)(targetSpeed - MIN_SPEED) / (rampDownTime / interval) : targetSpeed;

    for (float speed = MIN_SPEED; speed <= targetSpeed; speed += max(1.0f, stepsUp))
    {
        moveFunction(min((int)speed, (int)targetSpeed));
        delay(interval);
    }
    moveFunction(targetSpeed);
    delay(holdTime);
    for (float speed = targetSpeed; speed >= MIN_SPEED; speed -= max(1.0f, stepsDown))
    {
        moveFunction(max((int)speed, (int)MIN_SPEED));
        delay(interval);
    }
    release();
}

void sendToASR(String command)
{
    Serial2.println(command);
}

void processASRCommand(String command)
{
    if (command.startsWith("RGB:"))
    {
        String rgbCommand = command.substring(4);
        if (rgbCommand == "ON")
        {
            fill_solid(leds, NUM_RGB_LEDS, CRGB::White);
            FastLED.show();
        }
        else if (rgbCommand == "OFF")
        {
            fill_solid(leds, NUM_RGB_LEDS, CRGB::Black);
            FastLED.show();
        }
        else if (rgbCommand.startsWith("COLOR:"))
        {
            String color = rgbCommand.substring(6);
            CRGB colorValue;
            if (color == "RED")
                colorValue = CRGB::Red;
            else if (color == "GREEN")
                colorValue = CRGB::Green;
            else if (color == "BLUE")
                colorValue = CRGB::Blue;
            fill_solid(leds, NUM_RGB_LEDS, colorValue);
            FastLED.show();
        }
    }
}

void printMenu()
{
    Serial.println("\n==================================================");
    Serial.println("===== ESP32-S3 硬件功能测试菜单 (V3.2) =====");
    Serial.println("==================================================");
    Serial.println("--- 串口通信测试 ---");
    Serial.println("  1. 串口通信测试 (ESP32S3 <-> PC)");
    Serial.println("  2. 串口通信测试 (ESP32S3 <-> K210)");
    Serial.println("  3. 串口通信测试 (ESP32S3 <-> ASR-PRO)");
    Serial.println("  4. 软串口测试 (GPIO19, GPIO20)");
    Serial.println("--- 总线与接口测试 ---");
    Serial.println("  5. I2C 测试");
    Serial.println("  6. Grove 模拟接口测试");
    Serial.println("  7. Grove 数字接口测试");
    Serial.println("  8. PS2 手柄通信测试");
    Serial.println("--- 驱动测试 ---");
    Serial.println("  9. 麦克纳姆轮电机测试");
    Serial.println(" 10. 舵机功能测试");
    Serial.println(" 11. RGB LED 功能测试");
    Serial.println("--- 无线功能测试 ---");
    Serial.println(" 12. WiFi 功能测试 (扫描, AP, 连接)");
    Serial.println(" 13. 蓝牙低功耗 (BLE) 功能测试");
    Serial.println("--- 板载按键测试 ---");
    Serial.println(" 14. 用户按键 A/B 测试");
    Serial.println("--------------------------------------------------");
    Serial.println("输入 'a' 自动运行所有测试");
    Serial.print("请输入测试项目编号 (1-14)，或 'm' 返回菜单: ");
}

void wait_for_exit()
{
    Serial.println("\n** 测试结束。输入任意字符并回车返回主菜单... **");
    while (Serial.available() == 0)
    {
        delay(100);
    }
    while (Serial.available() > 0)
    {
        Serial.read();
    }
}

void testK210BasicCommunication()
{
    Serial.println("\n发送 'PingK210'，期待 'PongK210' 回复...");
    Serial1.println("PingK210\n");
    long startTime = millis();
    bool received = false;
    while (millis() - startTime < 3000)
    {
        if (Serial1.available() > 0)
        {
            String response = Serial1.readStringUntil('\n');
            response.trim();
            Serial.print("从K210收到: ");
            Serial.println(response);
            if (response == "PongK210")
            {
                Serial.println("测试成功!");
                received = true;
                break;
            }
        }
    }
    if (!received)
        Serial.println("测试失败: 未收到正确回复。");
}

void testK210FaceRecognition()
{
    Serial.println("\n测试K210人脸识别数据...");
    Serial.println("持续读取人脸框数据 (x, y, w, h)。按任意键退出。");
    wk->update_data();
    while (Serial.available() == 0)
    {
        wk->update_data();
        if (wk->recive_box(result, K210_FIND_FACE_YOLO))
        {
            Serial.print("x: ");
            Serial.println(result->x);
            Serial.print("y: ");
            Serial.println(result->y);
            Serial.print("w: ");
            Serial.println(result->w);
            Serial.print("h: ");
            Serial.println(result->h);
        }
        else
        {
            Serial.println("未收到人脸识别数据");
        }
        delay(100);
    }
    while (Serial.available() > 0)
        Serial.read();
}

void testASRBasicCommunication()
{
    Serial.println("\n发送 'PingASR'，期待 'PongASR' 回复...");
    sendToASR("PingASR\n");
    long startTime = millis();
    bool received = false;
    while (millis() - startTime < 3000)
    {
        if (Serial2.available() > 0)
        {
            String response = Serial2.readStringUntil('\n');
            response.trim();
            Serial.print("从ASR-PRO收到: ");
            Serial.println(response);
            if (response == "PongASR")
            {
                Serial.println("基础通信测试成功!");
                received = true;
                break;
            }
        }
    }
    if (!received)
        Serial.println("测试失败: 未收到正确回复。");
}

void testASRWakeup()
{
    Serial.println("\n发送 'Wakeup'，期待ASR-Pro进入唤醒状态...");
    sendToASR("Wakeup\n");
    Serial.println("请观察ASR-Pro是否进入唤醒状态（通常有语音提示）。");
}

void testASRAudioPlayback()
{
    Serial.println("\n发送 'PlayVol:100'，期待播放'我是未来科技盒'...");
    sendToASR("PlayVol:100\n");
    Serial.println("请确认是否听到音频播放。");
}

void testASRRGBControl()
{
    Serial.println("\n请对ASR-Pro说'打开灯光'或'关闭灯光'，观察RGB灯变化...");
    Serial.println("发送 'Wakeup' 并等待10秒以接收RGB命令（输入'exit'提前退出）。");
    sendToASR("Wakeup\n");
    long startTime = millis();
    while (millis() - startTime < 10000)
    {
        if (Serial.available() > 0)
        {
            String exitInput = Serial.readStringUntil('\n');
            exitInput.trim();
            if (exitInput == "exit")
                break;
        }
        if (Serial2.available() > 0)
        {
            String command = Serial2.readStringUntil('\n');
            command.trim();
            Serial.print("收到RGB命令: ");
            Serial.println(command);
            processASRCommand(command);
        }
        delay(10);
    }
}

void testI2CBusScan()
{
    Serial.println("\n--- I2C 总线扫描 ---");
    byte count = 0;
    for (byte i = 1; i < 127; i++)
    {
        Wire.beginTransmission(i);
        if (Wire.endTransmission() == 0)
        {
            Serial.print("发现I2C设备，地址: 0x");
            if (i < 16)
                Serial.print("0");
            Serial.println(i, HEX);
            count++;
        }
    }
    Serial.print("扫描完成。共发现 ");
    Serial.print(count);
    Serial.println(" 个设备。");
}

void testI2CAccelerometer()
{
    Serial.println("\n--- 加速度传感器测试 (LIS3DHTR) ---");
    if (LIS)
    {
        Serial.println("读取加速度数据 (10次，间隔500ms)：");
        for (int i = 0; i < 10; i++)
        {
            Serial.print("x: ");
            Serial.print(LIS.getAccelerationX());
            Serial.print("  ");
            Serial.print("y: ");
            Serial.print(LIS.getAccelerationY());
            Serial.print("  ");
            Serial.print("z: ");
            Serial.println(LIS.getAccelerationZ());
            delay(500);
        }
    }
    else
    {
        Serial.println("加速度传感器 (LIS3DHTR) 未连接。");
    }
}

void testAnalogADC()
{
    Serial.println("\n--- 模拟接口ADC测试 ---");
    Serial.println("将持续读取模拟接口的ADC值 (12位, 0-4095)。");
    Serial.println("按任意键退出。");
    while (Serial.available() == 0)
    {
        int adc1a = analogRead(ANALOG1_PIN_A);
        int adc1b = analogRead(ANALOG1_PIN_B);
        int adc2a = analogRead(ANALOG2_PIN_A);
        int adc2b = analogRead(ANALOG2_PIN_B);
        Serial.print("模拟接口1 (J16): ");
        Serial.print("P1(GPIO1):");
        Serial.print(adc1a);
        Serial.print(" P2(GPIO2):");
        Serial.print(adc1b);
        Serial.print("  |  ");
        Serial.print("模拟接口2 (J17): ");
        Serial.print("P1(GPIO3):");
        Serial.print(adc2a);
        Serial.print(" P2(GPIO4):");
        Serial.print(adc2b);
        Serial.println();
        delay(500);
    }
    while (Serial.available() > 0)
        Serial.read();
}

void testServo()
{
    Serial.println("\n舵机1 (GPIO48) 从0度转到180度，舵机2 (GPIO47) 从180度转到0度，然后反向。");
    Serial.println("按任意键退出。");
    for (int pos = 0; pos <= 180; pos += 1)
    {
        Servos_SetAngle(1, pos);
        Servos_SetAngle(2, 180 - pos);
        delay(15);
        if (Serial.available() > 0)
            break;
    }
    if (Serial.available() == 0)
    {
        for (int pos = 180; pos >= 0; pos -= 1)
        {
            Servos_SetAngle(1, pos);
            Servos_SetAngle(2, 180 - pos);
            delay(15);
            if (Serial.available() > 0)
                break;
        }
    }
    Servos_SetAngle(1, 0);
    Servos_SetAngle(2, 0);
}

void testServoSerialControl()
{
    Serial.println("\n--- 舵机串口控制测试 ---");
    Serial.println("通过串口输入 'S1:<angle>' 或 'S2:<angle>' 设置舵机1或舵机2角度 (0-180)。");
    Serial.println("输入 'exit' 退出测试。");

    while (true)
    {
        if (Serial.available() > 0)
        {
            String input = Serial.readStringUntil('\n');
            input.trim();
            if (input == "exit")
                break;
            if (input.startsWith("S1:") || input.startsWith("S2:"))
            {
                int servo_num = input.startsWith("S1:") ? 1 : 2;
                String angle_str = input.substring(3);
                int angle = angle_str.toInt();
                if (angle >= 0 && angle <= 180)
                {
                    Servos_SetAngle(servo_num, angle);
                    Serial.printf("舵机%d 设置角度: %d\n", servo_num, angle);
                }
                else
                {
                    Serial.println("角度值无效，必须在0-180之间。");
                }
            }
            else
            {
                Serial.println("无效命令，格式为 'S1:<angle>' 或 'S2:<angle>'。");
            }
        }
    }
    Servos_SetAngle(1, 0);
    Servos_SetAngle(2, 0);
}

void testLightSensor()
{
    Serial.println("\n--- 光线传感器测试 (GPIO2) ---");
    Serial.println("将持续读取光线传感器的ADC值 (12位, 0-4095)。");
    Serial.println("按任意键退出。");
    while (Serial.available() == 0)
    {
        int lightValue = analogRead(LIGHT_PIN);
        Serial.print("光线传感器值: ");
        Serial.println(lightValue);
        delay(100);
    }
    while (Serial.available() > 0)
        Serial.read();
}

void testDHT11Sensor()
{
    Serial.println("\n--- 温湿度传感器测试 (DHT11, Grove1) ---");
    Serial.println("读取温湿度数据 (5次，间隔1500ms)：");
    for (int i = 0; i < 5; i++)
    {
        float temp_hum_val[2] = {0};
        if (!dht.readTempAndHumidity(temp_hum_val))
        {
            Serial.print("Humidity: ");
            Serial.print(temp_hum_val[0]);
            Serial.print(" %\t");
            Serial.print("Temperature: ");
            Serial.print(temp_hum_val[1]);
            Serial.println(" *C");
        }
        else
        {
            Serial.println("无法获取温湿度值。");
        }
        delay(1500);
    }
}

void testUltrasonicSensor()
{
    Serial.println("\n--- 超声波传感器测试 (Grove2) ---");
    Serial.println("读取距离数据 (5次，间隔500ms)：");
    for (int i = 0; i < 5; i++)
    {
        long RangeInCentimeters = ultrasonic.MeasureInCentimeters();
        Serial.print("Distance: ");
        Serial.print(RangeInCentimeters);
        Serial.println(" cm");
        delay(500);
    }
}

void testLineFollowerSensors()
{
    Serial.println("\n--- 双组循迹红外传感器测试 (Grove3: GPIO3,4; Grove4: GPIO26,38) ---");
    Serial.println("将持续读取四路循迹传感器状态 (0=检测到线, 1=未检测到线)。");
    Serial.println("按任意键退出。");

    const int LINE_FOLLOWER_G3_A = GROVE3_PIN_A;
    const int LINE_FOLLOWER_G3_B = GROVE3_PIN_B;
    const int LINE_FOLLOWER_G4_A = GROVE4_PIN_A;
    const int LINE_FOLLOWER_G4_B = GROVE4_PIN_B;

    pinMode(LINE_FOLLOWER_G3_A, INPUT);
    pinMode(LINE_FOLLOWER_G3_B, INPUT);
    pinMode(LINE_FOLLOWER_G4_A, INPUT);
    pinMode(LINE_FOLLOWER_G4_B, INPUT);

    while (Serial.available() == 0)
    {
        int g3_a = digitalRead(LINE_FOLLOWER_G3_A);
        int g3_b = digitalRead(LINE_FOLLOWER_G3_B);
        int g4_a = digitalRead(LINE_FOLLOWER_G4_A);
        int g4_b = digitalRead(LINE_FOLLOWER_G4_B);

        Serial.print("Grove3 A (GPIO3): ");
        Serial.print(g3_a);
        Serial.print("  Grove3 B (GPIO4): ");
        Serial.print(g3_b);
        Serial.print("  |  Grove4 A (GPIO26): ");
        Serial.print(g4_a);
        Serial.print("  Grove4 B (GPIO38): ");
        Serial.println(g4_b);

        delay(200);
    }

    while (Serial.available() > 0)
        Serial.read();
}

void testSerialPC()
{
    Serial.println("\n--- 1. PC串口通信测试 ---");
    Serial.println("您发送的任何内容都将被返回。输入 'exit' 退出。");
    while (true)
    {
        if (Serial.available() > 0)
        {
            String input = Serial.readStringUntil('\n');
            input.trim();
            if (input == "exit")
                break;
            Serial.print("收到: ");
            Serial.println(input);
        }
    }
}
void testSerialK210()
{
    Serial.println("\n--- 2. K210串口通信测试 (UART1) ---");
    Serial.println("测试选项：");
    Serial.println("  1. 测试基础通信 (PING/PONG)");
    Serial.println("  2. 测试人脸识别数据");
    Serial.println("  0. 退出");
    Serial.print("请选择测试选项: ");

    while (true)
    {
        if (Serial.available() > 0)
        {
            String input = Serial.readStringUntil('\n');
            input.trim();
            int choice = input.toInt();
            bool should_exit = false;

            switch (choice)
            {
            case 1:
                testK210BasicCommunication();
                break;
            case 2:
                testK210FaceRecognition();
                break;
            case 0:
                should_exit = true;
                break;
            default:
                Serial.println("无效输入。");
                break;
            }

            if (should_exit)
                break;
            Serial.print("\n请选择下一个测试选项 (0退出): ");
        }
    }
}
void testSerialASR()
{
    Serial.println("\n--- 3. ASR-PRO串口通信测试 (UART2) ---");
    Serial.println("测试选项：");
    Serial.println("  1. 测试基础通信 (PING/PONG)");
    Serial.println("  2. 测试唤醒功能");
    Serial.println("  3. 测试音频播放 (ID: 100)");
    Serial.println("  4. 测试语音控制RGB (需说'打开灯光'或'关闭灯光')");
    Serial.println("  0. 退出");
    Serial.print("请选择测试选项: ");

    while (true)
    {
        if (Serial.available() > 0)
        {
            String input = Serial.readStringUntil('\n');
            input.trim();
            int choice = input.toInt();
            bool should_exit = false;

            switch (choice)
            {
            case 1:
                testASRBasicCommunication();
                break;
            case 2:
                testASRWakeup();
                break;
            case 3:
                testASRAudioPlayback();
                break;
            case 4:
                testASRRGBControl();
                break;
            case 0:
                should_exit = true;
                break;
            default:
                Serial.println("无效输入。");
                break;
            }

            if (should_exit)
                break;
            Serial.print("\n请选择下一个测试选项 (0退出): ");
        }
    }
}
void testSoftSerial()
{
    Serial.println("\n--- 4. 软串口测试  ---");
    Serial.println("请将GPIO19(TX)和GPIO20(RX)短接。");
    Serial.println("您在PC串口发送的内容将通过软串口发送并接收回来。");
    Serial.println("输入 'exit' 退出。");

    softSerial.listen();

    while (true)
    {
        if (Serial.available())
        {
            String input = Serial.readStringUntil('\n');
            input.trim();
            if (input == "exit")
                break;
            softSerial.println(input);
        }
        if (softSerial.available())
        {
            String s = softSerial.readStringUntil('\n');
            s.trim();
            Serial.print("软串口收到: ");
            Serial.println(s);
        }
    }
}

void testI2C()
{
    Serial.println("\n--- 5. I2C 测试 ---");
    Serial.println("测试选项：");
    Serial.println("  1. I2C 总线扫描");
    Serial.println("  2. 加速度传感器测试 (LIS3DHTR)");
    Serial.println("  0. 退出");
    Serial.print("请选择测试选项: ");

    while (true)
    {
        if (Serial.available() > 0)
        {
            String input = Serial.readStringUntil('\n');
            input.trim();
            int choice = input.toInt();
            bool should_exit = false;

            switch (choice)
            {
            case 0:
                should_exit = true;
                break;
            case 1:
                testI2CBusScan();
                break;
            case 2:
                testI2CAccelerometer();
                break;
            default:
                Serial.println("无效输入。");
                break;
            }

            if (should_exit)
                break;
            Serial.print("\n请选择下一个测试选项 (0退出): ");
        }
    }
}

void testAnalogInterfaces()
{
    Serial.println("\n--- 6. 模拟接口测试 (J16, J17) ---");
    Serial.println("测试选项：");
    Serial.println("  1. 模拟接口ADC测试");
    Serial.println("  2. 光线传感器测试 (GPIO2)");
    Serial.println("  0. 退出");
    Serial.print("请选择测试选项: ");

    while (true)
    {
        if (Serial.available() > 0)
        {
            String input = Serial.readStringUntil('\n');
            input.trim();
            int choice = input.toInt();
            bool should_exit = false;

            switch (choice)
            {
            case 1:
                testAnalogADC();
                break;
            case 2:
                testLightSensor();
                break;
            case 0:
                should_exit = true;
                break;
            default:
                Serial.println("无效输入。");
                break;
            }

            if (should_exit)
                break;
            Serial.print("\n请选择下一个测试选项 (0退出): ");
        }
    }
}

void testGroveInterfaces()
{
    Serial.println("\n--- 7. Grove 接口测试 ---");
    Serial.println("测试选项：");
    Serial.println("  1. 温湿度传感器测试 (DHT11, Grove5)");
    Serial.println("  2. 超声波传感器测试 (Grove2)");
    Serial.println("  3. 双组循迹红外传感器测试 (Grove3, Grove4)");
    Serial.println("  0. 退出");
    Serial.print("请选择测试选项: ");

    while (true)
    {
        if (Serial.available() > 0)
        {
            String input = Serial.readStringUntil('\n');
            input.trim();
            int choice = input.toInt();
            bool should_exit = false;

            switch (choice)
            {
            case 1:
                testDHT11Sensor();
                break;
            case 2:
                testUltrasonicSensor();
                break;
            case 3:
                testLineFollowerSensors();
                break;
            case 0:
                should_exit = true;
                break;
            default:
                Serial.println("无效输入。");
                break;
            }

            if (should_exit)
                break;
            Serial.print("\n请选择下一个测试选项 (0退出): ");
        }
    }
}

void testPS2()
{
    Serial.println("\n--- 8. PS2 手柄通信测试 ---");
    if (ps2_error != 0)
    {
        Serial.print("错误: PS2手柄初始化失败，错误码: ");
        Serial.println(ps2_error);
        return;
    }
    Serial.println("请按键或移动摇杆进行测试。按任意键退出。");

    struct ButtonMap
    {
        uint16_t button;
        const char *name;
    };
    const ButtonMap buttons[] = {
        {PSB_PAD_UP, "Up"},
        {PSB_PAD_RIGHT, "Right"},
        {PSB_PAD_DOWN, "Down"},
        {PSB_PAD_LEFT, "Left"},
        {PSB_SELECT, "Select"},
        {PSB_START, "Start"},
        {PSB_L1, "L1"},
        {PSB_R1, "R1"},
        {PSB_L2, "L2"},
        {PSB_R2, "R2"},
        {PSB_TRIANGLE, "Triangle"},
        {PSB_CIRCLE, "Circle"},
        {PSB_CROSS, "Cross"},
        {PSB_SQUARE, "Square"},
        {PSB_L3, "L3"},
        {PSB_R3, "R3"}};
    const int buttonCount = sizeof(buttons) / sizeof(buttons[0]);

    unsigned long lastUpdate = millis();
    while (Serial.available() == 0)
    {
        ps2x.read_gamepad(false, false);

        for (int i = 0; i < buttonCount; i++)
        {
            if (ps2x.Button(buttons[i].button))
            {
                Serial.println(buttons[i].name);
            }
        }

        int stick_lx = ps2x.Analog(PSS_LX);
        int stick_ly = ps2x.Analog(PSS_LY);
        if (stick_lx != 128 || stick_ly != 127)
        {
            Serial.printf("Left Stick: (%d, %d)\n", stick_lx, stick_ly);
        }
        int stick_rx = ps2x.Analog(PSS_RX);
        int stick_ry = ps2x.Analog(PSS_RY);
        if (stick_rx != 128 || stick_ry != 127)
        {
            Serial.printf("Right Stick: (%d, %d)\n", stick_rx, stick_ry);
        }

        if (millis() - lastUpdate >= 50)
        {
            lastUpdate = millis();
        }
        else
        {
            continue;
        }
    }
}
void testMecanumMotors()
{
    Serial.println("\n--- 9. 麦克纳姆轮电机测试 (自动模式) ---");
    Serial.println("将依次执行以下测试项：");
    Serial.println("  1. 前进 (2秒)");
    Serial.println("  2. 后退 (2秒)");
    Serial.println("  3. 左平移 (2秒)");
    Serial.println("  4. 右平移 (2秒)");
    Serial.println("  5. 顺时针旋转 (2秒)");
    Serial.println("  6. 逆时针旋转 (2秒)");
    Serial.println("  7. 加速-保持-减速演示 (前进)");
    Serial.println("按任意键中断测试。");

    struct TestCase
    {
        const char *name;
        void (*action)(uint8_t);
        uint8_t speed;
        unsigned long duration;
        bool isRamp;
    };
    const TestCase tests[] = {
        {"前进", moveForward, 200, 2000, false},
        {"后退", moveBackward, 200, 2000, false},
        {"左平移", strafeLeft, 200, 2000, false},
        {"右平移", strafeRight, 200, 2000, false},
        {"顺时针旋转", rotateClockwise, 200, 2000, false},
        {"逆时针旋转", rotateCounterClockwise, 200, 2000, false},
        {"加速-保持-减速演示", moveForward, 255, 4000, true}
    };
    const int testCount = sizeof(tests) / sizeof(tests[0]);

    for (int i = 0; i < testCount; i++)
    {
        if (Serial.available() > 0)
        {
            Serial.println("测试被用户中断！");
            while (Serial.available() > 0)
                Serial.read();
            release();
            return;
        }

        Serial.printf("\n[%lu] 执行: %s\n", millis(), tests[i].name);

        if (tests[i].isRamp)
        {
            rampMovement(tests[i].action, tests[i].speed, 1000, 2000, 1000);
        }
        else
        {
            tests[i].action(tests[i].speed);
            unsigned long startTime = millis();
            while (millis() - startTime < tests[i].duration)
            {
                if (Serial.available() > 0)
                {
                    Serial.println("测试被用户中断！");
                    while (Serial.available() > 0)
                        Serial.read();
                    release();
                    return;
                }
            }
            release();
        }

        Serial.printf("[%lu] %s 测试完成\n", millis(), tests[i].name);

        unsigned long pauseStart = millis();
        while (millis() - pauseStart < 500)
        {
            if (Serial.available() > 0)
            {
                Serial.println("测试被用户中断！");
                while (Serial.available() > 0)
                    Serial.read();
                return;
            }
        }
    }

    Serial.println("\n所有麦克纳姆轮测试完成！");
}

void testServos()
{
    Serial.println("\n--- 10. 舵机功能测试 (软件 PWM) ---");
    Serial.println("测试选项：");
    Serial.println("  1. 自动角度测试");
    Serial.println("  2. 串口控制测试");
    Serial.println("  0. 退出");
    Serial.print("请选择测试选项: ");

    while (true)
    {
        if (Serial.available() > 0)
        {
            String input = Serial.readStringUntil('\n');
            input.trim();
            int choice = input.toInt();
            bool should_exit = false;

            switch (choice)
            {
            case 1:
                testServo();
                break;
            case 2:
                testServoSerialControl();
                break;
            case 0:
                should_exit = true;
                break;
            default:
                Serial.println("无效输入。");
                break;
            }

            if (should_exit)
                break;
            Serial.print("\n请选择下一个测试选项 (0退出): ");
        }
    }
}

void testRGB_FastLED()
{
    Serial.println("\n--- 11. RGB LED 功能测试 (FastLED) ---");

    Serial.println("所有灯: 红色");
    fill_solid(leds, NUM_RGB_LEDS, CRGB::Red);
    FastLED.show();
    delay(1000);

    Serial.println("所有灯: 绿色");
    fill_solid(leds, NUM_RGB_LEDS, CRGB::Green);
    FastLED.show();
    delay(1000);

    Serial.println("所有灯: 蓝色");
    fill_solid(leds, NUM_RGB_LEDS, CRGB::Blue);
    FastLED.show();
    delay(1000);

    Serial.println("亮度测试: 50%");
    FastLED.setBrightness(RGB_BRIGHTNESS / 2);
    fill_solid(leds, NUM_RGB_LEDS, CRGB::White);
    FastLED.show();
    delay(1000);

    Serial.println("亮度测试: 10%");
    FastLED.setBrightness(RGB_BRIGHTNESS / 10);
    fill_solid(leds, NUM_RGB_LEDS, CRGB::White);
    FastLED.show();
    delay(1000);

    FastLED.setBrightness(RGB_BRIGHTNESS);

    Serial.println("彩虹流动效果");
    for (int i = 0; i < 2; i++)
    {
        for (int j = 0; j < 255; j++)
        {
            fill_rainbow(leds, NUM_RGB_LEDS, j, 7);
            FastLED.show();
            delay(10);
        }
    }

    fill_solid(leds, NUM_RGB_LEDS, CRGB::Black);
    FastLED.show();
    wait_for_exit();
}

void testWiFi()
{
    Serial.println("\n--- 12. WiFi 功能测试 ---");
    WiFi.mode(WIFI_AP_STA);

    Serial.println("--- 扫描网络 ---");
    int n = WiFi.scanNetworks();
    if (n > 0)
    {
        Serial.print(n);
        Serial.println(" 个网络被发现:");
        for (int i = 0; i < n; ++i)
        {
            Serial.printf("  %d: %s (%d) %s\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i), (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
        }
    }
    else
    {
        Serial.println("未扫描到任何网络。");
    }

    Serial.println("\n--- 创建AP热点 ---");
    const char *ap_ssid = "FTB_Test_AP";
    WiFi.softAP(ap_ssid);
    Serial.print("AP热点 '");
    Serial.print(ap_ssid);
    Serial.print("' 已创建, IP: ");
    Serial.println(WiFi.softAPIP());

    Serial.println("\n--- 连接到指定WiFi ---");
    Serial.print("正在连接到: ");
    Serial.println(wifi_ssid);
    WiFi.begin(wifi_ssid, wifi_password);
    int retries = 0;
    while (WiFi.status() != WL_CONNECTED && retries < 20)
    {
        delay(500);
        Serial.print(".");
        retries++;
    }
    if (WiFi.status() == WL_CONNECTED)
    {
        Serial.println("\nWiFi连接成功!");
        Serial.print("IP地址: ");
        Serial.println(WiFi.localIP());
    }
    else
    {
        Serial.println("\nWiFi连接失败。");
    }

    wait_for_exit();
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}

void testBLE()
{
    Serial.println("\n--- 13. 蓝牙低功耗 (BLE) 功能测试 ---");
    Serial.println("正在启动BLE服务...");

    BLEDevice::init("FTB_BT");
    BLEServer *pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    BLEService *pService = pServer->createService(SERVICE_UUID);

    pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_NOTIFY);
    pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
    pCharacteristic->setValue("Hello, FTB!");

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println("BLE服务已启动并开始广播。");
    Serial.println("请使用手机BLE调试App连接 'FTB_BT' 并测试读写特征值。");
    Serial.println("在串口监视器输入 'exit' 退出测试。");

    while (true)
    {
        if (Serial.available() > 0)
        {
            String input = Serial.readStringUntil('\n');
            if (input.indexOf("exit") != -1)
                break;
        }
        delay(100);
    }

    BLEDevice::stopAdvertising();
    Serial.println("BLE服务已停止。");
}

void testButtons()
{
    Serial.println("\n--- 14. 用户按键测试 ---");
    Serial.println("请按下板载的用户按键A和B。按任意键退出。");
    while (Serial.available() == 0)
    {
        if (digitalRead(USER_BUTTON_A_PIN) == LOW)
        {
            Serial.println("用户按键 A 被按下!");
            while (digitalRead(USER_BUTTON_A_PIN) == LOW)
                ;
        }
        if (digitalRead(USER_BUTTON_B_PIN) == LOW)
        {
            Serial.println("用户按键 B 被按下!");
            while (digitalRead(USER_BUTTON_B_PIN) == LOW)
                ;
        }
        delay(20);
    }
    while (Serial.available() > 0)
        Serial.read();
}

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("\n\nESP32-S3 测试程序 启动...");

    Serial1.begin(115200, SERIAL_8N1, K210_RX_PIN, K210_TX_PIN);
    Serial2.begin(115200, SERIAL_8N1, ASR_RX_PIN, ASR_TX_PIN);
    softSerial.begin(9600, SWSERIAL_8N1, SOFT_SERIAL_RX_PIN, SOFT_SERIAL_TX_PIN, false);

    Wire.begin(I2C_SDA, I2C_SCL);
    LIS.begin(Wire, 0x19);
    LIS.openTemp();
    delay(100);
    LIS.setFullScaleRange(LIS3DHTR_RANGE_2G);
    LIS.setOutputDataRate(LIS3DHTR_DATARATE_50HZ);
    dht.begin();

    wk = new WonderK210(&Serial1);
    result = new Find_Box_st();

    pinMode(LIGHT_PIN, INPUT);

    pinMode(USER_BUTTON_A_PIN, INPUT_PULLUP);
    pinMode(USER_BUTTON_B_PIN, INPUT_PULLUP);

    pinMode(SERVO1_PIN, OUTPUT);
    pinMode(SERVO2_PIN, OUTPUT);
    g_servo_timer = timerBegin(SERVO_TIMER_FREQUENCY);
    timerAttachInterrupt(g_servo_timer, &on_servo_timer);
    timerAlarm(g_servo_timer, SERVO_TIMER_TICK_US, true, 0);
    Servos_SetAngle(1, 90);
    Servos_SetAngle(2, 90);

    uint8_t motorPins[] = {LF_MOTOR_FWD_PWM, LF_MOTOR_REV_PWM, RF_MOTOR_FWD_PWM, RF_MOTOR_REV_PWM,
                           LR_MOTOR_FWD_PWM, LR_MOTOR_REV_PWM, RR_MOTOR_FWD_PWM, RR_MOTOR_REV_PWM};
    for (uint8_t pin : motorPins)
    {
        pinMode(pin, OUTPUT);
        analogWriteFrequency(pin, 5000);
    }

    FastLED.addLeds<LED_TYPE, RGB_PIN, COLOR_ORDER>(leds, NUM_RGB_LEDS).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(RGB_BRIGHTNESS);
    fill_solid(leds, NUM_RGB_LEDS, CRGB::Black);
    FastLED.show();

    delay(1000);
    ps2_error = ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_CS_PIN, PS2_DATA_PIN, true, true);
    if (ps2_error == 0)
        Serial.println("PS2手柄配置成功。");
    else
    {
        Serial.print("PS2手柄配置失败，错误码: ");
        Serial.println(ps2_error);
    }

    printMenu();
}


void loop()
{
    if (Serial.available() > 0)
    {
        char input[16];
        int index = 0;

        while (Serial.available() && index < sizeof(input) - 1)
        {
            char c = Serial.read();
            if (c == '\n')
                break;
            input[index++] = c;
        }
        input[index] = '\0';

        while (Serial.available())
            Serial.read();

        char *trimmed = input;
        while (*trimmed == ' ')
            trimmed++;
        char *end = trimmed + strlen(trimmed) - 1;
        while (end >= trimmed && *end == ' ')
            *end-- = '\0';

        struct TestCase
        {
            int id;
            const char *name;
            void (*testFunc)();
        };
        const TestCase tests[] = {
            {1, "串口通信测试 (PC)", testSerialPC},
            {2, "串口通信测试 (K210)", testSerialK210},
            {3, "串口通信测试 (ASR-PRO)", testSerialASR},
            {4, "软串口测试", testSoftSerial},
            {5, "I2C 测试", testI2C},
            {6, "Grove 模拟接口测试", testAnalogInterfaces},
            {7, "Grove 数字接口测试", testGroveInterfaces},
            {8, "PS2 手柄通信测试", testPS2},
            {9, "麦克纳姆轮电机测试", testMecanumMotors},
            {10, "舵机功能测试", testServos},
            {11, "RGB LED 功能测试", testRGB_FastLED},
            {12, "WiFi 功能测试", testWiFi},
            {13, "蓝牙低功耗 (BLE) 测试", testBLE},
            {14, "用户按键测试", testButtons}};
        const int testCount = sizeof(tests) / sizeof(tests[0]);

        if (trimmed[0] == 'm' || trimmed[0] == 'M')
        {
            printMenu();
        }
        else if (trimmed[0] == 'a' || trimmed[0] == 'A')
        {
            Serial.println("\n--- 自动运行所有测试 ---");
            for (int i = 0; i < testCount; i++)
            {
                Serial.printf("开始测试: %s\n", tests[i].name);
                tests[i].testFunc();
                Serial.printf("结束测试: %s\n", tests[i].name);
                delay(500);
            }
            printMenu();
        }
        else
        {
            int choice = atoi(trimmed);
            bool valid = false;
            for (int i = 0; i < testCount; i++)
            {
                if (choice == tests[i].id)
                {
                    Serial.printf(" 开始测试: %s\n", tests[i].name);
                    tests[i].testFunc();
                    Serial.printf(" 结束测试: %s\n", tests[i].name);
                    valid = true;
                    break;
                }
            }
            if (!valid)
            {
                Serial.println("无效输入，请输入 1-14 或 'm' 显示菜单，'a' 运行所有测试。");
            }
            printMenu();
        }
    }
}
