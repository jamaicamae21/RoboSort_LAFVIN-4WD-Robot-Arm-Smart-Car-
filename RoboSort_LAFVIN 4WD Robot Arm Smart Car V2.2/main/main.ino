#include "IR_remote.h"
#include "keymap.h"
#include <Wire.h>
#include <VL53L0X.h>

IRremote ir(3);
VL53L0X tofSensor;

#include <Servo.h>

/*
 * Uncomment this line to use long range mode. This
 * increases the sensitivity of the sensor and extends its
 * potential range, but increases the likelihood of getting
 * an inaccurate reading because of reflections from objects
 * other than the intended target. It works best in dark
 * conditions.
 */

#define LONG_RANGE

// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

// #define HIGH_SPEED
#define HIGH_ACCURACY

String BLE_val;
int base_degrees;
int arm_degrees;
int claw_degrees;
boolean forward_flag;
boolean backward_flag;
boolean left_flag;
boolean right_flag;
boolean claw_close_flag;
boolean claw_open_flag;
boolean arm_forward_flag;
boolean claw_recracted_flag;
boolean base_anticlockwise_flag;
boolean base_clockwise_flag;
boolean menory_action_flag;
boolean Avoidance_Function_flag;
boolean Following_Function_flag;
boolean Anti_drop_Function_flag;
boolean Line_tracking_Function_flag;
boolean Gravity_sensor_Function_flag;
int Left_Tra_Value;
int Center_Tra_Value;
int Right_Tra_Value;
int Black_Line;
int distance;
int actions_count;
int auto_count;
int low_speed;
int medium_speed;
int high_speed;
int speed_car;

uint8_t is_left_black;
uint8_t is_right_black;
uint8_t is_center_black;

bool searching;

bool isMetal;
bool isNonmetal;

int claw_read_degress[20] = {0, 0, 0};
int arm_read_degress[20] = {0, 0, 0};
int base_read_degress[20] = {0, 0, 0};

int capacityVal;
int inductiveVal;

unsigned long lastMillis = 0;

Servo myservo1;
Servo myservo2;
Servo myservo3;

void read_degress()
{
    if (actions_count <= 19)
    {
        claw_read_degress[(int)((actions_count + 1) - 1)] = myservo1.read();
        delay(50);
        arm_read_degress[(int)((actions_count + 1) - 1)] = myservo2.read();
        delay(50);
        base_read_degress[(int)((actions_count + 1) - 1)] = myservo3.read();
        delay(50);
        actions_count = actions_count + 1;
        auto_count = actions_count;
        Serial.println(auto_count);
    }
}

void IR_control_Function()
{
    if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_UP)
    {
        Move_Forward(120);
        delay(300);
        Stop();
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_DOWN)
    {
        Move_Backward(120);
        delay(300);
        Stop();
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_LEFT)
    {
        Rotate_Left(110);
        delay(300);
        Stop();
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_RIGHT)
    {
        Rotate_Right(110);
        delay(300);
        Stop();
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_OK)
    {
        Stop();
    }
    else if (false)
    {
    }
    else if (false)
    {
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_7)
    {
        claw_degrees = claw_degrees + 5;
        if (claw_degrees >= 180)
        {
            claw_degrees = 180;
        }
        myservo1.write(claw_degrees);
        delay(2);
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_9)
    {
        claw_degrees = claw_degrees - 5;
        if (claw_degrees <= 50)
        {
            claw_degrees = 50;
        }
        myservo1.write(claw_degrees);
        delay(2);
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_2)
    {
        arm_degrees = arm_degrees + 5;
        if (arm_degrees >= 180)
        {
            arm_degrees = 180;
        }
        myservo2.write(arm_degrees);
        delay(2);
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_8)
    {
        arm_degrees = arm_degrees - 5;
        if (arm_degrees <= 0)
        {
            arm_degrees = 0;
        }
        myservo2.write(arm_degrees);
        delay(2);
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_4)
    {
        base_degrees = base_degrees + 5;
        if (base_degrees >= 180)
        {
            base_degrees = 180;
        }
        myservo3.write(base_degrees);
        delay(2);
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_6)
    {
        base_degrees = base_degrees - 5;
        if (base_degrees <= 0)
        {
            base_degrees = 0;
        }
        myservo3.write(base_degrees);
        delay(2);
    }
}

void AutoPickAndSort()
{
    bool metal;
    bool plastic;
    if (CapacitiveTrue() && InductiveTrue())
    {
        metal = true;
        plastic = false;
        Serial.println("Metal Detected");
    }
    else if (CapacitiveTrue() && !InductiveTrue())
    {
        metal = false;
        plastic = true;
        Serial.println("Plastic Detected");
    }
    delay(1000);
    Move_Backward(85);
    delay(510);
    Stop();

    ArmReadyGrabPos();
    Serial.println("Done");

    Move_Forward(45);
    delay(350);
    
    // int tof = CheckTOFDist();
    
    // while (tof > 43.0)
    // {
    //     tof = CheckTOFDist();
    //     Move_Forward(45);
    // }

    Stop();

    if (metal)
    {
        PickMetal();
    }
    else if (plastic)
    {
        PickNonMetal();
    }
}

void auto_do()
{
    Serial.println(auto_count);
    if (0 != auto_count)
    {
        menory_action_flag = true;
    }
    actions_count = 0;
    claw_degrees = myservo1.read();
    arm_degrees = myservo2.read();
    base_degrees = myservo3.read();
    while (menory_action_flag)
    {
        for (int i = (1); i <= (auto_count); i = i + (1))
        {
            if (Serial.read() == 's')
            {
                menory_action_flag = false;
                break;
            }
            if (claw_degrees < claw_read_degress[(int)(i - 1)])
            {
                while (claw_degrees < claw_read_degress[(int)(i - 1)])
                {
                    claw_degrees = claw_degrees + 1;
                    myservo1.write(claw_degrees);
                    delay(15);
                }
            }
            else
            {
                while (claw_degrees > claw_read_degress[(int)(i - 1)])
                {
                    claw_degrees = claw_degrees - 1;
                    myservo1.write(claw_degrees);
                    delay(15);
                }
            }
            if (Serial.read() == 's')
            {
                menory_action_flag = false;
                break;
            }
            if (arm_degrees < arm_read_degress[(int)(i - 1)])
            {
                while (arm_degrees < arm_read_degress[(int)(i - 1)])
                {
                    arm_degrees = arm_degrees + 1;
                    myservo2.write(arm_degrees);
                    delay(15);
                }
            }
            else
            {
                while (arm_degrees > arm_read_degress[(int)(i - 1)])
                {
                    arm_degrees = arm_degrees - 1;
                    myservo2.write(arm_degrees);
                    delay(15);
                }
            }
            if (Serial.read() == 's')
            {
                menory_action_flag = false;
                break;
            }
            if (base_degrees < base_read_degress[(int)(i - 1)])
            {
                while (base_degrees < base_read_degress[(int)(i - 1)])
                {
                    base_degrees = base_degrees + 1;
                    myservo3.write(base_degrees);
                    delay(15);
                }
            }
            else
            {
                while (base_degrees > base_read_degress[(int)(i - 1)])
                {
                    base_degrees = base_degrees - 1;
                    myservo3.write(base_degrees);
                    delay(15);
                }
            }
            if (Serial.read() == 's')
            {
                menory_action_flag = false;
                break;
            }
        }
    }
}

void claw_close()
{
    claw_close_flag = true;
    while (claw_close_flag)
    {
        claw_degrees = claw_degrees + 1;
        myservo1.write(claw_degrees);
        Serial.println(claw_degrees);
        delay(10);
        if (claw_degrees >= 180)
        {
            claw_degrees = 180;
        }
        if (Serial.read() == 's')
        {
            claw_close_flag = false;
        }
    }
}

void claw_open()
{
    claw_close_flag = true;
    while (claw_close_flag)
    {
        claw_degrees = claw_degrees - 1;
        myservo1.write(claw_degrees);
        Serial.println(claw_degrees);
        delay(10);
        if (claw_degrees <= 50)
        {
            claw_degrees = 50;
        }
        if (Serial.read() == 's')
        {
            claw_close_flag = false;
        }
    }
}

void arm_up()
{
    arm_forward_flag = true;
    while (arm_forward_flag)
    {
        arm_degrees = arm_degrees + 1;
        myservo2.write(arm_degrees);
        delay(10);
        Serial.println(arm_degrees);
        if (arm_degrees >= 180)
        {
            arm_degrees = 180;
        }
        if (Serial.read() == 's')
        {
            arm_forward_flag = false;
        }
    }
}

void arm_down()
{
    claw_recracted_flag = true;
    while (claw_recracted_flag)
    {
        arm_degrees = arm_degrees - 1;
        myservo2.write(arm_degrees);
        Serial.println(arm_degrees);
        delay(10);
        if (arm_degrees <= 0)
        {
            arm_degrees = 0;
        }
        if (Serial.read() == 's')
        {
            claw_recracted_flag = false;
        }
    }
}

void Stop()
{
    digitalWrite(2, LOW);
    analogWrite(5, 0);
    digitalWrite(4, HIGH);
    analogWrite(6, 0);
}

void arm_base_anticlockwise()
{
    base_anticlockwise_flag = true;
    while (base_anticlockwise_flag)
    {
        base_degrees = base_degrees + 1;
        myservo3.write(base_degrees);
        Serial.println(base_degrees);
        delay(10);
        if (base_degrees >= 180)
        {
            base_degrees = 180;
        }
        if (Serial.read() == 's')
        {
            base_anticlockwise_flag = false;
        }
    }
}

void arm_base_clockwise()
{
    base_clockwise_flag = true;
    while (base_clockwise_flag)
    {
        base_degrees = base_degrees - 1;
        myservo3.write(base_degrees);
        Serial.println(base_degrees);
        delay(10);
        if (base_degrees <= 0)
        {
            base_degrees = 0;
        }
        if (Serial.read() == 's')
        {
            base_clockwise_flag = false;
        }
    }
}

void Line_tracking_Function()
{
    Line_tracking_Function_flag = true;
    
    while (Line_tracking_Function_flag)
    {
        Left_Tra_Value = digitalRead(7);
        Center_Tra_Value = digitalRead(8);
        Right_Tra_Value = digitalRead(A1);
        if (Left_Tra_Value != Black_Line && (Center_Tra_Value == Black_Line && Right_Tra_Value != Black_Line))
        {
            if(CapacitiveTrue() || CheckSonicDist() < 3) {
                Stop();
                AutoPickAndSort();
            } else {
                Move_Forward(60);
            }
        }
        else if (Left_Tra_Value == Black_Line && (Center_Tra_Value == Black_Line && Right_Tra_Value != Black_Line))
        {
            Rotate_Left(85);
        }
        else if (Left_Tra_Value == Black_Line && (Center_Tra_Value != Black_Line && Right_Tra_Value != Black_Line))
        {
            Rotate_Left(110);
        }
        else if (Left_Tra_Value != Black_Line && (Center_Tra_Value != Black_Line && Right_Tra_Value == Black_Line))
        {
            Rotate_Right(110);
        }
        else if (Left_Tra_Value != Black_Line && (Center_Tra_Value == Black_Line && Right_Tra_Value == Black_Line))
        {
            Rotate_Right(85);
        }
        else if (Left_Tra_Value == Black_Line && (Center_Tra_Value == Black_Line && Right_Tra_Value == Black_Line))
        {
            Stop();
        }
        if (Serial.read() == 'S')
        {
            Line_tracking_Function_flag = false;
            Stop();
        }
    }
}

void Move_Backward(int speed)
{
    digitalWrite(2, LOW);
    analogWrite(5, speed);
    digitalWrite(4, HIGH);
    analogWrite(6, speed);
}

float checkdistance()
{
    digitalWrite(12, LOW);
    delayMicroseconds(2);
    digitalWrite(12, HIGH);
    delayMicroseconds(10);
    digitalWrite(12, LOW);
    float distance = pulseIn(13, HIGH) / 58.00;
    delay(10);
    return distance;
}

void Following_Function()
{
    int Following_distance = 0;
    Following_Function_flag = true;
    while (Following_Function_flag)
    {
        Following_distance = checkdistance();
        if (Following_distance < 15)
        {
            Move_Backward(100);
        }
        else if (15 <= Following_distance && Following_distance <= 20)
        {
            Stop();
        }
        else if (20 <= Following_distance && Following_distance <= 25)
        {
            Move_Forward(100);
        }
        else if (25 <= Following_distance && Following_distance <= 30)
        {
            Move_Forward(120);
        }
        else
        {
            Stop();
        }
        if (Serial.read() == 'S')
        {
            Following_Function_flag = false;
            Stop();
        }
    }
}

void Rotate_Left(int speed)
{
    digitalWrite(2, LOW);
    analogWrite(5, speed + 30);
    digitalWrite(4, LOW);
    analogWrite(6, speed + 30);
}

void Rotate_Right(int speed)
{
    digitalWrite(2, HIGH);
    analogWrite(5, speed + 30);
    digitalWrite(4, HIGH);
    analogWrite(6, speed + 30);
}

void Move_Forward(int speed)
{
    digitalWrite(2, HIGH);
    analogWrite(5, speed);
    digitalWrite(4, LOW);
    analogWrite(6, speed);
}

void Anti_drop_Function()
{
    Anti_drop_Function_flag = true;
    while (Anti_drop_Function_flag)
    {
        Left_Tra_Value = digitalRead(7);
        Center_Tra_Value = digitalRead(8);
        Right_Tra_Value = digitalRead(A1);
        if (Left_Tra_Value != Black_Line && (Center_Tra_Value != Black_Line && Right_Tra_Value != Black_Line))
        {
            Move_Forward(90);
        }
        else
        {
            Move_Backward(90);
            delay(600);
            Rotate_Left(100);
            delay(500);
        }
        if (Serial.read() == 'S')
        {
            Anti_drop_Function_flag = false;
            Stop();
        }
    }
}

void Move_backward_Function()
{
    backward_flag = true;
    while (backward_flag)
    {
        Move_Backward(speed_car);
        if (Serial.read() == 'S')
        {
            backward_flag = false;
            Stop();
        }
    }
}

void Move_forward_Function()
{
    forward_flag = true;
    while (forward_flag)
    {
        Move_Forward(speed_car);
        if (Serial.read() == 'S')
        {
            forward_flag = false;
            Stop();
        }
    }
}

void Avoidance_Function()
{
    int Avoidance_distance = 0;
    Avoidance_Function_flag = true;
    while (Avoidance_Function_flag)
    {
        Avoidance_distance = checkdistance();
        if (Avoidance_distance <= 25)
        {
            if (Avoidance_distance <= 15)
            {
                Stop();
                delay(100);
                Move_Backward(120);
                delay(600);
                Rotate_Right(120);
                delay(200);
            }
            else
            {
                Stop();
                delay(100);
                Rotate_Left(120);
                delay(600);
            }
        }
        else
        {
            Move_Forward(90);
        }
        if (Serial.read() == 'S')
        {
            Avoidance_Function_flag = false;
            Stop();
        }
    }
}

void Turn_right_Function()
{
    right_flag = true;
    while (right_flag)
    {
        Rotate_Right(speed_car);
        if (Serial.read() == 'S')
        {
            right_flag = false;
            Stop();
        }
    }
}

void Turn_left_Function()
{
    left_flag = true;
    while (left_flag)
    {
        Rotate_Left(speed_car);
        if (Serial.read() == 'S')
        {
            left_flag = false;
            Stop();
        }
    }
}

char bluetooth_val;

bool CapacitiveTrue()
{
    int capacitive = digitalRead(A0);
    Serial.println("Capacitive = " + String(capacitive));
    if (capacitive == HIGH)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool InductiveTrue()
{
    int inductive = digitalRead(A3);
    Serial.println("Inductive = " + String(inductive));
    if (inductive == HIGH)
    {
        return true;
    }
    else
    {
        return false;
    }
}

float CheckSonicDist()
{
    digitalWrite(12, LOW);
    delayMicroseconds(2);
    digitalWrite(12, HIGH);
    delayMicroseconds(10);
    digitalWrite(12, LOW);
    float distance = pulseIn(13, HIGH) / 58.00;
    delay(10);
    return distance;
}

int CheckTOFDist()
{
    if (tofSensor.timeoutOccurred())
    {
        Serial.print(" TIMEOUT");
    }

    return tofSensor.readRangeSingleMillimeters();
}

void ArmIdlePos()
{

    for (int i = myservo3.read(); i > 83; i--)
    {
        myservo3.write(i);
        delay(10);
    }

    for (int i = myservo2.read(); i > 0; i--)
    {
        myservo2.write(i);
        delay(10);
    }

    for (int i = myservo1.read(); i < 90; i++)
    {
        myservo1.write(i);
        delay(10);
    }
}

void ArmReadyGrabPos()
{

    myservo1.write(90);
    delay(1000);

    for (int i = myservo3.read(); i > 83; i--)
    {
        myservo3.write(i);
        delay(10);
    }

    for (int i = myservo2.read(); i < 135; i++)
    {
        myservo2.write(i);
        delay(10);
    }
}

void PickMetal()
{

    myservo1.write(180);
    delay(1000);

    for (int i = myservo2.read(); i > 0; i--)
    {
        myservo2.write(i);
        delay(10);
    }

    for (int i = myservo3.read(); i > 0; i--)
    {
        myservo3.write(i);
        delay(10);
    }

    for (int i = myservo1.read(); i > 90; i--)
    {
        myservo1.write(i);
        delay(10);
    }

    for (int i = myservo3.read(); i < 83; i++)
    {
        myservo3.write(i);
        delay(10);
    }
}

void PickNonMetal()
{
    myservo1.write(180);
    delay(1000);

    for (int i = myservo2.read(); i > 0; i--)
    {
        myservo2.write(i);
        delay(10);
    }

    for (int i = myservo3.read(); i < 180; i++)
    {
        myservo3.write(i);
        delay(10);
    }

    for (int i = myservo1.read(); i > 90; i--)
    {
        myservo1.write(i);
        delay(10);
    }

    for (int i = myservo3.read(); i > 83; i--)
    {
        myservo3.write(i);
        delay(10);
    }
}

void Sort()
{
    Serial.println(CheckSonicDist());
    if (CapacitiveTrue())
    {
        delay(2000);
        if (InductiveTrue())
        {
            ArmReadyGrabPos();
            PickMetal();
        }
        else
        {
            ArmReadyGrabPos();
            PickNonMetal();
        }
        ArmIdlePos();
    }
}

void Gravity_sensor_Function()
{
    Gravity_sensor_Function_flag = true;
    while (Gravity_sensor_Function_flag)
    {
        if (Serial.available())
        {
            bluetooth_val = Serial.read();
            Serial.println(bluetooth_val);
            if (bluetooth_val == 'F')
            {
                Move_Forward(speed_car);
            }
            else if (bluetooth_val == 'B')
            {
                Move_Backward(speed_car);
            }
            else if (bluetooth_val == 'L')
            {
                Rotate_Left(speed_car);
            }
            else if (bluetooth_val == 'R')
            {
                Rotate_Right(speed_car);
            }
            else if (bluetooth_val == 'p')
            {
                Stop();
            }
            else if (bluetooth_val == 'X')
            {
                speed_car = low_speed;
            }
            else if (bluetooth_val == 'Y')
            {
                speed_car = medium_speed;
            }
            else if (bluetooth_val == 'Z')
            {
                speed_car = high_speed;
            }
            if (bluetooth_val == 'S')
            {
                Gravity_sensor_Function_flag = false;
                Stop();
            }
        }
    }
}

void setup()
{
    IRremote ir(3);

    Serial.begin(9600);
    Wire.begin();

    tofSensor.setTimeout(500);
    if (!tofSensor.init())
    {
        Serial.println("Failed to detect and initialize tofSensor!");
        while (1)
            ;
        {
        }
    }

    #if defined LONG_RANGE
        // lower the return signal rate limit (default is 0.25 MCPS)
        tofSensor.setSignalRateLimit(0.1);
        // increase laser pulse periods (defaults are 14 and 10 PCLKs)
        tofSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
        tofSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    #endif

    #if defined HIGH_ACCURACY
        // increase timing budget to 200 ms
        tofSensor.setMeasurementTimingBudget(200000);

    #elif defined HIGH_SPEED
        // reduce timing budget to 20 ms (default is about 33 ms)
        tofSensor.setMeasurementTimingBudget(20000);
    #endif

    BLE_val = "";
    base_degrees = 90;
    arm_degrees = 90;
    claw_degrees = 90;
    forward_flag = false;
    backward_flag = false;
    left_flag = false;
    right_flag = false;
    claw_close_flag = false;
    claw_open_flag = false;
    arm_forward_flag = false;
    claw_recracted_flag = false;
    base_anticlockwise_flag = false;
    base_clockwise_flag = false;
    menory_action_flag = false;
    Avoidance_Function_flag = false;
    Following_Function_flag = false;
    Anti_drop_Function_flag = false;
    Line_tracking_Function_flag = false;
    Gravity_sensor_Function_flag = false;
    Left_Tra_Value = 1;
    Center_Tra_Value = 1;
    Right_Tra_Value = 1;
    Black_Line = 1;
    distance = 0;
    actions_count = 0;
    auto_count = 0;
    low_speed = 80;
    medium_speed = 120;
    high_speed = 160;
    speed_car = 80;

    searching = true;

    isMetal = false;
    isNonmetal = false;

    myservo1.attach(9);
    myservo2.attach(10);
    myservo3.attach(11);
    
    Stop();

    pinMode(A0, INPUT);
    pinMode(A3, INPUT_PULLUP);

    pinMode(2, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(4, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, INPUT);
    pinMode(8, INPUT);
    pinMode(A1, INPUT);
    pinMode(12, OUTPUT);
    pinMode(13, INPUT);

    ArmIdlePos();
}

void loop()
{
    while (Serial.available() > 0)
    {
        BLE_val = BLE_val + ((char)(Serial.read()));
        delay(2);
    }
    if (0 < String(BLE_val).length() && 2 >= String(BLE_val).length())
    {
        Serial.println(String(BLE_val).length());
        Serial.println(BLE_val);
        switch (String(BLE_val).charAt(0))
        {
        // case 'o':
        //     claw_open();
        //     break;
        case 'c':
            // claw_close();
            Stop();
            if(!CapacitiveTrue()) break;
            AutoPickAndSort();
            break;
        // case 'u':
        //     arm_up();
        //     break;
        // case 'd':
        //     arm_down();
        //     break;
        // case 'l':
        //     arm_base_anticlockwise();
        //     break;
        // case 'r':
        //     arm_base_clockwise();
        //     break;
        case 'F':
            Move_forward_Function();
            break;
        case 'B':
            Move_backward_Function();
            break;
        case 'L':
            Turn_left_Function();
            break;
        case 'R':
            Turn_right_Function();
            break;
        case 'S':
            Stop();
            break;
        case 'm':
            Serial.println(actions_count);
            read_degress();
            break;
        case 'a':
            auto_do();
            break;
        case 'X':
            speed_car = low_speed;
            break;
        case 'Y':
            speed_car = medium_speed;
            break;
        case 'Z':
            speed_car = high_speed;
            break;
        // case 'A':
        //     Avoidance_Function();
        //     break;
        // case 'D':
        //     Anti_drop_Function();
        //     break;
        // case 'W':
        //     Following_Function();
        //     break;
        case 'T':
            Line_tracking_Function();
            break;
        // case 'G':
        //     Gravity_sensor_Function();
        //     break;
        }
        BLE_val = "";
    }
    else
    {
        BLE_val = "";
    }
    IR_control_Function();
    // Serial.println("Base = " + String(myservo3.read()));
    // Serial.println("Arm = " + String(myservo2.read()));
    // Serial.println("Claw = " + String(myservo1.read()));
}