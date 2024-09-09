#include <Wire.h>
//#include <Adafruit_VL53L0X.h>
#include <VL53L0X.h>
#include <Servo.h>
#include "movements.h"
#include "IR_remote.h"
#include "keymap.h"

IRremote ir(3);
//Adafruit_VL53L0X tof_sensor = Adafruit_VL53L0X();
VL53L0X tofSensor;

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

bool forward_flag;
bool backward_flag;
bool left_flag;
bool right_flag;
bool claw_close_flag;
bool claw_open_flag;
bool arm_forward_flag;
bool claw_recracted_flag;
bool base_anticlockwise_flag;
bool base_clockwise_flag;
bool menory_action_flag;
bool Line_tracking_Function_flag;

uint8_t is_left_black;
uint8_t is_right_black;
uint8_t is_center_black;

bool mode_1_flag;
bool mode_2_flag;
bool mode_3_flag;
bool mode_4_flag;

bool searching;

bool isMetal;
bool isNonmetal;

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

int capacityVal;
int inductiveVal;

unsigned long lastMillis = 0;

int claw_read_degress[20] = {0, 0, 0};
int arm_read_degress[20] = {0, 0, 0};
int base_read_degress[20] = {0, 0, 0};

Servo clawServo;
Servo armServo;
Servo baseServo;

void read_degress()
{
    if (actions_count <= 19)
    {
        claw_read_degress[(int)((actions_count + 1) - 1)] = clawServo.read();
        delay(50);
        arm_read_degress[(int)((actions_count + 1) - 1)] = armServo.read();
        delay(50);
        base_read_degress[(int)((actions_count + 1) - 1)] = baseServo.read();
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
        Move_Forward(80);
        delay(100);
        Stop();
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_DOWN)
    {
        Move_Backward(80);
        delay(100);
        Stop();
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_LEFT)
    {
        Rotate_Left(80);
        delay(100);
        Stop();
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_RIGHT)
    {
        Rotate_Right(80);
        delay(100);
        Stop();
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_OK)
    {
        Stop();
        AutoPick();
    }
    /*
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
        clawServo.write(claw_degrees);
        delay(2);
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_9)
    {
        claw_degrees = claw_degrees - 5;
        if (claw_degrees <= 50)
        {
            claw_degrees = 50;
        }
        clawServo.write(claw_degrees);
        delay(2);
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_2)
    {
        arm_degrees = arm_degrees + 5;
        if (arm_degrees >= 180)
        {
            arm_degrees = 180;
        }
        armServo.write(arm_degrees);
        delay(2);
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_8)
    {
        arm_degrees = arm_degrees - 5;
        if (arm_degrees <= 0)
        {
            arm_degrees = 0;
        }
        armServo.write(arm_degrees);
        delay(2);
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_4)
    {
        base_degrees = base_degrees + 5;
        if (base_degrees >= 180)
        {
            base_degrees = 180;
        }
        baseServo.write(base_degrees);
        delay(2);
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_6)
    {
        base_degrees = base_degrees - 5;
        if (base_degrees <= 0)
        {
            base_degrees = 0;
        }
        baseServo.write(base_degrees);
        delay(2);
    }
    */
}

void AutoPick() {
    bool metal;
    bool plastic;
    if(CapacitiveTrue() && InductiveTrue()) {
        metal = true;
        plastic = false;
        Serial.println("Metal Detected");
    } else if(CapacitiveTrue() && !InductiveTrue()) {
        metal = false;
        plastic = true;
        Serial.println("Plastic Detected");
    }
    delay(1000);
    Move_Backward(80);
    delay(500);
    Stop();

    ArmReadyGrabPos();
    Serial.println("Done");

    int tof = CheckTOFDist();

    while(tof > 40) {
        tof = CheckTOFDist();
        Move_Forward(35);
    }

    Stop();

    if(metal) {
        PickMetal();
    } else if(plastic) {
        PickNonMetal();
    }
}

// This function is for IR auto function
void IR_AUTO_FUNCTION()
{
    /*
     *   If the keyvalue 1 is pressed = MODE 1 (Non-metal Only)
     *   If the keyvalue 2 is pressed = MODE 2 (Metal Only)
     *   If the keyvalue 3 is pressed = MODE 3 (Both Metal and Non-metal)
     */

    if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_1)
    {
        mode_2_flag = mode_3_flag = mode_4_flag = false;
        mode_1_flag = true;
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_2)
    {
        mode_1_flag = mode_3_flag = mode_4_flag = false;
        mode_2_flag = true;
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_3)
    {
        mode_1_flag = mode_2_flag = mode_4_flag = false;
        mode_3_flag = true;
    }
    else if (ir.getIrKey(ir.getCode(), 1) == IR_KEYCODE_4)
    {
        mode_1_flag = mode_2_flag = mode_3_flag = false;
        mode_4_flag = true;
    }
}

void claw_close()
{
    claw_close_flag = true;
    while (claw_close_flag)
    {
        claw_degrees = claw_degrees + 1;
        clawServo.write(claw_degrees);
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
        clawServo.write(claw_degrees);
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
        armServo.write(arm_degrees);
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
        armServo.write(arm_degrees);
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

void arm_base_anticlockwise()
{
    base_anticlockwise_flag = true;
    while (base_anticlockwise_flag)
    {
        base_degrees = base_degrees + 1;
        baseServo.write(base_degrees);
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
        baseServo.write(base_degrees);
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
    Left_Tra_Value = digitalRead(7);
    Center_Tra_Value = digitalRead(8);
    Right_Tra_Value = digitalRead(A1);

    if (Left_Tra_Value != Black_Line && (Center_Tra_Value == Black_Line && Right_Tra_Value != Black_Line))
    {
        Move_Forward(60);
    }
    else if (Left_Tra_Value == Black_Line && (Center_Tra_Value == Black_Line && Right_Tra_Value != Black_Line))
    {
        Rotate_Left(50);
    }
    else if (Left_Tra_Value == Black_Line && (Center_Tra_Value != Black_Line && Right_Tra_Value != Black_Line))
    {
        Rotate_Left(50);
    }
    else if (Left_Tra_Value != Black_Line && (Center_Tra_Value != Black_Line && Right_Tra_Value == Black_Line))
    {
        Rotate_Right(50);
    }
    else if (Left_Tra_Value != Black_Line && (Center_Tra_Value == Black_Line && Right_Tra_Value == Black_Line))
    {
        Rotate_Right(50);
    }
    else if (Left_Tra_Value != Black_Line && (Center_Tra_Value != Black_Line && Right_Tra_Value != Black_Line))
    {
        Stop();
    }
}

void CheckLine() {
    is_left_black = digitalRead(7);
    is_right_black = digitalRead(8);
    is_center_black = digitalRead(A1);

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

int CheckTOFDist() {
    if (tofSensor.timeoutOccurred())
    {
        Serial.print(" TIMEOUT");
    }

    return tofSensor.readRangeSingleMillimeters();
}



void ArmIdlePos() {

    for (int i = baseServo.read(); i > 90; i--) {
        baseServo.write(i);
        delay(10);
    }

    for (int i = armServo.read(); i > 0; i--)
    {
        armServo.write(i);
        delay(10);
    }

    for (int i = clawServo.read(); i < 90; i++) {
        clawServo.write(i);
        delay(10);
    }
}

void ArmReadyPickPos() {
    
    clawServo.write(90);
    delay(1000);
    
    for (int i = baseServo.read(); i > 90; i--) {
        baseServo.write(i);
        delay(10);
    }

    for (int i = armServo.read(); i < 128; i++) {
        armServo.write(i);
        delay(10);
    }
}

void ArmReadyGrabPos()
{

    clawServo.write(90);
    delay(1000);

    for (int i = baseServo.read(); i > 80; i--)
    {
        baseServo.write(i);
        delay(10);
    }

    for (int i = armServo.read(); i < 135; i++)
    {
        armServo.write(i);
        delay(10);
    }
}

void PickMetal() {

    clawServo.write(180);
    delay(1000);

    for (int i = armServo.read(); i > 0; i--) {
        armServo.write(i);
        delay(10);
    }

    for (int i = baseServo.read(); i > 0; i--) {
        baseServo.write(i);
        delay(10);
    }

    for (int i = clawServo.read(); i > 90; i--) {
        clawServo.write(i);
        delay(10);
    }

    for (int i = baseServo.read(); i < 80; i++)
    {
        baseServo.write(i);
        delay(10);
    }
}

void PickNonMetal()
{
    clawServo.write(180);
    delay(1000);

    for (int i = armServo.read(); i > 0; i--)
    {
        armServo.write(i);
        delay(10);
    }

    for (int i = baseServo.read(); i < 180; i++)
    {
        baseServo.write(i);
        delay(10);
    }

    for (int i = clawServo.read(); i > 90; i--)
    {
        clawServo.write(i);
        delay(10);
    }

    for (int i = baseServo.read(); i > 80; i--) {
        baseServo.write(i);
        delay(10);
    }
}

void Sort() {
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

void setup()
{
    Serial.begin(115200);
    Wire.begin();

    tofSensor.setTimeout(500);
    if (!tofSensor.init())
    {
        Serial.println("Failed to detect and initialize tofSensor!");
        while (1); {
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

        // flags variables
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
        Line_tracking_Function_flag = false;

        mode_1_flag = false;
        mode_2_flag = false;
        mode_3_flag = false;
        mode_4_flag = false;

        searching = true;

        isMetal = false;
        isNonmetal = false;

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

        Stop();

        // proximity sensor pins
        pinMode(A0, INPUT);
        pinMode(A3, INPUT_PULLUP);

        // pins for MOTORS
        pinMode(2, OUTPUT);
        pinMode(5, OUTPUT);
        pinMode(4, OUTPUT);
        pinMode(6, OUTPUT);

        // pin for line sensor
        pinMode(7, INPUT);
        pinMode(8, INPUT);
        pinMode(A1, INPUT);

        pinMode(12, OUTPUT);
        pinMode(13, INPUT);

        while (!Serial)
        {
        delay(1);
    }

    // Serial.println("Adafruit VL53L0X test.");
    // if (!tof_sensor.begin())
    // {
    //     Serial.println(F("Failed to boot VL53L0X"));
    //     while (1);
    // }

    // power
    // Serial.println(F("VL53L0X API Continuous Ranging example\n\n"));

    // start continuous ranging
    // tof_sensor.startRangeContinuous();

    // servo attachments
    clawServo.attach(9);
    armServo.attach(10);
    baseServo.attach(11);

    delay(1000);

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
        // case 'c':
        //     claw_close();
        //     break;
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
        // case 'm':
        //     Serial.println(actions_count);
        //     read_degress();
        //     break;
        case 'a':
            //auto_do();
            Stop();
            AutoPick();
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

/**
 * @brief

 while(searching == true) {
        ArmIdlePos();
        bool capVal = CapacitiveTrue();
        bool indVal = InductiveTrue();
        float sonicVal = CheckSonicDist();
        int tofVal = CheckTOFDist();

        Serial.println("Sonic = " + String(sonicVal));
        Serial.println("TOF = " + String(tofVal));

        if (capVal == false && sonicVal > 3.50)
        {
            Move_Forward(50);
        }
        else if (capVal == true && sonicVal < 3.50 || capVal == true && indVal == true)
        {
            Stop();
            bool capValCheck = CapacitiveTrue();
            bool indValCheck = InductiveTrue();
            delay(2000);
            if (capValCheck == true && indValCheck == true)
            {
                isMetal = true;
                isNonmetal = false;
            }
            else if (capValCheck == true && indValCheck == false)
            {
                isMetal = false;
                isNonmetal = true;
            }
            Move_Backward(50);
            delay(1000);
            ArmReadyGrabPos();
            searching = false;
        }
    }

    while(searching == false) {
        bool capVal = CapacitiveTrue();
        bool indVal = InductiveTrue();
        float sonicVal = CheckSonicDist();
        int tofVal = CheckTOFDist();

        Serial.println("Sonic false = " + String(sonicVal));
        Serial.println("TOF false = " + String(tofVal));

        if(tofVal > 44) {
            Move_Forward(50);
            Serial.println("Looping here");
        } else if (tofVal < 43) {
            Stop();
            Serial.println("Try here");
            if(isMetal) {
                PickMetal();
            } else if(isNonmetal) {
                PickNonMetal();
            }
            isMetal = false;
            isNonmetal = false;
            searching = true;
        }
    }

 *
 */