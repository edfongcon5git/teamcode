/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.I2cDevice;

import java.util.concurrent.locks.Lock;

import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * An example linear op mode where the robot will drive in
 * a straight line (where the driving direction is guided by
 * the Yaw angle from a navX-Model device).
 *
 * This example uses a simple PID controller configuration
 * with a P coefficient, and will likely need tuning in order
 * to achieve optimal performance.
 *
 * Note that for the best accuracy, a reasonably high update rate
 * for the navX-Model sensor should be used.  This example uses
 * the default update rate (50Hz), which may be lowered in order
 * to reduce the frequency of the updates to the drive system.
 */
@Autonomous(name = "AutoSingle Shot 00 To Ramp Untested", group = "Autonomous")
//@Disabled //Comment this in to remove this from the Driver Station OpMode List
public class AutonSingleShot_00_ToRampLOGADA extends LinearOpMode {

    // motors
    DcMotor front_left, front_right;
    DcMotor back_left, back_right;

    /* This is the port on the Core Device Interface Module        */
    /* in which the navX-Model Device is connected.  Modify this  */
    /* depending upon which I2C port you are using.               */
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private navXPIDController yawPIDController;
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private final double TARGET_ANGLE_DEGREES = 0.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.02; //gain
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.075;

    private boolean calibration_complete = false;

    static final double     COUNTS_PER_MOTOR_REV    = 1120/2 ;    // eg: andymark 20 motor, 1120 for 40 motor
    //   static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.4;
    static final double     TURN_SPEED              = 0.5;

    double position; //servo pos
    double yawValue1, yawValue2;

    DecimalFormat df;

    navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();
    int DEVICE_TIMEOUT_MS = 500;

    // Define LED class members
    DeviceInterfaceModule dim;
    static final int    BLUE_LED    = 0;     // Blue LED channel on DIM
    static final int    RED_LED     = 1;     // Red LED Channel on DIM

    private boolean GlobalDebugEnabled = false;

    //light sensor
    OpticalDistanceSensor lineSensor;

    //VEX light sensors
    AnalogInput lineSensorL, lineSensorR;

    //LEGO light sensors
    LightSensor lightSensorL, lightSensorR;

    //servo
    //This servo is used to push the beacon button
    Servo servo;

    //touch sensor
    TouchSensor wallSensor;

    //public enum beacon_returnPosition{
    static final int  BEACON_LEFT_CUSTOM = 1;
    static final int  BEACON_RIGHT_CUSTOM = 2;
    //};

    // public enum beacon_color{
    static final int BEACON_BLUE_CUSTOM = 3;
    static final int  BEACON_RED_CUSTOM = 4;

    boolean continue_loop;

    int distance_fl, distance_fr, distance_bl, distance_br;
    double bl, br, fl, fr;

    //////////////CATAPULT SERVO SYSTEM////////////////////////////////////////////

    //Catapult Servo
    Servo CatServo;
    //Ramp Servo- this controls the ball to transistion into the catapult cup
    Servo RampServo;

    OpticalDistanceSensor ODS;   // CATAPULT LIGHT SENSER
    // Servo test for "485 servo"
    double ServoPosition1;

    //Ramp Servo
    double ServoPosition2;
    boolean bool_gamepad1yPressed;
    boolean bool_gamepad1bPressed;
    static final double     ZERO_RAMP_PUSHBALL_SERVO_POSITION       = 0.00;

    //Collector
    DcMotor BallCollector;
    boolean bool_gamepad1xPressed;
    boolean bool_gamepad1aPressed;
    int BallCollectorServoPosition;
    Servo SpinServo;
    double SpinServoPos;

    //INITIALIZATION
    boolean targetRed;
    String target;


    double finalValL;
    double finalValR;

    double logSpeed;
    double logDistance;
    double logTimeouts;
    double logDirection;

    int     logTargetangle;
    int     logTimeoutN;


    static final int  DRV_TIMEOUT_1_SEC           = 1;
    static final int  DRV_TIMEOUT_2_SEC           = 2;
    static final int  DRV_TIMEOUT_3_SEC           = 3;
    static final int  DRV_TIMEOUT_4_SEC           = 4;
    static final int  DRV_TIMEOUT_5_SEC           = 5;


    static final int  NAVX_TIMEOUT_1_SEC          = 1;
    static final int  NAVX_TIMEOUT_2_SEC          = 2;
    static final int  NAVX_TIMEOUT_3_SEC          = 3;
    static final int  NAVX_TIMEOUT_4_SEC          = 4;
    static final int  NAVX_TIMEOUT_5_SEC          = 5;

    int     logFrontLeftPosition;
    int     logFrontRightPosition;
    int     logBackLeftPosition;
    int     logBackRightPosition;

    double  logLightSensorMiddle;
    double  logLightSensorLeft;
    double  logLightSensorRight;

    // DIRECTIONS
    // Forward           = 0
    // Left 45 Degrees   = 1
    // Left              = 2 //driveToLine default
    // Left -45 Degrees  = 3
    // Backward          = 4 //driveToWall default
    // Right -45 Degrees = 5
    // Right             = 6
    // Right 45 Degrees  = 7

    // Declare variables for the switch
    int     dim_status_byte;   // 8 bits to indicate whether input or output mode
    boolean dim_value_bit;;    // single bit for D7


    double  pushButtonSavedPosition;
    double  lightDifferenceReading;

    private final double  RIGHT_LIGHT_READS_LESS_POINT_1 = 0.1;

    ColorSensor colorSensorL;  // Hardware Device Object


    // bLedOn represents the state of the LED.
    boolean bLedOn = true;

    // hsvValues is an array that will hold the hue, saturation, and value information.
    float hsvValuesL[] = {0F,0F,0F};

    int redColorReading;
    int blueColorReading;
    int alphaColorReading;
    int greenColorReading;

    /**
     * Created by Chris D on 10/5/2016.
     *
     * Partially based on:
     * https://github.com/OliviliK/FTC_Library/blob/master/TCS34725_ColorSensor.java
     */


    // Registers
    static final int ENABLE = 0x80;
    static final int ATIME = 0x81;
    static final int CONTROL = 0x8F;
    static final int ID = 0x92;
    static final int STATUS = 0x93;
    static final int CDATAL = 0x94;

    // Default I2C address for multiplexer. The address can be changed to any
    // value from 0x70 to 0x77, so this line would need to be changed if a
    // non-default address is to be used.
    static final I2cAddr MUX_ADDRESS = new I2cAddr(0x70);
    private I2cDevice mux;
    private I2cDeviceSynch muxReader;

    // Only one color sensor is needed in code as the multiplexer switches
    // between the physical sensors
    private byte[] adaCache;
    // I2C address for color sensor
    static final I2cAddr ADA_ADDRESS = new I2cAddr(0x29);
    private I2cDevice ada;
    private I2cDeviceSynch adaReader;

    private int[] sensorPorts;

    public static int GAIN_1X =  0x00;
    public static int GAIN_4X =  0x01;
    public static int GAIN_16X = 0x02;
    public static int GAIN_60X = 0x03;

    public int[] crgbArray;

    /**
     * Initializes Adafruit color sensors on the specified ports of the I2C
     * multiplexer.
     *

     */
    public void MultiplexColorSensor(


    ) {
        //sensorPorts = ports;

        //mux = hardwareMap.i2cDevice.get(muxName);
        //muxReader = new I2cDeviceSynchImpl(mux, MUX_ADDRESS, false);
        //muxReader.engage();

        // Loop over the ports activating each color sensor
        //for (int i = 0; i < sensorPorts.length; i++) {
        // Write to given output port on the multiplexer
        //muxReader.write8(0x0, 1 << sensorPorts[0], true);

        ada = hardwareMap.i2cDevice.get("ADAColorL");
        adaReader = new I2cDeviceSynchImpl(ada, ADA_ADDRESS, false);
        adaReader.engage();

        final int time = integrationByte(100);
        adaReader.write8(ENABLE, 0x03, true);  // Power on and enable ADC
        adaReader.read8(ID);                   // Read device ID
        adaReader.write8(CONTROL, 1, true); // Set gain
        adaReader.write8(ATIME, time, true);   // Set integration time
        // }
    }







    /**
     * Set the integration time on all the color sensors
     * @param milliSeconds Time in millseconds
     */
    public void setIntegrationTime(double milliSeconds) {
        int val = integrationByte(milliSeconds);

        for (int i = 0; i < sensorPorts.length; i++) {
            muxReader.write8(0x0, 1 << sensorPorts[i], true);
            adaReader.write8(ATIME, val, true);
        }
    }

    private int integrationByte(double milliSeconds) {
        int count = (int)(milliSeconds/2.4);
        if (count<1)    count = 1;   // Clamp the time range
        if (count>256)  count = 256;
        return (256 - count);
    }

    // Un-needed?
    public void startPolling() {
        for (int i = 0; i < sensorPorts.length; i++) {
            muxReader.write8(0x0, 1 << sensorPorts[i], true);
            adaReader.read8(STATUS);
        }
    }

    /**
     * Retrieve the color read by the given color sensor
     *

     * @return Array containing the Clear, Red, Green, and Blue color values
     */
    public int[] getCRGB() {
        // Write to I2C port on the multiplexer
        //muxReader.write8(0x0, 1 << port, true);

        // Read color registers
        adaCache = adaReader.read(CDATAL, 8);

        // Combine high and low bytes
        int[] crgb = new int[4];
        for (int i=0; i<4; i++) {
            crgb[i] = (adaCache[2*i] & 0xFF) + (adaCache[2*i+1] & 0xFF) * 256;
        }
        return crgb;
    }














    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }

    public void init1() {

        // motors
        front_left = hardwareMap.dcMotor.get("front_left");
        front_right = hardwareMap.dcMotor.get("front_right");

        back_left = hardwareMap.dcMotor.get("back_left");
        back_right = hardwareMap.dcMotor.get("back_right");

        //front_left.setDirection(DcMotor.Direction.REVERSE);
        //back_left.setDirection(DcMotor.Direction.REVERSE);
        front_right.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.REVERSE);

        front_left.setMaxSpeed(2800); // max speed = 150 rpm this is for the andymark40 motors
        front_right.setMaxSpeed(2800); // max speed = 150 rpm this is for the andymark40 motors
        back_left.setMaxSpeed(2800); // max speed = 150 rpm this is for the andymark40 motors
        back_right.setMaxSpeed(2800); // max speed = 150 rpm this is for the andymark40 motors

        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //can be.FLOAT if required
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        dim = this.hardwareMap.deviceInterfaceModule.get("dim");

        //navx
        navx_device = AHRS.getInstance(this.hardwareMap.deviceInterfaceModule.get("dim"),
              NAVX_DIM_I2C_PORT,
              AHRS.DeviceDataType.kProcessedData,
              NAVX_DEVICE_UPDATE_RATE_HZ);

        //light sensor
        lineSensor = hardwareMap.opticalDistanceSensor.get("line");

        //VEX sensors
        lineSensorL = hardwareMap.analogInput.get("lineL");
        lineSensorR = hardwareMap.analogInput.get("lineR");

        //LEGO sensors
      //  lightSensorL = hardwareMap.lightSensor.get("lightL");
     //   lightSensorR = hardwareMap.lightSensor.get("lightR");

        //servo
        //This is for pushing the beacon button
        servo = hardwareMap.servo.get("servo");

        //touch sensor
        wallSensor =  hardwareMap.touchSensor.get("touch");

        //reset servo
        //This needs to be in the init function
        position = 0.45;
        servo.setPosition(position);

        ///////////////////////Related to the catapult system/////////////////
        CatServo = hardwareMap.servo.get("CatServo");
        //This is for catapult
        ServoPosition1 = 0.5;
        CatServo.setPosition(ServoPosition1);
        bool_gamepad1bPressed = false;

        RampServo = hardwareMap.servo.get("RampServo");
        ServoPosition2 = ZERO_RAMP_PUSHBALL_SERVO_POSITION;
        RampServo.setPosition(ServoPosition2);
        bool_gamepad1yPressed = false;

        ODS = hardwareMap.opticalDistanceSensor.get("CatODSensor");

        //////////BALL COLLECTOR////////////////
        BallCollector = hardwareMap.dcMotor.get("ball_collector");

        BallCollector.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BallCollector.setPower(0);

        BallCollector.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SpinServo = hardwareMap.servo.get("SpinServo");
        SpinServoPos = 0.5; //stopped
        SpinServo.setPosition(SpinServoPos);

        MultiplexColorSensor();

        // get a reference to our ColorSensor object.
        //colorSensorL = hardwareMap.colorSensor.get("ADAColorL");

        telemetry.addData("00 updated", "4-07 615pm");

        dim_status_byte = dim.getDigitalIOControlByte();
        dim_value_bit  = dim.getDigitalChannelState(7);

        //Switch Definitions
        //False = RED
        //True = BLUE

        if (dim_value_bit == false )
        {
            //Running RED
            targetRed = true;
            target = "RED";
        }
        else
        {
            //Running BLUE
            targetRed = false;
            target = "BLUE";
        }

        //if (gamepad1.x) { //blue target
        //    targetRed = false;
        //    target = "BLUE";
        //}else if(gamepad1.b) { //red target
        //    targetRed = true;
        //    target = "RED";
        //}else { //default to red target
        //    targetRed = true;
        //    target = "None selected! RED is default";
        //}

        telemetry.addData("Target", target);
    }


    public void NavXCalibrateYawErrorHandle() throws InterruptedException
    {
        // If calibration does not get done, set the red light
        dim.setLED(RED_LED,   true); // Red for even
        while ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (!calibration_complete) {
                //   telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        }
        dim.setLED(RED_LED,   false); // Red for even


        if (!calibration_complete) {
            telemetry.addData("navX", "Startup Calibration in Progress");
        }
        else
        {
            telemetry.addData("navX", "Startup Calibration in DONE");
        }

        navx_device.zeroYaw();
        sleep(500);  // wait a small time for Yaw to update.
        telemetry.addData("Yaw Init", df.format(navx_device.getYaw()));
        telemetry.update();

        idle();
        yawValue1 = navx_device.getYaw();
        idle();
        yawValue2 = navx_device.getYaw();
        // If the yawValue is not near 0, check if the yawValue is stuck
        //if (( yawValue1 > 5.0) || ( yawValue1 < -5.0 ))
        {
            // when zeroYaw is issued, saw it at 1.75
            while (   !( ( yawValue1 < 2.0) && ( yawValue1 > -2.0))
                    && ( yawValue1 == yawValue2 ) )
            {
                // Flash the LED in a loop and indicate failure
                dim.setLED(RED_LED,   true); // Red for even
                yawValue1 = navx_device.getYaw();
                telemetry.addData("Yaw Init STUCK1", df.format(navx_device.getYaw()));
                telemetry.update();
                sleep(1000);               // optional pause after each move
                dim.setLED(RED_LED,   false); // Red for even
                yawValue2 = navx_device.getYaw();
                sleep(1000);
                telemetry.addData("Yaw Init STUCK2", df.format(navx_device.getYaw()));
                telemetry.update();
            }
            telemetry.clear();
            telemetry.update();
        }
        dim.setLED(RED_LED,   false); // Red for even
    }

    private void CheckDebugEnabled() throws InterruptedException
    {
        runtime.reset();
        if(gamepad1.left_bumper)
        {
            while(gamepad1.left_bumper && runtime.seconds() < 5)
            {
                //wait
            }
        }
        if(gamepad1.left_bumper && runtime.seconds() >= 5)
        {
            GlobalDebugEnabled = true;
            telemetry.addData("DebugModeOn", "Wait 10 seconds");
            telemetry.update();
            runtime.reset();
            while(gamepad1.left_bumper && runtime.seconds() < 10)
            {
                //wait
            }
        }
    }

    private void DebugDisplayAndWait( int MilliSecondsToWait, int MotionType,
                                      double speed,
                                      double distance,
                                      double timeoutS,
                                      int direction
    ) throws InterruptedException {
        /* Conditional code, to use, add astrick and slash at end of this line #IF_DEBUG_DISPLAY */
        if (GlobalDebugEnabled == true) {
            if (MotionType == 1) {
                // Display straight drive mode and parameters
                telemetry.addData("drive function", MilliSecondsToWait);
                telemetry.addData("Arguments", "Running at %.1f , %.1f , %.1f , %d",
                        speed,
                        distance,
                        timeoutS,
                        direction);
            }
            if (MotionType == 2) {
                // Display the Naxv Turn and parameters
                telemetry.addData("navxturn function", MilliSecondsToWait);
                telemetry.addData("Arguments", "Running at %.1f , %d ",
                        timeoutS,
                        direction);

            }
            telemetry.update();
            sleep(MilliSecondsToWait);
            }
            /* Conditional code, end of the #IF_DEBUG_DISPLAY */
    }

    @Override
    public void runOpMode() throws InterruptedException {

        df = new DecimalFormat("#.##");
        // Setup the hardware map to the objects
        init1();
        // Navx calibrate and Yaw Error Handling
        NavXCalibrateYawErrorHandle();

        DbgLog.msg("ABCD  ---NavXCalibrateYawErrorHandle "  +  String.format(" %.1f", yawValue1)
                                                            +  String.format(" %.1f", yawValue2)
                  );

        /* Create a PID Controller which uses the Yaw Angle as input. */
        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        // yawPIDController may need to go into a function for
        //    each movement.
        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);		// THIS MAY NEED TO BE PLACE IN A FUNCTION AS INPUT
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);


        waitForStart();

        final double TOTAL_RUN_TIME_SECONDS = 50;
        yawPIDController.enable(true);

        dim_value_bit  = dim.getDigitalChannelState(7);

        //Switch Definitions
        //False = RED
        //True = BLUE

        if (dim_value_bit == false )
        {
            //Running RED
            targetRed = true;
            target = "RED";
            DbgLog.msg("ABCD  ------------------------------ AutonSingleShot_00_ToRampLOGADA Red");
        }
        else
        {
            //Running BLUE
            targetRed = false;
            target = "BLUE";
            DbgLog.msg("ABCD  ------------------------------ AutonSingleShot_00_ToRampLOGADA Blue");
        }



        //////////////////////
        // DRIVE FROM START //
        //////////////////////


        // DISTANCES
        // if distance > 0, it will go to that position (encoders have reached values)
        // if distance == -1, it will  go until it hits the wall (jerk in acceleration)
        // if distance == -2, -3, or -4 it will go until it sees the white line (light > 0.1)
        // -2 = MIDDLE SENSOR
        // -3 = LEFT SENSOR
        // -4 = RIGHT SENSOR

        // DIRECTIONS
        // Forward           = 0
        // Left 45 Degrees   = 1
        // Left              = 2 //driveToLine default
        // Left -45 Degrees  = 3
        // Backward          = 4 //driveToWall default
        // Right -45 Degrees = 5
        // Right             = 6
        // Right 45 Degrees  = 7

//          5                ^               3
//       -45 \               |  4           / -45
//             _________________________        _
//          | /        |\     /|        \        |
//          |/        _|_|___|_|_        \       |
//          |  _ '|   |_________|         \      |
//         /| | | |  |    |_|              \     |
//         || |_|====|                      |    |
//         ||   | |  | |    |       __    __|    |
//         ||   | |  | | <> |     ||<>||||| |    |18 INCHES
//    5 <- ||   |-|--| |   _|_____||  ||||| | ->2|
//         ||   | |  | | <>   <>  ||  ||||| |    |
//         ||   | |  | |   _______||__|||||_|    |
//         ||   | LJ | |   /|               |    |
//         \|    /<>\   | / |              /     |
//          |\   \__/___|___|             /      |
//          | \__________________________/      _|
//            /            0 |            \
//           6 +45           v           +45 1
//                         Battery
//                      <> are particles
//

        if (targetRed==true)
        {
            // was 20
            drive(.5, 12, DRV_TIMEOUT_2_SEC, 4); //drive , arg1 = speed, arg2 = inches, arg3 = timeout
            //arg4 = direction
            navxturn(-45, NAVX_TIMEOUT_3_SEC); //turn to 45 degrees, arg1 = angle, arg2 = timeout

            shootBall();

            navxturn(-45, NAVX_TIMEOUT_3_SEC);
            //drive(.5, 10, 5, 5);

            sleep(10000);

            drive(.5, 65, DRV_TIMEOUT_5_SEC, 5);

            //sleep(2500);

            drive(.45, 45, DRV_TIMEOUT_5_SEC, 6);
        }



        if (targetRed==false)
        { //since we start the robot off to the right when on
            drive(.5, 6, DRV_TIMEOUT_2_SEC,4); //drive right -45 degrees for 48 in, arg1 = speed, arg2 = inches, arg3 = timeout
            //arg4 = direction
            //sleep(3000);
            navxturn(40, NAVX_TIMEOUT_3_SEC); //turn to -90 degrees, arg1 = angle, arg2 = timeout

            //sleep(3000);
            shootBall();

            navxturn(45, NAVX_TIMEOUT_3_SEC);
            // sleep(3000);
            drive(.5, 8, DRV_TIMEOUT_5_SEC, 5);

            sleep(10000);

            drive(.5, 60, DRV_TIMEOUT_5_SEC, 3);

            //sleep(2500);

            drive(0.45,18,DRV_TIMEOUT_5_SEC, 2 );
            // drive(0.40, 8, 3, 4);
            //navxturn(-90, 3);
        }


        //stop all motion
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);

        //yawPIDController.enable(false);
        navx_device.close();
        telemetry.addData("LinearOp", "Complet Wait 4 secs");
        telemetry.addData("Path2", "Running at %7d :%7d :%7d :%7d",
                back_left.getCurrentPosition(),
                back_right.getCurrentPosition(),
                front_right.getCurrentPosition(),
                front_left.getCurrentPosition());
        telemetry.update();

        DbgLog.msg("ABCD  ---------------------END AutonDoubleShot_00_ToRampLOGADA");

        //sleep(4000); Don't use because of stop button not working with this in.;
    }

    public void shootBall() {
        if (!opModeIsActive())
            return;
        //move intake out of the way
        BallCollector.setPower(0.2);
        BallCollectorServoPosition = 120;
        BallCollector.setTargetPosition(BallCollectorServoPosition);
        sleep(300);


        //SHOOT FIRST BALL
        ServoPosition1 = 1;
        CatServo.setPosition(ServoPosition1); //move catapult motor

        sleep(300);
        while (ODS.getRawLightDetected() < 3.0) {
            //wait for bar to get back in place
        }

        //Stop rotation
        ServoPosition1 = 0.5;
        CatServo.setPosition(ServoPosition1);

        sleep(100); //wait 100 millis

//        //PUSH SECOND BALL INTO PLACE
//        ServoPosition2 = .3;
//        RampServo.setPosition(ServoPosition2);
//
//        sleep(800); //wait 500 millis for ball to be pushed
//
//        ServoPosition2 = ZERO_RAMP_PUSHBALL_SERVO_POSITION;
//        RampServo.setPosition(ServoPosition2);
//
//
//        //SHOOT SECOND BALL
//        ServoPosition1 = 1;
//        CatServo.setPosition(ServoPosition1); //move catapult motor
//
//        sleep(300);
//        while (ODS.getRawLightDetected() < 3.0) {
//            //wait for bar to get back in place
//        }
        //Stop rotation
        ServoPosition1 = 0.5;
        CatServo.setPosition(ServoPosition1);

        //put intake back
        BallCollector.setPower(0.2);
        BallCollectorServoPosition = 0;
        BallCollector.setTargetPosition(BallCollectorServoPosition);
        DbgLog.msg("ABCD  shootBall");
    }

    public void drive(double speed,
                             double distance,
                             double timeoutS,
                             int direction) throws InterruptedException {

        logSpeed = speed;
        logDistance = distance;
        logTimeouts = timeoutS;
        logDirection = direction;

        if (!opModeIsActive())
            return;
        // DRIVE FUNCTION:
        // drive(speed, distance, timeout, direction)

        // DISTANCES
        // if distance > 0, it will go to that position (encoders have reached values)
        // if distance == -1, it will  go until it hits the wall (jerk in acceleration)
        // if distance == -2, -3, or -4 it will go until it sees the white line (light > 0.1)
        // -2 = MIDDLE SENSOR
        // -3 = LEFT SENSOR
        // -4 = RIGHT SENSOR

        // DIRECTIONS
        // Forward           = 0
        // Left 45 Degrees   = 1
        // Left              = 2 //driveToLine default
        // Left -45 Degrees  = 3
        // Backward          = 4 //driveToWall default
        // Right -45 Degrees = 5
        // Right             = 6
        // Right 45 Degrees  = 7

//        DebugDisplayAndWait( 5000,1,speed,distance,timeoutS,direction);
        telemetry.addData("SpeedDrive",df.format(distance));
        telemetry.update();

        sleep(100);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(100);
        //wait for encoder values to get to zero

        if (distance > 0) { //if driving a specific distance
            distance_fl = front_left.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            distance_fr = front_right.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            distance_bl = back_left.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
            distance_br = back_right.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        }
        // if distance == -2, -3, or -4 it will go until it sees the white line (light < 3)
        if (distance < 0)
        {
            // Set max limit distance to keep from crashing in case the light does not work
            distance_fl = front_left.getCurrentPosition() + (int) (12 * COUNTS_PER_INCH);
            distance_fr = front_right.getCurrentPosition() + (int) (12 * COUNTS_PER_INCH);
            distance_bl = back_left.getCurrentPosition() + (int) (12 * COUNTS_PER_INCH);
            distance_br = back_right.getCurrentPosition() + (int) (12 * COUNTS_PER_INCH);
            // this actually doesnt do anything because if distance < 0, it wont check these values
            // (you can lower the timeout value instead if this is a problem)
        }

        double last_world_linear_accel_y = 0; //set values for acceleration detection
        double currentJerkY = 0;
        double curr_world_linear_accel_y;

        //control directions
        set_direction_speeds(direction, speed);

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            //navx_device.zeroYaw();

            // reset the timeout time and start motion.
            runtime.reset();
            front_left.setPower(fl);
            front_right.setPower(fr);
            back_left.setPower(bl);
            back_right.setPower(br);

            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            continue_loop = true;

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (    opModeIsActive()
                    && (runtime.seconds() < timeoutS)
                    && continue_loop)
            {

                //NEW NAVX CONTROL
                if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                    double output = yawPIDResult.getOutput();
                    front_left.setPower(limit(fl - output));
                    front_right.setPower(limit(fr + output));
                    back_left.setPower(limit(bl - output)); //adjust each motor value based on angle
                    back_right.setPower(limit(br + output));

                    telemetry.addData("PIDOutput", df.format(limit(speed + output)) + ", " +
                            df.format(limit(speed - output)));
                    telemetry.addData("Yaw", df.format(navx_device.getYaw()));


                    //SET CONDTIONS FOR STOPPING LOOP BASED ON MODE IT IS IN
                    if(distance > 0) { //if driving a specific distance
                        if ((Math.abs(back_right.getCurrentPosition() ) < Math.abs(distance_br))
                                && (Math.abs(back_left.getCurrentPosition() ) < Math.abs(distance_bl))
                                && (Math.abs(front_right.getCurrentPosition() ) < Math.abs(distance_fr))
                                && (Math.abs(back_left.getCurrentPosition() ) < Math.abs(distance_bl)))
                        {
                            continue_loop = true;
                        }else{
                            continue_loop = false;
                        }
                    }else if(distance == -1) { //if driving to wall
                        //curr_world_linear_accel_y = navx_device.getWorldLinearAccelY();
                        //currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
                        //last_world_linear_accel_y = curr_world_linear_accel_y;
                        //
                        //if(!(runtime.seconds() > 0.5 && (Math.abs(currentJerkY) >  0.2))) {
                        if(!wallSensor.isPressed()) {
                            continue_loop = true;
                        }else{
                            continue_loop = false;
                        }
                    }else if(distance == -2) { //if driving to line using light sensor
                        if (lineSensor.getLightDetected() < 0.1) {
                            continue_loop = true;
                        } else {
                            continue_loop = false;
                        }
                    }

                    // Display it for the driver.
                    telemetry.addData("Path1",  "Running to %7d :%7d", distance_bl, distance_br);
                    telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                            back_left.getCurrentPosition(),
                            back_right.getCurrentPosition(),
                            front_right.getCurrentPosition(),
                            front_left.getCurrentPosition());
                    telemetry.addData("main light sensor", ": " + String.format("%.2f", lineSensor.getLightDetected()));
                    telemetry.update();

                    // Allow time for other processes to run.
                    idle();
                } else{
                    /* A timeout occurred */
                    telemetry.addData("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }
            }  // end of while loop

            logFrontLeftPosition = front_left.getCurrentPosition();
            logFrontRightPosition = front_right.getCurrentPosition();
            logBackLeftPosition  = back_left.getCurrentPosition();
            logBackRightPosition = back_right.getCurrentPosition();

            // Stop all motion;
            front_left.setPower(0);
            front_right.setPower(0);
            back_left.setPower(0);
            back_right.setPower(0);

        }



        DbgLog.msg("ABCD  drive " + String.format(" %.1f,", logSpeed)
                                  + String.format(" %.1f,", logDistance)
                                  + String.format(" %.1f,", logTimeouts)
                                  + String.format(" %.1f", logDirection)
                   );
        DbgLog.msg("ABCD       MOTOR FRONT LEFT  "  + String.format(" %7d", distance_fl)
                                                   + String.format(" %7d", logFrontLeftPosition)
                  );

        DbgLog.msg("ABCD       MOTOR FRONT RIGHT " + String.format(" %7d", distance_fr)
                                                   + String.format(" %7d", logFrontRightPosition)
                  );

        DbgLog.msg("ABCD       MOTOR BACK LEFT   " + String.format(" %7d", distance_bl)
                                                   + String.format(" %7d", logBackLeftPosition)
                  );

        DbgLog.msg("ABCD       MOTOR BACK RIGHT  " + String.format(" %7d", distance_br)
                                                   + String.format(" %7d", logBackRightPosition)
                  );

        if(distance == -2)
        {
            //This is where you drive looking for the white line
            DbgLog.msg("ABCD       -2 LIGHT MIDDLE" + String.format(" %.2f", logLightSensorMiddle));
            DbgLog.msg("ABCD       -2 LIGHT LEFT  " + String.format(" %.2f", logLightSensorLeft));
            DbgLog.msg("ABCD       -2 LIGHT RIGHT " + String.format(" %.2f", logLightSensorRight));
        }

        if(distance == -1)
        {
            //This is when the button is pushed
            DbgLog.msg("ABCD       -1 LIGHT MIDDLE" + String.format(" %.2f", logLightSensorMiddle));
            DbgLog.msg("ABCD       -1 LIGHT LEFT  " + String.format(" %.2f", logLightSensorLeft));
            DbgLog.msg("ABCD       -1 LIGHT RIGHT " + String.format(" %.2f", logLightSensorRight));
        }


    }

    public void navxturn(int target_angle, int timeoutS) throws InterruptedException {

        logTargetangle = target_angle;
        logTimeoutN = timeoutS;

        if (!opModeIsActive())
            return;

        //      DebugDisplayAndWait( 5000,2,0,0,target_angle,timeoutS);
        telemetry.addData("Navturn",df.format(target_angle));
        telemetry.update();
        //sleep(1000);

//        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        yawPIDController.setSetpoint(target_angle);



        final double TOTAL_RUN_TIME_SECONDS = 10.0;

    //    yawPIDController.enable(true);

        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */

        runtime.reset();

        boolean BoolTurnComplete = false;

        while (    (runtime.time() < timeoutS)
                && ( BoolTurnComplete == false )
                && opModeIsActive()
                ) {
            if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS))
            {
                if (yawPIDResult.isOnTarget()) {
                    telemetry.addData("TURN ONTARGET Yaw", df.format(navx_device.getYaw()));
                    telemetry.update();
                    if(( Math.abs(target_angle-navx_device.getYaw()) < TOLERANCE_DEGREES)) {
                        BoolTurnComplete = true;
                    }



                } else {
                    double output = yawPIDResult.getOutput();
                    if (output < 0.08 && output > 0) {
                        output = 0.08;
                    }
                    if (output > -0.08 && output < 0) {
                        output = -0.08;
                    }
                    front_right.setPower(limit(output));
                    front_left.setPower(limit(-output));
                    back_right.setPower(limit(output));
                    back_left.setPower(limit(-output));
                    telemetry.addData("TURN PIDOutput", df.format(output) + ", " +
                            df.format(-output));
                    telemetry.addData("TURN Yaw", df.format(navx_device.getYaw()));
                    telemetry.update();

                }
            }
            else
            {
	            /* A timeout occurred */
                telemetry.addData("TURN PID TIMEOUT", df.format(runtime.time()));
                sleep(500);
               // BoolTurnComplete = true;
             }
        }
        telemetry.addData("TURN Time", df.format(runtime.time()));
        telemetry.addData("TURN Yaw", df.format(navx_device.getYaw()));
        telemetry.update();
        //sleep(2000);   // optional pause after each move

        //stop all motion

        back_left.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);

        DbgLog.msg("ABCD  navxturn " + String.format("%4d", logTargetangle)
                                     + String.format("%4d", logTimeoutN)
                  );
    }

    public void pushButton(boolean targetRed) {
        if (!opModeIsActive())
            return;

        crgbArray = getCRGB();
        redColorReading = crgbArray[1];
        blueColorReading = crgbArray[3];

        // log the light readings after the button is pushed
        DbgLog.msg("ABCD pushButton 1 OVER OVER RED      " + String.format(" %7d", redColorReading));
        DbgLog.msg("ABCD pushButton 1 OVER OVER BLUE     " + String.format(" %7d", blueColorReading));

        //NEED TO WAIT AT LEASt 100 MILLISECONDS
        runtime.reset();
        while (runtime.seconds() < 0.1) {

        }

        crgbArray = getCRGB();
        redColorReading = crgbArray[1];
        blueColorReading = crgbArray[3];

        // log the light readings after the button is pushed
        DbgLog.msg("ABCD pushButton 2 OVER OVER RED      " + String.format(" %7d", redColorReading));
        DbgLog.msg("ABCD pushButton 2 OVER OVER BLUE     " + String.format(" %7d", blueColorReading));
        // convert the RGB values to HSV values.
        //Color.RGBToHSV((colorSensorL.red() * 255) / 800, (colorSensorL.green() * 255) / 800, (colorSensorL.blue() * 255) / 800, hsvValuesL);



        //lightDifferenceReading = Math.abs(initValL-initValR);
        if( redColorReading > blueColorReading) {
            //this is reading red
            if (targetRed) { //if red is target
                position = 0; //PUSH LEFT
            }else{ //if red is not target
                position = 1; //PUSH RIGHT
            }
        }else{ //this is a blue reading
            if (targetRed) { //if red is target
                position = 1; //PUSH RIGHT
            }else{ //if red is not target
                position = 0; //PUSH LEFT
            }
        }

        //saving this value to be logged later
        pushButtonSavedPosition = position;

        servo.setPosition(position);

        back_left.setPower(-0.3);
        back_right.setPower(-0.3);
        front_left.setPower(-0.3);
        front_right.setPower(-0.3);
        

        runtime.reset();
        while(runtime.seconds() < 0.8) {} //i changed this to 0.8 from 0.6

        //check if colors have changed
        //
        //while (Math.abs(lightSensorR.getLightDetected() - lightSensorL.getLightDetected()) > 0.05) {}
        position = 0.45;
        servo.setPosition(position);

        back_left.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
        front_right.setPower(0);



        // log the light readings before the button is pushed
        DbgLog.msg("ABCD pushButton SERVO POSITION " + String.format(" %.1f", pushButtonSavedPosition));


        crgbArray = getCRGB();
        redColorReading = crgbArray[1];
        blueColorReading = crgbArray[3];

        // log the light readings after the button is pushed
        DbgLog.msg("ABCD pushButton 3 OVER OVER RED      " + String.format(" %7d", redColorReading));
        DbgLog.msg("ABCD pushButton 3 OVER OVER BLUE     " + String.format(" %7d", blueColorReading));


        //NEED TO WAIT AT LEASt 100 MILLISECONDS
        runtime.reset();
        while (runtime.seconds() < 0.1) {
        }

        crgbArray = getCRGB();
        redColorReading = crgbArray[1];
        blueColorReading = crgbArray[3];

        // log the light readings after the button is pushed
        DbgLog.msg("ABCD pushButton AFTER RED      " + String.format(" %7d", redColorReading));
        DbgLog.msg("ABCD pushButton AFTER BLUE     " + String.format(" %7d", blueColorReading));

        //lightDifferenceReading = Math.abs(initValL-initValR);
        if(
                ( targetRed == true) && (blueColorReading > redColorReading)
        ||
                ( targetRed == false) && (blueColorReading < redColorReading)
          )
        {
            runtime.reset();
            while (runtime.seconds() < 5.0) {


            }
            //i changed this to 0.8 from 0.6
            servo.setPosition(pushButtonSavedPosition);
            runtime.reset();
            while(runtime.seconds() < 1.0) {} //i changed this to 0.8 from 0.6
            position = 0.45;
            servo.setPosition(position);


            DbgLog.msg("ABCD      RETRY 1 " );

            crgbArray = getCRGB();
            redColorReading = crgbArray[1];
            blueColorReading = crgbArray[3];

            //NEED TO WAIT AT LEASt 100 MILLISECONDS
            runtime.reset();
            while (runtime.seconds() < 0.1) {}

            crgbArray = getCRGB();
            redColorReading = crgbArray[1];
            blueColorReading = crgbArray[3];

            // log the light readings after the button is pushed
            DbgLog.msg("ABCD pushButton AFTER RED      " + String.format(" %7d", redColorReading));
            DbgLog.msg("ABCD pushButton AFTER BLUE     " + String.format(" %7d", blueColorReading));
        }

    }


    public void set_direction_speeds(int direction, double speed) {
        if (!opModeIsActive())
            return;
        //   if (!targetRed) { //if on blue side
        //       direction = ((-(direction-4))+4)%8; //mirror all motor directions
        //   }

        if (direction == 0) { //forwards
            bl = speed;
            br = speed;
            fl = speed;
            fr = speed;
        }
        if (direction == 1) { //left 45
            bl = speed;
            br = 0;
            fl = 0;
            fr = speed;
        }
        if (direction == 2) { //left
            bl = speed;
            br = -speed;
            fl = -speed;
            fr = speed;
        }
        if (direction == 3) { //left -45
            bl = 0;
            br = -speed;
            fl = -speed;
            fr = 0;
        }
        if (direction == 4 ) { //backwards
            bl = -speed;
            br = -speed;
            fl = -speed;
            fr = -speed;
        }
        if (direction == 5) { //right -45
            bl = -speed;
            br = 0;
            fl = 0;
            fr = -speed;
        }
        if (direction == 6) { //right
            bl = -speed;
            br = speed;
            fl = speed;
            fr = -speed;
        }
        if (direction == 7) { //right 45
            bl = 0;
            br = speed;
            fl = speed;
            fr = 0;
        }
    }
}
