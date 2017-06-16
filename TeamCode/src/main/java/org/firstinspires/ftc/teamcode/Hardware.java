package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.hitechnic.HiTechnicNxtGyroSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * Created by Oscar on 6/13/2017.
 */

public class Hardware {
    //Hardware Constants
    final static double ENCODER_TICKS_PER_REVOLUTION = 1120;
    //final static double ROBOT_HEIGHT = 24.0;   //inches
    final static double WHEEL_DIAMETER = 4.0;  //inches
    final static double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    //Hello there testing

    //Configuration names
    final static String LEFT_MOTOR = "lm";
    final static String RIGHT_MOTOR = "rm";
    final static String GYRO = "g";
    final static String HTGYRO = "htg";
    final static String IMU = "imu";
    final static String FRONT_ULTRASONIC = "fu";
    final static String REAR_ULTRASONIC = "ru";

    //Motor Directions
    final static DcMotorSimple.Direction LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    final static DcMotorSimple.Direction RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;


    DcMotor leftMotor;
    DcMotor rightMotor;
    //ModernRoboticsI2cGyro gyro;
    HiTechnicNxtGyroSensor hTGyro;
    BNO055IMU imu;
    DeviceInterfaceModule deviceInterfaceModule;
    UltrasonicSensor frontUltrasonic;
    UltrasonicSensor rearUltrasonic;

    Orientation angles;
    Acceleration gravity;

    Hardware(HardwareMap hardwareMap) {
        initialize(hardwareMap);
        imuParameters();
    }

    private void initialize(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.dcMotor.get(LEFT_MOTOR);
        rightMotor = hardwareMap.dcMotor.get(RIGHT_MOTOR);
        //gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, GYRO);
        deviceInterfaceModule = hardwareMap.deviceInterfaceModule.get("dim");
        imu = hardwareMap.get(BNO055IMU.class, IMU );

        //frontUltrasonic = hardwareMap.ultrasonicSensor.get(FRONT_ULTRASONIC);
        //rearUltrasonic = hardwareMap.ultrasonicSensor.get(REAR_ULTRASONIC);

        leftMotor.setDirection(LEFT_MOTOR_DIRECTION);
        leftMotor.setDirection(RIGHT_MOTOR_DIRECTION);
    }

    public void imuParameters() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }


    public void resetEncoders() {
        DcMotor.RunMode leftRunMode = leftMotor.getMode();
        DcMotor.RunMode rightRunMode = rightMotor.getMode();
        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRightMotorRunMode(rightRunMode);
        setLeftMotorRunMode(leftRunMode);
    }

    private void setLeftMotorRunMode(DcMotor.RunMode runMode) {
        leftMotor.setMode(runMode);
    }

    private void setRightMotorRunMode(DcMotor.RunMode runMode) {
        rightMotor.setMode(runMode);
    }

    void setMotorRunMode(DcMotor.RunMode runMode) {
        setLeftMotorRunMode(runMode);
        setRightMotorRunMode(runMode);
    }

    public void stop() {
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }

    double balance(double gyroHeading) {
        //double gyroHeading = gyro.getHeading();
        if (gyroHeading > 180) {
            gyroHeading -= 360;

        }
        /**
         if (gyroHeading == 0)
         return 0;
         double speed = ROBOT_HEIGHT *  Math.abs (gyro.getAngularVelocity(AngleUnit.RADIANS)) / Math.cos((gyroHeading / 360) * 2 * Math.PI);
         if (gyroHeading < 0){
         speed = -speed;
         }
         double revolutionPerSecond = speed / WHEEL_CIRCUMFERENCE;
         return scaleRevolutionsPerSecond(revolutionPerSecond);
         */

        if (Math.abs(gyroHeading) > 20) {
            return 0;
        }

        int gyroRange = 15 ;
        if (Math.abs(gyroHeading) > gyroRange){
            deviceInterfaceModule.setLED(1, true);
        }else{
            deviceInterfaceModule.setLED(1, false);
        }
        int targetAngleConst = 1;

        int targetAngle = 0;
        if (gyroHeading > 0){
            targetAngle = -targetAngleConst;
        }else if (gyroHeading < 0){
            targetAngle = targetAngleConst;
        }
        gyroHeading -= targetAngle;
        gyroHeading = Range.clip(gyroHeading, -gyroRange, gyroRange);

        double power = Range.scale(gyroHeading, -gyroRange, gyroRange , -1, 1);

        //power = Math.pow(power, 1.2);
        //if (gyroHeading < 0 && power > 0)
        //    power = -power;




        return power;

    }

    static double scaleRevolutionsPerSecondToPower(double revolutionsPerSecond) {
        revolutionsPerSecond = Range.clip(revolutionsPerSecond, -2.286, 2.286);
        return Range.scale(revolutionsPerSecond, -2.286, 2.286, -1, 1);
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
