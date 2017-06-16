package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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
    final static String FRONT_ULTRASONIC = "fu";
    final static String REAR_ULTRASONIC = "ru";

    //Motor Directions
    final static DcMotorSimple.Direction LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    final static DcMotorSimple.Direction RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;


    DcMotor leftMotor;
    DcMotor rightMotor;
    ModernRoboticsI2cGyro gyro;
    UltrasonicSensor frontUltrasonic;
    UltrasonicSensor rearUltrasonic;

    Hardware(HardwareMap hardwareMap) {
        initialize(hardwareMap);
    }

    private void initialize(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.dcMotor.get(LEFT_MOTOR);
        rightMotor = hardwareMap.dcMotor.get(RIGHT_MOTOR);
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, GYRO);
        //frontUltrasonic = hardwareMap.ultrasonicSensor.get(FRONT_ULTRASONIC);
        //rearUltrasonic = hardwareMap.ultrasonicSensor.get(REAR_ULTRASONIC);

        leftMotor.setDirection(LEFT_MOTOR_DIRECTION);
        leftMotor.setDirection(RIGHT_MOTOR_DIRECTION);
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

    double balance() {
        double gyroHeading = gyro.getHeading();
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
        int gyroRange = 5 ;
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
        if (gyroHeading < 0 && power > 0)
            power = -power;
        return power;

    }

    static double scaleRevolutionsPerSecondToPower(double revolutionsPerSecond) {
        revolutionsPerSecond = Range.clip(revolutionsPerSecond, -2.286, 2.286);
        return Range.scale(revolutionsPerSecond, -2.286, 2.286, -1, 1);
    }
}
