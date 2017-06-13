package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.robocol.RobocolDatagramSocket;

/**
 * Created by Oscar on 6/13/2017.
 */

public class Hardware {

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
    GyroSensor gyro;
    UltrasonicSensor frontUltrasonic;
    UltrasonicSensor rearUltrasonic;

    Hardware(HardwareMap hardwareMap) {
        initialize(hardwareMap);
    }

    private void initialize(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.dcMotor.get(LEFT_MOTOR);
        rightMotor = hardwareMap.dcMotor.get(RIGHT_MOTOR);
        gyro = hardwareMap.gyroSensor.get(GYRO);
        frontUltrasonic = hardwareMap.ultrasonicSensor.get(FRONT_ULTRASONIC);
        rearUltrasonic = hardwareMap.ultrasonicSensor.get(REAR_ULTRASONIC);

        leftMotor.setDirection(LEFT_MOTOR_DIRECTION);
        leftMotor.setDirection(RIGHT_MOTOR_DIRECTION);
    }


    void resetEncoders(){
        DcMotor.RunMode leftRunMode = leftMotor.getMode();
        DcMotor.RunMode rightRunMode = rightMotor.getMode();
        setMotorRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRightMotorRunMode(rightRunMode);
        setLeftMotorRunMode(leftRunMode);
    }

    private void setLeftMotorRunMode(DcMotor.RunMode runMode){
        leftMotor.setMode(runMode);
    }

    private void setRightMotorRunMode(DcMotor.RunMode runMode){
        rightMotor.setMode(runMode);
    }

    void setMotorRunMode(DcMotor.RunMode runMode){
        setLeftMotorRunMode(runMode);
        setRightMotorRunMode(runMode);
    }

    void balance(){

    }
}
