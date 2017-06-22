package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by Oscar & Toni on 6/13/2017.
 */

public class Hardware {

/*==================================================================================================
                                         INITIALIZATION
==================================================================================================*/

    //Configuration names
    final static String LEFT_MOTOR = "lm";
    final static String RIGHT_MOTOR = "rm";
    final static String IMU = "imu";

    //Motor Directions
    final static DcMotorSimple.Direction LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    final static DcMotorSimple.Direction RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    DcMotor leftMotor;
    DcMotor rightMotor;
    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    Hardware(HardwareMap hardwareMap) {
        initialize(hardwareMap);
        initializeImuParameters();
    }

    //Initialize motors
    private void initialize(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.dcMotor.get(LEFT_MOTOR);
        rightMotor = hardwareMap.dcMotor.get(RIGHT_MOTOR);
        imu = hardwareMap.get(BNO055IMU.class, IMU);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        leftMotor.setDirection(LEFT_MOTOR_DIRECTION);
        leftMotor.setDirection(RIGHT_MOTOR_DIRECTION);
    }

    //Initialize IMU
    public void initializeImuParameters() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

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
/*==================================================================================================
                                         CALCULATIONS
==================================================================================================*/

    //Find angle from gyro
    double getPitch() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES)
                .toAngleUnit(AngleUnit.DEGREES).thirdAngle;
    }

    //Method to balance the robot autonomously
    double balance(double gyroHeading) {

        //offsets gyro so it can balance correctly
        gyroHeading += 0.8;

        //scales the gyro so any number above the scale is classed
        // as the max and the motors are given full power.
        if (Math.abs(gyroHeading) > 40 || Math.abs(gyroHeading) < 1) {
            return Range.scale(gyroHeading, -15, 15, -.02, .02);
        }
        //this tells the robot what angle to stop sending power to the motors.
        double gyroRange = 20;

        //tells the robot when to not send power to the motors,
        //gives a range of values at which the robot does not move.
        double targetAngle = 0;
        if (gyroHeading > 0) {
            targetAngle = -.9;
        } else if (gyroHeading < 0) {
            targetAngle = .9;
        }

        gyroHeading -= targetAngle;
        gyroHeading = Range.clip(gyroHeading, -gyroRange, gyroRange);

        //gives power to each motor depending on how the robot is positioned.
        double power = Range.scale(gyroHeading, -gyroRange, gyroRange, -1, 1);

        //converts power to negative power so the robot can move in both directions
        power = Math.pow(Math.abs(power), 1.05   );
        if (gyroHeading < 0 && power > 0)
            power = -power;

        return power;
    }
}
