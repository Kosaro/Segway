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

    //Segway variables
    double centerOfGravityAngle = -.15;
    double degreesToFullPower = 30.0;
    double overshootAngle = 0;
    double deadzoneAngle = .5;
    double exponent = 0.5;

    //Motor Directions
    final static DcMotorSimple.Direction LEFT_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    final static DcMotorSimple.Direction RIGHT_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;

    //devices
    DcMotor leftMotor;
    DcMotor rightMotor;
    BNO055IMU imu;

    Hardware(HardwareMap hardwareMap) {
        initialize(hardwareMap);
        initializeImuParameters();
    }

    //Initialize devices
    private void initialize(HardwareMap hardwareMap) {
        leftMotor = hardwareMap.dcMotor.get(LEFT_MOTOR);
        rightMotor = hardwareMap.dcMotor.get(RIGHT_MOTOR);
        imu = hardwareMap.get(BNO055IMU.class, IMU);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        leftMotor.setDirection(LEFT_MOTOR_DIRECTION);
        leftMotor.setDirection(RIGHT_MOTOR_DIRECTION);
    }

    //Initialize IMU
    private void initializeImuParameters() {
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
                .toAngleUnit(AngleUnit.DEGREES).thirdAngle - 90;
    }

    //Allows the variables do be incremented while the program is running
    void incrementVariable(int mode, double incrementValue) {
        switch (mode) {
            case 0:
                centerOfGravityAngle += incrementValue;
                break;
            case 1:
                degreesToFullPower += incrementValue;
                break;
            case 2:
                overshootAngle += incrementValue;
                break;
            case 3:
                deadzoneAngle += incrementValue;
                break;
            case 4:
                exponent += incrementValue;
                break;
        }
    }

    //Method to balance the robot autonomously
    double balance(double gyroHeading) {
        //offsets gyro so it can balance correctly
        gyroHeading += centerOfGravityAngle;

        //this tells the robot at what angle to send full power to the motors.
        double gyroRange = degreesToFullPower;

        //If robot is leaning more than 30 degrees or less than the deadzone angle, stop the motors
        if (Math.abs(gyroHeading) > 30 || Math.abs(gyroHeading) < deadzoneAngle)
            return 0;

        //Modifies the gyro heading so that instead of aiming at 0,
        //it aims for a angle a bit beyond 0 (overshoot angle)
        double modifiedOvershootAngle = overshootAngle;
        if (gyroHeading > 0) {
            modifiedOvershootAngle = -overshootAngle;
        } else if (gyroHeading < 0) {
            modifiedOvershootAngle = overshootAngle;
        }
        gyroHeading -= modifiedOvershootAngle;

        //Gives power to each motor depending on how the robot is positioned.
        //Scales the gyro so any number above the scale is classed
        //as the max and the motors are given full power.
        gyroHeading = Range.clip(gyroHeading, -gyroRange, gyroRange);
        double power = Range.scale(gyroHeading, -gyroRange, gyroRange, -1, 1);

        //Changes power curve of the power. Numbers less than one mean faster speeds closer to 0,
        //and vice versa for bigger numbers
        power = Math.pow(Math.abs(power), exponent);
        if (gyroHeading < 0 && power > 0) // makes sure that the result is negative if the input was negative
            power = -power;

        return power;
    }
}
