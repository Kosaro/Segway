package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by 516 on 6/13/2017.
 */

@TeleOp(name = "SegwayTest")
//@Disabled
public class SegwayTest extends LinearOpMode {
    Hardware robot;


    //constants relative to wheel size
    double kGyroAngle = 3.5;
    double kGyroSpeed = 1.15;
    double kPos = 0.07;
    double kSpeed = 0.1;

    // aids in drive control
    double kDrive = -0.02;
    double kSteer = 0.25;

    double emaOffset = 0.0005;
    double timeFallLimit = 1000;

    long nPgmTime = 0;
//==========================================================================
    //Global Variables

    // motorControlDrive is the target speed for the sum of the two motors
    // motorControlSteer is the target change in difference for two motors
    double motorControlDrive = 0.0;
    double motorControlSteer = 0.0;

    // target motor differential - which way the robot should be pointing
    double motorDiffTarget = 0.0;

    // time that the robot starts to balance
    long tCalcStart;

    // the time for each iteration of balance loop
    double tInterval;

    // relative wheel size compared to standard 1" wheel
    // (2" wheel has 0.7 ratio)
    double ratioWheel = 8.0;

    double gOffset;
    double gAngleGlobal;

    double motorPos = 0;
    long mrcSum = 0, mrcSumPrev;
    long motorDiff;
    long mrcDeltaP3 = 0;
    long mrcDeltaP2 = 0;
    long mrcDeltaP1 = 0;
//==========================================================================

    public void runOpMode() throws InterruptedException {
        robot = new Hardware(hardwareMap);

        //resets and calibrates encoders
        robot.resetEncoders();
        robot.gyro.calibrate();
        double calibrationStartTime = getRuntime();
        while (robot.gyro.isCalibrating() && opModeIsActive()) {
            telemetry.addData("Gyro calibrating", String.format("%1.2f", getRuntime() - calibrationStartTime));
            telemetry.update();
            idle();
        }

        balance();
    }
//===================================================================================

    public void getGyroData(double gyroSpeed, double gyroAngle) {
        int heading = robot.gyro.getHeading();

        //find offset of gyro
        int zAxisOffset = robot.gyro.getZAxisOffset();

        double gOffset = emaOffset * heading + (1 - emaOffset) * zAxisOffset;
        gyroSpeed = heading - gOffset;

        gAngleGlobal += gyroSpeed * tInterval;
    }

    //===============================================================================
    public void getMotorData(double motorSpeed, double motorPos) {
        long mrcLeft, mrcRight, mrcDelta;

        // Keep track of motor position and speed
        mrcLeft = robot.leftMotor.getCurrentPosition();
        mrcRight = robot.rightMotor.getCurrentPosition();

        // new mrcSum and Diff values
        mrcSumPrev = mrcSum;
        mrcSum = mrcLeft + mrcRight;
        motorDiff = mrcLeft - mrcRight;

        // mrcDelta is the sum of the motor encoders
        // motorPos is based on this delta
        mrcDelta = mrcSum - mrcSumPrev;
        motorPos += mrcDelta;

        // motorSpeed is based on the average of the last four deltas.
        motorSpeed = (mrcDelta + mrcDeltaP1 + mrcDeltaP2 + mrcDeltaP3) / (4 * tInterval);

        // Shift the latest mrcDelta into the previous three saved delta values
        mrcDeltaP3 = mrcDeltaP2;
        mrcDeltaP2 = mrcDeltaP1;
        mrcDeltaP1 = mrcDelta;
    }
//====================================================================================

    // Calculate the interval time from one iteration of the loop to the next.
    public void calcInterval(long cLoop) {
        if (cLoop == 0) {
            // First time through, set an initial tInterval time and
            // record start time
            tInterval = 0.0055;
            tCalcStart = nPgmTime;
        } else {
            // Take average of number of times through the loop and use for interval time.
            tInterval = (nPgmTime - tCalcStart) / (cLoop * 1000.0);
        }
    }

    //================================================================================
    public void steerControl(double power) {
        double powerSteer;

        motorDiffTarget += motorControlSteer * tInterval;

        powerSteer = kSteer * (motorDiffTarget - motorDiff);
        double powerLeft = power + powerSteer;
        double powerRight = power - powerSteer;

        if (powerLeft > 100)
            powerLeft = 100;
        else if (powerLeft < -100)
            powerLeft = -100;
        if (powerRight > 100)
            powerRight = 100;
        else if (powerRight < -100)
            powerRight = -100;
    }
//====================================================================================

    public void balance() {
        robot.resetEncoders();

        double gyroSpeed = 0.0;
        double gyroAngle = 0.0;
        double motorSpeed = 0.0;
        double power;
        double powerLeft = 0;
        double powerRight = 0;
        long tMotorPosOK;
        long cLoop = 0;

        tMotorPosOK = nPgmTime;

        while(opModeIsActive()) {
            calcInterval(cLoop++);
            getGyroData(gyroSpeed, gyroAngle);
            getMotorData(motorSpeed, motorPos);
            motorPos -= motorControlDrive * tInterval;

            power = (kGyroSpeed * gyroSpeed + kGyroAngle * gyroAngle) / ratioWheel
                    + kPos * motorPos + kDrive * motorControlDrive + kSpeed * motorSpeed;
            if (Math.abs(power) < 100)
                tMotorPosOK = nPgmTime;
            steerControl(power);
            robot.leftMotor.setPower(powerLeft);
            robot.rightMotor.setPower(powerRight);

            if ((nPgmTime - tMotorPosOK) > timeFallLimit) {
                break;
            }
        }
        sleep(2000);

        robot.stop();
    }
}

