package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Created by 516 on 6/13/2017.
 */

@TeleOp(name = "Segway")
//@Disabled
public class SegwayTest extends LinearOpMode{
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
    double ratioWheel;

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

        robot.resetEncoders();
        robot.gyro.calibrate();
        double calibrationStartTime = getRuntime();
        while (robot.gyro.isCalibrating() && opModeIsActive()){
            telemetry.addData("Gyro calibrating", String.format("%1.2f", getRuntime() - calibrationStartTime));
            telemetry.update();
            idle();
        }
//===========================================================================

        int heading = robot.gyro.getHeading();

        AngularVelocity rates = robot.gyro.getAngularVelocity(AngleUnit.DEGREES);
        float zAngle = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int zAxisOffset = robot.gyro.getZAxisOffset();

        double gOffset = emaOffset * heading + (1-emaOffset) * zAxisOffset;
        double gyroSpeed = heading - gOffset;

        gAngleGlobal += gyroSpeed*tInterval;

//===========================================================================


    }
}