package org.firstinspires.ftc.teamcode;

/**
 * Created by okosa on 6/13/2017.
 */

import android.util.Log;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Segway")
//@Disabled
public class SegwayOpMode extends LinearOpMode{
    Hardware robot;

    enum State{
        DRIVER_CONTROLLED("Driver Controlled");


        private String name;
        State(String name){
            this.name = name;
        }

        public String toString() {
            return name;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Hardware(hardwareMap);
        robot.setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.deviceInterfaceModule.setLED(0, true);
        /**
        robot.gyro.calibrate();
        double calibrationStartTime = getRuntime();
        while (robot.gyro.isCalibrating()&& !isStopRequested()){
            telemetry.addData("Gyro calibrating", String.format("%1.1f", getRuntime() - calibrationStartTime));
            telemetry.update();
            idle();
        }
        telemetry.addData("Gyro calibration finished in", String.format("%1.1f seconds", getRuntime() - calibrationStartTime));
        telemetry.update();
         */
        robot.hTGyro.calibrate(3000, 100);
        double gyroHeading = 0;
        robot.deviceInterfaceModule.setLED(0, false);

        waitForStart();
        telemetry.update();

        robot.resetEncoders();

        int counter = 0;
        double counterTime = getRuntime();
        double cyclesPerSecond = 0;
        double lastTime = getRuntime();
        while (opModeIsActive()){
            double currentTime = getRuntime();
            counter ++;
            if (currentTime > counterTime + .5){
               cyclesPerSecond = counter / (currentTime - counterTime);
                counterTime = currentTime;
            }

            gyroHeading += (robot.hTGyro.getAngularVelocity(AngleUnit.DEGREES).zRotationRate * (currentTime - lastTime));
            lastTime = currentTime;


            telemetry.addData("Miliseconds per cycle", String.format("%1.4f",(1 / cyclesPerSecond) * 1000));

            double currentAngle = gyroHeading;
            if (currentAngle > 180){
                currentAngle -= 360;
            }

            double power = robot.balance(gyroHeading);

            robot.leftMotor.setPower(power);
            robot.rightMotor.setPower(power);

            telemetry.addData("rate", robot.hTGyro.getAngularVelocity(AngleUnit.DEGREES).zRotationRate);
            telemetry.addData("Gyro", currentAngle);
            telemetry.addData("Power", power);
            telemetry.update();
            idle();
        }


    }
}
