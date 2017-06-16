package org.firstinspires.ftc.teamcode;

/**
 * Created by okosa on 6/13/2017.
 */

import android.util.Log;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        robot.gyro.calibrate();
        double calibrationStartTime = getRuntime();
        robot.deviceInterfaceModule.setLED(0, true);
        while (robot.gyro.isCalibrating()&& !isStopRequested()){
            telemetry.addData("Gyro calibrating", String.format("%1.1f", getRuntime() - calibrationStartTime));
            telemetry.update();
            idle();
        }
        telemetry.addData("Gyro calibration finished in", String.format("%1.1f seconds", getRuntime() - calibrationStartTime));
        telemetry.update();
        robot.deviceInterfaceModule.setLED(0, false);

        waitForStart();
        telemetry.update();

        robot.resetEncoders();

        int counter = 0;
        double counterTime = getRuntime();
        double cyclesPerSecond = 0;
        while (opModeIsActive()){
            counter ++;
            if (getRuntime() > counterTime + .5){
               cyclesPerSecond = counter / (getRuntime() - counterTime);
                counterTime = getRuntime();
            }
            telemetry.addData("Miliseconds per cycle", String.format("%1.4f",1 / cyclesPerSecond * 1000));

            double currentAngle = robot.gyro.getHeading();
            if (currentAngle > 180){
                currentAngle -= 360;
            }

            double power = robot.balance();
            robot.leftMotor.setPower(power);
            robot.rightMotor.setPower(power);


            telemetry.addData("Gyro", currentAngle);
            telemetry.addData("Power", power);
            telemetry.update();
            idle();
        }


    }
}
