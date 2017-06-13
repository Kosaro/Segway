package org.firstinspires.ftc.teamcode;

/**
 * Created by okosa on 6/13/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

        robot.resetEncoders();
        robot.gyro.calibrate();
        double calibrationStartTime = getRuntime();
        while (robot.gyro.isCalibrating() && opModeIsActive()){
            telemetry.addData("Gyro calibrating", String.format("%1.2f", getRuntime() - calibrationStartTime));
            telemetry.update();
            idle();
        }
        telemetry.addData("Gyro calibration finished in", String.format("%1.2f seconds", getRuntime() - calibrationStartTime));
        telemetry.update();

        waitForStart();


    }
}
