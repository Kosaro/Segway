package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Oscar & Toni on 6/13/2017.
 */

@TeleOp(name = "Segway")
//@Disabled
public class SegwayOpMode extends LinearOpMode {
    Hardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        // INITIALIZE OPMODE
        robot = new Hardware(hardwareMap);
        robot.setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        robot.resetEncoders();

        //DECLARE VARIABLES
        int counter = 0;
        double counterTime = getRuntime();
        double cyclesPerSecond = 0;

        //START OPMODE
        while (opModeIsActive()) {

            //Calculate the average loop duration
            double currentTime = getRuntime();
            counter++;
            if (currentTime > counterTime + .5) {
                cyclesPerSecond = counter / (currentTime - counterTime);
                counterTime = currentTime;
            }
            double msPerCycle = (1/ cyclesPerSecond) * 1000000;

            //Calculate power to give to the motors
            double pitch = robot.getPitch();
            double power = robot.balance(pitch);

            //Give power to the motors
            robot.leftMotor.setPower(power);
            robot.rightMotor.setPower(power);

            //Displays the information on the screen
            telemetry.addData("Microseconds per cycle", "%1.4f", msPerCycle);
            telemetry.addData("Pitch Angle", "%1.4f", pitch);
            telemetry.addData("Power", "%1.4f", power);
            telemetry.update();
            idle();
        }


    }
}