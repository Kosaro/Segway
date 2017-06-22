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
        robot.leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        robot.resetEncoders();

        //DECLARE VARIABLES
        int counter = 0;
        double counterTime = getRuntime();
        double cyclesPerSecond = 0;
        int mode = 0;
        boolean lastUp = false;
        boolean lastDown = false;
        boolean lastRight = false;
        boolean lastLeft = false;

        //START OPMODE
        while (opModeIsActive()) {

            //Calculate the average loop duration
            double currentTime = getRuntime();
            counter++;
            if (currentTime > counterTime + .5) {
                cyclesPerSecond = counter / (currentTime - counterTime);
                counterTime = currentTime;
            }
            double msPerCycle = (1 / cyclesPerSecond) * 1000000;

            //Calculate power to give to the motors
            double pitch = robot.getPitch();
            double power = robot.balance(pitch);

            //Give power to the motors
            robot.leftMotor.setPower(power);
            robot.rightMotor.setPower(power);

            //Change values with controller
            double incrementValue = 0;
            double activeVariable = 0.0;

            switch (mode) {
                case 0:
                    activeVariable = robot.angleOffset;
                    incrementValue = .05;
                    telemetry.addData("Variable", "Angle Offset: " + activeVariable);
                    break;
                case 1:
                    activeVariable = robot.degreesToFullPower;
                    incrementValue = .5;
                    telemetry.addData("Variable", "Degrees to Full Power: " + activeVariable);
                    break;
                case 2:
                    activeVariable = robot.targetAngle;
                    incrementValue = .1;
                    telemetry.addData("Variable", "Target Angle: " + activeVariable);
                    break;
                case 3:
                    activeVariable = robot.zeroRange;
                    incrementValue = .1;
                    telemetry.addData("Variable", "Zero Range: " + activeVariable);
                    break;
                case 4:
                    activeVariable = robot.exponent;
                    incrementValue = .05;
                    telemetry.addData("Variable", "Exponent: " + activeVariable);
                    break;
            }
            if (gamepad1.dpad_up != lastUp) {
                if (gamepad1.dpad_up) {
                    robot.setActiveVariable(mode, incrementValue);
                }
                lastUp = gamepad1.dpad_up;
            }
            if (gamepad1.dpad_down != lastDown) {
                if (gamepad1.dpad_down) {
                    robot.setActiveVariable(mode, -incrementValue);
                }
                lastDown = gamepad1.dpad_down;
            }
            if (gamepad1.dpad_right != lastRight) {
                if (gamepad1.dpad_right) {
                    mode++;
                    if (mode > 4) {
                        mode = 0;
                    }
                }
                lastRight = gamepad1.dpad_right;
            }
            if (gamepad1.dpad_left != lastLeft) {
                if (gamepad1.dpad_left) {
                    mode--;
                    if (mode < 0) {
                        mode = 4;
                    }
                }
                lastLeft = gamepad1.dpad_left;
            }

            //Displays the information on the screen
            telemetry.addData("Microseconds per cycle", "%1.4f", msPerCycle);
            telemetry.addData("Pitch Angle", "%1.4f", pitch);
            telemetry.addData("Power", "%1.4f", power);
            telemetry.update();
            idle();
        }


    }
}