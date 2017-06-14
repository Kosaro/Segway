package org.firstinspires.ftc.teamcode;

/**
 * Created by okosa on 6/13/2017.
 */

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
        while (robot.gyro.isCalibrating() && opModeIsActive()){
            telemetry.addData("Gyro calibrating", String.format("%1.2f", getRuntime() - calibrationStartTime));
            telemetry.update();
            idle();
        }
        telemetry.addData("Gyro calibration finished in", String.format("%1.2f seconds", getRuntime() - calibrationStartTime));
        telemetry.update();

        waitForStart();
        telemetry.update();

        robot.resetEncoders();

        double lastAngle = robot.gyro.getHeading();
        if (lastAngle > 180){
            lastAngle -= 360;
        }
        double lastTime = getRuntime();
        double angularVelocity = 0;
        while (opModeIsActive()){

            double currentAngle = robot.gyro.getHeading();
            if (currentAngle > 180){
                currentAngle -= 360;
            }
            double timeSinceLast = getRuntime() - lastTime;
            if (timeSinceLast > .05) {
                angularVelocity = (currentAngle - lastAngle ) / timeSinceLast;
                angularVelocity /= 360;
                angularVelocity *= 2 * Math.PI;

                lastAngle = currentAngle;
                lastTime = getRuntime();
            }
            double revolutionsPerSecond = robot.balance(angularVelocity);
            //robot.leftMotor.setPower(robot.scaleRevolutionsPerSecondToPower(revolutionsPerSecond));
            //robot.rightMotor.setPower(robot.scaleRevolutionsPerSecondToPower(revolutionsPerSecond));
            robot.leftMotor.setPower(revolutionsPerSecond);
            robot.rightMotor.setPower(revolutionsPerSecond);

            telemetry.addData("Gyro", currentAngle);
            telemetry.addData("Power", robot.scaleRevolutionsPerSecondToPower(revolutionsPerSecond));
            telemetry.update();
            idle();
        }


    }
}
