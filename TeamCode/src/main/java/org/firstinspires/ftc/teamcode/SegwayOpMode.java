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

        robot.resetEncoders();
        //robot.gyro.calibrate();
        double calibrationStartTime = getRuntime();
        //while (robot.gyro.isCalibrating() && opModeIsActive()){
          //  telemetry.addData("Gyro calibrating", String.format("%1.2f", getRuntime() - calibrationStartTime));
            //telemetry.update();
            //idle();
        //}
        //telemetry.addData("Gyro calibration finished in", String.format("%1.2f seconds", getRuntime() - calibrationStartTime));
        //telemetry.update();

        waitForStart();

        robot.setMotorRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.resetEncoders();

        //double lastAngle = robot.gyro.getHeading();
        double lastPositionLeft = robot.leftMotor.getCurrentPosition();
        //double lastPositionRight = robot.rightMotor.getCurrentPosition();
        double lastTime = getRuntime();
        double angularVelocity = 0;
        double leftRevolutionsPerSecond = 0;
        double rightRevolutionsPerSecond = 0;
        robot.leftMotor.setPower(.4);
        double lastLongTimer = 0;
        double lastLostValue = 0;
        double averageValue = 0;
        while (opModeIsActive()){
            if (getRuntime() - lastLongTimer > 10){
                averageValue = (robot.leftMotor.getCurrentPosition() - lastLostValue) / (getRuntime() - lastLongTimer);
                lastLongTimer = getRuntime();
                lastLostValue = robot.leftMotor.getCurrentPosition();
            }
          //  double currentAngle = robot.gyro.getHeading();
            double currentLeftPosition = robot.leftMotor.getCurrentPosition();
            //double currentRightPosition = robot.rightMotor.getCurrentPosition();
            double timeSinceLast = getRuntime() - lastTime;
            if (timeSinceLast > .05) {
              //  angularVelocity = (lastAngle - currentAngle ) / timeSinceLast;
                leftRevolutionsPerSecond = (lastPositionLeft - currentLeftPosition) / Hardware.ENCODER_TICKS_PER_REVOLUTION / timeSinceLast;
                //rightRevolutionsPerSecond = (lastPositionRight - currentRightPosition) / timeSinceLast;

                //lastAngle = currentAngle;
                lastPositionLeft = currentLeftPosition;
                //lastPositionRight = currentRightPosition;
                lastTime = getRuntime();
            }
            telemetry.addData("RPS", leftRevolutionsPerSecond);
            telemetry.addData("Average", averageValue / Hardware.ENCODER_TICKS_PER_REVOLUTION);
            telemetry.update();
            //double revolutionsPerSecond = robot.balance(angularVelocity);

            idle();
        }


    }
}
