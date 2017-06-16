package org.firstinspires.ftc.teamcode;

/**
 * Created by okosa on 6/13/2017.
 */

import android.util.Log;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Segway")
//@Disabled
public class SegwayOpMode extends LinearOpMode {
    Hardware robot;

    enum State {
        DRIVER_CONTROLLED("Driver Controlled");


        private String name;

        State(String name) {
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


        waitForStart();

        robot.resetEncoders();

        int counter = 0;
        double counterTime = getRuntime();
        double cyclesPerSecond = 0;
        while (opModeIsActive()) {
            double currentTime = getRuntime();
            counter++;
            if (currentTime > counterTime + .5) {
                cyclesPerSecond = counter / (currentTime - counterTime);
                counterTime = currentTime;
            }


            telemetry.addData("Milliseconds per cycle", "%1.4f", (1 / cyclesPerSecond) * 1000);


            double power = robot.balance();

            robot.leftMotor.setPower(power);
            robot.rightMotor.setPower(power);

            double pitch = robot.getPitch() + 90;
            telemetry.addData("Pitch Angle", pitch);
            telemetry.addData("Power", power);
            telemetry.update();
            idle();
        }


    }
}
