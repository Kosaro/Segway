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


        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double pitch = robot.angles.thirdAngle;

        waitForStart();

        robot.resetEncoders();

        int counter = 0;
        double counterTime = getRuntime();
        double cyclesPerSecond = 0;
        while (opModeIsActive()){
            double currentTime = getRuntime();
            counter ++;
            if (currentTime > counterTime + .5){
               cyclesPerSecond = counter / (currentTime - counterTime);
                counterTime = currentTime;
            }



            telemetry.addData("Milliseconds per cycle", "%1.4f",(1 / cyclesPerSecond) * 1000);


            if (pitch > 180){
                pitch -= 360;
            }

            double power = robot.balance(pitch);

            robot.leftMotor.setPower(power);
            robot.rightMotor.setPower(power);

            if (pitch > 180){
                pitch -= 360;
            }

            telemetry.addData("Pitch Angle", pitch);
            telemetry.addData("Power", power);
            telemetry.update();
            idle();
        }


    }
}
