

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.firstinspires.ftc.robotcontroller.external.samples.Autonomous;

import java.util.Iterator;
import java.util.List;
import java.util.SortedSet;



@Autonomous(name="SailorBot Auto", group="Robot")
public class SailorBotAuto extends LinearOpMode {
    private DcMotor rightMotor = null;
    private DcMotor leftMotor = null;


    @Override
    public void runOpMode() throws InterruptedException {
       waitForStart();
        rightMotor = hardwareMap.get(DcMotor.class,"right_drive");
        leftMotor = hardwareMap.get(DcMotor.class,"left_drive");
        armMotor = hardwareMap.get(DcMotor.class,"arm");
//        showAttachedDevices();
          //moveRightMotor();
//        parkRobot();
       // moveIncrementally();
       // doAll();
        armMoving();

    }

    private void doAll()
    {
        moving();
        turnRight();
        armMoving();
    }

    private void turnRight() {
        int j =1;
        while (j <= 118)
        {


            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int tick_Distance = 4304;
            rightMotor.setPower(1);
            int MOTOR_MAX_TICK = 538;
            rightMotor.setTargetPosition(tick_Distance);


            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setPower(1);
            leftMotor.setTargetPosition(tick_Distance);

            j++;
        }
        telemetry.addData("I moved: ", j);
        telemetry.update();

    }

    private void moving() {
        int h = 1;
        int moveDis = 204;
        while (h <= moveDis) // 85 or 84 is the rough estimate of tick per rotation //204
        {
            // this is the forward moving method
            //not the backward. do not mix it up

            rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int tick_Distance = 4304; //4304
            rightMotor.setPower(1); //0.5
            int MOTOR_MAX_TICK = 538;
            rightMotor.setTargetPosition(tick_Distance);


            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setPower(1);
            leftMotor.setTargetPosition(tick_Distance);

            h++;
        }
        telemetry.addData("I moved: ", h);
        telemetry.update();

        sleep(3000);

    }

    private void armMoving()
    {
        int h = 1;
        int moveDis = 100;
        while (h <= moveDis) // 85 or 84 is the rough estimate of tick per rotation //204
        {
            // this is the forward moving method
            //not the backward. do not mix it up

            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            int tick_Distance = 4304; //4304
             armMotor.setPower(1); //0.5
            int MOTOR_MAX_TICK = 538;
            armMotor.setTargetPosition(tick_Distance);


            h++;
        }
        telemetry.addData("I moved (arm): ", h);
        telemetry.update();

        sleep(3000);

    }


    private void tellMe(String caption, int currentPosition) {
        telemetry.addData(caption, currentPosition);
        telemetry.update();
        sleep(2000);
    }


}
