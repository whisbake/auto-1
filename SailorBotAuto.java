/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.Autonomous;

import java.util.Iterator;
import java.util.List;
import java.util.SortedSet;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="SailorBot Auto", group="Robot")
public class SailorBotAuto extends LinearOpMode {
    private DcMotor rightMotor = null;
    private DcMotor leftMotor = null;


    @Override
    public void runOpMode() throws InterruptedException {
       waitForStart();
        rightMotor = hardwareMap.get(DcMotor.class,"right_drive");
        leftMotor = hardwareMap.get(DcMotor.class,"left_drive");
//        showAttachedDevices();
          //moveRightMotor();
//        parkRobot();
       // moveIncrementally();
        doAll();

    }

    private void doAll()
    {
        moving();
        turnRight();
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


    @Override
    public void waitForStart() {
        super.waitForStart();
    }

    private void parkRobot()
    {
//        DcMotor rightMotor = (DcMotor) hardwareMap.get("right_motor");
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setPower(1.0);
        int currentPosition = rightMotor.getCurrentPosition();
        tellMe("Current position is: ", currentPosition);
        for(int i=1; i<50; i++)
        {
            tellMe("Advancing to ", currentPosition);
            rightMotor.setTargetPosition(currentPosition + i);
        }
    }

    private void moving() {
        int h = 1;
        int moveDis = 204;
        while (h <= 204) // 85 or 84 is the rough estimate of tick per rotation //204
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
    private void moveIncrementally()
    {
//        DcMotor rightMotor = (DcMotor) hardwareMap.get("right_motor");

        int tick_Distance = 4304;
        rightMotor.setPower(1.0);
        int cumTicks = 0;
        int currentPosition = rightMotor.getCurrentPosition();
        int MOTOR_MAX_TICK = 538;
        int MOTOR_MIN_TICK = 1;
        if(currentPosition != 1)
        {
            // move from current position to the end of a wheel rotation
            tellMe("Current position is: ", currentPosition);
            rightMotor.getCurrentPosition();
            rightMotor.setTargetPosition(538);
            //left motor movement = cumticks
            cumTicks += MOTOR_MAX_TICK - currentPosition;
            int leftMotorTicks = cumTicks;
        }
        while((tick_Distance - cumTicks) <= 538)
        {
            // preform full wheel rotations until there is less than one rotation remaining
            tellMe("Current position is: ", currentPosition);
            rightMotor.setTargetPosition(MOTOR_MAX_TICK / 2);
            rightMotor.setTargetPosition(MOTOR_MAX_TICK);
            cumTicks += 538;
        }
        // move remaining distance

        if((tick_Distance - cumTicks) != 0)
        {
            tellMe("Current position is: ", currentPosition);
            rightMotor.setTargetPosition(tick_Distance - cumTicks);
        }
        double leftCurrentPos = leftMotor.getCurrentPosition();

        if (leftCurrentPos + MOTOR_MAX_TICK > 538)
        {
            double leftMotorPartialTicks = leftCurrentPos + MOTOR_MAX_TICK;

        }

    }

    private void tellMe(String caption, int currentPosition) {
        telemetry.addData(caption, currentPosition);
        telemetry.update();
        sleep(2000);
    }

    private void moveRightMotor()
    {
//        DcMotor rightMotor = (DcMotor) hardwareMap.get("right_motor");
        int currentPosition = rightMotor.getCurrentPosition();
        final int MOTOR_MAX_TICK = 538;

        telemetry.addData("Current Position is: ", currentPosition);
        telemetry.update();
        sleep(5000);

        rightMotor.setPower(.3);

        if (currentPosition != 0)
        {
            telemetry.addData("Current Position is not equal to 0. Setting target position to ", MOTOR_MAX_TICK);
            telemetry.update();
            sleep(5000);
            rightMotor.setTargetPosition(MOTOR_MAX_TICK);

            telemetry.addData("Setting target position to ", currentPosition);
            telemetry.update();
            sleep(5000);
            rightMotor.setTargetPosition(currentPosition);
        }
        else
        {
            telemetry.addData("Current Position is equal to 0. Setting target position to ", MOTOR_MAX_TICK);
            telemetry.update();
            sleep(5000);
            rightMotor.setTargetPosition(MOTOR_MAX_TICK);
        }


    }


//
//    /* Declare OpMode members. */
//    private DcMotor         leftDrive   = null;
//    private DcMotor         rightDrive  = null;
//
//    private ElapsedTime     runtime = new ElapsedTime();
//
//    // Calculate the COUNTS_PER_INCH for your specific drive train.
//    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
//    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
//    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
//    // This is gearing DOWN for less speed and more torque.
//    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
//    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
//    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
//    static final double     DRIVE_SPEED             = 0.6;
//    static final double     TURN_SPEED              = 0.5;
//
//    @Override
//    public void runOpMode() {
//
//        // Initialize the drive system variables.
//        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
//        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
//
//        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
//        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
//        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        leftDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightDrive.setDirection(DcMotor.Direction.FORWARD);
//
//        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        // Send telemetry message to indicate successful Encoder reset
//        telemetry.addData("Starting at",  "%7d :%7d",
//                          leftDrive.getCurrentPosition(),
//                          rightDrive.getCurrentPosition());
//        telemetry.update();
//
//        // Wait for the game to start (driver presses START)
//        waitForStart();
//
//        // Step through each leg of the path,
//        // Note: Reverse movement is obtained by setting a negative distance (not speed)
//        encoderDrive(DRIVE_SPEED,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
//        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
//
//        telemetry.addData("Path", "Complete");
//        telemetry.update();
//        sleep(1000);  // pause to display final telemetry message.
//    }
//
//    /*
//     *  Method to perform a relative move, based on encoder counts.
//     *  Encoders are not reset as the move is based on the current position.
//     *  Move will stop if any of three conditions occur:
//     *  1) Move gets to the desired position
//     *  2) Move runs out of time
//     *  3) Driver stops the OpMode running.
//     */
//    public void encoderDrive(double speed,
//                             double leftInches, double rightInches,
//                             double timeoutS) {
//        int newLeftTarget;
//        int newRightTarget;
//
//        // Ensure that the OpMode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            newLeftTarget = leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
//            newRightTarget = rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
//            leftDrive.setTargetPosition(newLeftTarget);
//            rightDrive.setTargetPosition(newRightTarget);
//
//            // Turn On RUN_TO_POSITION
//            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // reset the timeout time and start motion.
//            runtime.reset();
//            leftDrive.setPower(Math.abs(speed));
//            rightDrive.setPower(Math.abs(speed));
//
//            // keep looping while we are still active, and there is time left, and both motors are running.
//            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
//            // its target position, the motion will stop.  This is "safer" in the event that the robot will
//            // always end the motion as soon as possible.
//            // However, if you require that BOTH motors have finished their moves before the robot continues
//            // onto the next step, use (isBusy() || isBusy()) in the loop test.
//            while (opModeIsActive() &&
//                   (runtime.seconds() < timeoutS) &&
//                   (leftDrive.isBusy() && rightDrive.isBusy())) {
//
//                // Display it for the driver.
//                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
//                telemetry.addData("Currently at",  " at %7d :%7d",
//                                            leftDrive.getCurrentPosition(), rightDrive.getCurrentPosition());
//                telemetry.update();
//            }
//
//            // Stop all motion;
//            leftDrive.setPower(0);
//            rightDrive.setPower(0);
//
//            // Turn off RUN_TO_POSITION
//            leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            sleep(250);   // optional pause after each move.

    /*
    Gets all the device names from the hardware map and displays them on the driver hub
     */
    public void showAttachedDevices() {
        Iterator<HardwareDevice> hwDevList = hardwareMap.iterator();
        for (Iterator<HardwareDevice> it = hwDevList; it.hasNext(); ) {
            HardwareDevice hd = it.next();
            telemetry.addData("Found", hd.getDeviceName());
        }
        telemetry.update();
        sleep(5000);
    }



}
