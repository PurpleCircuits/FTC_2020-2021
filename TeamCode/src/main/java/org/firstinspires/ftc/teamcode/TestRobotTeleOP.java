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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="TestRobotTeleOP", group="Linear Opmode")
public class TestRobotTeleOP<pose> extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // Declare our hardware
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor theClawMotor = null;
    private Servo theClawServo = null;

    private static final double SERVO_MIN_POS = 0.0; // Minimum rotational position
    private static final double SERVO_MAX_POS = 1.0; // Maximum rotational position
    private static final double SERVO_HALFWAY_POSITION = (SERVO_MAX_POS - SERVO_MIN_POS) / 2;

    private int a = 0;
    private int pose[] = new int[6];
    private int savedPosition = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        theClawMotor = hardwareMap.get(DcMotor.class, "the_claw_motor");
        theClawServo = hardwareMap.get(Servo.class, "the_claw_servo");

        // These are the encoder positions for the motor to go to in the array
        pose[0]=0;
        pose[1]=383;
        pose[2]=553;
        pose[3]=538;
        pose[4]=724;
        pose[5]=916;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        theClawMotor.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            clawAction();
            driveAction();
            //Conveyer right stick #2
            //intake switch#2 X button
            //Launch#2 Y (add servo gate + conveyerAction)
            //Open Claw#2 B


            telemetry.update();
        }
    }

    private void clawAction() {
        // close the claw
        if (gamepad2.a) {
            theClawServo.setPosition(SERVO_MIN_POS);
        }

        // open the claw
        if (gamepad2.b) {
            theClawServo.setPosition(SERVO_MAX_POS);
        }

        // Log the encoder value of the claw motor
        telemetry.addData("Claw Motor Encoder: ", "%d %d" , theClawMotor.getCurrentPosition(), a);
        telemetry.addData("A value:","%d",a);

        // This is a recreation of an exponential graph we decided to create
        // y = ax^2 with x being the joystick input and y being the motor power
        /*
        float x = gamepad2.left_stick_y;
        telemetry.addData("Claw Joystick: ", "%.2f", x);
        if ( x > 0) {
            double power = 0.90 * x * x;
            telemetry.addData("Claw Power (Positive): ", "%.2f", power);
            theClawMotor.setPower(power);
        } else {
            double power = -0.90 * x * x;
            telemetry.addData("Claw Power (Negative): ", "%.2f", power);
            theClawMotor.setPower(power);
        }
         */

        // Linear speed
       double power = gamepad2.left_stick_y;
        // Slow down the robot by factor 5 or 2 when right bumper pressed
        if (!gamepad2.right_bumper) {
            power = power/5;
        }else{
            power = power/2;
        }
        theClawMotor.setPower(power);

        // Save the position of the encoder when bumper is pressed
        if(gamepad2.left_bumper) {
            savedPosition = theClawMotor.getTargetPosition();
        }

        // Loop to get to position
        /*
        if (gamepad2.left_bumper){
            if (a<5){
                ++a;qqqmnaqaa[
            }else
               a = 0;
        }
        theClawMotor.setTargetPosition(pose[a]);
        theClawMotor.setPower(.5);
        theClawMotor.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        */
        //sleep so bot does not over correct and to make sure one button press is one increment in 'a' value
        sleep(100);

/*
       if (gamepad2.right_bumper){
           a= ++a;
       }
        if (a==1){
            theClawMotor.setTargetPosition(0);
            theClawMotor.setPower(1);
            theClawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if  (a==2){
            theClawMotor.setTargetPosition(383);
            theClawMotor.setPower(1);
            theClawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if  (a==3){
            theClawMotor.setTargetPosition(682);
            theClawMotor.setPower(1);
            theClawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (a==4){
            theClawMotor.setTargetPosition(706);
            theClawMotor.setPower(1);
            theClawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (a==5){
            theClawMotor.setTargetPosition(753);
            theClawMotor.setPower(1);
            theClawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (a==6){
            theClawMotor.setTargetPosition(762);
            theClawMotor.setPower(1);
            theClawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (a==7){
            theClawMotor.setTargetPosition(0);
            theClawMotor.setPower(1);
            theClawMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            a=a-6;
        }

 */


    }

    private void driveAction() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn  =  gamepad1.right_stick_x;
        leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Show the wheel power
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }


}
