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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Summer: Auto")

public class SummerAuto extends LinearOpMode {


    HardwareSummer         robot   = new HardwareSummer();
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static boolean cornerC = false;
    public boolean isblue;
    static char walORcor;
    static double ballAvgColor = 0;


    @Override
    public void runOpMode() {


        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.motor_left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor_left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor_right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motor_right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motor_right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor_right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor_left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motor_left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.motor_left_back.getCurrentPosition(),
                robot.motor_left_front.getCurrentPosition(),
                robot.motor_right_back.getCurrentPosition(),
                robot.motor_right_front.getCurrentPosition());

        telemetry.update();

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Ask", "press x for blue or b for red");    //
            telemetry.update();
            while (cornerC == false) {
                if (gamepad1.b) {
                    isblue = false;
                    cornerC = true;
                }
                if (gamepad1.x) {
                    isblue = true;
                    cornerC = true;
                }

            }
            cornerC = false;
            telemetry.addData("Ask", "press DOWN for wall or RIGHT for corner");    //
            telemetry.update();
            while (cornerC == false) {
                if (gamepad1.dpad_down) {
                    walORcor = 'w';
                }
                if (gamepad1.dpad_right) {
                    walORcor = 'c';
                }

            }



            robot.BallArmUpDown.setPosition(80);
            robot.BallArmRightLeft.setPosition(85);
            while(robot.BallArmUpDown.getPosition()<90 && opModeIsActive()){
                ballAvgColor +=robot.colorLeft.blue();
                robot.BallArmRightLeft.setPosition(robot.BallArmRightLeft.getPosition() + 1);
            }
            ballAvgColor=ballAvgColor/5;
            if(isblue && ballAvgColor >90 && ballAvgColor < 110){
                robot.BallArmUpDown.setPosition(90);
                robot.BallArmRightLeft.setPosition(100);
            }
            if (!isblue && ballAvgColor <90 && ballAvgColor > 110){
                robot.BallArmUpDown.setPosition(90);
                robot.BallArmRightLeft.setPosition(100);
            }
            else{
                robot.BallArmUpDown.setPosition(90);
                robot.BallArmRightLeft.setPosition(80);
            }
        }


        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftTargetfront;
        int newRightTargetfront;


            robot.motor_right_front.setPower(0);
            robot.motor_right_back.setPower(0);
            robot.motor_left_back.setPower(0);
            robot.motor_left_front.setPower(0);

            newLeftTargetfront = robot.motor_left_front.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTargetfront = robot.motor_right_back.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftTarget = robot.motor_left_back.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.motor_right_front.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.motor_left_front.setTargetPosition(newLeftTargetfront);
            robot.motor_right_back.setTargetPosition(newRightTarget);
            robot.motor_left_back.setTargetPosition(newLeftTarget);
            robot.motor_right_front.setTargetPosition(newRightTargetfront);
            robot.motor_right_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor_right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor_left_back.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motor_left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            robot.motor_left_front.setPower(Math.abs(speed));
            robot.motor_left_back.setPower(Math.abs(speed));
            robot.motor_right_front.setPower(Math.abs(speed));
            robot.motor_right_back.setPower(Math.abs(speed));
            while((runtime.seconds() < timeoutS) &&
                    (robot.motor_right_back.isBusy() && robot.motor_left_back.isBusy()) &&
                    (robot.motor_right_front.isBusy() && robot.motor_left_front.isBusy()) ) {



                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.motor_left_front.getCurrentPosition(),
                        robot.motor_left_back.getCurrentPosition(),
                        robot.motor_right_front.getCurrentPosition(),
                        robot.motor_right_back.getCurrentPosition());
                telemetry.update();
            }

            robot.motor_right_front.setPower(0);
            robot.motor_right_back.setPower(0);
            robot.motor_left_back.setPower(0);
            robot.motor_left_front.setPower(0);

        }
    }


