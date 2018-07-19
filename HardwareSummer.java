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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


public class HardwareSummer {

    public DcMotor motor_left_front = null;
    public DcMotor motor_right_front = null;
    public DcMotor motor_left_back = null;
    public DcMotor motor_right_back = null;
    public DcMotor liftLeft = null;
    public DcMotor liftRight = null;
    public DcMotor spinner = null;
    public Servo BallArmUpDown = null;
    public Servo BallArmRightLeft = null;
    public ColorSensor colorLeft = null;
    public ColorSensor colorRight = null;
    public DistanceSensor DistanceFront = null;
    public DistanceSensor DistanceBack = null;
    BNO055IMU imu;
    DigitalChannel touchSpinnerUp;
    DigitalChannel touchSpinnerDown;
    DistanceSensor sensorDistanceDown;
    DistanceSensor sensorDistanceUp;

    HardwareMap hwMap = null;


    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;


        spinner = hwMap.get(DcMotor.class, "sp");
        motor_left_back = hwMap.get(DcMotor.class, "dlb");
        motor_right_back = hwMap.get(DcMotor.class, "drb");
        motor_left_front = hwMap.get(DcMotor.class, "dlf");
        motor_right_front = hwMap.get(DcMotor.class, "drf");
        liftRight = hwMap.get(DcMotor.class, "liftR");
        liftLeft = hwMap.get(DcMotor.class, "liftL");

        motor_left_back.setDirection(DcMotor.Direction.FORWARD);
        motor_right_back.setDirection(DcMotor.Direction.REVERSE);
        motor_left_front.setDirection(DcMotor.Direction.REVERSE);
        motor_right_front.setDirection(DcMotor.Direction.FORWARD);
        liftLeft.setDirection(DcMotor.Direction.FORWARD);
        liftRight.setDirection(DcMotor.Direction.REVERSE);
        spinner.setDirection(DcMotor.Direction.REVERSE); 


        setPowerAllMotors(0);
        liftRight.setPower(0);
        liftLeft.setPower(0);
        spinner.setPower(0);


        setDriveMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        BallArmRightLeft = hwMap.get(Servo.class, "arl");
        BallArmUpDown = hwMap.get(Servo.class, "aud");



        colorRight = hwMap.get(ColorSensor.class, "cr");
        colorLeft = hwMap.get(ColorSensor.class, "cl");
        DistanceFront = hwMap.get(DistanceSensor.class, "cf");
        DistanceBack = hwMap.get(DistanceSensor.class, "cb");
        sensorDistanceDown = hwMap.get(DistanceSensor.class, "sd");
        sensorDistanceUp = hwMap.get(DistanceSensor.class, "sdu");


        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        touchSpinnerDown = hwMap.get(DigitalChannel.class, "tsd");

        touchSpinnerDown.setMode(DigitalChannel.Mode.INPUT);

        touchSpinnerUp = hwMap.get(DigitalChannel.class, "tsu");

        touchSpinnerUp.setMode(DigitalChannel.Mode.INPUT);
    }

    public void setDriveMotorsMode(DcMotor.RunMode runMode) {
        motor_left_back.setMode(runMode);
        motor_right_back.setMode(runMode);
        motor_left_front.setMode(runMode);
        motor_right_front.setMode(runMode);
    }


    public void setPowerAllMotors(double speed) {
        motor_left_back.setPower(speed);
        motor_right_back.setPower(speed);
        motor_left_front.setPower(speed);
        motor_right_front.setPower(speed);
    }


    public void setPowerLeftMotors(double speed) {
        motor_left_back.setPower(speed);
        motor_left_front.setPower(speed);
    }


    public void setPowerRightMotors(double speed) {
        motor_right_back.setPower(speed);
        motor_right_front.setPower(speed);
    }

    public void setPowerLifts( double LIFT_SPEED){
        liftLeft.setPower(LIFT_SPEED);
        liftRight.setPower(LIFT_SPEED);
    }

}

