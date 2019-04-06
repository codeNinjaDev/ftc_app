package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
/**
 * Created by peter on 4/11/18.
 */

public class RobotModel {
    int TICKS_PER_ROTATION = 1120;
    double WHEEL_DIAMETER = 6;
    double INCHES_PER_TICK = (WHEEL_DIAMETER * Math.PI) / TICKS_PER_ROTATION;
    private double DEGREE_PER_TICK = 2;

    public HardwareMap hardwareMap = null;

    DcMotor frontLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor backRightMotor = null;

    public DcMotor armLeftMotor = null;
    public DcMotor armRightMotor = null;

    Servo leftServo = null;
    Servo rightServo = null;
    public RobotModel(HardwareMap hardwareMap) {
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");

        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        armLeftMotor = hardwareMap.dcMotor.get("armLeftMotor");
        armRightMotor = hardwareMap.dcMotor.get("armRightMotor");

        armLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftServo = hardwareMap.servo.get("leftServo");
        rightServo = hardwareMap.servo.get("rightServo");

        armLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


    }

    public void resetEncoders() {

        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void reset() {
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void driveForward(double power) {
        frontLeftMotor.setPower(power);
        frontRightMotor.setPower(power);
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    public void setRightDrive(double power) {
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
    }
    public void setLeftDrive(double power) {
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
    }

    public void stopDriving() {
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void setArmMotors(double power) {
        armLeftMotor.setPower(power);
        armRightMotor.setPower(power);

    }

    public void runArmWithEncoders() {
        armLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runArmWithoutEncoders() {
        armLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setLeftServoPosition(double position) {
        leftServo.setPosition(position);
    }
    public void setRightServoPosition(double position) {
        rightServo.setPosition(position);
    }


    public void runDriveWithEncoders(int position) {
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        backLeftMotor.setTargetPosition(position);
        frontLeftMotor.setTargetPosition(position);
        backRightMotor.setTargetPosition(position);
        backRightMotor.setTargetPosition(position);

    }

    public double getLeftDriveEncoderDistance() {
        return backLeftMotor.getCurrentPosition() * INCHES_PER_TICK;
    }

    public double getRightDriveEncoderDistance() {
        return backRightMotor.getCurrentPosition() * INCHES_PER_TICK;
    }

    public double getArmEncoderAngle() {
        return armLeftMotor.getCurrentPosition() * DEGREE_PER_TICK;
    }
}
