package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by peter on 11/22/17.
 */

public class DriveController {
    private HumanControl humanControl;
    double MAX_SPEED = 1;
    int TICKS_PER_ROTATION = 1120;
    double WHEEL_DIAMETER = 6;
    double INCHES_PER_TICK = (WHEEL_DIAMETER * Math.PI) / TICKS_PER_ROTATION;
    private double DEGREE_PER_TICK = 2;

    public HardwareMap hardwareMap = null;

    DcMotor frontLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor backRightMotor = null;
    ElapsedTime pidTimer;

    PIDController drivePID;
    public DriveController(RobotModel robot, HumanControl humanControl) {
        pidTimer = new ElapsedTime();
        this.humanControl = humanControl;
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");

        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");


        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);


        resetEncoders();
        drivePID = new PIDController(.03, .0001, .001, 1, 1);

    }



    void arcadeDrive(double moveValue, double rotateValue) {
        double left = moveValue + rotateValue;
        double right = moveValue - rotateValue;

        if(left > 1) {
            left = 1;
        } else if(left < -1) {
            left = -1;
        }

        if(right > 1) {
            right = 1;
        } else if(right < -1) {
            right = -1;
        }

        setLeftDrive(Math.abs(left)*left*MAX_SPEED);
        setRightDrive(Math.abs(right)*right*MAX_SPEED);
    }
    void tankDrive(double left, double right) {
        setLeftDrive(-left);
        setRightDrive(right);
    }

    public void runToDistance(double distance) {
        pidTimer.reset();
        resetEncoders();
        drivePID.setSetpoint(distance);
        do {
            arcadeDrive(drivePID.run(backLeftMotor.getCurrentPosition()*INCHES_PER_TICK), 0);
            pidTimer.reset();
        } while(!drivePID.onTarget() && (pidTimer.milliseconds() <= 20));

        arcadeDrive(0, 0);

    }
    void update() {
        if(humanControl.isBrakeDesired()) {
            MAX_SPEED = .7;
        } else {
            MAX_SPEED = 1;
        }
        arcadeDrive(humanControl.getDriverLeftJoyY(), humanControl.getDriverRightJoyX());
        //arcadeDrive(humanControl.getDriverRightJoyX(), humanControl.getDriverLeftJoyY());
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

}
