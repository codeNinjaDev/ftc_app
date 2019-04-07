package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HumanControl;
import org.firstinspires.ftc.teamcode.libs.PIDController;

/**
 * Created by peter on 11/22/17.
 */

public class DriveSubsystem implements Subsystem {
    private HumanControl humanControl;
    double MAX_SPEED = 1;
    int TICKS_PER_ROTATION = 288;
    double WHEEL_DIAMETER = 6;
    double INCHES_PER_TICK = (WHEEL_DIAMETER * Math.PI) / TICKS_PER_ROTATION;
    private double DEGREE_PER_TICK = 2;
    private Telemetry tl;
    public HardwareMap hardwareMap = null;

    DcMotor backLeftMotor = null;
    DcMotor backRightMotor = null;
    ElapsedTime pidTimer;

    public DriveSubsystem(HardwareMap hardwareMap, HumanControl humanControl, Telemetry tl) {
        pidTimer = new ElapsedTime();
        this.humanControl = humanControl;
        this.tl = tl;
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");


        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);


        resetEncoders();

    }

    public void init() {
        //resetEncoders();
    }

    public void arcadeDrive(double moveValue, double rotateValue) {
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
        tl.addData("ArcadeDriveLeftPower", Math.abs(left)*left*MAX_SPEED);
        tl.addData("ArcadeDriveRightPower", Math.abs(right)*right*MAX_SPEED);
        tl.addData("LeftMotor", backLeftMotor);
        tl.addData("LeftMotorPower", backLeftMotor.getPower());
        tl.addData("LeftMotorPort", backLeftMotor.getPortNumber());
        tl.addData("rightMotorPort", backRightMotor.getPortNumber());


        setLeftDrive(Math.abs(left)*left*MAX_SPEED);
        setRightDrive(Math.abs(right)*right*MAX_SPEED);
    }
    void tankDrive(double left, double right) {
        setLeftDrive(-left);
        setRightDrive(right);
    }

    public double getAverageDistance() {
        return (backLeftMotor.getCurrentPosition()*INCHES_PER_TICK + backRightMotor.getCurrentPosition()*INCHES_PER_TICK) / 2;
    }
    public void update(Telemetry telemetry) {
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
        resetEncoders();


        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void driveForward(double power) {

        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    public void setRightDrive(double power) {
        backRightMotor.setPower(power);
    }
    public void setLeftDrive(double power) {
        backLeftMotor.setPower(power);
    }

    public void stopDriving() {

        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void stop() {
        stopDriving();
        reset();
    }

}
