package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HumanControl;
import org.firstinspires.ftc.teamcode.libs.PIDController;
import org.firstinspires.ftc.teamcode.Parameters;
/**
 * Created by peter on 11/22/17.
 */

public class DriveSubsystem implements Subsystem {
    private HumanControl humanControl;
    double MAX_SPEED = 1;

    private Telemetry tl;
    public HardwareMap hardwareMap = null;

    public DcMotor backLeftMotor = null;
    DcMotor backRightMotor = null;
    ElapsedTime pidTimer;

    public DriveSubsystem(HardwareMap hardwareMap, HumanControl humanControl, Telemetry tl) {
        pidTimer = new ElapsedTime();
        this.humanControl = humanControl;
        this.tl = tl;
        this.hardwareMap = hardwareMap;
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
    public void tankDrive(double left, double right) {
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
        setLeftDrive(left);
        setRightDrive(right);
    }

    public double getAverageDistance() {
        return (backLeftMotor.getCurrentPosition()*Parameters.kInchesPerTick + backRightMotor.getCurrentPosition()*Parameters.kInchesPerTick) / 2;
    }
    
    public double getLeftDistance() {
        return (backLeftMotor.getCurrentPosition()*Parameters.kInchesPerTick);
    }
    
    public double getRightDistance() {
        return (backRightMotor.getCurrentPosition()*Parameters.kInchesPerTick);
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
        
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public void reset() {
        resetEncoders();




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

    // Computes the current battery voltage
    public double getBatteryVoltage() {
        try {
            double result = Double.POSITIVE_INFINITY;
            for (VoltageSensor sensor : hardwareMap.voltageSensor) {
                double voltage = sensor.getVoltage();
                if (voltage > 0) {
                    result = Math.min(result, voltage);
                }
            }
            tl.addData("Voltage ", result);
            tl.update();
            return result;
        } catch(Exception e) {
            tl.addData("DriveException ", e);
            tl.update();
            return 12;
        }
    }
}
