package org.firstinspires.ftc.teamcode;

/**
 * Created by peter on 11/22/17.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class HumanControl {

    private Gamepad driverJoy;
    private Gamepad operatorJoy;

    private double driverLeftJoyX, driverLeftJoyY, driverRightJoyX, driverRightJoyY;
    private double operatorLeftJoyX, operatorLeftJoyY, operatorRightJoyX, operatorRightJoyY;

    boolean clampOpenDesired, clampClosedDesired, armDownDesired, armUpDesired, clampMiddleDesired, brakeDesired;



    public HumanControl(Gamepad driverJoy, Gamepad operatorJoy) {
        this.driverJoy = driverJoy;
        this.operatorJoy = operatorJoy;

        driverLeftJoyX = 0;
        driverLeftJoyY = 0;
        driverRightJoyX = 0;
        driverRightJoyY = 0;

        operatorLeftJoyX = 0;
        operatorLeftJoyY = 0;
        operatorRightJoyX = 0;
        operatorRightJoyY = 0;


        clampClosedDesired = false;
        clampOpenDesired = false;
        armDownDesired = false;
        armUpDesired = false;
        clampMiddleDesired = false;
        brakeDesired = false;
    }

    public void update() {
        driverLeftJoyX = getDriverLeftJoyX();
        driverLeftJoyY = getDriverLeftJoyY();
        driverRightJoyX = getDriverRightJoyX();
        driverRightJoyY = getDriverRightJoyY();

        operatorLeftJoyX = getOperatorLeftJoyX();
        operatorLeftJoyY = getOperatorLeftJoyY();
        operatorRightJoyX = getOperatorRightJoyX();
        operatorRightJoyY = getOperatorRightJoyY();

        clampClosedDesired = isClampClosedDesired();
        clampOpenDesired = isClampOpenDesired();
        armUpDesired = isArmUpDesired();
        armDownDesired = isArmDownDesired();
        clampMiddleDesired = isClampMiddleDesired();
        brakeDesired = isBrakeDesired();
    }

    public double getDriverLeftJoyY() {
        return -driverJoy.left_stick_y;
    }
    public double getDriverLeftJoyX() {
        return driverJoy.left_stick_x;
    }
    public double getDriverRightJoyY() {
        return driverJoy.right_stick_y;
    }
    public double getDriverRightJoyX() {
        return driverJoy.right_stick_x;
    }

    public double getOperatorLeftJoyY() {
        return -operatorJoy.left_stick_y;
    }
    public double getOperatorLeftJoyX() {
        return operatorJoy.left_stick_x;
    }
    public double getOperatorRightJoyY() {
        return operatorJoy.right_stick_y;
    }
    public double getOperatorRightJoyX() {
        return operatorJoy.right_stick_x;
    }

    public boolean isArmUpDesired() {
        if(getOperatorLeftJoyY() > 0) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isArmDownDesired() {
        if(getOperatorLeftJoyY() < 0) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isClampOpenDesired() {
        if(operatorJoy.a) {
            return true;
        } else {
            return false;
        }
    }
    public boolean isClampClosedDesired() {
        if(operatorJoy.y) {
            return true;
        } else {
            return false;
        }
    }

    public boolean isClampMiddleDesired() {
        if(operatorJoy.right_trigger )
    }

    public boolean isBrakeDesired() {
        if(operatorJoy.left_trigger > -1) {
            return true;
        } else {
            return false;
        }
    }

}
