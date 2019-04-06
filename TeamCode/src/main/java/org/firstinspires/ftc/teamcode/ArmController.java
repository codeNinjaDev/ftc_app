package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by peter on 11/22/17.
 */

public class ArmController {
    private RobotModel robot;
    private HumanControl humanControl;
    private PIDController armPID;
    Thread armPIDThread;

    public ArmController(RobotModel robot, HumanControl humanControl) {

        this.humanControl = humanControl;
        this.robot = robot;
        armPID = new PIDController(0.01, 0.0001, 0.009, .6, 5);
        robot.armLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }




    public void update() {


        if(humanControl.isArmUpDesired()) {
            robot.setArmMotors(humanControl.getOperatorLeftJoyY()*0.4);
        } else if(humanControl.isArmDownDesired()) {
            robot.setArmMotors(humanControl.getOperatorLeftJoyY()*0.4);
        } else {
            robot.setArmMotors(0);
        }
    }

    public void goToTopPosition() {

        //Go to 100 degrees
        armPID.setSetpoint(100);
        //Enable PID with encoder
        armPID.run(robot.getArmEncoderAngle());
    }

    public void goToMidPosition() {
        armPID.setSetpoint(60);
        armPID.run(robot.getArmEncoderAngle());

    }

    public void goToBottomPosition() {
        armPID.setSetpoint(30);
        armPID.run(robot.getArmEncoderAngle());

    }

    public boolean armPIDOnTarget() {
        return armPID.onTarget();
    }

}
