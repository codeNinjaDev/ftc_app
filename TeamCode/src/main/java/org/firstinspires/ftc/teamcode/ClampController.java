package org.firstinspires.ftc.teamcode;

/**
 * Created by peter on 11/22/17.
 */

import com.qualcomm.robotcore.hardware.Servo;


public class ClampController {
    private HumanControl humanControl;
    private RobotModel robot;
    double OPEN_POSITION_LEFT, CLAMPED_POSITION_LEFT, OPEN_POSITION_RIGHT, CLAMPED_POSITION_RIGHT, MIDDLE_POSITION_LEFT, MIDDLE_POSITION_RIGHT;

    //leftServo port 1 rightServo 0 Hub Top 10

    public ClampController(RobotModel robot, HumanControl humanControl) {
        this.robot = robot;
        this.humanControl = humanControl;

        OPEN_POSITION_RIGHT = 0;
        MIDDLE_POSITION_LEFT = .6;
        CLAMPED_POSITION_LEFT = 1;

        MIDDLE_POSITION_RIGHT = 1.0 - MIDDLE_POSITION_LEFT;
        OPEN_POSITION_RIGHT = 1.0 - OPEN_POSITION_LEFT;
        CLAMPED_POSITION_RIGHT = 1.0 - CLAMPED_POSITION_LEFT;
    }

    public void openClamp() {
        robot.setLeftServoPosition(OPEN_POSITION_LEFT);
        robot.setRightServoPosition(OPEN_POSITION_RIGHT);
    }

    public void clampClamp() {
        robot.setLeftServoPosition(CLAMPED_POSITION_LEFT);
        robot.setRightServoPosition(CLAMPED_POSITION_RIGHT);
    }

    public void middleClamp() {
        robot.setLeftServoPosition(MIDDLE_POSITION_LEFT);
        robot.setRightServoPosition(MIDDLE_POSITION_RIGHT);
    }



    public void update() {
        if(humanControl.isClampClosedDesired()) {
            clampClamp();
        } else if(humanControl.isClampMiddleDesired()) {
            middleClamp();
        }
    }
}
