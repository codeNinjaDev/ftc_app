package org.firstinspires.ftc.teamcode.commands;
/*** Allows sequential actions in autonomous ***/
public interface Command {
    /*** Configures everything ***/
    void init();
    /*** Runs in a loop ***/
    void update();
    /*** Checks if command is finished ***/
    boolean isFinished();
    /*** Runs when command is finished ***/
    void finish();

}
