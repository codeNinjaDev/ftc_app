package org.firstinspires.ftc.teamcode.commands;
/*** Allows sequential actions in autonomous ***/
import org.firstinspires.ftc.robotcore.external.Telemetry;

public interface Command {
    /*** Configures everything ***/
    void init();
    /*** Runs in a loop ***/
    void update(Telemetry tl);
    /*** Checks if command is finished ***/
    boolean isFinished();
    /*** Runs when command is finished ***/
    void finish();

}
