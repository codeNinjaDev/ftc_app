package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.Command;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
/***Executes Command***/
public class CommandRunner {
    private LinearOpMode opMode;
    private Command command;
    private Telemetry telemetry;
    public CommandRunner(LinearOpMode opMode, Command command, Telemetry telemetry) {
        this.opMode = opMode;
        this.command = command;
        this.telemetry = telemetry;
    }
    //Runs command
    public void runCommand() {
        command.init();
        telemetry.addData("Command Status", command.isFinished());
        while(opMode.opModeIsActive() && (!command.isFinished())) {
            command.update();
            telemetry.update();
        }
        command.finish();
    }

}
