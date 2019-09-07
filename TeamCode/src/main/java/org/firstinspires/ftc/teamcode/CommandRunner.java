package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.Command;
import java.util.concurrent.*;
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
        final ScheduledExecutorService scheduledExecutorService = Executors.newScheduledThreadPool(3);
        

        Runnable updateMethod = new Runnable() {
            @Override
            public void run() {
                try {
                    telemetry.addData("Running: ", true);
                    command.update(telemetry);
                    telemetry.update();
                } catch(Exception e) {
                    telemetry.addData("Running: ", false);
                    telemetry.addData("Exception: ", e);

                    telemetry.update();
                }
            }
        };
        try {
            scheduledExecutorService.scheduleAtFixedRate(updateMethod, 0,20, TimeUnit.MILLISECONDS);
            while(!command.isFinished() && opMode.opModeIsActive()) {
                
            }
            scheduledExecutorService.shutdownNow();

        } catch (Exception e) {
            command.finish();
            throw e;
        }
        command.finish();

    }

}
