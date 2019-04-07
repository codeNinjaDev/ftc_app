package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/*** Drives a set distance in inches
 *
 *
 * ***/
public class DriveDistanceCommand implements Command {
    private DriveSubsystem driveSubsystem;
    private PIDController pidController;
    private double targetVelocity;
    private double targetDistance;
    private double timeout;
    private double[] PID;
    private ElapsedTime timer;

    private double currTime, pastTime, currPos, pastPos, deltaTime, deltaPos, currVelocity;
    public DriveDistanceCommand(DriveSubsystem driveSubsystem, double[] PID, double targetDistance, double targetVelocity, double timeout) {
        this.driveSubsystem = driveSubsystem;
        this.pidController = new PIDController(PID[0], PID[1], PID[2], 1, 5);
        this.targetVelocity = targetVelocity;
        this.targetDistance = targetDistance;
        this.timeout = timeout;
        currTime = pastTime = currPos = pastPos = deltaTime = deltaPos = currVelocity = 0;
        timer = new ElapsedTime();
    }

    @Override
    public void init() {
        driveSubsystem.resetEncoders();
        pidController.setSetpoint(targetVelocity);
        timer.reset();

    }

    @Override
    public void update() {

        //Calculate Velocity
        currPos = driveSubsystem.getAverageDistance();
        currTime = timer.milliseconds();
        deltaTime = currTime - pastTime; // In milliseconds
        deltaPos = currPos - pastPos;
        currVelocity = deltaPos / (deltaTime * .001);
        pastTime = currTime;
        currTime = pastTime;

        driveSubsystem.arcadeDrive(pidController.run(currVelocity), 0);

    }

    @Override
    public boolean isFinished() {
        boolean timeoutReached = timer.seconds() >= timeout;
        boolean positionReached = driveSubsystem.getAverageDistance() >= targetDistance;
        return timeoutReached || positionReached;
    }

    @Override
    public void finish() {

        driveSubsystem.stopDriving();
    }
}
