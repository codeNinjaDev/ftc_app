package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.libs.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.Parameters;

/*** Drives a set distance in inches
 *
 *
 * ***/
public class DriveDistanceCommand implements Command {
    private DriveSubsystem driveSubsystem;
    private PIDController pidController;
    private double targetVelocity, leftVelocity, rightVelocity;
    private double targetDistance;
    private double timeout;
    private double kV, kS;
    private double[] pastLVelocities;
    private double[] pastRVelocities;
    private double leftErrorDeriv, leftPositionError, lastLeftPositionError;
    private double rightErrorDeriv, rightPositionError, lastRightPositionError;

    private ElapsedTime timer;

    private double currLeftPos, pastLeftPos, deltaTime, deltaLeftPos, currRightPos, pastRightPos, deltaRightPos, currTime, pastTime;
    public DriveDistanceCommand(DriveSubsystem driveSubsystem, double[] PID, double targetDistance, double targetVelocity, double timeout) {
        this.driveSubsystem = driveSubsystem;
        this.pidController = new PIDController(PID[0], PID[1], PID[2], 1, 5);
        this.targetVelocity = targetVelocity;
        this.targetDistance = targetDistance;
        this.leftVelocity = 0;
        this.rightVelocity = 0;
        this.pastLeftPos = 0;
        this.pastRightPos = 0;
        this.timeout = timeout;
        this.deltaTime = 0;
        this.pastTime = 0;
        this.currTime = 0;
        lastLeftPositionError = 0;
        lastRightPositionError = 0;
        timer = new ElapsedTime();
        pastLVelocities = new double[5];
        pastRVelocities = new double[5];

    }

    @Override
    public void init() {
        driveSubsystem.resetEncoders();
        pidController.setSetpoint(targetVelocity);
        timer.reset();


    }


    @Override
    public void update(Telemetry tl) {

        //Calculate Velocity
        currRightPos = driveSubsystem.getRightDistance();
        currLeftPos = driveSubsystem.getLeftDistance();
        currTime = timer.milliseconds();
        deltaTime = currTime - pastTime;
        deltaLeftPos = currLeftPos - pastLeftPos;
        leftVelocity = deltaLeftPos / (deltaTime * 0.001) ;

        deltaRightPos = currRightPos - pastRightPos;
        rightVelocity = deltaRightPos / (deltaTime * 0.001);
        leftPositionError = targetDistance - currLeftPos;
        rightPositionError = targetDistance - currRightPos;
        leftErrorDeriv = (leftPositionError-  lastLeftPositionError) / (deltaTime * 0.001);
        rightErrorDeriv = (rightPositionError-  lastRightPositionError) / (deltaTime * 0.001);

        pastRightPos = currRightPos;
        pastLeftPos = currLeftPos;
        lastLeftPositionError = leftPositionError;
        lastRightPositionError = rightPositionError;
        pastTime = currTime;
        rightVelocity = sampleEncoderVelocity(rightVelocity, pastRVelocities);
        leftVelocity = sampleEncoderVelocity(leftVelocity, pastLVelocities);
        tl.addData("dS ", deltaLeftPos);
        tl.addData("dT ", deltaTime);
        tl.addData("LVelocity", leftVelocity);
        tl.addData("RVelocity", rightVelocity);

        kV = (Parameters.kV * targetVelocity) / driveSubsystem.getBatteryVoltage(); //kV part
        kS =  (Math.signum(kV) * Parameters.kVIntercept) / driveSubsystem.getBatteryVoltage();
        pidController.setArbitraryFeedForward(kV + kS);
        tl.addData("kV ", kV);
        tl.addData("PkV ", Parameters.kV);
        tl.addData("targetVelo ", targetVelocity);
        double kP = 0.1;
        double kD = 0.05;

        //double leftOutput = Math.min(pidController.run(leftVelocity) + kP*leftPositionError + kD*leftErrorDeriv, 1);
        //double rightOutput =Math.min(pidController.run(rightVelocity) + kP*rightPositionError + kD*rightErrorDeriv, 1);

        double leftOutput = Math.min(kV + kS + kP*leftPositionError + kD*leftErrorDeriv, 1);
        double rightOutput =Math.min(kV + kS + kP*rightPositionError + kD*rightErrorDeriv, 1);
        driveSubsystem.tankDrive(leftOutput, rightOutput);

    }

    @Override
    public boolean isFinished() {
        boolean timeoutReached = timer.seconds() >= timeout;
        boolean positionReached = Math.abs(targetDistance - driveSubsystem.getAverageDistance()) <= .2 ;
        return timeoutReached || positionReached;
    }

    private double sampleEncoderVelocity(double currentVelocity, double[] pastVelocities) {
        for(int i = 0; i < pastVelocities.length - 1; i++) {
            pastVelocities[i + 1] = pastVelocities[i];
        }
        pastVelocities[0] = currentVelocity;
        double average = 0;
        for(int i = 0; i < pastVelocities.length; i++) {
            average += pastVelocities[i];
        }
        average = average / pastVelocities.length;
        return average;
    }
    @Override
    public void finish() {

        driveSubsystem.stopDriving();
    }
}



