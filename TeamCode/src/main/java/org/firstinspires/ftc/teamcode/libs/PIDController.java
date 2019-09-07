package org.firstinspires.ftc.teamcode.libs;

/**
 * Created by peter on 4/11/18.
 */

public class PIDController {
    double P, I, D, maxRange, percentTolerance, setpoint, summation, lastError, currentError, feedback;
    double deltaError;
    double output;
    double aff;
    public PIDController(double P, double I, double D, double maxRange, double percentTolerance) {
        this.P = P;
        this.I = I;
        this.D = D;
        summation = 0;
        feedback = 0;
        output = 0;
        this.maxRange = maxRange;
        this.percentTolerance = percentTolerance;
        aff = 0;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double run(double feedback) {
        this.feedback = feedback;
        currentError = setpoint - feedback;
        summation += currentError * I;
        deltaError = currentError - lastError;
        output = P * currentError + I * summation + D * deltaError;
        lastError = currentError;
        output += aff;
        if(output > maxRange)
            output = maxRange;
        else if(output < -maxRange)
            output = -maxRange;
        return output;
    }
    public void setArbitraryFeedForward(double ff) {
        aff = ff;
    }
    public boolean onTarget() {
        currentError = setpoint -feedback;
        double percent = percentTolerance * .01;
        if(Math.abs(currentError) < setpoint*percent)
            return true;
        else
            return false;

    }






}
