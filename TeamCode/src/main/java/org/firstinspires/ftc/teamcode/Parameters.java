package org.firstinspires.ftc.teamcode;


public class Parameters {
    public static double kV = 1 / 2.05; //Units are V*s/in
    public static double kVIntercept = 0.449268; //Units are V
    public static double[] kVelocityPID = {0, 0, 0};
    public static double kWheelDiameter = 3.54; //Units are inches
    public static double kTicksPerRev = 288; //Hex Core Motor
    public static double kInchesPerTick = (kWheelDiameter * Math.PI) / kTicksPerRev;
    // todo: write your code here
}