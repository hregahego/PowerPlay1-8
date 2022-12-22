package org.firstinspires.ftc.teamcode.controls;

import com.qualcomm.robotcore.util.ElapsedTime;

import dalvik.system.DelegateLastClassLoader;

public class PIDController {
    public static double kP, kI, kD;
    public static double integralSum;
    private double lastError;
    private ElapsedTime timer;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integralSum = 0;
        this.lastError = 0;
        this.timer = new ElapsedTime();
    }

    public double calculate(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * kP) + (derivative * kD) + (integralSum * kI);
        return output;
    }

    public void setPID(double P, double I, double D) {
        this.kP = P;
        this.kI = I;
        this.kD = D;
    }
}
