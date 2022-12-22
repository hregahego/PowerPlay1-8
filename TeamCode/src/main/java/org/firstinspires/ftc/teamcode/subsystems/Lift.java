package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.controls.PIDController;

public class Lift implements Subsystem {

    private DcMotorEx motor1;
    private DcMotorEx motor2;

    public static double kP = 0.023, kI = 0, kD = 0.0004; //0.6, 0, 0.01
    private static double ff = 0.16;
    public static double integralSum = 0;
    private double lastError = 0;

    private double targetHeight = 0;
    private double currentHeight;

    private final double ticks_to_inches = 1.0;
    private final double MAX_POWER = 1;

    private PIDController pidController;

    ElapsedTime timer = new ElapsedTime();

    public Lift(HardwareMap hardwareMap, Telemetry telemetry) {
        //pidController = new PIDController(kP, kI, kD);
        motor1 = hardwareMap.get(DcMotorEx.class, "lift1");
        motor2 = hardwareMap.get(DcMotorEx.class, "lift2");

        //reverse correctly
        motor1.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void init() {
        setTargetHeight(0);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {

        if (targetHeight > 1000) {
            targetHeight = 1000;
        }
        currentHeight = getCurrentHeight();

        if (targetHeight == 0 && currentHeight < 0.3) {
            setLiftPower(0);
        } else if (motor1.getPower() > 0.5 && motor1.getCurrent(CurrentUnit.AMPS) > 10) {
            setLiftPower(-1);
        } else if (Math.abs(targetHeight - currentHeight) < 5) {
            setLiftPower(ff);
        } else {
            //setLiftPower(pidController.calculate(targetHeight, currentHeight));
            setLiftPower(PIDController(targetHeight, currentHeight));
        }
    }

    @Override
    public void update(TelemetryPacket packet) {

    }

    public void setLiftPower(double power) {
        power = Range.clip(power, -0.3,MAX_POWER);
        motor2.setPower(power);
        motor1.setPower(power);
    }

    public double PIDController(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * kP) + (derivative * kD) + (integralSum * kI);
        return output;
    }

    public void setTargetHeight(double height) {
        targetHeight = height;
    }

    public Double getTargetHeight() {
        return targetHeight;
    }


    public double getCurrentHeight() {
        double height = motor2.getCurrentPosition() * ticks_to_inches;
        return height;
    }

    public double getMotorPower() {
        return motor2.getPower();
    }

    public double getPID() {
        return PIDController(targetHeight, currentHeight);
    }

    public void setFF(double input) {
        ff = input;
    }

    public void setPID (double P, double I, double D) {
        kP = P;
        kI = I;
        kD = D;
    }
}
