package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.onbotjava.handlers.admin.ResetOnBotJava;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turret implements Subsystem {

    public DcMotorEx tmotor;

    public static double kP = 0.2, kI = 0, kD = 0, ff = 0;
    public static double integralSum = 0;
    private double lastError = 0;

    private double targetAngle = 0;
    private double currentAngle;
    private final double deadzone = 5;
    public double rotation = 0;

    private final double ticks_to_degrees = 1.0;
    private final double MAX_POWER = 0.8;

    ElapsedTime timer = new ElapsedTime();

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        tmotor = hardwareMap.get(DcMotorEx.class, "turret");
    }

    @Override
    public void init() {
        setTargetAngle(0);
        tmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        currentAngle = getCurrentAngle();
        if (Math.abs(currentAngle - targetAngle) < deadzone) {
            setTurretPower(0);
        } else if (Math.abs(rotation) > 0.1) {
            setTurretPower(rotation * ff);
        } else {
            setTurretPower(PIDController(targetAngle, currentAngle));
        }
    }

    @Override
    public void update(TelemetryPacket packet) {
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

    public double getTargetAngle() {
        return targetAngle;
    }

    public void setTargetAngle(double angle) {
        targetAngle = angle;
    }

    public double getCurrentAngle() {
        double angle = tmotor.getCurrentPosition() * ticks_to_degrees;
        return angle;
    }

    public void setTurretPower(double power) {
        power = Range.clip(power, -MAX_POWER, MAX_POWER);
        tmotor.setPower(power);
    }

    public void setRotation(double gamepadInput) {
        rotation = gamepadInput;
    }

    public double getPID() { return PIDController(targetAngle, currentAngle); }

    public double getMotorPower() { return tmotor.getPower(); }

    public void setFF(double input) {
        ff = input;
    }

    public void setPID (double P, double I, double D) {
        kP = P;
        kI = I;
        kD = D;
    }

}
