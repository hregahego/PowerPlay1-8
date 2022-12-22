package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.threeten.bp.DayOfWeek;

public class Intake implements Subsystem {

    //Arm constants
    private double lowArm = 0.1;
    private double idleArm = 0.75;
    private double placingArm = 0.6;
    private double groundArm = 0.0;

    //Claw constants
    private double openClaw = 0;
    private double closeClaw = 0;

    public Servo clawServoB;
    public Servo clawServo;
    public Servo armServo1;
    public Servo armServo2;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        clawServo = hardwareMap.get(Servo.class, "claw");
        clawServoB = hardwareMap.get(Servo.class, "claw2");
        armServo1 = hardwareMap.get(Servo.class, "arm");
        armServo2 = hardwareMap.get(Servo.class, "arm2");
    }

    @Override
    public void init() {

    }

    @Override
    public void update() {

    }

    @Override
    public void update(TelemetryPacket packet) {

    }

    public void closeClaw(){ setClawPos(closeClaw); }

    public void openClaw(){ setClawPos(openClaw); }

    public void setArmLow(){ setArmPos(lowArm); }

    public void setArmIdle(){ setClawPos(idleArm); }

    public void setArmPlacing(){ setClawPos(placingArm); }

    public void setArmGround(){ setClawPos(groundArm); }

    public void setClawPos(double position){
        clawServo.setPosition(position);
        clawServoB.setPosition(1-position);
    }

    public void setArmPos(double position){
        armServo1.setPosition(1-position);
        armServo2.setPosition(position);
    }

}
