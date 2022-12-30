package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Autonomous
public class StrafingTest extends LinearOpMode {
    //    Pose2d START_POSE = new Pose2d(0, 0, 0);
    Drivetrain drivetrain;
    private ElapsedTime timer;

    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        Drivetrain drivetrain;
        drivetrain = new Drivetrain(hardwareMap);
        timer = new ElapsedTime();
        waitForStart();
        timer.reset();
        double power = 0.3;
        while (!isStopRequested() && opModeIsActive()) {
            if (timer.seconds() < 3) {
                drivetrain.leftFront.setPower(-1 * power);
                drivetrain.rightFront.setPower(power * 1.075);
                drivetrain.leftRear.setPower(power);
                drivetrain.rightRear.setPower(-1 * power);
            } else {
                drivetrain.leftFront.setPower(0);
                drivetrain.rightFront.setPower(0);
                drivetrain.rightRear.setPower(0);
                drivetrain.leftRear.setPower(0);
            }
            telemetry.addData("time", timer);
            telemetry.update();

        }
    }
}