package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.dashboard.FtcDashboard;

@Config
@Autonomous
public class CycleAutoRedRightLow extends LinearOpMode {
    Pose2d START_POSE = new Pose2d(38,-64,Math.toRadians(180));
    Pose2d Preload_POSE = new Pose2d(38,-12,Math.toRadians(180));
    Robot robot;
    SleeveDetector detector = new SleeveDetector();
    SleeveDetectionPipeline.Color parkingPos = SleeveDetectionPipeline.Color.BLUE;
    private ElapsedTime timer;

    public double liftHigh = 1150;
    public double liftMid = 700;
    public double liftLow = 350;
    public double liftGround = 0;
    public double liftIdle = 200;
    public double liftIntaking = 0;

    public double turretFront = 0;
    public double turretLeft = 330;
    public double turretBack = 630;
    public double turretRight = 990;

    //change after tuning horizontal slides
    public double hzslidesout = 1.0;
    public double hzslidesin = 0.3;

    public void runOpMode() {
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();
        robot.intake.closeClaw();
        sleep(1500);
        robot.intake.dropArm();
        detector.init(hardwareMap, telemetry);

        robot.turret.MAX_POWER = 0.65;
//        robot.drive.voltagemode = "teleop";

        TrajectorySequence preload = robot.drive.trajectorySequenceBuilder(START_POSE)
                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))

                // Preplaced
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    robot.intake.centerArm();
                })
                .lineToLinearHeading(Preload_POSE)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    robot.lift.setTargetHeight(liftHigh);
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    robot.turret.setTargetAngle(530);
                })
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.openClaw();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    robot.lift.setHorizontalPosition(hzslidesin);
                })
                .build();

        //Cycle 1
        TrajectorySequence cycleLow = robot.drive.trajectorySequenceBuilder(Preload_POSE)
                .lineToLinearHeading(new Pose2d(47.5, -8.5, Math.toRadians(180)))

                //Pick Up Cone
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    robot.lift.setHorizontalPosition(hzslidesout);
                })
                .waitSeconds(1.0)

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    robot.intake.setArmPos(0.55);
                    robot.intake.closeClaw();
                })
                .waitSeconds(1.0)
//                .addTemporalMarker(() -> {
//                    robot.lift.setTargetHeight(liftLow);
//                    robot.lift.setHorizontalPosition(hzslidesin);
//                })
//                .addTemporalMarker(() -> {
//                    robot.turret.setTargetAngle(150);
//                })
//                .waitSeconds(0.7)
//                .addTemporalMarker(() -> {
//                    robot.lift.setHorizontalPosition(0.6);
//                })
//                .waitSeconds(1)
//                .addTemporalMarker(() -> {
//                    robot.lift.setTargetHeight(liftLow);
//                    robot.intake.fullyOpenClaw();
//                })
//                .waitSeconds(1)
//                .addTemporalMarker(() -> {
//                    robot.turret.setTargetAngle(turretBack);
//                })
//                .waitSeconds(0.3)
//                .build();
                .build();


        robot.drive.setPoseEstimate(START_POSE);

        waitForStart();

        robot.drive.followTrajectorySequenceAsync(preload);
//        robot.drive.followTrajectorySequenceAsync(cycleLow);

        while (opModeIsActive()) {
            Pose2d poseEstimate = robot.drive.getPoseEstimate();
            //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addData("turret pos", robot.turret.getCurrentAngle());
            telemetry.addData("turret target", robot.turret.getTargetAngle());
            telemetry.addData("slide pos", robot.lift.getCurrentHeight());
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.addData("time", timer.seconds());
            telemetry.update();
            robot.update();
            //drive.update();
        }
    }
}