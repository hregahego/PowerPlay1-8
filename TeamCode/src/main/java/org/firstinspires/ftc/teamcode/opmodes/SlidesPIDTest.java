package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetectionPipeline;
import org.firstinspires.ftc.teamcode.subsystems.SleeveDetector;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@TeleOp
public class SlidesPIDTest extends LinearOpMode{
    private Robot robot;
    private ElapsedTime timer;
    public enum Height {
        DOWN,
        LOW,
        MID,
        HIGH
    }

    FtcDashboard dashboard;

    public static double TARGET_POS = 0;

    public void runOpMode(){
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        robot.init();

        dashboard = FtcDashboard.getInstance();

        waitForStart();
        //robot.lift.setHeight(LiftConstants.IdleArm);
        telemetry.addData("loop:", "started");
        Height height = Height.DOWN;
        timer.reset();

        while(!isStopRequested()) {
            telemetry.addData("slide height", robot.lift.getCurrentHeight());
            telemetry.addData("lift PID", robot.lift.getPID());

            robot.update();
            telemetry.update();

            switch (height) {
                case DOWN:
                    robot.lift.setTargetHeight(0);
                    if (timer.milliseconds() > 1500) {
                        height = Height.LOW;
                        timer.reset();
                    }
                    break;
                case LOW:
                    robot.lift.setTargetHeight(250);
                    if (timer.milliseconds() > 1500) {
                        height = Height.MID;
                        timer.reset();
                    }
                    break;
                case MID:
                    robot.lift.setTargetHeight(500);
                    if (timer.milliseconds() > 2500) {
                        height = Height.HIGH;
                        timer.reset();
                    }
                    break;
                case HIGH:
                    robot.lift.setTargetHeight(800);
                    if (timer.milliseconds() > 1500) {
                        height = Height.LOW;
                        timer.reset();
                    }
                    break;
            }
        }
    }

    @Autonomous
    public static class CycleAutoRedRightLow extends LinearOpMode {
        Pose2d START_POSE = new Pose2d(33, -66.5, Math.toRadians(270));
        Robot robot;
        SleeveDetector detector = new SleeveDetector();
        SleeveDetectionPipeline.Color parkingPos = SleeveDetectionPipeline.Color.BLUE;
        private ElapsedTime timer;

        public double liftHigh = 1000;
        public double liftMid = 500;
        public double liftLow = 200;
        public double liftGround = 0;
        public double liftIdle = 200;
        public double liftIntaking = 0;

        //change after tuning horizontal slides
        public double hzslidesout = 0.4;
        public double hzslidesin = 0;

        public void runOpMode() {
            robot = new Robot(telemetry, hardwareMap);
            timer = new ElapsedTime();
            robot.init();
            robot.intake.closeClaw();
            sleep(1500);
            robot.intake.dropArm();
            detector.init(hardwareMap, telemetry);
    //        robot.lift.setOpmode("auto");

            TrajectorySequence parking1 = robot.drive.trajectorySequenceBuilder(START_POSE)
                    .setVelConstraint(robot.drive.getVelocityConstraint(10, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                    .setReversed(true)
                    // Preplaced
                    .lineTo(new Vector2d(36,-12))
                    .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                        robot.lift.setTargetHeight(liftHigh);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        robot.turret.setTargetAngle(45);
                        robot.intake.centerArm();
                        robot.lift.setHorizontalPosition(hzslidesout);
                        robot.intake.openClaw();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                        robot.turret.setTargetAngle(180);
                        robot.lift.setHorizontalPosition(hzslidesin);
                    })

                    // Cycle #1

                    .addTemporalMarker(() -> {
                    })
                    .setReversed(false)
                    .waitSeconds(0.9)
                    .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                    .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                        robot.lift.setTargetHeight(7.5);
                    })
                    .waitSeconds(0.2)
                    .lineTo(new Vector2d(52, -12))
                    //pick up cone
                    .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        robot.lift.setHorizontalPosition(hzslidesout);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                        robot.intake.closeClaw();
                    })
                    .waitSeconds(0.5)
                    .addTemporalMarker(() -> {
                        robot.lift.setTargetHeight(liftLow);
                    })
                    .setReversed(true)
                    .waitSeconds(0.3)
                    .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
                    .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                        robot.turret.setTargetAngle(280);
                    })
                    .waitSeconds(0.5)
                    .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                        robot.intake.openClaw();
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                        robot.turret.setTargetAngle(180);
                        robot.lift.setHorizontalPosition(hzslidesin);
                    })

    //                // Cycle #2
    //
    //                .addTemporalMarker(() -> {
    //                })
    //                .setReversed(false)
    //                .waitSeconds(0.9)
    //                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
    //                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
    //                    robot.lift.setTargetHeight(6.5);
    //                })
    //                .waitSeconds(0.2)
    //                .lineToLinearHeading(new Pose2d(70, -12, Math.toRadians(328)))
    //                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
    //                .forward(12)
    //                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
    //                //pick up cone
    //                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
    //                    robot.intake.closeClaw();
    //                })
    //                .waitSeconds(0.5)
    //                .addTemporalMarker(() -> {
    //                    robot.lift.setTargetHeight(liftLow);
    //                })
    //                .waitSeconds(0.1)
    //                .addTemporalMarker(() -> {
    //                    robot.intake.dropArm();
    //                })
    //                .setReversed(true)
    //                .waitSeconds(0.3)
    //                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
    //                .lineToLinearHeading(new Pose2d(50, 2, Math.toRadians(328)))
    //                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
    //                    robot.turret.setTargetAngle(360);
    //                    robot.lift.setTargetHeight(36);
    //                })
    //                .waitSeconds(0.5)
    //                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
    //                    robot.intake.centerArm();
    //                    robot.intake.openClaw();
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
    //                    robot.turret.setTargetAngle(240);
    //                    robot.intake.dropArm();
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(0.55, () -> {
    //                    robot.lift.setTargetHeight(liftIdle);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(0.55, () -> {
    //                    robot.intake.centerArm();
    //                })
    //                // Cycle #3
    //
    //
    //                .addTemporalMarker(() -> {
    //                })
    //                .setReversed(false)
    //                .waitSeconds(0.9)
    //                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
    //                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
    //                    robot.lift.setTargetHeight(5.5);
    //                })
    //                .waitSeconds(0.2)
    //                .lineToLinearHeading(new Pose2d(70, -12, Math.toRadians(324)))
    //                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
    //                .forward(12)
    //                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
    //                //pick up cone
    //                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
    //                    robot.intake.closeClaw();
    //                })
    //                .waitSeconds(0.5)
    //                .addTemporalMarker(() -> {
    //                    robot.lift.setTargetHeight(liftLow);
    //                })
    //                .waitSeconds(0.1)
    //                .addTemporalMarker(() -> {
    //                    robot.intake.dropArm();
    //                })
    //                .setReversed(true)
    //                .waitSeconds(0.3)
    //                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
    //                .lineToLinearHeading(new Pose2d(44, 2, Math.toRadians(324)))
    //                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
    //                    robot.turret.setTargetAngle(360);
    //                    robot.lift.setTargetHeight(36);
    //                })
    //                .waitSeconds(0.5)
    //                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
    //                    robot.intake.centerArm();
    //                    robot.intake.openClaw();
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
    //                    robot.turret.setTargetAngle(240);
    //                    robot.intake.dropArm();
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(0.55, () -> {
    //                    robot.lift.setTargetHeight(liftIdle);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(0.55, () -> {
    //                    robot.intake.centerArm();
    //                })
    //                // Cycle #4
    //
    //
    //                .addTemporalMarker(() -> {
    //                })
    //                .setReversed(false)
    //                .waitSeconds(0.9)
    //                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
    //                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
    //                    robot.lift.setTargetHeight(4);
    //                })
    //                .waitSeconds(0.2)
    //                .lineToLinearHeading(new Pose2d(70, -12, Math.toRadians(320)))
    //                .setVelConstraint(robot.drive.getVelocityConstraint(20, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
    //                .forward(12)
    //                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
    //                //pick up cone
    //                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
    //                    robot.intake.closeClaw();
    //                })
    //                .waitSeconds(0.5)
    //                .addTemporalMarker(() -> {
    //                    robot.lift.setTargetHeight(liftLow);
    //                })
    //                .waitSeconds(0.1)
    //                .addTemporalMarker(() -> {
    //                    robot.intake.dropArm();
    //                })
    //                .setReversed(true)
    //                .waitSeconds(0.3)
    //                .setVelConstraint(robot.drive.getVelocityConstraint(40, Math.toRadians(180), DriveConstants.TRACK_WIDTH))
    //                .lineToLinearHeading(new Pose2d(44, 2, Math.toRadians(332.5)))
    //                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
    //                    robot.turret.setTargetAngle(360);
    //                    robot.lift.setTargetHeight(36);
    //                })
    //                .waitSeconds(0.5)
    //                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
    //                    robot.intake.centerArm();
    //                    robot.intake.openClaw();
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
    //                    robot.turret.setTargetAngle(240);
    //                    robot.intake.dropArm();
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(0.55, () -> {
    //                    robot.lift.setTargetHeight(liftIdle);
    //                })
    //                .UNSTABLE_addTemporalMarkerOffset(0.55, () -> {
    //                    robot.intake.centerArm();
    //                })

                    //park

                    .back(10)
                    .build();


            robot.drive.setPoseEstimate(START_POSE);

            // Waiting for start
            while (!isStarted() && !isStopRequested()) {
                //parkingPos = detector.getColor();
                telemetry.addData("timer", timer.milliseconds());
                telemetry.update();
            }

            // Start...
            detector.stop();
            waitForStart();

            if (parkingPos == SleeveDetectionPipeline.Color.MAGENTA) {
                robot.drive.followTrajectorySequenceAsync(parking1);
                detector.stop();
            } else if (parkingPos == SleeveDetectionPipeline.Color.BLUE) {
                robot.drive.followTrajectorySequenceAsync(parking1);
                detector.stop();
            } else if (parkingPos == SleeveDetectionPipeline.Color.RED) {
                robot.drive.followTrajectorySequenceAsync(parking1);
                detector.stop();
            }

            while (opModeIsActive()) {
                telemetry.addData("turret pos", robot.turret.getCurrentAngle());
                telemetry.addData("turret target", robot.turret.getTargetAngle());
    //            telemetry.addData("opmode", robot.lift.getOpmode());
                telemetry.addData("slide pos", robot.lift.getCurrentHeight());
                telemetry.update();
                robot.update();
            }
        }
    }
}