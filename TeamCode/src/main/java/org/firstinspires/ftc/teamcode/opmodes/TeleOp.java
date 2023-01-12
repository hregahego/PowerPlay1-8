//Imports
package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    //Constants
    private Robot robot;
    private robotState robotState;
    private robotMode robotMode;
    private switchToAUTOCYCLE switchToAUTOCYCLE;
    private switchNormal switchNormal;
    private switchStack switchStack;
    private ElapsedTime timer;
    private ElapsedTime turrettimer;
    private ElapsedTime armtimer;
    private ElapsedTime autotimer;
    private ElapsedTime timer2;
    private ElapsedTime timer3;
    private boolean canTurn = false;
    double turretaddition = 10;
    double dtspeed = 1;
    public double up = 1150;
    public double mid = 700;
    public double low = 350;
    public double ground = 0;
    public double idle = 200;
    public double intaking = 10;
    private double front = 0;
    private double back = 660;
    private double right = 330;
    private double left = -330;
    private double droppedvalue = 150;
    private boolean armup = true;
    private boolean lowheight = false;
    private double horizontalback = 0.44;
    private double horizontallifted = 0.5;
    private double horizontalmiddle = 0.61;
    private double horizontalextended = 1;
    private String horizontalpos = "back";
    private String liftedpos = "lifted";
    private boolean running = false;
    private double highposarm = 0.6;

    private double prev_time = System.currentTimeMillis();

    public enum robotState {
        IDLE,
        IDLE2,
        INTAKING,
        INTAKING2,
        GRABBED,
        LIFTED,
        DROPPED
    }

    public enum robotMode {
        STACK,
        NORMAL,
        AUTOCYCLE
    }

    public enum switchToAUTOCYCLE {
        Idle,
        FirstActions,
        SecondActions,
        ThirdActions
    }

    public enum switchNormal {
        Idle,
        FirstActions,
        SecondActions
    }
    public enum switchStack {
        Idle,
        FirstActions,
        SecondActions
    }

    public void runOpMode() throws InterruptedException {
//Init
        robot = new Robot(telemetry, hardwareMap);
        timer = new ElapsedTime();
        turrettimer = new ElapsedTime();
        armtimer = new ElapsedTime();
        autotimer = new ElapsedTime();
        timer2 = new ElapsedTime();
        timer3 = new ElapsedTime();
        waitForStart();
        robot.init();
        robotState = robotState.INTAKING;
        robot.intake.openClaw();
        robot.drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.turret.tmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armup = true;
        robot.turret.MAX_POWER = 1;
        robotMode = robotMode.NORMAL;
        switchToAUTOCYCLE = switchToAUTOCYCLE.Idle;
        switchNormal = switchNormal.Idle;
        switchStack = switchStack.Idle;
        robot.drive.voltagemode = "teleop";
        double TurretPower = gamepad2.right_stick_x;

        double distance;

//Drivetrain
        while (!isStopRequested() && opModeIsActive()) {

            double dt = System.currentTimeMillis() - prev_time;
            prev_time = System.currentTimeMillis();

            telemetry.addData("loop time", dt);


            //anti-tip + regular teleop code - ONLY ON PITCH RIGHT NOW
            double antiTipMulti = 1;
//            double correctedpitch = robot.drive.getOrientation().thirdAngle + 3.13;
            robot.drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * dtspeed, //  controls forward
                            -gamepad1.left_stick_x * dtspeed, +//controls strafing
                            -gamepad1.right_stick_x * dtspeed //controls turning
                    )
            );

            //Slow-Mode
            if (gamepad1.right_bumper == true) {
                dtspeed = 0.4;
            } else {
                dtspeed = 1;
            }

            //Vertical Slides Manual Control
            if (gamepad2.left_stick_y > 0.5) {
                robot.lift.setTargetHeight(robot.lift.getCurrentHeight() + 10);
            } else if (gamepad2.left_stick_y < -0.5) {
                robot.lift.setTargetHeight(robot.lift.getCurrentHeight() - 10);
            }

            //Turret Manual Control
            if (canTurn = true) {
                if (TurretPower > 0.5) {
                    robot.turret.setTargetAngle(robot.turret.getCurrentAngle() - turretaddition);
                } else if (TurretPower < -0.5) {
                    robot.turret.setTargetAngle(robot.turret.getCurrentAngle() + turretaddition);
                }
            }

            switch (robotMode) {
                case NORMAL:
                    //Switching Modes
//                    switch(switchToAUTOCYCLE) {
//                        case Idle:
//                                if (gamepad1.x) {
//                                    switchToAUTOCYCLE = switchToAUTOCYCLE.FirstActions;
//                                    robot.intake.stopIntake();
//                                }
//                            break;
//                        case FirstActions:
//                            robot.lift.setTargetHeight(idle);
//                            robot.intake.liftArm();
//                            robot.intake.closeClaw();
//                            robot.lift.setHorizontalPosition(horizontallifted);
//                            switchToAUTOCYCLE = switchToAUTOCYCLE.SecondActions;
//                            timer.reset();
//                            break;
//                        case SecondActions:
//                            if (timer.milliseconds() > 700) {
//                                robot.turret.setTargetAngle(front);
//                                switchToAUTOCYCLE = switchToAUTOCYCLE.ThirdActions;
//                                timer.reset();
//                            }
//                            break;
//                        case ThirdActions:
//                            if (timer.milliseconds() > 300) {
//                                robot.lift.setTargetHeight(intaking);
//                                robot.intake.centerArm();
//                                robot.intake.fullyOpenClaw();
//                                robotMode = robotMode.AUTOCYCLE;
//                                switchToAUTOCYCLE = switchToAUTOCYCLE.Idle;
//                                timer.reset();
//                            }
                    //}

                    switch(switchStack) {
                        case Idle:
                            if (gamepad1.a && gamepad1.start == false) {
                                switchStack = switchStack.FirstActions;
                                robot.intake.stopIntake();
                                timer.reset();
                            }
                            break;
                        case FirstActions:
                            robot.lift.setTargetHeight(idle);
                            robot.intake.liftArm();
                            robot.intake.closeClaw();
                            robot.lift.setHorizontalPosition(horizontallifted);
                            switchStack = switchStack.SecondActions;
                            timer.reset();
                            break;
                        case SecondActions:
                            if (timer.milliseconds() > 700) {
                                robot.turret.setTargetAngle(back);
                                robotMode = robotMode.STACK;
                                robotState = robotState.IDLE;
                                switchStack = switchStack.Idle;
                                timer.reset();
                            }
                    }



                    //Horizontal Slides Extend/Retract


                    //Vertical Slides SlightDrop/SlightLift
                    if (robotState == robotState.LIFTED && gamepad2.left_bumper && liftedpos == "lifted") {
                        robot.lift.setTargetHeight(robot.lift.getCurrentHeight() - droppedvalue);
                        horizontalpos = "dropped";
                    } else if (robotState == robotState.LIFTED && gamepad2.left_bumper && liftedpos == "dropped") {
                        robot.lift.setTargetHeight(robot.lift.getCurrentHeight() + droppedvalue);
                        horizontalpos = "lifted";
                    }

                    //Turret Presets
                    if (canTurn = true) {
                        if (turrettimer.milliseconds() > 500) {
                            if (gamepad2.a) {
                                robot.turret.setTargetAngle(back);
                                turrettimer.reset();
                            }
                            if (gamepad2.x) {
                                robot.turret.setTargetAngle(right);
                                turrettimer.reset();
                            }
                            if (gamepad2.b && gamepad2.start == false) {
                                robot.turret.setTargetAngle(left);
                                turrettimer.reset();
                            }
                            if (gamepad2.y) {
                                robot.turret.setTargetAngle(front);
                                turrettimer.reset();
                            }
                            if (gamepad1.y) {
                                robot.turret.setTargetAngle(front);
                                turrettimer.reset();
                            }
                        }
                    }

                    //FSM
                    switch (robotState) {
                        case IDLE:
                            canTurn = true;
                            robot.intake.closeClaw();
                            robot.intake.liftArm();
                            robot.lift.setHorizontalPosition(horizontallifted);
                            robot.lift.setTargetHeight(idle);
                            if (timer.milliseconds() > 500) {
                                if (gamepad1.left_bumper || timer.milliseconds()>1250) {
                                    robot.intake.dropArm();
                                    timer.reset();
                                    timer2.reset();
                                    robotState = robotState.IDLE2;
                                }
                            }
                            break;
                        case IDLE2:
                            if (timer.milliseconds()>100){
                                robot.lift.setTargetHeight(intaking);
                                robotState = robotState.INTAKING;
                                timer.reset();
                                timer2.reset();
                            }
                            break;
                        case INTAKING:
                            canTurn = false;
                            horizontalpos = "back";
                            robot.lift.setHorizontalPosition(horizontalback);
                            robot.intake.dropArm();
                            robot.lift.setTargetHeight(intaking);
                            robot.intake.intake(1);
                            if (timer2.milliseconds() > 500) {
                                robot.intake.openClaw();
                                if (timer.milliseconds() > 500) {
                                    if (gamepad1.left_bumper || gamepad2.left_bumper || robot.intake.getDistance() < 3) {
                                        robot.intake.closeClaw();
                                        robot.intake.stopIntake();
                                        robotState = robotState.INTAKING2;
                                        timer2.reset();
                                        timer.reset();
                                    }
                                }
                            }
                            break;
                        case INTAKING2:
                            if(timer.milliseconds()>600){
                                robotState = robotState.GRABBED;
                            }
                            break;
                        case GRABBED:
                            canTurn = true;
                            robot.lift.setHorizontalPosition(horizontallifted);
                            if (timer.milliseconds() > 300) {
                                robot.intake.liftArm();
                                robot.lift.setTargetHeight(idle);
                            }
                            if (timer.milliseconds() > 500) {
                                if (gamepad1.left_bumper) {
                                    robot.intake.dropArm();
                                    timer.reset();
                                    robotState = robotState.IDLE2;
                                }
                                if (gamepad2.dpad_up) {
                                    robot.lift.setTargetHeight(up);
                                    robot.intake.setArmPos(highposarm);
                                    timer.reset();
                                    robotState = robotState.LIFTED;
                                    armup = true;
                                    lowheight = false;
                                }
                                if (gamepad2.dpad_right) {
                                    robot.lift.setTargetHeight(mid);
                                    robot.intake.centerArm();
                                    timer.reset();
                                    robotState = robotState.LIFTED;
                                    armup = true;
                                    lowheight = false;
                                }
                                if (gamepad2.dpad_left) {
                                    robot.lift.setTargetHeight(low);
                                    robot.intake.centerArm();
                                    timer.reset();
                                    robotState = robotState.LIFTED;
                                    armup = true;
                                    lowheight = false;
                                }
                                if (gamepad2.dpad_down) {
                                    if (Math.abs(robot.turret.getCurrentAngle() - back) < 5) {
                                        robot.lift.setTargetHeight(ground);
                                        robot.intake.dropArm();
                                        timer.reset();
                                        robotState = robotState.LIFTED;
                                        armup = false;
                                        lowheight = true;
                                    } else {
                                        robot.turret.setTargetAngle(back);
                                        if (Math.abs(robot.turret.getCurrentAngle() - back) < 5) {
                                            robot.lift.setTargetHeight(ground);
                                            robot.intake.dropArm();
                                            timer.reset();
                                            robotState = robotState.LIFTED;
                                            armup = false;
                                            lowheight = true;
                                        }
                                    }
                                }
                            }
                            break;
                        case LIFTED:
                            canTurn = true;
                            dtspeed = 0.4;
                            horizontalpos = "middle";
                            liftedpos = "lifted";
                            robot.intake.intake(-1);
                            robot.lift.setHorizontalPosition(horizontalmiddle);
                            if (timer.milliseconds() > 750) {
                                if (gamepad2.dpad_down || gamepad1.dpad_down) {
                                    timer.reset();
                                    robot.turret.setTargetAngle(front);
                                    robot.lift.setTargetHeight(idle);
                                    robotState = robotState.GRABBED;
                                }
                                if (gamepad1.left_bumper) {
                                    if (liftedpos == "lifted") {
                                        robot.lift.setTargetHeight(robot.lift.getCurrentHeight() - droppedvalue);
                                    }
                                    timer.reset();
                                    robotState = robotState.DROPPED;
                                }
                                if (gamepad2.dpad_up) {
                                    robot.lift.setTargetHeight(up);
                                    robot.intake.setArmPos(highposarm);
                                    timer.reset();
                                }
                                if (gamepad2.dpad_left) {
                                    robot.lift.setTargetHeight(low);
                                    robot.intake.centerArm();
                                }
                                if (gamepad2.dpad_right) {
                                    robot.lift.setTargetHeight(mid);
                                    robot.intake.centerArm();
                                }
                            }
                            if (gamepad2.right_bumper && horizontalpos == "middle") {
                                robot.lift.setHorizontalPosition(horizontalextended);
                                horizontalpos = "extended";
                            } else if (gamepad2.right_bumper && horizontalpos == "extended") {
                                robot.lift.setHorizontalPosition(horizontalmiddle);
                                horizontalpos = "middle";
                            }
                            break;
                        case DROPPED:
                            dtspeed = 1;
                            canTurn = false;
                            horizontalpos = "back";
                            robot.intake.openClaw();
                            if (timer.milliseconds() > 450) {
                                double marker3 = timer.milliseconds()+300;
                                if (lowheight == false) {
                                    robot.lift.setTargetHeight(idle);
                                    robot.intake.liftArm();
                                    robot.turret.setTargetAngle(front);
                                    robotState = robotState.IDLE;
                                    timer.reset();
                                } else {
                                    robot.lift.setTargetHeight(idle);
                                    robot.intake.liftArm();
                                    if(timer.milliseconds()>marker3) {
                                        robot.turret.setTargetAngle(front);
                                        robotState = robotState.IDLE;
                                        timer.reset();
                                    }
                                }

                            }
                            break;

                    }
                    break;


                case STACK:
                    //Switching Modes
//                    switch(switchToAUTOCYCLE) {
//                        case Idle:
//                                if (gamepad1.x) {
//                                    switchToAUTOCYCLE = switchToAUTOCYCLE.FirstActions;
//                                    timer.reset();
//                                }
//                            break;
//                        case FirstActions:
//                            robot.lift.setTargetHeight(idle);
//                            robot.intake.liftArm();
//                            robot.intake.closeClaw();
//                            robot.lift.setHorizontalPosition(horizontallifted);
//                            switchToAUTOCYCLE = switchToAUTOCYCLE.SecondActions;
//                            timer.reset();
//                            break;
//                        case SecondActions:
//                            if (timer.milliseconds() > 700) {
//                                robot.turret.setTargetAngle(front);
//                                switchToAUTOCYCLE = switchToAUTOCYCLE.ThirdActions;
//                                timer.reset();
//                            }
//                            break;
//                        case ThirdActions:
//                            if (timer.milliseconds() > 300) {
//                                robot.lift.setTargetHeight(intaking);
//                                robot.intake.centerArm();
//                                robot.intake.fullyOpenClaw();
//                                robotMode = robotMode.AUTOCYCLE;
//                                switchToAUTOCYCLE = switchToAUTOCYCLE.Idle;
//                                timer.reset();
//                            }
//                            break;
//                    }

                    switch(switchNormal) {
                        case Idle:
                            if (gamepad1.y) {
                                switchNormal = switchNormal.FirstActions;
                                timer.reset();
                            }
                            break;
                        case FirstActions:
                            robot.lift.setTargetHeight(idle);
                            robot.intake.liftArm();
                            robot.intake.closeClaw();
                            robot.lift.setHorizontalPosition(horizontallifted);
                            switchNormal = switchNormal.SecondActions;
                            timer.reset();
                            break;
                        case SecondActions:
                            if (timer.milliseconds() > 700) {
                                robot.turret.setTargetAngle(front);
                                robotMode = robotMode.NORMAL;
                                robotState = robotState.IDLE;
                                switchNormal = switchNormal.Idle;
                                timer.reset();
                            }
                    }


                    //Horizontal Slides Extend/Retract
                    if (robotState == robotState.LIFTED && gamepad2.right_bumper && horizontalpos == "middle") {
                        robot.lift.setHorizontalPosition(horizontalextended);
                        horizontalpos = "extended";
                    } else if (robotState == robotState.LIFTED && gamepad2.right_bumper && horizontalpos == "extended") {
                        robot.lift.setHorizontalPosition(horizontalmiddle);
                        horizontalpos = "middle";
                    }

                    //Horizontal Slides Extend/Retract for Intaking
                    if (robotState == robotState.INTAKING && gamepad2.right_bumper) {
                        robot.lift.setTargetHeight(100);
                    }
                    if (robotState == robotState.INTAKING && gamepad2.left_bumper) {
                        robot.lift.setTargetHeight(60);
                    }


                    //Vertical Slides SlightDrop/SlightLift
                    if (robotState == robotState.LIFTED && gamepad2.left_bumper && liftedpos == "lifted") {
                        robot.lift.setTargetHeight(robot.lift.getCurrentHeight() - droppedvalue);
                        liftedpos = "dropped";
                    } else if (robotState == robotState.LIFTED && gamepad2.left_bumper && liftedpos == "dropped") {
                        robot.lift.setTargetHeight(robot.lift.getCurrentHeight() + droppedvalue);
                        liftedpos = "lifted";
                    }

                    //Turret Presets
                    if (canTurn = true) {
                        if (turrettimer.milliseconds() > 500) {
                            if (gamepad2.a) {
                                robot.turret.setTargetAngle(back);
                                turrettimer.reset();
                            }
                            if (gamepad2.x) {
                                robot.turret.setTargetAngle(right);
                                turrettimer.reset();
                            }
                            if (gamepad2.b && gamepad2.start == false) {
                                robot.turret.setTargetAngle(back-left);
                                turrettimer.reset();
                            }
                            if (gamepad2.y) {
                                robot.turret.setTargetAngle(front);
                                turrettimer.reset();
                            }
                            if (gamepad2.a && gamepad2.start == false) {
                                robot.turret.setTargetAngle(back);
                                turrettimer.reset();
                            }
                        }
                    }

                    //FSM
                    switch (robotState) {
                        case IDLE:
                            canTurn = true;
                            robot.intake.closeClaw();
                            robot.intake.liftArm();
                            robot.lift.setHorizontalPosition(horizontallifted);
                            robot.lift.setTargetHeight(idle);
                            if (timer.milliseconds() > 500) {
                                if (gamepad1.left_bumper) {
                                    robot.lift.setTargetHeight(intaking);
                                    timer.reset();
                                    robotState = robotState.INTAKING;
                                }
                            }
                            break;
                        case INTAKING:
                            canTurn = false;
                            robot.lift.setHorizontalPosition(horizontalback);
                            robot.intake.dropArm();
                            robot.intake.openClaw();
                            robot.intake.fullyOpenClaw();
                            robot.lift.setTargetHeight(30);
                            if (timer.milliseconds() > 500) {
                                if (gamepad1.left_bumper || robot.intake.getDistance() < 3) {
                                    robot.intake.closeClaw();
                                    robotState = robotState.GRABBED;
                                    timer.reset();
                                }
                            }
                            break;
                        case GRABBED:
                            canTurn = true;
                            robot.lift.setHorizontalPosition(horizontallifted);
                            if (timer.milliseconds() > 300) {
                                robot.intake.liftArm();
                                robot.lift.setTargetHeight(idle);
                            }
                            if (timer.milliseconds() > 500) {
                                if (gamepad1.left_bumper) {
                                    robot.lift.setTargetHeight(intaking);
                                    timer.reset();
                                    timer2.reset();
                                    robotState = robotState.INTAKING;
                                }
                                if (gamepad2.dpad_up) {
                                    robot.lift.setTargetHeight(up);
                                    robot.intake.setArmPos(highposarm);
                                    timer.reset();
                                    robotState = robotState.LIFTED;
                                    armup = true;
                                    lowheight = false;
                                }
                                if (gamepad2.dpad_right) {
                                    robot.lift.setTargetHeight(mid);
                                    robot.intake.centerArm();
                                    timer.reset();
                                    robotState = robotState.LIFTED;
                                    armup = true;
                                    lowheight = false;
                                }
                                if (gamepad2.dpad_left) {
                                    robot.lift.setTargetHeight(low);
                                    robot.intake.centerArm();
                                    timer.reset();
                                    robotState = robotState.LIFTED;
                                    armup = true;
                                    lowheight = false;
                                }
                                if (gamepad2.dpad_down) {
                                    if (Math.abs(robot.turret.getCurrentAngle() - back) < 5) {
                                        robot.lift.setTargetHeight(ground);
                                        robot.intake.dropArm();
                                        timer.reset();
                                        robotState = robotState.LIFTED;
                                        armup = false;
                                        lowheight = true;
                                    } else {
                                        robot.turret.setTargetAngle(back);
                                        if (Math.abs(robot.turret.getCurrentAngle() - back) < 5) {
                                            robot.lift.setTargetHeight(ground);
                                            robot.intake.dropArm();
                                            timer.reset();
                                            robotState = robotState.LIFTED;
                                            armup = false;
                                            lowheight = true;
                                        }
                                    }
                                }
                            }
                            break;
                        case LIFTED:
                            canTurn = true;
                            dtspeed = 0.4;
                            horizontalpos = "middle";
                            liftedpos = "lifted";
                            robot.lift.setHorizontalPosition(horizontalmiddle);
                            if (timer.milliseconds() > 750) {
                                if (gamepad2.dpad_down || gamepad1.dpad_down) {
                                    timer.reset();
                                    robot.turret.setTargetAngle(back);
                                    robot.lift.setTargetHeight(idle);
                                    robotState = robotState.GRABBED;
                                }
                                if (gamepad1.left_bumper) {
                                    robot.lift.setTargetHeight(robot.lift.getCurrentHeight() - droppedvalue);
                                    timer.reset();
                                    robotState = robotState.DROPPED;
                                }
                                if (gamepad2.dpad_up) {
                                    robot.lift.setTargetHeight(up);
                                    robot.intake.setArmPos(highposarm);
                                    timer.reset();
                                }
                                if (gamepad2.dpad_left) {
                                    robot.lift.setTargetHeight(low);
                                    robot.intake.centerArm();
                                }
                                if (gamepad2.dpad_right) {
                                    robot.lift.setTargetHeight(mid);
                                    robot.intake.centerArm();
                                }
                            }
                            break;
                        case DROPPED:
                            dtspeed = 1;
                            canTurn = false;
                            robot.intake.openClaw();
                            if (timer.milliseconds() > 450) {
                                double marker2 = timer.milliseconds() + 300;
                                if (lowheight == false) {
                                    robot.lift.setTargetHeight(idle);
                                    robot.intake.liftArm();
                                    robot.turret.setTargetAngle(back);
                                    robotState = robotState.IDLE;
                                    timer.reset();
                                } else {
                                    robot.lift.setTargetHeight(idle);
                                    robot.intake.liftArm();
                                    if (timer.milliseconds()>marker2) {
                                        robot.turret.setTargetAngle(back);
                                        robotState = robotState.IDLE;
                                        timer.reset();
                                    }
                                }

                            }
                            break;

                    }
                    break;


                case AUTOCYCLE:
//                    switch(switchNormal) {
//                        case Idle:
//                            if (gamepad1.y) {
//                                switchNormal = switchNormal.FirstActions;
//                                timer.reset();
//                            }
//                            break;
//                        case FirstActions:
//                            running = false;
//                            robot.turret.MAX_POWER=1;
//                            robot.lift.setTargetHeight(idle);
//                            robot.intake.liftArm();
//                            robot.intake.closeClaw();
//                            robot.lift.setHorizontalPosition(horizontallifted);
//                            switchNormal = switchNormal.SecondActions;
//                            timer.reset();
//                            break;
//                        case SecondActions:
//                            if (timer.milliseconds() > 700) {
//                                robot.turret.setTargetAngle(front);
//                                robotMode = robotMode.NORMAL;
//                                robotState = robotState.IDLE;
//                                switchNormal = switchNormal.Idle;
//                                timer.reset();
//                            }
//                    }
//                    switch(switchStack) {
//                        case Idle:
//                            if (gamepad1.a && gamepad1.start == false) {
//                                switchStack = switchStack.FirstActions;
//                                running=false;
//                                timer.reset();
//                            }
//                            break;
//                        case FirstActions:
//                            robot.turret.MAX_POWER=1;
//                            running = false;
//                            robot.lift.setTargetHeight(idle);
//                            robot.intake.liftArm();
//                            robot.intake.closeClaw();
//                            robot.lift.setHorizontalPosition(horizontallifted);
//                            switchStack = switchStack.SecondActions;
//                            timer.reset();
//                            break;
//                        case SecondActions:
//                            if (timer.milliseconds() > 700) {
//                                robot.turret.setTargetAngle(back);
//                                robotMode = robotMode.STACK;
//                                robotState = robotState.IDLE;
//                                switchStack = switchStack.Idle;
//                                timer.reset();
//                            }
//                    }
//                    if(gamepad1.left_bumper){
//                        running = true;
//                        autotimer.reset();
//                    }
//                    if(gamepad2.right_bumper){
//                        running = false;
//                    }
//                    if (running == true) {
//                        if(timer.milliseconds()<300) {
//                            robot.turret.MAX_POWER = 0.6;
//                            robot.intake.centerArm();
//                            robot.intake.fullyOpenClaw();
//                            robot.lift.setHorizontalPosition(horizontalextended);
//                        }else if (autotimer.milliseconds() > 300 && autotimer.milliseconds() < 400) {
//                            robot.intake.closeClaw();
//                        } else if (autotimer.milliseconds() > 700 && autotimer.milliseconds() < 1300) {
//                            robot.lift.setHorizontalPosition(horizontalback);
//                            robot.lift.setTargetHeight(up);
//                            robot.intake.setArmPos(0.6);
//                            robot.turret.setTargetAngle(back);
//                        } else if (autotimer.milliseconds() > 1600 && autotimer.milliseconds() < 1700) {
//                            robot.lift.setHorizontalPosition(0.6);
//                        } else if (autotimer.milliseconds() > 1900 && autotimer.milliseconds() < 2100) {
//                            robot.lift.setTargetHeight(robot.lift.getCurrentHeight() - droppedvalue);
//                        } else if (autotimer.milliseconds() > 2100 && autotimer.milliseconds() < 2800) {
//                            robot.intake.openClaw();
//                            robot.intake.centerArm();
//                            robot.lift.setTargetHeight(200);
//                            robot.turret.setTargetAngle(front);
//                        } else if (autotimer.milliseconds() > 2800) {
//                            autotimer.reset();
//                        }
//                    }
            }

            telemetry.addData("turret pos", robot.turret.getCurrentAngle());
            telemetry.addData("turret goal", robot.turret.getTargetAngle());
            telemetry.addData("State", robotState);
            telemetry.addData("Height", robot.lift.getCurrentHeight());
            telemetry.addData("TargetHeight", robot.lift.getTargetHeight());
            telemetry.addData("Distance", robot.intake.getDistance());
//                telemetry.addData("Orientation", robot.drive.getOrientation());
            telemetry.addData("timer", timer.milliseconds());
            telemetry.addData("turrettimer", turrettimer.milliseconds());
            telemetry.addData("dtspeed", dtspeed);
            telemetry.addData("slide power", robot.lift.motor2.getPower());
            telemetry.addData("armup?", armup);
            telemetry.addData("lowheight?", lowheight);
            telemetry.addData("arm timer", armtimer.milliseconds());
            telemetry.addData("mode", robotMode);
            telemetry.addData("liftedpos", liftedpos);
            telemetry.addData("horizontalpos", horizontalpos);
            telemetry.addData("horzontalservopos", robot.lift.horizontalServo1.getPosition());
            telemetry.addData("timer2", timer2);
            telemetry.addData("autotimer", autotimer.milliseconds());
            telemetry.addData("running?", running);
            robot.update();
        }
    }
}