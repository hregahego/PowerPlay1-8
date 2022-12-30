package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class OdoTest extends LinearOpMode{
    private Encoder leftEncoder, rightEncoder, frontEncoder;
    public void runOpMode(){
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightRear"));
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
//        leftEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        rightEncoder.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        leftEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        rightEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        while(!isStopRequested()){
            telemetry.addData("encoder left value: ", leftEncoder.getCurrentPosition());
            telemetry.addData("encoder right value: ", rightEncoder.getCurrentPosition());
            telemetry.addData("encoder front value: ", frontEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}