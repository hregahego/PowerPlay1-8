package org.firstinspires.ftc.teamcode.opmodes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class OdoTest extends LinearOpMode{
    private Encoder leftEncoder, rightEncoder, frontEncoder;
//    DcMotorEx encoder1;
//    DcMotorEx encoder2;
    public void runOpMode(){
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightFront"));
//        encoder1 = hardwareMap.get(DcMotorEx.class, "leftFront");//new DcMotorEx(hardwareMap.get(DcMotorEx.class, "leftFront"));
//        encoder2 = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
//        encoder1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        encoder2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        encoder1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        encoder2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        while(!isStopRequested()){
            telemetry.addData("encoder1 value: ", leftEncoder.getCurrentPosition());
            telemetry.addData("encoder2 value: ", rightEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}