package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class ArmAlonebecausecodingstupidandbuildersnoputpeniscablestodriveandarmatsametime extends LinearOpMode{


    private DcMotor lift,reach;

    private CRServo intake;

    @Override
    public void runOpMode() throws InterruptedException {
        lift = hardwareMap.get(DcMotor.class,"lift");
        reach = hardwareMap.get(DcMotor.class,"reach");
        intake = hardwareMap.get(CRServo.class,"intake");
        lift.setZeroPowerBehavior(BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        reach.setZeroPowerBehavior(BRAKE);
        reach.setDirection(DcMotorSimple.Direction.REVERSE);
        //hardwareMap.get(DcMotor.class,"leftRear").setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()){

            intake.setPower(this.gamepad2.right_trigger);
            if(lift.getCurrentPosition()<-1600){
                lift.setPower(0.1+this.gamepad2.left_stick_y);
                reach.setPower(0.1+this.gamepad2.left_stick_x);
            }else{
                lift.setPower(this.gamepad2.left_stick_y);
                reach.setPower(this.gamepad2.left_stick_x);
            }
            telemetry.addData("lift",lift.getCurrentPosition());
            telemetry.addData("reach",reach.getCurrentPosition());
            telemetry.update();
        }

    }
}
