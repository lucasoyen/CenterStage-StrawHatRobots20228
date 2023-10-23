package org.firstinspires.ftc.teamcode.util;



import android.content.Context;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.ColorSensor;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardwareMaps.MecanumDrive;
import com.qualcomm.robotcore.util.ElapsedTime;



@Config
@TeleOp(group = "drive")

public class Drive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    boolean aPressable = true;


    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;
    @Override

    public void runOpMode() {

        boolean y2Pressable = true;

        String color = "NONE";

        double speed = 0.8;

        frontLeft = hardwareMap.dcMotor.get("leftFront"); //port 3
        frontRight = hardwareMap.dcMotor.get("rightFront"); //port 2
        backLeft = hardwareMap.dcMotor.get("leftRear"); //port 1
        backRight = hardwareMap.dcMotor.get("rightRear");  //port 0


        boolean drivingReversed = false;


        int position = 0;
        boolean holding = false;

        frontLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);

        backLeft.setDirection(DcMotor.Direction.FORWARD);

        backRight.setDirection(DcMotor.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MecanumDrive driveController = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        waitForStart();

        runtime.reset();


        while (opModeIsActive()) {


            //drive
            double stickLx = this.gamepad1.left_stick_x;
            double stickLy = this.gamepad1.left_stick_y;
            double stickRx = this.gamepad1.right_stick_x;
            boolean rb = this.gamepad1.right_bumper;
            boolean lb = this.gamepad1.left_bumper;
            boolean a = this.gamepad1.a;

            speed = 0.8;
            if (rb) {
                speed = 0.3;
            } else if (lb) {
                speed = 0.15;
            }
            if (drivingReversed) {
                driveController.moveInTeleop(-stickLx, -stickLy, stickRx, speed);
            } else {
                driveController.moveInTeleop(stickLx, stickLy, stickRx, speed);
            }

            //drive

            //arm

            double stickLy2 = this.gamepad2.left_stick_y;
            boolean a2 = this.gamepad2.a;
            boolean b2 = this.gamepad2.b;
            boolean y2 = this.gamepad2.y;
            boolean rb2 = this.gamepad2.right_bumper;
            boolean lb2 = this.gamepad2.left_bumper;
            boolean x2 = this.gamepad2.x;
            double rt2 = this.gamepad2.right_trigger;
            double lt2 = this.gamepad2.left_trigger;
            boolean r2 = this.gamepad2.dpad_right;
            boolean l2 = this.gamepad2.dpad_left;
            boolean u2 = this.gamepad2.dpad_up;
            boolean d2 = this.gamepad2.dpad_down;

            if (a && aPressable) {
                drivingReversed = !drivingReversed;
            }
            aPressable = !a;
            telemetry.update();



        }
    }

}
