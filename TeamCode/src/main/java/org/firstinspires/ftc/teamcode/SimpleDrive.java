package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import java.util.Vector;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getAccelerationConstraint;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(group = "drive")
public class SimpleDrive extends LinearOpMode{

    private Servo rwrist;
    private Servo lwrist;
    private Servo door;
    private DcMotor michael1;
    private DcMotor michael2;
    private DcMotor intake;

    public float ypower;
    public float xpower;
    public float rpower;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setZeroPowerBehavior(BRAKE);
        intake = hardwareMap.get(DcMotor.class,"intake");
        hardwareMap.get(DcMotor.class,"rightFront").setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareMap.get(DcMotor.class,"rightRear").setDirection(DcMotorSimple.Direction.REVERSE);
        rwrist = hardwareMap.get(Servo.class,"rwrist");
        rwrist.setDirection(Servo.Direction.REVERSE);
        lwrist = hardwareMap.get(Servo.class,"lwrist");
        door = hardwareMap.get(Servo.class,"doorServo");


        michael1 = hardwareMap.get(DcMotor.class,"arm1");
        michael1.setDirection(DcMotorSimple.Direction.REVERSE);
        michael2 = hardwareMap.get(DcMotor.class,"arm2");
        michael1.setZeroPowerBehavior(BRAKE);
        michael2.setZeroPowerBehavior(BRAKE);

        waitForStart();
        rwrist.setPosition(0);
        lwrist.setPosition(0);
        door.setPosition(0);
        //lwrist.setPosition(rwrist.getPosition());
        while (opModeIsActive()){
            boolean rb = this.gamepad1.right_bumper;
            michael1.setPower(this.gamepad2.left_stick_y==0?0.1:-gamepad2.left_stick_y);
            michael2.setPower(this.gamepad2.left_stick_y==0?0.1:-gamepad2.left_stick_y);

            /*if(rwrist.getPosition()<0.3322){
                rwrist.setPosition(rwrist.getPosition()+this.gamepad2.right_stick_y/50);
            }else{
                rwrist.setPosition(0.32);
            }
            if(lwrist.getPosition()<0.3322){
                lwrist.setPosition(lwrist.getPosition()+this.gamepad2.right_stick_y/50);
            }else{
                lwrist.setPosition(0.32);
            }

            if(michael1.getCurrentPosition()<1 || michael2.getCurrentPosition()<1){
                michael1.setPower(0.5);
                michael2.setPower(0.5);
            }
            if(michael1.getCurrentPosition()>3200||michael2.getCurrentPosition()>3200){
                michael1.setPower(-0.5);
                michael2.setPower(-0.5);
            }*/
            if(gamepad2.a){
                door.setPosition(0.6);
            }
            if(gamepad2.b){
                door.setPosition(0);
            }
            if(gamepad2.x){
                rwrist.setPosition(0.35444444);
                lwrist.setPosition(0.34166666);
            }
            intake.setPower(gamepad2.left_trigger*(gamepad2.left_bumper?-1:1));
            rwrist.setPosition(rwrist.getPosition()+this.gamepad2.right_stick_y/75);
            lwrist.setPosition(lwrist.getPosition()+this.gamepad2.right_stick_y/75);

            if(michael1.getCurrentPosition()<150){
                rwrist.setPosition(0.0983333333);
                lwrist.setPosition(0.7722222222);
            }else if(michael1.getCurrentPosition()<280){
                rwrist.setPosition(0.06999999999);
                lwrist.setPosition(0.06944444444);
            }

            float staticspeed = 1;

            float speed = staticspeed*(rb ? .5f : 1f);

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            gamepad1.right_stick_x
                    ).times(speed)
            );



            telemetry.addLine("Right Wrist: " + rwrist.getPosition());
            telemetry.addLine("Left Wrist: " + lwrist.getPosition());
            telemetry.addLine("Arm 1: " + michael1.getCurrentPosition());
            telemetry.addLine("Arm 2: " + michael1.getCurrentPosition());
            telemetry.update();



        }

    }
}
