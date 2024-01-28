package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import java.util.Vector;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import androidx.annotation.NonNull;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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

    private boolean reversible;
    private int reversed;
    private DcMotor lift, reach, pullup1,pullup2;
    private CRServo intake, pullupr1,pullupr2;
    public float ypower;
    public float xpower;
    public float rpower;

    @Override
    public void runOpMode() throws InterruptedException {
        reversed = 1;
        reversible = true;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setZeroPowerBehavior(BRAKE);
        lift = hardwareMap.get(DcMotor.class,"lift");
        reach = hardwareMap.get(DcMotor.class,"reach");
        intake = hardwareMap.get(CRServo.class,"intake");
        lift.setZeroPowerBehavior(BRAKE);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        reach.setZeroPowerBehavior(BRAKE);
        reach.setDirection(DcMotorSimple.Direction.REVERSE);
        pullupr1 = hardwareMap.get(CRServo.class,"pullupr1");
        pullupr2 = hardwareMap.get(CRServo.class,"pullupr2");
        pullup1 = hardwareMap.get(DcMotor.class,"pullup1");
        pullup2 = hardwareMap.get(DcMotor.class,"pullup2");
        pullupr2.setDirection(DcMotorSimple.Direction.REVERSE);
        pullup2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){



            if(gamepad1.left_bumper && reversible){
                reversed*=-1;
                reversible=false;
            }
            if(!gamepad1.left_bumper && !reversible){
                reversible=true;
            }

            intake.setPower(this.gamepad2.right_trigger);
            lift.setPower(lift.getCurrentPosition()<-1600?0.1:0+this.gamepad2.left_stick_y);
            reach.setPower(lift.getCurrentPosition()<-1600?0.1:0+this.gamepad2.left_stick_x);

            pullupr1.setPower(gamepad2.right_stick_y);
            pullupr2.setPower(gamepad2.right_stick_y);
            pullup1.setPower(gamepad2.right_stick_x);
            pullup2.setPower(gamepad2.right_stick_x);

            float staticspeed = 0.8f;

            float speed = staticspeed*(this.gamepad1.right_bumper ? .5f : 1f)*reversed;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y,
                            gamepad1.left_stick_x,
                            gamepad1.right_stick_x
                    ).times(speed)
            );

            telemetry.update();



        }

    }
}
