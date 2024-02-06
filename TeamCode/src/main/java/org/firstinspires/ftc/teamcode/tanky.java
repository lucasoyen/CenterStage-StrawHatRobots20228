package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;

@TeleOp(
        group = "drive"
)
public class tanky extends LinearOpMode {
    private double speed = 1.0;
    private boolean speedcanbechanged = true;

    public void runOpMode() throws InterruptedException {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        drive.setMode(RunMode.RUN_USING_ENCODER);
        drive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        waitForStart();

        while(opModeIsActive()){
            drive.setWeightedDrivePower((new Pose2d(
                    (double)this.gamepad1.left_stick_y,
                    (double)this.gamepad1.left_stick_x,
                    (double)(this.gamepad1.right_bumper?1:0)-(this.gamepad1.left_bumper?1:0)))
                    .times(this.speed*0.8));
            if (this.gamepad1.x && this.speedcanbechanged) {
                this.speedcanbechanged = false;
                this.speed = 0.5 / this.speed;
            }
            if (!this.gamepad1.x) {
                this.speedcanbechanged = true;
            }
        }

    }
}
