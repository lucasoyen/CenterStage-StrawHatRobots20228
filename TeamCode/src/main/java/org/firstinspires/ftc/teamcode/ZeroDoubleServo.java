package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ZeroDoubleServo", group = "drive")
public class ZeroDoubleServo extends LinearOpMode{
    private List<Servo> servos;

    @Override
    public void runOpMode() throws InterruptedException {
        servos = hardwareMap.getAll(Servo.class);

        waitForStart();
        while(opModeIsActive()){
            for (int i=0;i<servos.size();i++) {
                servos.get(i).setPosition(0+gamepad1.right_trigger-gamepad1.left_trigger);
            }
        }
    }
}
