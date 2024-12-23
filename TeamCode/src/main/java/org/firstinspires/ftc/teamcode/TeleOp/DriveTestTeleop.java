package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmTutorial;
import org.firstinspires.ftc.teamcode.Subsystems.CustomMecanumDrive;

@TeleOp
public class DriveTestTeleop extends OpMode {
    private CustomMecanumDrive drive = new CustomMecanumDrive();

    public void init() {
        drive.initialize(this);
    }

    public void start() {

    }

    public void loop() {
        // drive field-centric default
        // slowmode when hold right bumper
        // auto rotate to target
        drive.operate(this);
    }
}
