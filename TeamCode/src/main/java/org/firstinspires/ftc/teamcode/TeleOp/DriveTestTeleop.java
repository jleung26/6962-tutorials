package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.ArmTutorial;
import org.firstinspires.ftc.teamcode.Subsystems.CustomMecanumDrive;

import java.util.List;

@TeleOp
public class DriveTestTeleop extends OpMode {
    private CustomMecanumDrive drive = new CustomMecanumDrive();
    private ElapsedTime elapsedtime;
    private List<LynxModule> allHubs;

    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();

    public void init() {
        drive.initialize(this);

        // loop time stuff, not necessary
//        elapsedtime = new ElapsedTime();
//        elapsedtime.reset();

        // bulk caching
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void start() {

    }

    public void loop() {
        // bulk caching, because I am making like 5 IMU reads per loop, and that will really kill loop times
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        // for rising edge detection
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // drive field-centric default
        // left trigger - slowmode
        // right trigger - auto rotate to target
        // B- reset
        // A - set target
        drive.operateTesting();

        // most simple, basic rising edge detection, no need for expressway or Pasteurized
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            CustomMecanumDrive.SCALING_EXPONENT += 0.4;
        } else if (currentGamepad1.left_bumper && !previousGamepad2.left_bumper) {
            CustomMecanumDrive.SCALING_EXPONENT -= 0.4;
        }

        // loop time measuring
//        telemetry.addData("Loop Times", elapsedtime.milliseconds());
//        elapsedtime.reset();
    }
}
