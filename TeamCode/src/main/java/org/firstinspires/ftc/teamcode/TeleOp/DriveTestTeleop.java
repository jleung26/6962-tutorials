package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.ArmTutorial;
import org.firstinspires.ftc.teamcode.Subsystems.CustomMecanumDrive;

import java.util.List;

@TeleOp
public class DriveTestTeleop extends OpMode {
    private CustomMecanumDrive drive = new CustomMecanumDrive();
    private ElapsedTime elapsedtime;
    private List<LynxModule> allHubs;

    public void init() {
        drive.initialize(this);
        elapsedtime = new ElapsedTime();
        elapsedtime.reset();
        allHubs = hardwareMap.getAll(LynxModule.class);
        // bulk caching
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    public void start() {

    }

    public void loop() {
        // bulk caching, because I am making like 5 IMU reads per loop, and that will really kill auto caching loop times
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        // drive field-centric default
        // slowmode when hold right bumper
        // auto rotate to target
        drive.operate(this);

        // loop time measuring
        telemetry.addData("Loop Times", elapsedtime.milliseconds());
        elapsedtime.reset();
    }
}
