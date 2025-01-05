package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

import java.util.List;

@TeleOp
public class IntakeTestTeleOp extends OpMode {
    Intake intake = new Intake();
    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();

    private List<LynxModule> allHubs;

    private ElapsedTime elapsedtime;

    boolean testingManualMode = true;
    boolean rejectBlue = true;

    @Override
    public void init() {
        intake.initialize(this);
        elapsedtime = new ElapsedTime();
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }
    }

    @Override
    public void start() {
        elapsedtime.reset();
    }

    @Override
    public void loop() {
        // clearing bulk cache so we don't have egregious loop times from all the i2c calls I'm making
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
        // do this first
        // left_stick_y - manual intake motor control
        // right & left trigger - wrist incremental

        // next
        // stick stuff into chamber for color
        // tune detection threshold

        // write down, push, test
        // A intake, B neutral, Y reverse
        // dpad up and down- flip wrist up and down

        // boolean toggle
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            testingManualMode = !testingManualMode;
        }

        // another boolean toggle
        if (currentGamepad1.x && !previousGamepad1.x) {
            rejectBlue = !rejectBlue;
        }

        if (testingManualMode) {
            intake.operateTesting();
        }else {
            intake.operateColorChecking(rejectBlue);
        }

        // rising edge detection for extremely fine wrist adjustment
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
            intake.incremental(-1);
        }
        else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            intake.incremental(1);
        }

        telemetry.addData("loop time", elapsedtime.milliseconds());
        elapsedtime.reset();
    }
}
