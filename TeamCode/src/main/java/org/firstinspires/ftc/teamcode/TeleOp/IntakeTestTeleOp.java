package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;

@TeleOp
public class IntakeTestTeleOp extends OpMode {
    Intake intake = new Intake();
    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();


    @Override
    public void init() {
        intake.initialize(this);
    }

    @Override
    public void loop() {

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

        intake.operateTesting();

        // rising edge detection for extremely fine wrist adjustment
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper){
            intake.incremental(-1);
        }
        else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            intake.incremental(1);
        }

    }
}
