package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ArmTutorial;

@TeleOp
public class ArmTestTeleOp extends OpMode {
    private ArmTutorial intakeArm = new ArmTutorial();

    public void init() {
        intakeArm.initialize(this);
    }

    public void start() {

    }

    public void loop() {
        intakeArm.operate(this);
        if (gamepad1.a) {
            intakeArm.claw.closeClaw();
        }

        if (gamepad2.a && intakeArm.arm.armCurrentState == ArmTutorial.ArmStates.TRANSFER) {
            intakeArm.claw.closeClaw();
        }
    }
}
