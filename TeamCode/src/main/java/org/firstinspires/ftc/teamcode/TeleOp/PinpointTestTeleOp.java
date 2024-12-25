package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.PinpointManager;

public class PinpointTestTeleOp extends OpMode {
    PinpointManager pinpoint = new PinpointManager();

    @Override
    public void init() {
        pinpoint.initialize(this);
    }

    @Override
    public void loop() {
        pinpoint.operateTesting();
    }
}
