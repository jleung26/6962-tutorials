package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.HorizontalSlides;

public class HorizontalSlidesTestTeleOp extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dash.getTelemetry();
    HorizontalSlides horizontalSlides = new HorizontalSlides();

    final Gamepad currentGamepad1 = new Gamepad();
    final Gamepad currentGamepad2 = new Gamepad();
    final Gamepad previousGamepad1 = new Gamepad();
    final Gamepad previousGamepad2 = new Gamepad();

    private boolean tuningPID = false;

    @Override
    public void init() {
        horizontalSlides.autoInitialize(this);
    }

    @Override
    public void loop() {
        // for rising edge detection
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);

        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);
        if (currentGamepad1.a && !previousGamepad1.a) {
            tuningPID = !tuningPID;
        }

        if (tuningPID) {
            horizontalSlides.operateTuningPID();
        }
        else {
            horizontalSlides.operateTuningPositions();
        }

        dashboardTelemetry.addLine(String.format("Tuning Mode %s", tuningPID ? "PID tuning (trigger)" : "Position tuning (joystick)"));
        dashboardTelemetry.addData("Motor Encoder Pos: ", horizontalSlides.telemetryMotorPos());
        dashboardTelemetry.addData("Target: ", horizontalSlides.telemetryTarget());
        dashboardTelemetry.addData("PID Power: ", horizontalSlides.telemetryOutput());
        dashboardTelemetry.update();
    }
}
