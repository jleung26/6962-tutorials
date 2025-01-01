package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class HorizontalSlides {
    private OpMode opmode;
    public DcMotorEx slideMotor;
    private PIDController controller;

    // TODO: tune
    public static double MAPPING_EXPONENT = 0.4;
    public static double UPPER_LIMIT = 10;
    public static double LOWER_LIMIT = -2;
    public static double RETRACTED_THRESHOLD = 10;

    // PID constants
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;

    // variables for later modification
    private volatile double output;
    private volatile double target = 0;
    private volatile double slidePower;
    public volatile boolean slidesRetracted = true;

    public HorizontalSlides() {}

    public void teleInitialize(OpMode opmode) {
        this.opmode = opmode;
        slideMotor = opmode.hardwareMap.get(DcMotorEx.class, "");

        controller = new PIDController(Kp, Ki, Kd);

        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void autoInitialize(OpMode opmode) {
        this.opmode = opmode;
        slideMotor = opmode.hardwareMap.get(DcMotorEx.class, "");

        controller = new PIDController(Kp, Ki, Kd);

        slideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        slideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void operateTeleOp() {
        int currentPos = slideMotor.getCurrentPosition();
        target = mapTriggerToTarget(opmode.gamepad1.right_trigger);
        output = controller.calculate(currentPos, target);

        slideMotor.setPower(output);

        // updates boolean
        slidesRetracted = currentPos < RETRACTED_THRESHOLD;
    }

    public void operateTuningPID() {
        controller.setPID(Kp, Ki, Kd);
        int currentPos = slideMotor.getCurrentPosition();
        target = mapTriggerToTarget(opmode.gamepad1.right_trigger);
        output = controller.calculate(currentPos, target);
        slideMotor.setPower(output);
    }

    public void operateTuningPositions() {
        slideMotor.setPower(-opmode.gamepad1.left_stick_y);
    }

    public double mapTriggerToTarget(double input) {
        return Math.round(Math.pow(input, MAPPING_EXPONENT) * UPPER_LIMIT);
        // paste this into desmos to see graph: x^{0.4}\ \left\{0\le x\le1\right\}
        // making the mapping exponent smaller makes the graph steeper
    }

    public double telemetryMotorPos() {
        return slideMotor.getCurrentPosition();
    }
    public double telemetryTarget() {
        return target;
    }
    public double telemetryOutput() {
        return output;
    }
}
