package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CustomMecanumDrive {
    private DcMotorEx Fl, Fr, Bl, Br;
    private volatile double prevFrontLeftPower, prevBackLeftPower, prevFrontRightPower, prevBackRightPower;
    private GyroManager gyro = new GyroManager();

    public static double SLOW_MODE_FACTOR = 0.5;
    public static double CACHING_THRESHOLD = 0.005;

    public static double Kp = 0.01;
    public static double Kd = 0;

    // variables for later modification
    private double targetAngle = 0;
    // PID
    private double error, lastError;
    ElapsedTime timer = new ElapsedTime();

    public CustomMecanumDrive() {}

    public void initialize(OpMode opmode) {
        this.Fl = opmode.hardwareMap.get(DcMotorEx.class, "");
        this.Fr = opmode.hardwareMap.get(DcMotorEx.class, "");
        this.Bl = opmode.hardwareMap.get(DcMotorEx.class, "");
        this.Br = opmode.hardwareMap.get(DcMotorEx.class, "");

        Fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Fl.setDirection(DcMotorSimple.Direction.REVERSE);
        Bl.setDirection(DcMotorSimple.Direction.REVERSE);

        gyro.initialize(opmode);
    }

    public void operate(OpMode opmode) {

        // auto rotate to angle with PID test
        if (opmode.gamepad1.right_trigger > 0.1) {
            driveFieldCentric(opmode.gamepad1.left_stick_x, opmode.gamepad1.left_stick_y, PDTurning(gyro.getHeading(), targetAngle), gyro.getHeading());
        }
        // slow mode
        else if (opmode.gamepad1.right_bumper) {
            driveFieldCentric(opmode.gamepad1.left_stick_x * SLOW_MODE_FACTOR, -opmode.gamepad1.left_stick_y * SLOW_MODE_FACTOR, opmode.gamepad1.right_stick_x * SLOW_MODE_FACTOR, gyro.getHeading());
        }
        // normal drive
        else {
            driveFieldCentric(opmode.gamepad1.left_stick_x, -opmode.gamepad1.left_stick_y, opmode.gamepad1.right_stick_x, gyro.getHeading());
        }

        // gyro reset
        if (opmode.gamepad1.b) {gyro.relativeReset();}

        // sets target
        if (opmode.gamepad1.a) {targetAngle = gyro.getHeading();}

        opmode.telemetry.addData("current heading", gyro.getHeading());
        opmode.telemetry.addData("absolute heading", gyro.getAbsoluteHeading());
        opmode.telemetry.addData("target heading", targetAngle);
        opmode.telemetry.addData("gyro offset", gyro.offset);
        opmode.telemetry.addData("PID calculated rx", PDTurning(gyro.getHeading(), targetAngle));
        opmode.telemetry.addData("normalized error", normalizeError(targetAngle - gyro.getHeading()));
    }

    public void driveFieldCentric(double x, double y, double rx, double heading) {

        // calculating output
        double headingRads = -Math.toRadians(heading);

        double rotX = y * Math.cos(headingRads) + x * Math.sin(headingRads);

        double rotY = y * Math.sin(headingRads) - x * Math.cos(headingRads);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        // power caching
        if (comparePower(prevFrontLeftPower, frontLeftPower) || comparePower(prevBackLeftPower, backLeftPower) ||
            comparePower(prevFrontRightPower, frontRightPower) || comparePower(prevBackRightPower, backRightPower)) {
            // writing/assiging outputs
            Fl.setPower(frontLeftPower);
            Bl.setPower(backLeftPower);
            Fr.setPower(frontRightPower);
            Br.setPower(backRightPower);
        }
        // assigns for next loop
        prevFrontLeftPower = frontLeftPower;
        prevBackLeftPower = backLeftPower;
        prevFrontRightPower = frontRightPower;
        prevBackRightPower = backRightPower;
    }

    // checks if powers are different enough
    public boolean comparePower(double prevPower, double currentPower) {
        return Math.abs(currentPower - prevPower) >= CACHING_THRESHOLD;
    }

    public double PDTurning(double targetHeading, double currentHeading) {
        // calculate the error
        error = normalizeError(targetHeading - currentHeading);

        double derivative = (error - lastError) / timer.seconds();

        double output = Math.max(-1, Math.min(1, (Kp * error) + (Kd * derivative)));
        // square root PID, if robot is heavy
        // double output = Math.signum(output) * Math.min(1, Math.sqrt(Math.abs(output)));

        // reset stuff for next time
        timer.reset();
        lastError = error;

        return output;
    }

    private double normalizeError(double error) {
        // Normalize error to -180 to 180
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        return error;
    }
}
