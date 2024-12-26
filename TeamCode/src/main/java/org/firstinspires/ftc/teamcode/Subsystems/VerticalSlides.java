package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
public class VerticalSlides {
    OpMode opmode;

//    private final RobotHardware rHardware = new RobotHardware();
    private PIDController controller;
    private DcMotorEx leftSlideMotor, rightSlideMotor;

    // constants
    public static double Kp = 0.013;
    public static double Ki = 0;
    public static double Kd = 0.0003;
    public static double Kg = 0.1;
    public static double CACHING_THRESHOLD = 0.005;
    public static double RETRACTED_THRESHOLD = 10;
    public static int UPPER_LIMIT = 1600; // this is for 1150s
    public static int LOWER_LIMIT = -2;

    // encoder positions
    public static int highBucketPos = 1300;
    public static int retractedPos = 0;
    public static int pickupClipPos = 0;
    public static int prepClipPos = 555;
    public static int slamClipPos = 220;

    // declaring variables for later modification
    private volatile double target = 0;
    private volatile double slidePower;
    private volatile double output = 0;
    private volatile double previousOutput = 0;
    public volatile boolean slidesRetracted = true;

    public VerticalSlides() {}

    public void teleInitialize(OpMode opmode) {
        this.opmode = opmode;

        leftSlideMotor = opmode.hardwareMap.get(DcMotorEx.class, "");
        rightSlideMotor = opmode.hardwareMap.get(DcMotorEx.class, "");

        controller = new PIDController(Kp, Ki, Kd);

        leftSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

//        leftSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void autoInitialize(OpMode opmode) {
        this.opmode = opmode;

        leftSlideMotor = opmode.hardwareMap.get(DcMotorEx.class, "");
        rightSlideMotor = opmode.hardwareMap.get(DcMotorEx.class, "");

        controller = new PIDController(Kp, Ki, Kd);
        leftSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

//        leftSlideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        leftSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void operateTeleOp() {
        int currentPos = rightSlideMotor.getCurrentPosition();
        slidesRetracted = currentPos < RETRACTED_THRESHOLD;
        output = controller.calculate(currentPos, target) + (slidesRetracted ? 0: Kg);
        output = (Math.abs(output) > 1 ? (output > 0 ? 1 : -1) : output);

        // includes power caching
        if (isDifferent(output, previousOutput)) {
            leftSlideMotor.setPower(output);
            rightSlideMotor.setPower(output);
        }
        previousOutput = output;
    }

    public void operateTuning() {
        if (opmode.gamepad2.y) {
            target = highBucketPos;
        } else if (opmode.gamepad2.a) {
            target = retractedPos;
        }

        int currentPos = rightSlideMotor.getCurrentPosition();

        slidePower = -opmode.gamepad2.left_stick_y;
        if (Math.abs(slidePower) > 0.05) {
            // move freely
            leftSlideMotor.setPower(slidePower);
            rightSlideMotor.setPower(slidePower);
            target = currentPos;

            // if out of range, sets target to back in range
            if (currentPos > UPPER_LIMIT) {
                target = UPPER_LIMIT;
            } else if (currentPos < LOWER_LIMIT) {
                target = LOWER_LIMIT;
            }
        } else {
            // else use PID
            output = controller.calculate(currentPos, target) + Kg;
            leftSlideMotor.setPower(output);
            rightSlideMotor.setPower(output);
        }

        // updates boolean
        slidesRetracted = currentPos < RETRACTED_THRESHOLD;
    }

    public void operateFix() {
        // manual control
        slidePower = -opmode.gamepad2.left_stick_y;

        if (Math.abs(slidePower) > 0.05)
        {
            leftSlideMotor.setPower(slidePower);
            rightSlideMotor.setPower(slidePower);
        }

        if (opmode.gamepad2.left_stick_button) {
            leftSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rightSlideMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

    }

    public void moveToPosition(int targetPos) {target = targetPos;}
    public void raiseToHighBucket() { moveToPosition(highBucketPos);}
    public void raiseToPickupClip() { moveToPosition(pickupClipPos);}
    public void raiseToPrepClip()   { moveToPosition(prepClipPos);}
    public void retract()           { moveToPosition(retractedPos);}
    public void slamToScoreClip()   { moveToPosition(slamClipPos);}

    public double telemetryMotorPos() {
        return rightSlideMotor.getCurrentPosition();
    }
    public double telemetryTarget() {
        return target;
    }
    public double telemetryOutput() {
        return output;
    }

    private boolean isDifferent(double val1, double val2) {
        return Math.abs(val1 - val2) >= CACHING_THRESHOLD;
    }
}

