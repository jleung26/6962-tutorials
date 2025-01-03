package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake {
    OpMode opmode;
    public DcMotorEx intakeMotor;
    public Servo wrist;
    public ColorRangeSensor colorSensor;

    public enum IntakeChamberState {
        BLUE,
        RED,
        YELLOW,
        EMPTY,
        UNKNOWN
    }

    public enum IntakeState {
        INTAKING,
        REVERSE,
        NEUTRAL
    }

    public volatile boolean chamberFull = false;
    public volatile IntakeChamberState chamberColor = IntakeChamberState.EMPTY;
    public volatile IntakeState intakeState = IntakeState.NEUTRAL;


    // motor constants
    private double INTAKING_POWER = 1;
    private double REVERSE_POWER = -1;
    private double NEUTRAL_POWER = 0;

    // wrist constants
    private double DROP_DOWN_POS = 0;
    private double TRANSFER_POS = 0;
    private double WRIST_TUNING_INCREMENT = 0.001;

    // sensor constants
    public static double DETECTION_THRESHOLD = 0.75; // inches

    public void initialize(OpMode opmode) {
        this.opmode = opmode;
        intakeMotor = opmode.hardwareMap.get(DcMotorEx.class, "");
        wrist = opmode.hardwareMap.get(Servo.class, "");
        colorSensor = opmode.hardwareMap.get(ColorRangeSensor.class, "");

        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
//        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void operateTesting() {
        intakeMotor.setPower(-opmode.gamepad1.left_stick_y);

        if (opmode.gamepad1.left_trigger > 0.5) {
            incremental(-1);
        } else if (opmode.gamepad1.right_trigger > 0.5) {
            incremental(1);
        }

        if (opmode.gamepad1.a) {
            intake();
        } else if (opmode.gamepad1.b) {
            neutral();
        } else if (opmode.gamepad1.y) {
            reverse();
        }

        if (opmode.gamepad1.dpad_up) {
            flipUp();
        } else if (opmode.gamepad1.dpad_down) {
            dropDown();
        }

        chamberColor = getPieceColor();

        opmode.telemetry.addData("b", colorSensor.blue());
        opmode.telemetry.addLine(String.format("Red %s, Green %s, Blue %s, Opacity %s", colorSensor.red(), colorSensor.green(), colorSensor.blue(), colorSensor.alpha()));
        opmode.telemetry.addData("distance detected", colorSensor.getDistance(DistanceUnit.INCH));
        opmode.telemetry.addData("chamber color enum: ", chamberColor);
        opmode.telemetry.addData("intake state enum: ", intakeState);
    }

    public void operateTeleOp() {}

    // intake method
    private void setIntake(double power) {intakeMotor.setPower(power);}
    public void intake() {
        setIntake(INTAKING_POWER);
        intakeState = IntakeState.INTAKING;
    }
    public void reverse() {
        setIntake(REVERSE_POWER);
        intakeState = IntakeState.REVERSE;
    }
    public void neutral() {
        setIntake(NEUTRAL_POWER);
        intakeState = IntakeState.NEUTRAL;
    }

    // wrist methods
    public void incremental(int sign) {wrist.setPosition(wrist.getPosition() + sign * WRIST_TUNING_INCREMENT);}
    public void dropDown() { wrist.setPosition(DROP_DOWN_POS);}
    public void flipUp() { wrist.setPosition(TRANSFER_POS);}

    public IntakeChamberState getPieceColor() {
        if (colorSensor.getDistance(DistanceUnit.INCH) < DETECTION_THRESHOLD) {
            double blue = colorSensor.blue();
            double red = colorSensor.red();
            double green = colorSensor.green();

            // Color detection comparisons courtesy of chatgpt because I'm lazy
            // Calculate the maximum of the three raw values to normalize them
            double maxColor = Math.max(red, Math.max(green, blue));

            // Avoid division by zero by checking if maxColor is greater than a threshold
            if (maxColor > 0.1) {
                // Normalize RGB values by the maximum color value to account for varying brightness
                red /= maxColor;
                green /= maxColor;
                blue /= maxColor;
            }

            // Minimal comparisons based on normalized values
            if (blue > 0.5) {
                return IntakeChamberState.BLUE; // Blue is dominant
            } else if (red > 0.5 && green > 0.5) {
                return IntakeChamberState.YELLOW; // Red and green are dominant (Yellow)
            } else {
                return IntakeChamberState.RED; // If not blue or yellow, it's red
            }
            // safer code, but more comparisons and less optimal, if keeps returning RED above, try this instead
            // if (blue > red && blue > green) {
            //        chamberColor = IntakeChamberState.BLUE; // Blue dominates
            //    } else if (red > blue && green > blue) {
            //        chamberColor = IntakeChamberState.YELLOW; // Red and green dominate = Yellow
            //    } else if (red > blue && red > green) {
            //        chamberColor = IntakeChamberState.RED; // Red dominates
            //    } else {
            //        chamberColor = IntakeChamberState.UNKNOWN; // This case shouldn't happen if inputs are guaranteed
            //    }
        } else {
            return IntakeChamberState.EMPTY;
        }
    }
}
