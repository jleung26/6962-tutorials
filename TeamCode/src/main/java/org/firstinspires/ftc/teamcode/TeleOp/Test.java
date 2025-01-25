package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp
public class Test extends OpMode {
    public RevColorSensorV3 colorSensor;

    public enum IntakeChamberState {
        BLUE,
        RED,
        YELLOW,
        EMPTY,
        UNKNOWN
    }

    public static double DETECTION_THRESHOLD = 0.75; // inches
    public static double RGB_THRESHOLD = 600;

    public volatile IntakeChamberState chamberState = IntakeChamberState.EMPTY;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorsensor");

        colorSensor.enableLed(true);
    }

    @Override
    public void loop() {
        chamberState = getPieceColor();
        telemetry.addLine(String.format(Locale.US, "Red %d, \nGreen %d, \nBlue %d, \nOpacity %d", colorSensor.red(), colorSensor.green(), colorSensor.blue(), colorSensor.alpha()));
        telemetry.addData("distance detected", colorSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("chamber color enum: ", chamberState);
    }

    public IntakeChamberState getPieceColor() {
        if (colorSensor.getDistance(DistanceUnit.INCH) < DETECTION_THRESHOLD) {
            double blue = colorSensor.blue();
            double green = colorSensor.green();

            // Minimal comparisons, breaks if the reflected light is too saturated and nearly white
            if (blue > RGB_THRESHOLD) {
                return IntakeChamberState.BLUE; // Blue is dominant -> blue
            } else if (green > RGB_THRESHOLD) {
                return IntakeChamberState.YELLOW; // Green dominant, blue not dominant -> (Yellow)
            } else {
                return IntakeChamberState.RED; // Not blue or yellow -> red
            }
        } else {
            return IntakeChamberState.EMPTY;
        }
    }
}
