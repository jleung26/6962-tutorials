package org.firstinspires.ftc.teamcode.Subsystems;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class GyroManager {
public IMU imu;

        double globalHeading;
        double relativeHeading;
        double offset;

        public void initialize(OpMode opmode) {
                imu = opmode.hardwareMap.get(IMU.class, "imu");
                // Initialize IMU directly
                imu.initialize(
                        new IMU.Parameters(
                                new RevHubOrientationOnRobot(
                                        // TODO: hub directions
                                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                        RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                                )
                        )
                );

        }

        public double getHeading()
        {
                return getAbsoluteHeading() - offset;
        }

        public double getAbsoluteHeading() {
                return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }

        public void relativeReset() {
                offset += getHeading();
        }
}
