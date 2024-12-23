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
        // IMU orientation
        // https://ftc-docs.firstinspires.org/en/latest/programming_resources/imu/imu.html?highlight=imu#physical-hub-mounting
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        private RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        double globalHeading;
        double relativeHeading;
        double offset;

        public void initialize(OpMode opmode) {
                imu = opmode.hardwareMap.get(IMU.class, "imu");
                imu.initialize(new IMU.Parameters(orientation));
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
