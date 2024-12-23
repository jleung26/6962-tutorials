package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmTutorial {
    OpMode opmode;

    public Claw claw = new Claw();
    public Arm arm = new Arm();
    public Wrist wrist = new Wrist();

    public ArmTutorial() {}

    public void initialize(OpMode opmode) {
        this.opmode = opmode;
        claw.initialize();
        arm.initialize();
        wrist.initialize();
    }

    public void operate(OpMode opmode) {
        if (opmode.gamepad1.left_bumper) {
            claw.incremental(-1);
        } else if (opmode.gamepad1.right_bumper) {
            claw.incremental(1);
        }

        claw.incremental(getSign(opmode.gamepad1.left_stick_x)); // [-1,1] -> -1,0,1 -> smooth movement

        opmode.telemetry.addData("claw pos: ", claw.clawServo.getPosition());
    }

    public class Claw {
        Servo clawServo;

        double openPosition = 0.5;
        double closedPosition = 0;
        double increment = 0.001;

        public void initialize() {
            clawServo = opmode.hardwareMap.get(Servo.class, "claw");
        }
        public void incremental(int sign) {
            clawServo.setPosition(clawServo.getPosition() + sign * increment);
        }

        public void openClaw() {
            clawServo.setPosition(openPosition);
        }
        public void closeClaw() {
            clawServo.setPosition(closedPosition);
        }
    }

    public class Arm {
        Servo armServo;
        public ArmStates armCurrentState = ArmStates.TRANSFER;

        double grabOffWallPosition = 0;
        double transferPosition = 0;
        double scoringBucketPosition = 0;
        double scoringClipPosition = 0;

        public void initialize() {
            armServo = opmode.hardwareMap.get(Servo.class, "arm");
        }

        public void toTransfer() {
            armServo.setPosition(transferPosition);
            armCurrentState = ArmStates.TRANSFER;
        }

        public void toScoringBucket() {
            armServo.setPosition(scoringBucketPosition);
            armCurrentState = ArmStates.SCORING_BUCKET;
        }

        public void toScoringClipt() {
            armServo.setPosition(scoringClipPosition);
            armCurrentState = ArmStates.SCORING_CLIP;
        }

        public void toGrabWall() {
            armServo.setPosition(grabOffWallPosition);
            armCurrentState = ArmStates.GRAB_OFF_WALL;
        }
    }

    public class Wrist {
        Servo wristFlipServo, wristRotateServo;

        double rotateIntakePosition = 0;
        double rotateStowPosition = 0;
        double rotateTransferPosition = 0;

        double flipIntakePosition = 0;
        double flipStowPosition = 0;
        double flipTransferPosition = 0;

        public void initialize() {
            wristFlipServo = opmode.hardwareMap.get(Servo.class, "wristFlip");
            wristRotateServo = opmode.hardwareMap.get(Servo.class, "wristRotate");
        }
    }

    public enum ArmStates {
        TRANSFER,
        SCORING_BUCKET,
        SCORING_CLIP,
        GRAB_OFF_WALL
    }

    public int getSign(double input) {
        return input > 0 ? 1: input == 0 ? 0: -1;
    }
}
