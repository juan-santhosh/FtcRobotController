package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class AdaptedAccelerationCurves extends LinearOpMode {
    final double MAX_ACCELERATION_DIFFERENCE = 0.7, MAX_VELOCITY_DIFFERENCE = 1400;

    private double stabilizeAcceleration(double newAcceleration, double currentAcceleration) {
        double deltaAcceleration = newAcceleration - currentAcceleration;
        return Math.abs(deltaAcceleration) > 0.1 ? currentAcceleration + MAX_ACCELERATION_DIFFERENCE * deltaAcceleration / Math.abs(deltaAcceleration) : newAcceleration;
    }

    private double stabilizeVelocity(double newVelocity, double currentVelocity) {
        double deltaVelocity = newVelocity - currentVelocity;
        return Math.abs(deltaVelocity) > 20 ? currentVelocity + MAX_VELOCITY_DIFFERENCE * deltaVelocity / Math.abs(deltaVelocity) : newVelocity;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        final int SENSITIVITY_MULTIPLIER_JOYSTICK = 40, SENSITIVITY_DIVISOR_SERVOS = 1;
        final double MAX_PITCH_POS = 1.0, MIN_PITCH_POS = -1.0, MAX_CLAW_POS = 0.95, MIN_CLAW_POS = 0.4;

        double currentFL, currentFR, currentBL, currentBR, currentBase, currentFore;
        currentFL = currentFR = currentBL = currentBR = currentBase = currentFore = 0;

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx motorBaseArm = hardwareMap.get(DcMotorEx.class, "motorBaseArm");
        DcMotorEx motorForearm = hardwareMap.get(DcMotorEx.class, "motorForearm");

        motorBaseArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorForearm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBaseArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorForearm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo servoPitch = hardwareMap.servo.get("servoPitch");
        Servo servoClaw = hardwareMap.servo.get("servoClaw");

        int baseArmPos = 0, forearmPos = 0;

        waitForStart();

        double pitchPos = 0.5, clawPos = 0.4;

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double joystickLeftY = -gamepad2.left_stick_y, joystickRightY = gamepad2.right_stick_y;

            if (gamepad2.right_trigger > 0.1 && pitchPos < MAX_PITCH_POS) { pitchPos += 0.05 / SENSITIVITY_DIVISOR_SERVOS; }
            else if (gamepad2.left_trigger > 0.1 && pitchPos > MIN_PITCH_POS) { pitchPos -= 0.05 / SENSITIVITY_DIVISOR_SERVOS; }

            if (gamepad2.right_bumper && clawPos < MAX_CLAW_POS) { clawPos += 0.05 / SENSITIVITY_DIVISOR_SERVOS; }
            else if (gamepad2.left_bumper && clawPos > MIN_CLAW_POS) { clawPos -= 0.05 / SENSITIVITY_DIVISOR_SERVOS; }

            servoPitch.setPosition(pitchPos);
            servoClaw.setPosition(clawPos);

            telemetry.addData("Pitch Required Pos: ", pitchPos);
            telemetry.addData("Claw Required Pos: ", clawPos);

            baseArmPos += (int) (joystickLeftY * SENSITIVITY_MULTIPLIER_JOYSTICK);
            forearmPos += (int) (joystickRightY * SENSITIVITY_MULTIPLIER_JOYSTICK);

            if (baseArmPos < 0) { baseArmPos = 0; }
            if (forearmPos < 0) { forearmPos = 0; }

            motorBaseArm.setTargetPosition(baseArmPos);
            motorForearm.setTargetPosition(forearmPos);

            motorBaseArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorForearm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double stableBase = stabilizeVelocity(joystickLeftY * 2000, currentBase);
            double stableFore = stabilizeVelocity(joystickRightY * 2000, currentFore);

            motorBaseArm.setVelocity(stableBase);
            motorForearm.setVelocity(stableFore);

            currentBase = stableBase;
            currentFore = stableFore;

            double x = gamepad1.left_stick_x, y = gamepad1.left_stick_y, rotation = -gamepad1.right_stick_x;
            double resultant = Math.hypot(x, y), resultantAngle = Math.atan2(y, x);
            double possibleAbsolutePower = resultant + Math.abs(rotation);
            double possiblePower = resultant + rotation;

            double xComponent = Math.sin(resultantAngle - Math.PI/4), yComponent = Math.cos(resultantAngle - Math.PI/4);
            double maxComponent = Math.max(Math.abs(xComponent), Math.abs(yComponent));

            double xComponentRatio = xComponent / maxComponent;
            double yComponentRatio = yComponent / maxComponent;

            double frontLeftPower  = resultant * yComponentRatio + rotation;
            double frontRightPower = resultant * xComponentRatio - rotation;
            double backLeftPower   = resultant * xComponentRatio + rotation;
            double backRightPower  = resultant * yComponentRatio - rotation;

            if (possibleAbsolutePower > 1) {
                frontLeftPower  /= possiblePower;
                frontRightPower /= possiblePower;
                backLeftPower   /= possiblePower;
                backRightPower  /= possiblePower;
            }

            double stableFL = stabilizeAcceleration(frontLeftPower, currentFL);
            double stableFR = stabilizeAcceleration(frontRightPower, currentFR);
            double stableBL = stabilizeAcceleration(backLeftPower, currentBL);
            double stableBR = stabilizeAcceleration(backRightPower, currentBR);

            motorFrontLeft.setPower(stableFL);
            motorFrontRight.setPower(stableFR);
            motorBackLeft.setPower(stableBL);
            motorBackRight.setPower(stableBR);

            currentFL = stableFL;
            currentFR = stableFR;
            currentBL = stableBL;
            currentBR = stableBR;
        }
    }
}

