package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DriverControl extends LinearOpMode {
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackLeft;
    DcMotor motorBackRight;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
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

            motorFrontLeft.setPower(frontLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackLeft.setPower(backLeftPower);
            motorBackRight.setPower(backRightPower);
        }
    }
}
