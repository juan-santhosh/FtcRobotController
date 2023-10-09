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

    DcMotor motorIntake;
    DcMotorEx motorSlider;

    Servo servoClaw;

    int sliderPos = 0;

    double servoPos = 0;
    double intakePower = 0;

    final double SERVO_INCREMENT = 0.005;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorIntake = hardwareMap.dcMotor.get("motorIntake");

        motorSlider = hardwareMap.get(DcMotorEx.class, "motorSlider");
        motorSlider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSlider.setVelocity(50);

        servoClaw = hardwareMap.servo.get("servoClaw");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            sliderPos += (int) (-gamepad2.right_stick_y);
            motorSlider.setTargetPosition(sliderPos);

            servoPos += (gamepad2.right_trigger > 0.1 && servoPos < (1 - SERVO_INCREMENT)) ? SERVO_INCREMENT : ((gamepad2.left_trigger > 0.1 && servoPos > SERVO_INCREMENT) ? -SERVO_INCREMENT : 0);
            servoClaw.setPosition(servoPos);

            motorIntake.setPower(gamepad2.a ? 1 : (gamepad2.b ? 0 : intakePower));

            double x = gamepad1.left_stick_x, y = gamepad1.left_stick_y, rotation = -gamepad1.right_stick_x;
            double resultant = Math.hypot(x, y), resultantAngle = Math.atan2(y, x);
            double possibleAbsPower = resultant + Math.abs(rotation);
            double possiblePower = resultant + rotation;

            double xComponent = Math.sin(resultantAngle - Math.PI/4), yComponent = Math.cos(resultantAngle - Math.PI/4);
            double maxComponent = Math.max(Math.abs(xComponent), Math.abs(yComponent));

            double xComponentRatio = xComponent / maxComponent;
            double yComponentRatio = yComponent / maxComponent;

            double frontLeftPower  = resultant * yComponentRatio + rotation;
            double frontRightPower = resultant * xComponentRatio - rotation;
            double backLeftPower   = resultant * xComponentRatio + rotation;
            double backRightPower  = resultant * yComponentRatio - rotation;

            if (possibleAbsPower > 1) {
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
