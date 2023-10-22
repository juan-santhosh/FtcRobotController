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
    DcMotorEx motorSliderLeft;
    DcMotorEx motorSliderRight;

    Servo servoClaw;
    Servo servoPitch;
    Servo servoBaseLeft;
    Servo servoBaseRight;

    double sliderPos = 0;
    double sliderPower = 0;

    double clawPos = 0;
    double pitchPos = 0.55;
    double baseLeftPos = 0.8;
    double baseRightPos = 0.2;

    final int MAX_SLIDER = 3100;
    final int MIN_SLIDER = 0;

    final double MAX_CLAW = 0.22;
    final double MIN_CLAW = 0;

    final double MAX_PITCH = 1;
    final double MIN_PITCH = 0;

    final double MAX_BASE = 1;
    final double MIN_BASE = 0;

    final double SERVO_INCREMENT = 0.005;
    final double PRESET_INCREMENT = 0.05;
    final double SLIDER_SENSITIVITY = 10;

    public void grabPixel() {
        baseLeftPos += baseLeftPos > 0 ? -PRESET_INCREMENT : 0;
        baseRightPos += baseRightPos < 1 ? PRESET_INCREMENT : 0;

        pitchPos = 0.61;
        clawPos = 0.11;
    }

    public void retractArm() {
        baseLeftPos += baseLeftPos < 0.8 ? PRESET_INCREMENT : 0;
        baseRightPos += baseRightPos > 0.2 ? -PRESET_INCREMENT : 0;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorIntake = hardwareMap.dcMotor.get("motorIntake");

        motorSliderLeft = hardwareMap.get(DcMotorEx.class, "motorLeftSlider");
        motorSliderRight = hardwareMap.get(DcMotorEx.class, "motorRightSlider");

        motorSliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorSliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        servoClaw = hardwareMap.servo.get("servoClaw");
        servoPitch = hardwareMap.servo.get("servoPitch");
        servoBaseLeft = hardwareMap.servo.get("servoBaseLeft");
        servoBaseRight = hardwareMap.servo.get("servoBaseRight");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (gamepad2.right_trigger > 0.1) {
                sliderPos += sliderPos < MAX_SLIDER ? gamepad2.right_trigger * SLIDER_SENSITIVITY : 0;
                sliderPower = gamepad2.right_trigger;
            } else if (gamepad2.left_trigger > 0.1) {
                sliderPos -= sliderPos > MIN_SLIDER ? gamepad2.left_trigger * SLIDER_SENSITIVITY : 0;
                sliderPower = gamepad2.left_trigger;
            }

            motorSliderLeft.setTargetPosition((int) (-sliderPos));
            motorSliderRight.setTargetPosition((int) (sliderPos));

            motorSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorSliderLeft.setPower(sliderPower);
            motorSliderRight.setPower(sliderPower);

            telemetry.addData("sliderPos", sliderPos);

            telemetry.addData("Desired L", motorSliderLeft.getTargetPosition());
            telemetry.addData("Actual L", motorSliderLeft.getCurrentPosition());

            telemetry.addData("\nDesired R", motorSliderRight.getTargetPosition());
            telemetry.addData("Actual R", motorSliderRight.getCurrentPosition());

            if (gamepad2.y) {
                grabPixel();
            } else if (gamepad2.x) {
                retractArm();
            } else {
                clawPos += (gamepad2.dpad_up && clawPos < MAX_CLAW) ? SERVO_INCREMENT : ((gamepad2.dpad_down && clawPos > MIN_CLAW) ? -SERVO_INCREMENT : 0);
                pitchPos += (gamepad2.right_stick_y > 0.1 && pitchPos < MAX_PITCH) ? SERVO_INCREMENT : ((gamepad2.right_stick_y < -0.1 && pitchPos > MIN_PITCH) ? -SERVO_INCREMENT : 0);
                baseLeftPos += (gamepad2.left_stick_y > 0.1 && baseLeftPos < MAX_BASE) ? SERVO_INCREMENT : ((gamepad2.left_stick_y < -0.1 && baseLeftPos > MIN_BASE) ? -SERVO_INCREMENT : 0);
                baseRightPos = 1.0 - baseLeftPos;
            }

            servoClaw.setPosition(clawPos);
            servoPitch.setPosition(pitchPos);
            servoBaseLeft.setPosition(baseLeftPos);
            servoBaseRight.setPosition(baseRightPos);

            motorIntake.setPower(gamepad1.x ? 1 : 0);

            telemetry.addData("\nDesired Claw", clawPos);
            telemetry.addData("Actual Claw", servoClaw.getPosition());

            telemetry.addData("\nDesired Pitch", pitchPos);
            telemetry.addData("Actual Pitch", servoPitch.getPosition());

            telemetry.addData("\nDesired Base Left", baseLeftPos);
            telemetry.addData("Actual Base Left", servoBaseLeft.getPosition());

            telemetry.addData("\nDesired Base Right", baseRightPos);
            telemetry.addData("Actual Base Right", servoBaseRight.getPosition());

            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rotation = -gamepad1.right_stick_x;

            double resultant = Math.hypot(x, y);
            double resultantAngle = Math.atan2(y, x) - Math.PI/4;
            double possiblePower = resultant + rotation;
            double possibleAbsPower = resultant + Math.abs(rotation);

            double xComponent = Math.sin(resultantAngle);
            double yComponent = Math.cos(resultantAngle);
            double maxComponent = Math.max(Math.abs(xComponent), Math.abs(yComponent));

            double xResultant = resultant * xComponent / maxComponent;
            double yResultant = resultant * yComponent / maxComponent;

            double backLeftPower   = xResultant + rotation;
            double backRightPower  = yResultant - rotation;
            double frontLeftPower  = yResultant + rotation;
            double frontRightPower = xResultant - rotation;

            if (possibleAbsPower > 1) {
                backLeftPower   /= possiblePower;
                backRightPower  /= possiblePower;
                frontLeftPower  /= possiblePower;
                frontRightPower /= possiblePower;
            }

            motorBackLeft.setPower(backLeftPower);
            motorBackRight.setPower(backRightPower);
            motorFrontLeft.setPower(frontLeftPower);
            motorFrontRight.setPower(frontRightPower);

            telemetry.update();
        }
    }
}
