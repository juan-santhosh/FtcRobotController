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

    double sliderPower = 0;
    double sliderPos = 0;

    double clawPos = 0;
    double pitchPos = 0;
    double baseLeftPos = 1;
    double baseRightPos = 0;

    double currentBL, currentBR, currentFL, currentFR;
    double smoothBL, smoothBR, smoothFL, smoothFR;

    final int MAX_SLIDER = 4820;
    final int MIN_SLIDER = 0;

    final double SERVO_INCREMENT = 0.005;

    final double MAX_CLAW = 0.22;
    final double MIN_CLAW = 0;

    final double MAX_PITCH = 0.87;
    final double MIN_PITCH = 0;

    final double MAX_BASE = 1;
    final double MIN_BASE = 0.18;

    final double MAX_POWER_DIFFERENCE = 0.7;

    private double smoothPower(double newPower, double currentPower) {
        double outputPower = newPower;
        double powerDifference = newPower - currentPower;

        if (Math.abs(powerDifference) > 0.1) {
            outputPower = currentPower + MAX_POWER_DIFFERENCE * powerDifference / Math.abs(powerDifference);
        }

        return outputPower;
    }

    public void intake() {
        servoBaseLeft.setPosition(0);
        servoBaseRight.setPosition(0);
        servoPitch.setPosition(0);

        sleep(1000);

        servoClaw.setPosition(0);

        sleep(1000);

        servoBaseLeft.setPosition(0);
        servoBaseRight.setPosition(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorIntake = hardwareMap.dcMotor.get("motorIntake");
        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);

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
                if (sliderPos < MAX_SLIDER) {
                    sliderPos += gamepad2.right_trigger * 0.001;
                }
                sliderPower = gamepad2.right_trigger;
            } else if (gamepad2.left_trigger > 0.1) {
                if (sliderPos > MIN_SLIDER) {
                    sliderPos -= gamepad2.left_trigger * 0.001;
                }
                sliderPower = gamepad2.left_trigger;
            } else {
                sliderPower = 0.1;
            }

            motorSliderLeft.setTargetPosition((int) (-sliderPos));
            motorSliderRight.setTargetPosition((int) (sliderPos));

            motorSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorSliderLeft.setPower(sliderPower);
            motorSliderRight.setPower(sliderPower);

            telemetry.addData("Desired L", motorSliderLeft.getTargetPosition());
            telemetry.addData("Actual L", motorSliderLeft.getCurrentPosition());

            telemetry.addData("\nDesired R", motorSliderRight.getTargetPosition());
            telemetry.addData("Actual R", motorSliderRight.getCurrentPosition());

            clawPos += (gamepad2.dpad_up && clawPos < MAX_CLAW) ? SERVO_INCREMENT : ((gamepad2.dpad_down && clawPos > MIN_CLAW) ? -SERVO_INCREMENT : 0);
            servoClaw.setPosition(clawPos);

            telemetry.addData("\nDesired Claw", clawPos);
            telemetry.addData("Actual Claw", servoClaw.getPosition());

            pitchPos += (gamepad2.right_stick_y > 0.1 && pitchPos < MAX_PITCH) ? SERVO_INCREMENT : ((gamepad2.right_stick_y < -0.1 && pitchPos > MIN_PITCH) ? -SERVO_INCREMENT : 0);
            servoPitch.setPosition(pitchPos);

            telemetry.addData("\nDesired Pitch", pitchPos);
            telemetry.addData("Actual Pitch", servoPitch.getPosition());

            baseLeftPos += (gamepad2.left_stick_y > 0.1 && baseLeftPos < MAX_BASE) ? SERVO_INCREMENT : ((gamepad2.left_stick_y < -0.1 && baseLeftPos > MIN_BASE) ? -SERVO_INCREMENT : 0);
            baseRightPos += 1.0 - baseLeftPos; // (gamepad2.left_stick_y < -0.1 && baseRightPos < MAX_BASE) ? SERVO_INCREMENT : ((gamepad2.left_stick_y > 0.1 && baseRightPos > MIN_BASE) ? -SERVO_INCREMENT : 0);

            servoBaseLeft.setPosition(baseLeftPos);
            servoBaseRight.setPosition(baseRightPos);

            telemetry.addData("\nDesired Base Left", baseLeftPos);
            telemetry.addData("Actual Base Left", servoBaseLeft.getPosition());

            telemetry.addData("\nDesired Base Right", baseRightPos);
            telemetry.addData("Actual Base Right", servoBaseRight.getPosition());

            motorIntake.setPower(gamepad1.a ? 1 : 0);

            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rotation = -gamepad1.right_stick_x;

            double resultant = Math.hypot(x, y);
            double resultantAngle = Math.atan2(y, x);
            double possiblePower = resultant + rotation;
            double possibleAbsPower = resultant + Math.abs(rotation);

            double xComponent = Math.sin(resultantAngle - Math.PI/4);
            double yComponent = Math.cos(resultantAngle - Math.PI/4);
            double maxComponent = Math.max(Math.abs(xComponent), Math.abs(yComponent));

            double xComponentRatio = xComponent / maxComponent;
            double yComponentRatio = yComponent / maxComponent;

            double backLeftPower   = resultant * xComponentRatio + rotation;
            double backRightPower  = resultant * yComponentRatio - rotation;
            double frontLeftPower  = resultant * yComponentRatio + rotation;
            double frontRightPower = resultant * xComponentRatio - rotation;

            if (possibleAbsPower > 1) {
                frontLeftPower  /= possiblePower;
                frontRightPower /= possiblePower;
                backLeftPower   /= possiblePower;
                backRightPower  /= possiblePower;
            }

            smoothBL = smoothPower(backLeftPower, currentFL);
            smoothBR = smoothPower(backRightPower, currentFR);
            smoothFL = smoothPower(frontLeftPower, currentBL);
            smoothFR = smoothPower(frontRightPower, currentBR);

            motorBackLeft.setPower(smoothBL);
            motorBackRight.setPower(smoothBR);
            motorFrontLeft.setPower(smoothFR);
            motorFrontRight.setPower(smoothFR);

            currentBL = smoothBL;
            currentBR = smoothBR;
            currentFL = smoothFL;
            currentFR = smoothFR;

            telemetry.update();
        }
    }
}
