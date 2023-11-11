package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Field-Centric Drive", group="Driver Control")
public class DriverControlStabilisedPower extends LinearOpMode {
    private boolean presetActive;

    private double sliderPos = 0;
    private double sliderPower = 0;
    private double dronePos = 1;
    private double hookPos = 0;

    private double clawPos = 0;
    private double pitchPos = 0.55;
    private double baseLeftPos = 0.4;
    private double baseRightPos = 1.0 - baseLeftPos;

    private double currentBL;
    private double currentBR;
    private double currentFL;
    private double currentFR;

    @Override
    // @Disabled tag goes here if you want to hide a program from the TeleOp list.
    public void runOpMode() throws InterruptedException {
        /* Runs on INIT press. Creates instances of DcMotor(Ex)s and Servos */

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor motorIntake = hardwareMap.dcMotor.get("motorIntake");
        DcMotorEx motorSliderLeft = hardwareMap.get(DcMotorEx.class, "motorLeftSlider");
        DcMotorEx motorSliderRight = hardwareMap.get(DcMotorEx.class, "motorRightSlider");

        motorSliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorSliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Servo servoClaw = hardwareMap.servo.get("servoClaw");
        Servo servoPitch = hardwareMap.servo.get("servoPitch");
        Servo servoBaseLeft = hardwareMap.servo.get("servoBaseLeft");
        Servo servoBaseRight = hardwareMap.servo.get("servoBaseRight");

        Servo servoDrone = hardwareMap.servo.get("servoDrone");
        servoDrone.setPosition(dronePos);

        Servo servoHook = hardwareMap.servo.get("servoHook");
        servoHook.setPosition(hookPos);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        /* Constants that hold the maximum position values of the motors and servos */

        final int MAX_SLIDER = 3100;
        final double MAX_CLAW = 0.22;
        final double MAX_PITCH = 1;
        final double MAX_BASE = 1;

        final double SERVO_INCREMENT = 0.005;
        final double SLIDER_SENSITIVITY = 20;

        /* Presets that attempt to grab the pixels and retract the claw to it's starting position respectively. */

        Thread grabPixel = new Thread(() -> {
            presetActive = true;

            clawPos = 0.11;
            pitchPos = 0.878;
            servoClaw.setPosition(clawPos);
            servoPitch.setPosition(pitchPos);

            sleep(500);

            baseLeftPos = 0;
            baseRightPos = 1.0 - baseLeftPos;

            servoBaseLeft.setPosition(baseLeftPos);
            servoBaseRight.setPosition(baseRightPos);

            sleep(1000);

            clawPos = MAX_CLAW;
            servoClaw.setPosition(clawPos);

            sleep(500);

            baseLeftPos = 0.5;
            baseRightPos = 1.0 - baseLeftPos;

            servoBaseLeft.setPosition(baseLeftPos);
            servoBaseRight.setPosition(baseRightPos);

            presetActive = false;
        });

        Thread retractArm = new Thread(() -> {
            presetActive = true;

            clawPos = MAX_CLAW;
            servoClaw.setPosition(clawPos);

            sleep(500);

            baseLeftPos = 0.5;
            baseRightPos = 1.0 - baseLeftPos;

            servoBaseLeft.setPosition(baseLeftPos);
            servoBaseRight.setPosition(baseRightPos);

            presetActive = false;
        });

        Thread placePixel = new Thread(() -> {
            presetActive = true;

            pitchPos = 0.555;
            servoPitch.setPosition(pitchPos);

            sleep(500);

            baseLeftPos = 0.565;
            baseRightPos = 1.0 - baseLeftPos;

            servoBaseLeft.setPosition(baseLeftPos);
            servoBaseRight.setPosition(baseRightPos);

            sleep(500);

            pitchPos = 0.345;
            servoPitch.setPosition(pitchPos);

            presetActive = false;
        });

        Thread zeroArm = new Thread(() -> {
            presetActive = true;

            pitchPos = 0.555;
            servoPitch.setPosition(pitchPos);

            sleep(500);

            baseLeftPos = 0.5;
            baseRightPos = 1.0 - baseLeftPos;

            servoBaseLeft.setPosition(baseLeftPos);
            servoBaseRight.setPosition(baseRightPos);

            sleep(500);

            pitchPos = 0.878;
            servoPitch.setPosition(pitchPos);

            sliderPos = 0;
            motorSliderLeft.setTargetPosition((int) (-sliderPos));
            motorSliderRight.setTargetPosition((int) (sliderPos));

            motorSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorSliderLeft.setPower(1);
            motorSliderRight.setPower(1);

            while (true) {
                if (motorSliderLeft.getTargetPosition() == motorSliderLeft.getCurrentPosition()) {
                    motorSliderLeft.setPower(0);
                    motorSliderRight.setPower(0);
                    break;
                }
            }

            presetActive = false;
        });

        Thread launchDrone = new Thread(() -> {
            presetActive = true;

            dronePos = 0.5;
            servoDrone.setPosition(dronePos);

            sleep(1000);

            presetActive = false;

            dronePos = 1;
            servoDrone.setPosition(dronePos);
        });

        Thread latchHook = new Thread(() -> {
            presetActive = true;

            hookPos = 1;
            servoHook.setPosition(hookPos);

            sleep(500);

            presetActive = false;
        });

        waitForStart();

        retractArm.start();

        if (isStopRequested()) return; // Terminates program when STOP pressed.

        while (opModeIsActive()) { // Runs on PLAY press. This section handles all player inputs.
            if (gamepad2.right_trigger > 0.1) { // Raises sliders on holding right trigger.
                // The notation in the following code is: variable = condition ? value : other_value;
                // meaning if condition true then value, else other_value.
                sliderPos += sliderPos < MAX_SLIDER ? gamepad2.right_trigger * SLIDER_SENSITIVITY : 0;
                sliderPower = gamepad2.right_trigger; // Sets the power of the slider motors to how deep the right trigger is being pressed.
            } else if (gamepad2.left_trigger > 0.1) { // Lowers sliders on holding left trigger.
                sliderPos -= sliderPos > 0 ? gamepad2.left_trigger * SLIDER_SENSITIVITY : 0;
                sliderPower = gamepad2.left_trigger;
            }

            motorSliderLeft.setTargetPosition((int) (-sliderPos)); // Sets the slider positions to the previously calculated sliderPos values.
            motorSliderRight.setTargetPosition((int) (sliderPos)); // Note motorSliderLeft is negative as it spins in the opposite direction.

            motorSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Sets the sliders to run to a given position when supplied power.
            motorSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorSliderLeft.setPower(sliderPower); // Supplies sliders with previously calculated power.
            motorSliderRight.setPower(sliderPower);

            /* Servo control */

            if (!presetActive) {
                if (gamepad2.x) {
                    grabPixel.start();
                } else if (gamepad2.y) {
                    placePixel.start();
                } else if (gamepad2.b) {
                    zeroArm.start();
                } else if (gamepad1.back) {
                    launchDrone.start();
                } else if (gamepad2.back) {
                    latchHook.start();
                } else { // Manual control. Same notation as used for the sliders but a little more complex in this scenario.
                    // The nested second expression after the colons is equivalent to an else if statement.
                    clawPos += (gamepad2.dpad_up && clawPos < MAX_CLAW) ? SERVO_INCREMENT : ((gamepad2.dpad_down && clawPos > 0) ? -SERVO_INCREMENT : 0);
                    pitchPos += (gamepad2.right_stick_y > 0.1 && pitchPos < MAX_PITCH) ? SERVO_INCREMENT / 1.5 : ((gamepad2.right_stick_y < -0.1 && pitchPos > 0) ? -SERVO_INCREMENT / 1.5 : 0);
                    baseLeftPos += (gamepad2.left_stick_y > 0.1 && baseLeftPos < MAX_BASE) ? SERVO_INCREMENT : ((gamepad2.left_stick_y < -0.1 && baseLeftPos > 0) ? -SERVO_INCREMENT : 0);
                    baseRightPos = 1.0 - baseLeftPos;
                }
            }

            // Sets the servos to the calculated positions.
            servoClaw.setPosition(clawPos);
            servoPitch.setPosition(pitchPos);
            servoBaseLeft.setPosition(baseLeftPos);
            servoBaseRight.setPosition(baseRightPos);

            motorIntake.setPower(gamepad1.x ? 1 : (gamepad1.b ? -1 : 0)); // Intake motor control handled by the driver.

            // region Telemetry Logging
            telemetry.addData("sliderPos", sliderPos);

            telemetry.addData("\nDesired L", motorSliderLeft.getTargetPosition());
            telemetry.addData("Actual L", motorSliderLeft.getCurrentPosition());

            telemetry.addData("\nDesired R", motorSliderRight.getTargetPosition());
            telemetry.addData("Actual R", motorSliderRight.getCurrentPosition());

            telemetry.addData("\nDesired Claw", clawPos);
            telemetry.addData("Actual Claw", servoClaw.getPosition());

            telemetry.addData("\nDesired Pitch", pitchPos);
            telemetry.addData("Actual Pitch", servoPitch.getPosition());

            telemetry.addData("\nDesired Base Left", baseLeftPos);
            telemetry.addData("Actual Base Left", servoBaseLeft.getPosition());

            telemetry.addData("\nDesired Base Right", baseRightPos);
            telemetry.addData("Actual Base Right", servoBaseRight.getPosition());
            // endregion

            /* Field Centric Mecanum Drive */
            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rot = -gamepad1.right_stick_x;

            if (gamepad1.start) imu.resetYaw();

            double zRot = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS); // - Math.PI/30
            double xRes = x * Math.cos(-zRot) - y * Math.sin(-zRot) * 1.1;
            double yRes = x * Math.sin(-zRot) + y * Math.cos(-zRot);

            double maxRes = Math.max(Math.abs(yRes) + Math.abs(xRes) + Math.abs(rot), 1);
            double backLeftPower   = (yRes - xRes + rot) / maxRes;
            double backRightPower  = (yRes + xRes - rot) / maxRes;
            double frontLeftPower  = (yRes + xRes + rot) / maxRes;
            double frontRightPower = (yRes - xRes - rot) / maxRes;

            double stableBL = stabilisePower(backLeftPower, currentBL);
            double stableBR = stabilisePower(backRightPower, currentBR);
            double stableFL = stabilisePower(frontLeftPower, currentFL);
            double stableFR = stabilisePower(frontRightPower, currentFR);

            motorBackLeft.setPower(stableBL);
            motorBackRight.setPower(stableBR);
            motorFrontLeft.setPower(stableFL);
            motorFrontRight.setPower(stableFR);

            currentBL = stableBL;
            currentBR = stableBR;
            currentFL = stableFL;
            currentFR = stableFR;

            telemetry.update(); // Updates the telemetry to display all the previously logged information to the Driver Station.
        }
    }

    private double stabilisePower(double currentPower, double newPower) {
        final double MAX_POWER_DIFFERENCE = 0.7;

        double deltaPower = newPower - currentPower;
        return Math.abs(deltaPower) > MAX_POWER_DIFFERENCE ? currentPower + MAX_POWER_DIFFERENCE * deltaPower / Math.abs(deltaPower) : newPower;
    }
}
