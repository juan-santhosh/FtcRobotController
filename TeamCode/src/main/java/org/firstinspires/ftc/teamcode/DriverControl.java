package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Robot-Centric Drive", group="Driver Control")
public class DriverControl extends LinearOpMode {
    /* Create instances of DcMotor(Ex)s and Servos */

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
    Servo servoDrone;

    /* Variables that hold the position values of the servos
       and are the initial position the servos travel to upon pressing play. */

    boolean presetActive;

    double sliderPos = 0;
    double sliderPower = 0;
    double dronePos = 1;

    double clawPos = 0;
    double pitchPos = 0.55;
    double baseLeftPos = 0.4;
    double baseRightPos = 1.0 - baseLeftPos; // baseRightPos is the "inverse" of baseLeftPos so baseLeftPos + baseRightPos = 1

    /* Constants that hold the maximum and minimum position values of the servos */

    final int MAX_SLIDER = 3100;
    final int MIN_SLIDER = 0;

    final double MAX_CLAW = 0.22;
    final double MIN_CLAW = 0;

    final double MAX_PITCH = 1;
    final double MIN_PITCH = 0;

    final double MAX_BASE = 1;
    final double MIN_BASE = 0;

    /* Constants that hold the sensitivity multipliers of the servos and sliders
       Note PRESET_INCREMENT is the servo speed when using a preset whereas
       SERVO_INCREMENT is the speed when using manual control */

    final double SERVO_INCREMENT = 0.005; // Maximum value for a servo is 1.0 note the very small increments.
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
        baseRightPos = 1;

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

        sleep(1000);

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

        sleep(500);

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

        while (motorSliderLeft.getTargetPosition() != motorSliderLeft.getCurrentPosition()) { sleep(1); }

        motorSliderLeft.setPower(0);
        motorSliderRight.setPower(0);

        presetActive = false;
    });

    Thread launchDrone = new Thread(() -> {
        dronePos = 0.5;
        servoDrone.setPosition(dronePos);
        sleep(2000);
        dronePos = 1;
        servoDrone.setPosition(dronePos);
    });


    @Override
    // @Disabled tag goes here if you want to hide a program from the TeleOp list.
    public void runOpMode() throws InterruptedException {
        /* Runs on INIT press. Defines all the previously instanced motors and servos as well as
           configures motor RunModes and Directions. */

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

        servoDrone = hardwareMap.servo.get("servoDrone");
        servoDrone.setPosition(dronePos);

        waitForStart();

        grabPixel.start();

        if (isStopRequested()) return; // Terminates program when STOP pressed.

        while (opModeIsActive()) { // Runs on PLAY press. This section handles all player inputs.
            if (gamepad2.right_trigger > 0.1) { // Raises sliders on holding right trigger.
                /* The notation in the following code means: variable = condition ? value : other_value;

                   This is a compact form of writing the following:
                   if (condition) {
                       variable = value;
                   } else {
                       variable = other_value;
                   } */
                sliderPos += sliderPos < MAX_SLIDER ? gamepad2.right_trigger * SLIDER_SENSITIVITY : 0;
                sliderPower = gamepad2.right_trigger; // Sets the power of the slider motors to how deep the right trigger is being pressed.
            } else if (gamepad2.left_trigger > 0.1) { // Lowers sliders on holding left trigger.
                sliderPos -= sliderPos > MIN_SLIDER ? gamepad2.left_trigger * SLIDER_SENSITIVITY : 0;
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
                } else if (gamepad2.back) {
                    launchDrone.start();
                } else { // Manual control. Same notation as used for the sliders but a little more complex in this scenario.
                    // The nested second expression after the colons is equivalent to an else if statement.
                    clawPos += (gamepad2.dpad_up && clawPos < MAX_CLAW) ? SERVO_INCREMENT : ((gamepad2.dpad_down && clawPos > MIN_CLAW) ? -SERVO_INCREMENT : 0);
                    pitchPos += (gamepad2.right_stick_y > 0.1 && pitchPos < MAX_PITCH) ? SERVO_INCREMENT / 1.5 : ((gamepad2.right_stick_y < -0.1 && pitchPos > MIN_PITCH) ? -SERVO_INCREMENT / 1.5 : 0);
                    baseLeftPos += (gamepad2.left_stick_y > 0.1 && baseLeftPos < MAX_BASE) ? SERVO_INCREMENT : ((gamepad2.left_stick_y < -0.1 && baseLeftPos > MIN_BASE) ? -SERVO_INCREMENT : 0);
                    baseRightPos = 1.0 - baseLeftPos;
                }
            }

            servoClaw.setPosition(clawPos); // Sets the servos to the calculated positions.
            servoPitch.setPosition(pitchPos);
            servoBaseLeft.setPosition(baseLeftPos);
            servoBaseRight.setPosition(baseRightPos);

            motorIntake.setPower(gamepad1.x ? 1 : (gamepad1.b ? -1 : 0)); // Intake motor control handled by the driver.

            // region Telemetry Logging
            telemetry.addData("sliderPos", sliderPos);

            telemetry.addData("Desired L", motorSliderLeft.getTargetPosition());
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

            /* Mecanum Drive. I cannot really explain this through comments and I don't think
               I need to, but let me know if for some reason you require an explanation */

            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double rotation = -gamepad1.right_stick_x;

            double resultant = Math.hypot(x, y);
            double resultantAngle = Math.atan2(y, x) - Math.PI/4;
            double possiblePower = resultant + Math.abs(rotation);

            double xComponent = Math.sin(resultantAngle);
            double yComponent = Math.cos(resultantAngle);
            double maxComponent = Math.max(Math.abs(xComponent), Math.abs(yComponent));

            double xResultant = resultant * xComponent / maxComponent;
            double yResultant = resultant * yComponent / maxComponent;

            double backLeftPower   = xResultant + rotation;
            double backRightPower  = yResultant - rotation;
            double frontLeftPower  = yResultant + rotation;
            double frontRightPower = xResultant - rotation;

            if (possiblePower > 1) {
                backLeftPower   /= possiblePower;
                backRightPower  /= possiblePower;
                frontLeftPower  /= possiblePower;
                frontRightPower /= possiblePower;
            }

            motorBackLeft.setPower(backLeftPower);
            motorBackRight.setPower(backRightPower);
            motorFrontLeft.setPower(frontLeftPower);
            motorFrontRight.setPower(frontRightPower);

            telemetry.update(); // Updates the telemetry to display all the previously logged information to the Driver Station.
        }
    }
}
