package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class mecanumDrive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("rightRear");
        Servo axon = hardwareMap.servo.get("axon");

        waitForStart();

        if (isStopRequested()) return;
        boolean axonRotated = false;

        // change this to true when appropriate...
        boolean reverseAxonDirection = false;

        // change this to the right value through trial and error...
        // this value should be less than 0.33333333...
        double initialInclination = 0.0;

        while (opModeIsActive()) {
            // is this initial position sensible?
            axon.setPosition(reverseAxonDirection? (1-initialInclination):initialInclination);

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            double speedMod = gamepad1.right_trigger; // only for testing purposes - not final driver configuration

            frontLeftMotor.setPower(frontLeftPower*speedMod);
            backLeftMotor.setPower(backLeftPower*speedMod);
            frontRightMotor.setPower(frontRightPower*speedMod);
            backRightMotor.setPower(backRightPower*speedMod);
            double totalCurrentDT = frontLeftMotor.getCurrentPosition();



            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            if (gamepad1.y) {
                // will this turn in the right direction?
                // I assumed here that the maximum travel of our axon is 270 degrees...
                // is it okay to use the 'y' button of the controller?
                if (axonRotated) {
                    axon.setPosition(reverseAxonDirection? (1-initialInclination):initialInclination);
                    axonRotated = false;
                } else {
                    axon.setPosition(reverseAxonDirection? (1 - initialInclination - 0.6666666666):(initialInclination + 0.6666666666));
                    axonRotated = true;
                }
            }

        }
    }
}
