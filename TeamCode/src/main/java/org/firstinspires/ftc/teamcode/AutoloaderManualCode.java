//https://docs.revrobotics.com/duo-control/programming/using-encoder-feedback
//NEED TO DO:
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class AutoloaderManualCode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        int sensitivityMultiplierServos = 1;

        double MAXPITCHPOS = 0.75;
        double MINPITCHPOS = -0.75;
        
        double MAXBASEPOS = 0.75;
        double MINBASEPOS = -0.75;

        double MAXCLAWPOS = 1;
        double MINCLAWPOS = 0.4;

        double MAXVIPERS = 100; //the most the viperslides can extend themselves to
        double MINVIPERS = 0; //the lowest the viperslides can move to

        Servo servoBaseLeft = hardwareMap.servo.get("servoBaseLeft");
        Servo servoBaseRight = hardwareMap.servo.get("servoBaseRight");
        Servo servoPitch = hardwareMap.servo.get("servoPitch");
        Servo servoClaw = hardwareMap.servo.get("servoClaw");


        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        DcMotor motorIntake = hardwareMap.dcMotor.get("motorIntake");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx motorLeftViper = hardwareMap.get(DcMotorEx.class, "motorLeftViper");
        DcMotorEx motorRightViper = hardwareMap.get(DcMotorEx.class, "motorRightViper");

        motorLeftViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightViper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLeftViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightViper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int ViperPos = 0;
        
        waitForStart();

        double pitchPos = 0;
        double clawPos = 0;
        double BasePos = 0;

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double joystickLeftY = -gamepad2.left_stick_y;
            double joystickRightY = gamepad2.right_stick_y;

            if (gamepad2.right_trigger > 0.1 && pitchPos < MAXPITCHPOS) {
                pitchPos += 0.05 / sensitivityMultiplierServos;
            } else if (gamepad2.left_trigger > 0.1 && pitchPos > MINPITCHPOS) {
                pitchPos -= 0.05 / sensitivityMultiplierServos;
            }

            if (gamepad2.right_bumper && clawPos < MAXCLAWPOS) {
                clawPos += 0.05 / sensitivityMultiplierServos;
            } else if (gamepad2.left_bumper && clawPos > MINCLAWPOS) {
                clawPos -= 0.05 / sensitivityMultiplierServos;
            }
            if (BasePos < MINBASEPOS) { BasePos = MINBASEPOS; } //Make it not go past the calibration point (reduce damage)
            if (BasePos > MAXBASEPOS) { BasePos = MAXBASEPOS; }
            BasePos += (int) (joystickRightY / sensitivityMultiplierServos)

            servoPitch.setPosition(pitchPos);
            servoClaw.setPosition(clawPos);
            servoBaseRight.setPosition(BasePos);
            servoBaseLeft.setPosition(-BasePos);

            telemetry.addData("Pitch Required Pos: ", pitchPos);
            telemetry.addData("Claw Required Pos: ", clawPos);

            if (gamepad2.dpad_left) { // Go to Calibrate position


            } else if (gamepad2.dpad_down) { // Intake Close


            } else if (gamepad2.dpad_up) {   // Intake Far


            } else if (gamepad2.y) {         // High Junction
                

            } else if (gamepad2.b) {         // Middle Junction


            } else if (gamepad2.a) {         // Hover (Low Junction)


            } else {                         // Manual Control
                ViperPos += (int) (joystickLeftY * 40);
            }

            if (ViperPos < MINVIPERS) { ViperPos = MINVIPERS; } //Make it not go past the calibration point (reduce damage)
            if (forearmPos < 2) { forearmPos = 0; } //Make it not go past the calibration point (reduce damage)

            if (ViperPos > MAXVIPERS) { ViperPos = MAXVIPERS; } //make it not go past max
            if (forearmPos > -30) { forearmPos = -28; } //make it not go past max

            motorLeftViper.setTargetPosition(ViperPos);
            motorRightViper.setTargetPosition(-ViperPos);

            motorLeftViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightViper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorLeftViper.setVelocity(2000);
            motorRightViper.setVelocity(2000);

            if (gamepad2.a) {         // intakeactivate
                motorIntake.setPower(255);
            }

            telemetry.addData("Velocity base arm", motorLeftViper.getVelocity());
            telemetry.addData("Position base arm", motorLeftViper.getCurrentPosition());
            telemetry.addData("Base is at target", !motorLeftViper.isBusy());
            telemetry.addData("Velocity forearm", motorRightViper.getVelocity());
            telemetry.addData("Position forearm", motorRightViper.getCurrentPosition());
            telemetry.addData("Forearm is at target.", !motorRightViper.isBusy());
            telemetry.addData("Left Trigger", gamepad2.left_trigger);
            telemetry.addData("Right Trigger", gamepad2.right_trigger);
            telemetry.update();

            //DT Code:
            
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double rotation = gamepad1.right_stick_x;

            double theta = Math.atan2(y, x);
            double resultant = Math.hypot(x, y);

            double x_component = Math.sin(theta - Math.PI/4);
            double y_component = Math.cos(theta - Math.PI/4);
            double max_component = Math.max(Math.abs(x_component), Math.abs(y_component));

            double frontLeftPower  = resultant * (y_component / max_component) + rotation;
            double frontRightPower = resultant * (x_component / max_component) - rotation;
            double backLeftPower   = resultant * (x_component / max_component) + rotation;
            double backRightPower  = resultant * (y_component / max_component) - rotation;

            if ((resultant + Math.abs(rotation)) > 1) {
                frontLeftPower  /= resultant + rotation;
                frontRightPower /= resultant + rotation;
                backLeftPower   /= resultant + rotation;
                backRightPower  /= resultant + rotation;
            }

            motorFrontLeft.setPower(frontLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackLeft.setPower(backLeftPower);
            motorBackRight.setPower(backRightPower);

        }
    }
}

