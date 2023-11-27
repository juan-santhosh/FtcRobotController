package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

import java.security.cert.Extension;


public class ftclib_teleop extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration
        // dt motors below
        MotorEx frontLeftMotor = new MotorEx(hardwareMap,"leftFront", Motor.GoBILDA.RPM_435);
        MotorEx rearLeftMotor = new MotorEx(hardwareMap,"leftRear", Motor.GoBILDA.RPM_435);
        MotorEx frontRightMotor = new MotorEx(hardwareMap,"rightFront", Motor.GoBILDA.RPM_435);
        MotorEx rearRightMotor = new MotorEx(hardwareMap,"rightRear", Motor.GoBILDA.RPM_435);
// slide motors below
        MotorEx rightspool = new MotorEx(hardwareMap, "rightspool",537.6,340);
        rightspool.setInverted(true);
        MotorEx leftspool = new MotorEx(hardwareMap, "leftspool",537.6,340);

// Intake Motors below
        DcMotorEx intake = (DcMotorEx) hardwareMap.dcMotor.get("Intake");
// Elbow servos below
        ServoEx rightElbow = new SimpleServo(
                hardwareMap, "rightElbow", 0, 255);
        rightElbow.setInverted(true);
        ServoEx leftElbow = new SimpleServo(
                hardwareMap, "leftElbow", 0, 255);
        ServoEx wrist = new SimpleServo(
                hardwareMap, "wrist", 0, 270);
        ServoEx gripper = new SimpleServo(
                hardwareMap, "claw", 0, 300);

        ServoImplEx drone = (ServoImplEx) hardwareMap.servo.get("drone");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // Ensure spools are braking

            int spooltargetval= rightspool.getCurrentPosition();
            // Slides programming start

            //Max slide extension = 2350
            //mid slide position = 1750
            //low slide position = 1300


            // set the run mode
            leftspool.setRunMode(Motor.RunMode.PositionControl);
            rightspool.setRunMode(Motor.RunMode.PositionControl);

            // set and get the position coefficient
            leftspool.setPositionCoefficient(0.05);
            rightspool.setPositionCoefficient(0.05);

             int MAX_SLIDE_EXTENSION = -2000;
             int MID_SLIDE_EXTENSION = -1500;
             int MIN_SLIDE_EXTENSION = -750;
             int R_LOW_SLIDE_EXTENSION = -20;
             int L_lOW_SLIDE_EXTENSION = -40;
             int SLIDE_Extension_LEFT = 0;
             int SLIDE_EXTENSION_RIGHT = 0;
             leftspool.setTargetPosition(SLIDE_Extension_LEFT);
             rightspool.setTargetPosition(SLIDE_EXTENSION_RIGHT);
             if (gamepad2.dpad_down) {
                 SLIDE_Extension_LEFT = L_lOW_SLIDE_EXTENSION;
                 SLIDE_EXTENSION_RIGHT = R_LOW_SLIDE_EXTENSION;
                 break;
             }
             if (gamepad2.dpad_left) {
                 SLIDE_Extension_LEFT = MIN_SLIDE_EXTENSION;
                 SLIDE_EXTENSION_RIGHT = MIN_SLIDE_EXTENSION;
                 break;

             }

         }


            leftspool.set(0.2);
            rightspool.set(0.2);
            while (!leftspool.atTargetPosition()) {
                leftspool.set(0.1);
            }
            while (!rightspool.atTargetPosition()) {
                rightspool.set(0.1);
            }

            leftspool.stopMotor(); // stop the motor
            rightspool.stopMotor();



            // set the target position
            if (gamepad2.x){

                leftElbow.turnToAngle(20);
                rightElbow.turnToAngle(44); // right elbow offset value leftelbow + 24 degrees
                gripper.turnToAngle(0);

            }
            else if (gamepad2.a) {
                // an integer representing
                // desired tick count
                leftElbow.turnToAngle(60);
                rightElbow.turnToAngle(84);
            }
            else if(gamepad2.b){

                leftElbow.turnToAngle(120);
                rightElbow.turnToAngle(144);
            }
            else if (gamepad2.y){

                leftElbow.turnToAngle(200);
                rightElbow.turnToAngle(184);
            }
            if (leftspool.getCurrentPosition() < 20){
                wrist.turnToAngle(20);
            } else if (leftspool.get()>200) {


            }



            // Slide programming end
            // Transfer system

// Intake programming start
            intake.setPower(gamepad1.right_stick_y/3);
            // Intake programming end




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
            double speedMod =1.25;  // only for testing purposes - not final driver configuration

            frontLeftMotor.set(frontLeftPower*speedMod);
            rearLeftMotor.set(backLeftPower*speedMod);
            frontRightMotor.set(frontRightPower*speedMod);
            rearRightMotor.set(backRightPower*speedMod);
            double totalCurrentDT = frontLeftMotor.getCurrentPosition();

          frontLeftMotor.setInverted(true);
          rearLeftMotor.setInverted(true);




            telemetry.addData("rightspool",rightspool.getCurrentPosition());
            telemetry.addData("leftspool",leftspool.getCurrentPosition());
            telemetry.addData("leftelbow position",leftElbow.getAngle(AngleUnit.DEGREES));
            telemetry.addData("rightelbow position",rightElbow.getAngle(AngleUnit.DEGREES));
            telemetry.update();
           /* double totalROBOTCUrrent = frontLeftMotor.getCurrent(CurrentUnit.AMPS) +
                    frontRightMotor.getCurrent(CurrentUnit.AMPS)+
                    backLeftMotor.getCurrent(CurrentUnit.AMPS)+
                    backRightMotor.getCurrent(CurrentUnit.AMPS);
            telemetry.addData("total dt current draw",totalROBOTCUrrent);*/


        }
    }

