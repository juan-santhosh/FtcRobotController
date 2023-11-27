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

public class teleOP1 extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration
        // dt motors below
        DcMotorEx frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        DcMotorEx backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftRear");
        DcMotorEx frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        DcMotorEx backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightRear");
// slide motors below
        MotorEx rightspool = new MotorEx(hardwareMap, "rightspool");
        rightspool.setInverted(true);
        MotorEx leftspool = new MotorEx(hardwareMap, "leftspool");

// Intake Motors below
        DcMotorEx intake = (DcMotorEx) hardwareMap.dcMotor.get("Intake");
// Elbow servos below
        ServoEx rightElbow = new SimpleServo(
                hardwareMap, "rightElbow", 0, 255);
        rightElbow.setInverted(true);
        ServoEx leftElbow = new SimpleServo(
                hardwareMap, "leftElbow", 0, 255);
        ServoEx wrist = new SimpleServo(
                hardwareMap, "wrist", 0, 360);
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
            int MAX_SLIDE_EXTENSION = -2350;
            int MID_SLIDE_EXTENSION = -1750;
            int MIN_SLIDE_EXTENSION = -1300;

                        // set the run mode
                        leftspool.setRunMode(Motor.RunMode.PositionControl);
                        rightspool.setRunMode(Motor.RunMode.PositionControl);

            // set and get the position coefficient
                        leftspool.setPositionCoefficient(0.05);
                       // rightspool.setPositionCoefficient(0.05);
                        double kP = leftspool.getPositionCoefficient();



            // set the target position
           if (gamepad2.x){
               //leftspool.setTargetPosition(0);
               //rightspool.setTargetPosition(0);
               leftElbow.turnToAngle(18);
               rightElbow.turnToAngle(42); // right elbow offset value leftelbow + 24 degrees
               wrist.turnToAngle(350);
               gripper.turnToAngle(250);

           }
            else if (gamepad2.a) {
                //leftspool.setTargetPosition(MIN_SLIDE_EXTENSION);
              // rightspool.setTargetPosition(MIN_SLIDE_EXTENSION); // an integer representing
                // desired tick count
               leftElbow.turnToAngle(60);
               rightElbow.turnToAngle(84);
            }
            else if(gamepad2.b){
                //leftspool.setTargetPosition(MID_SLIDE_EXTENSION);
              // rightspool.setTargetPosition(MID_SLIDE_EXTENSION);
               leftElbow.turnToAngle(120);
               rightElbow.turnToAngle(144);
            }
            else if (gamepad2.y){
                //leftspool.setTargetPosition(MAX_SLIDE_EXTENSION);
              // rightspool.setTargetPosition(MAX_SLIDE_EXTENSION);
               leftElbow.turnToAngle(200);
               rightElbow.turnToAngle(184);
            }
            if (leftspool.getCurrentPosition() < 20){
                wrist.turnToAngle(20);
            } else if (leftspool.get()>200) {


            }

            leftspool.set(0.2);
            rightspool.set(0.2);

            // set the tolerance
                        leftspool.setPositionTolerance(20);
                        //rightspool.setPositionTolerance(20);// allowed maximum error

            // perform the control loop
                        while (!leftspool.atTargetPosition()) {
                            leftspool.set(0.10);
                        }
                        while(!rightspool.atTargetPosition()){
                            rightspool.set(0.10);
                        }
                        leftspool.stopMotor(); // stop the motor
                        rightspool.stopMotor();


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

            frontLeftMotor.setPower(frontLeftPower*speedMod);
            backLeftMotor.setPower(backLeftPower*speedMod);
            frontRightMotor.setPower(frontRightPower*speedMod);
            backRightMotor.setPower(backRightPower*speedMod);
            double totalCurrentDT = frontLeftMotor.getCurrentPosition();

            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);




            telemetry.addData("rightspool",rightspool.getCurrentPosition());
            telemetry.addData("leftspool",leftspool.getCurrentPosition());
            telemetry.addData("leftelbow position",leftElbow.getAngle(AngleUnit.DEGREES));
            telemetry.addData("rightelbow position",rightElbow.getAngle(AngleUnit.DEGREES));
            double totalROBOTCUrrent = frontLeftMotor.getCurrent(CurrentUnit.AMPS) +
                    frontRightMotor.getCurrent(CurrentUnit.AMPS)+
                    backLeftMotor.getCurrent(CurrentUnit.AMPS)+
                    backRightMotor.getCurrent(CurrentUnit.AMPS);
            telemetry.addData("total dt current draw",totalROBOTCUrrent);
            telemetry.update();

        }
    }
}