package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (group = "CompTELEOP",name = "OPTIMISED_TELEOP")
public class OPTIMISED_TELEOP extends OpMode {
    private DcMotorEx leftspool = null;
    private DcMotorEx rightspool = null;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;

    private DcMotorEx intake;

    private ServoEx rightElbow;
    private ServoEx leftElbow;
    private ServoEx wrist;
    // private CRServo gripper;
    private ServoEx gripper;
    private ServoImplEx drone;
    private ServoEx latch;


    public static final int MAX_SLIDE_VELOCITY = 3000;

    public static final int SLIDE_HIGH = 1600;
    public static final int SLIDE_MID = 1200;
    public static final int SLIDE_LOW = 400;

    @Override
    public void init_loop() {
        leftspool.setMotorEnable();
        rightspool.setMotorEnable();

        intake.setMotorEnable();

        frontLeftMotor.setMotorEnable();
        frontRightMotor.setMotorEnable();
        backLeftMotor.setMotorEnable();
        backRightMotor.setMotorEnable();
    }

    @Override
    public void init() {

        leftspool = hardwareMap.get(DcMotorEx.class, "leftspool");
        rightspool = hardwareMap.get(DcMotorEx.class, "rightspool");

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        rightspool.setDirection(DcMotorEx.Direction.FORWARD);
        leftspool.setDirection(DcMotorEx.Direction.REVERSE);
        leftspool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightspool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ServoEx rightElbow = new SimpleServo(
                hardwareMap, "rightElbow", 0, 255);
        rightElbow.setInverted(true);
        ServoEx leftElbow = new SimpleServo(
                hardwareMap, "leftElbow", 0, 255);
        ServoEx wrist = new SimpleServo(
                hardwareMap, "wrist", 0, 270);
        //wrist = hardwareMap.get(CRServo.class,"wrist");
        //gripper = hardwareMap.get(CRServo.class, "claw");
        ServoEx latch = new SimpleServo(
                hardwareMap, "latch", 0, 270);
        ServoEx gripper= new SimpleServo(
                hardwareMap, "claw", 0, 255);
        //latch = hardwareMap.get(CRServo.class,"latch");

        ServoImplEx drone = (ServoImplEx) hardwareMap.servo.get("drone");
        //telemetry.addLine("Ready to Rock n Roll")

        latch.turnToAngle(180);
        gripper.setInverted(true);
        gripper.turnToAngle(205);
    }

    @Override
    public void loop() {
        leftspool = hardwareMap.get(DcMotorEx.class, "leftspool");
        rightspool = hardwareMap.get(DcMotorEx.class, "rightspool");

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "rightRear");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        ServoEx rightElbow = new SimpleServo(
                hardwareMap, "rightElbow", 0, 255);
        rightElbow.setInverted(true);
        ServoEx leftElbow = new SimpleServo(
                hardwareMap, "leftElbow", 0, 255);
        ServoEx wrist = new SimpleServo(
                hardwareMap, "wrist", 0, 270);
        //wrist = hardwareMap.get(CRServo.class,"wrist");
        //gripper = hardwareMap.get(CRServo.class, "claw");
        ServoEx latch = new SimpleServo(
                hardwareMap, "latch", 0, 270);
        ServoEx gripper= new SimpleServo(
                hardwareMap, "claw", 0, 255);
        //latch = hardwareMap.get(CRServo.class,"latch");

        ServoImplEx drone = (ServoImplEx) hardwareMap.servo.get("drone");
        telemetry.addLine("Ready to Rock n Roll");

        //<Drive code>
        double y = -gamepad1.left_stick_y;// Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x; //gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        double speedMod = 2.05;  // only for testing purposes - not final driver configuration

        frontLeftMotor.setPower(frontLeftPower * speedMod);
        backLeftMotor.setPower(backLeftPower * speedMod);
        frontRightMotor.setPower(frontRightPower * speedMod);
        backRightMotor.setPower(backRightPower * speedMod);
        double totalCurrentDT = frontLeftMotor.getCurrentPosition();

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //</Drive code>

        ElapsedTime spacer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        leftElbow.setInverted(true);
        wrist.setInverted(true);
        gripper.setInverted(true);
        telemetry.addData("timer", spacer.time());
        if (gamepad1.right_bumper){
            speedMod =1;
        }
        // Intake run
        intake.setPower(gamepad2.left_stick_y);
        if (gamepad2.a){ // Intake

            leftElbow.turnToAngle(45);
            spacer.startTime();
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
                wrist.turnToAngle(0);
                spacer.reset();

            latch.turnToAngle(180);
            leftspool.setTargetPosition(-10);
            rightspool.setTargetPosition(-10); // FIXME: i highly recommend adding a separate threaded subsystem handler to prevent the spool motors from self combusting

            leftspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftspool.setVelocity(MAX_SLIDE_VELOCITY);
            rightspool.setVelocity(MAX_SLIDE_VELOCITY);
        }
        else if (gamepad2.x) { // transfer

            latch.turnToAngle(0);
            gripper.turnToAngle(205);
            spacer.startTime();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            leftElbow.turnToAngle(0);
                wrist.turnToAngle(0);
            try {
                Thread.sleep(200);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }


            gripper.turnToAngle(255);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            leftElbow.turnToAngle(45);
            try {
                Thread.sleep(800);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
                        leftspool.setTargetPosition(SLIDE_MID);
                        rightspool.setTargetPosition(SLIDE_MID);

                        leftspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        rightspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        leftspool.setVelocity(MAX_SLIDE_VELOCITY);
                        rightspool.setVelocity(MAX_SLIDE_VELOCITY);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            wrist.turnToAngle(75);
            leftElbow.turnToAngle(165);
                            spacer.reset();
        } // end of loop
        else if (gamepad2.y) { // transfer

            latch.turnToAngle(0);
            gripper.turnToAngle(205);
            spacer.startTime();
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            leftElbow.turnToAngle(0);
            wrist.turnToAngle(0);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }


            gripper.turnToAngle(255);
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            leftElbow.turnToAngle(35);
            try {
                Thread.sleep(800);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            leftspool.setTargetPosition(SLIDE_HIGH);
            rightspool.setTargetPosition(SLIDE_HIGH);

            leftspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            leftspool.setVelocity(MAX_SLIDE_VELOCITY);
            rightspool.setVelocity(MAX_SLIDE_VELOCITY);
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            wrist.turnToAngle(75);
            leftElbow.turnToAngle(165);
            spacer.reset();
        } // end of loop
        if (gamepad2.right_stick_button){
            gripper.turnToAngle(205);
        }
    }


    @Override
    public void stop() {
        leftspool.setMotorDisable();
        rightspool.setMotorDisable();

        intake.setMotorDisable();

        frontLeftMotor.setMotorDisable();
        frontRightMotor.setMotorDisable();
        backLeftMotor.setMotorDisable();
        backRightMotor.setMotorDisable();

    }
}

