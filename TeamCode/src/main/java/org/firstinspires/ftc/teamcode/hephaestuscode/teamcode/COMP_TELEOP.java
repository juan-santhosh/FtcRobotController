package org.firstinspires.ftc.teamcode;

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

@TeleOp (group = "CompTELEOP",name = "Regionals_MANUAL_TELEOP")
public class COMP_TELEOP extends OpMode {
    private DcMotorEx leftspool = null;
    private DcMotorEx rightspool = null;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;

    private DcMotorEx intake;

    private CRServo rightElbow;
    private CRServo leftElbow;
    private CRServo wrist;
    private CRServo gripper;
    private ServoImplEx drone;
    private CRServo latch;


    public static final int MAX_SLIDE_VELOCITY = 2000;

    public static final int SLIDE_HIGH = 2000;
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

   /* ServoEx rightElbow = new SimpleServo(
            hardwareMap, "rightElbow", 0, 255);
    rightElbow.setInverted(true);
    ServoEx leftElbow = new SimpleServo(
            hardwareMap, "leftElbow", 0, 255);*/
    rightElbow = hardwareMap.get(CRServo.class,"rightElbow");
    leftElbow = hardwareMap.get(CRServo.class,"leftElbow");
    //ServoEx wrist = new SimpleServo(
           // hardwareMap, "wrist", 0, 270);
    wrist = hardwareMap.get(CRServo.class,"wrist");
    gripper = hardwareMap.get(CRServo.class,"claw");
   // ServoEx latch = new SimpleServo(
       //     hardwareMap, "latch", 0, 270);
    latch = hardwareMap.get(CRServo.class,"latch");

    ServoImplEx drone = (ServoImplEx) hardwareMap.servo.get("drone");
    //telemetry.addLine("Ready to Rock n Roll")
}

@Override
public void loop() {

    //</Slides>
    if(gamepad2.dpad_up) {
        leftspool.setTargetPosition(SLIDE_HIGH);
        rightspool.setTargetPosition(SLIDE_HIGH);

        leftspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftspool.setVelocity(MAX_SLIDE_VELOCITY);
        rightspool.setVelocity(MAX_SLIDE_VELOCITY);
    } else if (gamepad2.dpad_down) {
        leftspool.setTargetPosition(0);
        rightspool.setTargetPosition(0); // FIXME: i highly recommend adding a separate threaded subsystem handler to prevent the spool motors from self combusting

        leftspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftspool.setVelocity(MAX_SLIDE_VELOCITY);
        rightspool.setVelocity(MAX_SLIDE_VELOCITY);
    }
    else if (gamepad2.dpad_left){
        leftspool.setTargetPosition(SLIDE_MID);
        rightspool.setTargetPosition(SLIDE_MID);

        leftspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightspool.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftspool.setVelocity(MAX_SLIDE_VELOCITY);
        rightspool.setVelocity(MAX_SLIDE_VELOCITY);
    }
    else {
        leftspool.setVelocity(0);
        rightspool.setVelocity(0);
    }
    
    telemetry.addData("Slide Pos: ", (leftspool.getCurrentPosition() + rightspool.getCurrentPosition())/2);
//</Slides end>



    //<Drive code>
    double y = - gamepad1.left_stick_y;// Remember, Y stick value is reversed
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
//</Drive code>

    // Intake
        intake.setPower(gamepad2.left_stick_y / 2); // brings down intake speed to 1500 rpm

        //

    /*ServoEx rightElbow = new SimpleServo(
            hardwareMap, "rightElbow", 0, 255);
    rightElbow.setInverted(true);
        ServoEx leftElbow = new SimpleServo(
            hardwareMap, "leftElbow", 0, 255);*/
    rightElbow = hardwareMap.get(CRServo.class,"rightElbow");
    leftElbow = hardwareMap.get(CRServo.class,"leftElbow");

    leftElbow.setPower(-gamepad2.right_stick_x /2);
    rightElbow.setPower(gamepad2.right_stick_x /2);
    //ServoEx wrist = new SimpleServo(
            //hardwareMap, "wrist", 0, 270);
    wrist = hardwareMap.get(CRServo.class,"wrist");
    wrist.setPower(gamepad2.right_stick_y);
    gripper = hardwareMap.get(CRServo.class,"claw");
    gripper.setPower(gamepad2.left_stick_y);
   // ServoEx latch = new SimpleServo(
          //  hardwareMap, "latch", 0, 270);
    latch = hardwareMap.get(CRServo.class,"latch");
    ServoEx drone = new SimpleServo(
            hardwareMap, "drone", 0, 270);
    
    //telemetry.addLine("Ready to Rock n Roll")
    // Drone
    if (gamepad1.right_bumper) {
        drone.setPosition(0.25);
    }
        //
    // latch
   latch.setPower(-gamepad2.right_trigger);
    latch.setPower(gamepad2.left_trigger);
    // latch

    // transfer system


        //leftElbow.setInverted(true);
       // wrist.setInverted(true);
      //  if (gamepad2.a){ // Intake position
           // 0 the wrist at intake position - back end of robot
          //  leftElbow.turnToAngle(0);
            //rightElbow.turnToAngle(5);
           // if ((leftElbow.getAngle() == 0)) {

                //wrist.turnToAngle(0);
            //}
            // rightElbow.turnToAngle(0);
        //}
       // if (gamepad2.x){ // transfer position

           // leftElbow.turnToAngle(100);


            //rightElbow.turnToAngle(100);
        //}
    //if (gamepad2.y) { // mid scoring
       // wrist.turnToAngle(140);
        //leftElbow.turnToAngle(140);
        //rightElbow.turnToAngle(140);
  //  }
   // wrist.setPower(gamepad2.right_stick_y);

   // if (gamepad2.b) { // high scoring
        //wrist.turnToAngle(90);
        //leftElbow.turnToAngle(124);
        //rightElbow.turnToAngle(100);
   // }

    // gripper

    /*if(gamepad2.left_stick_button){
        gripper.turnToAngle(65);
    }
    if(gamepad2.right_stick_button){
        gripper.turnToAngle(45);
    }*/

    //gripper.setPower(gamepad2.left_stick_x/1.25);
    //telemetry.addLine("POINTS TIME");



}


@Override
public void stop(){
    leftspool.setMotorDisable();
    rightspool.setMotorDisable();

    intake.setMotorDisable();

    frontLeftMotor.setMotorDisable();
    frontRightMotor.setMotorDisable();
    backLeftMotor.setMotorDisable();
    backRightMotor.setMotorDisable();

    }
}
