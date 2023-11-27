package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import com.qualcomm.robotcore.hardware.ServoImplEx;

public class basic_teleop extends OpMode {
    private DcMotorEx leftspool;
    private DcMotorEx rightspool;

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    private DcMotorEx backRightMotor;

    private DcMotorEx Intake ;




    ServoEx rightElbow;
    ServoEx leftElbow;
    ServoEx wrist;
    ServoEx gripper;
    ServoEx latch;
    ServoImplEx drone;
@Override
    public void init(){




}
    @Override
    public void init_loop() {
    }
    @Override
    public void loop(){
    DcMotorEx frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
    DcMotorEx backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftRear");
    DcMotorEx frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
    DcMotorEx backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightRear");

    DcMotorEx Intake = (DcMotorEx) hardwareMap.dcMotor.get("Intake");

    ServoEx rightElbow = new SimpleServo(
            hardwareMap, "rightElbow", 0, 255);
    rightElbow.setInverted(true);
    ServoEx leftElbow = new SimpleServo(
            hardwareMap, "leftElbow", 0, 255);
    ServoEx wrist = new SimpleServo(
            hardwareMap, "wrist", 0, 270);
    ServoEx gripper = new SimpleServo(
            hardwareMap, "claw", 0, 300);

    ServoEx latch = new SimpleServo(
            hardwareMap, "latch", 0, 300);
    ServoImplEx drone = (ServoImplEx) hardwareMap.servo.get("drone");

    DcMotorEx rightspool = (DcMotorEx) hardwareMap.dcMotor.get("rightspool");

    DcMotorEx leftspool = (DcMotorEx) hardwareMap.dcMotor.get("leftspool");

    leftspool.setTargetPosition(0);
    rightspool.setTargetPosition(0);


    // Intake
    if(gamepad1.a){
        Intake.setPower(0.5);
        latch.turnToAngle(25);
    } else if (gamepad1.x) {
        Intake.setPower(0);
        latch.turnToAngle(90);
    }
    // Drivetrain code
    double y = gamepad1.left_stick_y;// Remember, Y stick value is reversed
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

    // Slide code

    int MAX_SLIDE_EXTENSION = -1800;
    int MID_SLIDE_EXTENSION = -1800;
    int MIN_SLIDE_EXTENSION = -750;
    int R_LOW_SLIDE_EXTENSION = -20;
    int L_lOW_SLIDE_EXTENSION = -40;
    int SLIDE_Extension_LEFT = 0;
    int SLIDE_EXTENSION_RIGHT = 0;

    if (gamepad2.a){
        SLIDE_Extension_LEFT = L_lOW_SLIDE_EXTENSION;
        SLIDE_EXTENSION_RIGHT = R_LOW_SLIDE_EXTENSION;

    } else if (gamepad2.x) {
        SLIDE_Extension_LEFT=MIN_SLIDE_EXTENSION;
        SLIDE_EXTENSION_RIGHT=MIN_SLIDE_EXTENSION;
    }
    else if (gamepad2.y){
        SLIDE_Extension_LEFT=MID_SLIDE_EXTENSION;
        SLIDE_EXTENSION_RIGHT=MID_SLIDE_EXTENSION;
    }
    else if (gamepad2.b){
        SLIDE_Extension_LEFT=MAX_SLIDE_EXTENSION;
        SLIDE_EXTENSION_RIGHT=MAX_SLIDE_EXTENSION;

        while (leftspool.getCurrentPosition()<SLIDE_Extension_LEFT) {
            leftspool.setPower(0.5);
            rightspool.setPower(0.5);
        }
    }

    leftspool.setTargetPosition(SLIDE_Extension_LEFT);
    rightspool.setTargetPosition(SLIDE_EXTENSION_RIGHT);
    rightspool.setDirection(DcMotorEx.Direction.REVERSE);

    // End of slide code
    // drone code
    if (gamepad2.right_bumper){
        drone.setPosition(0.25);
    }

    // drone code done

// End of loop section
}

}
