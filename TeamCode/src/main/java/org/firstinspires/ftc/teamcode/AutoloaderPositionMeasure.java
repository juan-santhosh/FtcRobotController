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
public class AutoloaderPositionMeasure extends LinearOpMode {
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
            telemetry.addData("Base Left Pos: ", servoBaseLeft.getPosition());
            telemetry.addData("Base Right Pos: ", servoBaseRight.getPosition());
            telemetry.addData("Claw Pos: ", servoClaw.getPosition());
            telemetry.addData("Pitch Pos: ", servoPitch.getPosition());

            telemetry.addData("Position Left viper", motorLeftViper.getCurrentPosition());
            telemetry.addData("Position Right Viper", motorRightViper.getCurrentPosition());


            telemetry.addData("Left Trigger", gamepad2.left_trigger);
            telemetry.addData("Right Trigger", gamepad2.right_trigger);
            telemetry.update();
        }
    }
}

