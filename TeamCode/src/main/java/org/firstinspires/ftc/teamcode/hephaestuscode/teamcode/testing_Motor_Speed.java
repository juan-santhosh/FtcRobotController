package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


public class testing_Motor_Speed extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration

        DcMotorEx backLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("leftRear");
        DcMotorEx frontRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        DcMotorEx backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("rightRear");
        CRServo axon = hardwareMap.crservo.get("axon");
        DcMotorEx frontLeftMotor = (DcMotorEx) hardwareMap.dcMotor.get("lefFront");
// add motors as required


        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
           frontLeftMotor.setPower(gamepad1.left_stick_y);
           frontRightMotor.setPower(gamepad1.right_stick_y);
           backLeftMotor.setPower(gamepad2.left_stick_y);
           backRightMotor.setPower(gamepad2.right_stick_y);
           axon.setPower(gamepad1.left_stick_x);


           telemetry.addData("front left speed", frontLeftMotor.getVelocity());
           telemetry.addData("front right speed", frontRightMotor.getVelocity());
           telemetry.addData("back left speed", backLeftMotor.getVelocity());
           telemetry.addData("back right speed", backRightMotor.getVelocity());
           telemetry.addData("Axon direction", axon.getDirection());



        }
    }
}