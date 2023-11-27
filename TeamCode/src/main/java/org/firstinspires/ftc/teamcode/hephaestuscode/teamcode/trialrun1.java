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


public class trialrun1 extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        ServoImplEx wrist = (ServoImplEx) hardwareMap.servo.get("wrist");
        ServoImplEx leftElbow = (ServoImplEx) hardwareMap.servo.get("leftElbow");
        ServoImplEx rightElbow = (ServoImplEx) hardwareMap.servo.get("rightElbow");
        ServoImplEx gripper = (ServoImplEx) hardwareMap.servo.get("claw");
        ServoImplEx drone = (ServoImplEx) hardwareMap.servo.get("drone");
        ServoImplEx latch = (ServoImplEx) hardwareMap.servo.get("latch");
        MotorEx intake = new MotorEx(hardwareMap, "Intake", Motor.GoBILDA.BARE);

        waitForStart();

        if (isStopRequested()) return;
        while(opModeIsActive()){
            if (gamepad2.x){
                gripper.setPosition(0.5);
                leftElbow.setPosition(8);
                rightElbow.setPosition(0.9);

            }

           intake.set(gamepad2.left_stick_y);

        }
    }
}