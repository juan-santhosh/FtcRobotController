package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

// Some important notes for my teammates:
// you guys should tune the duration parameters through trial and error;
// and you guys should also insert the code for automatic transfer and release (where it is indicated down there to do so).
// good luck with your matches!

@Autonomous
class AutoFromRowTwoToBackstage extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        DcMotor rightRear = hardwareMap.get(DcMotor.class, "rightRear");

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized and Waiting");
        telemetry.update();

        waitForStart();

        double movementOneTime = 2000;  // this duration parameter (in ms) should be tuned through trial and error.
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        leftFront.setPower(1);
        leftRear.setPower(1);
        rightFront.setPower(1);
        rightRear.setPower(1);

        while (timer.time() < movementOneTime) {
            telemetry.addData("Status", "Running Mvt 1");
            telemetry.update();
        }

        telemetry.addData("Status", "Transitioning to Mvt 2");
        telemetry.update();
        double movementTwoTime = 2000;  // this duration parameter (in ms) should be tuned through trial and error;
        timer.reset();

        leftFront.setPower(1);
        leftRear.setPower(-1);
        rightFront.setPower(-1);
        rightRear.setPower(1);

        while (timer.time() < movementTwoTime) {
            telemetry.addData("Status", "Running Mvt 2");
            telemetry.update();
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

        telemetry.addData("Status", "Running Mvt 3 (Dropping Pixels)");
        telemetry.update();

        // insert here the code for automatic transfer and dropping.

        double MovementFourDuration = 2000;  // this duration parameter (in ms) should be tuned through trial and error;
        timer.reset();

        leftFront.setPower(-0.5);
        leftRear.setPower(-0.5);
        rightFront.setPower(-0.5);
        rightRear.setPower(-0.5);

        while (timer.time() < MovementFourDuration) {
            telemetry.addData("Status", "Running Mvt 4 (Parking)");
            telemetry.update();
        }

        leftFront.setPower(0);
        leftRear.setPower(0);
        rightFront.setPower(0);
        rightRear.setPower(0);

    }
}
