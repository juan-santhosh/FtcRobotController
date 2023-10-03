package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@TeleOp
public class Main extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    DcMotorEx motorFrontLeft;
    DcMotorEx motorFrontRight;
    DcMotorEx motorBackLeft;
    DcMotorEx motorBackRight;

    DcMotor motorSlider;

    Servo servoClaw;

    double servoPos = 0;
    final double SERVO_INCREMENT = 0.005;

    int DETECT_ID = 1;

    final double TAG_SIZE = 0.166;

    final double FX = 1445.26203593404;
    final double FY = 1458.020517466836;
    final double CX = 624.5189363050382;
    final double CY = 335.98784556504864;

    final double TICKS_PER_REVOLUTION = 537.7;
    final double WHEEL_CIRCUMFERENCE = Math.PI * 0.095;

    final double Y_TARGET = 0.2;

    AprilTagDetection detectTag = null;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(TAG_SIZE, FX, FY, CX, CY);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                telemetry.addData("Camera Status: ", "Streaming");
            }
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Status: ", "Error " + errorCode);
            }
        });

        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setVelocity(50);
        motorFrontRight.setVelocity(50);
        motorBackLeft.setVelocity(50);
        motorBackRight.setVelocity(50);

        motorSlider = hardwareMap.dcMotor.get("motorSlider");

        servoClaw = hardwareMap.servo.get("servoClaw");

        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == DETECT_ID) {
                        detectTag = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest has been detected.\n\nLocation data:");
                    tagToTelemetry(detectTag);
                } else {
                    telemetry.addLine("Tag of interest not detected.");

                    if (detectTag == null) {
                        telemetry.addLine("(The tag has yet to be detected)");
                    } else {
                        telemetry.addLine("\nTag last detected at:");
                        tagToTelemetry(detectTag);
                    }
                }
            } else {
                telemetry.addLine("Tag of interest not detected.");

                if (detectTag == null) {
                    telemetry.addLine("(The tag has yet to be detected)");
                } else {
                    telemetry.addLine("\nTag last detected at:");
                    tagToTelemetry(detectTag);
                }
            }

            telemetry.update();
            sleep(20);
        }

        if (detectTag != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(detectTag);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it wasn't detected during the init loop.");
            telemetry.update();
        }

        if (detectTag == null) {
            telemetry.addLine("No april tag detected.");
            telemetry.update();
        } else {
            if (detectTag.pose.y > Y_TARGET) {
                telemetry.addLine("April tag detected, now moving.");
                telemetry.update();

                double rotationsNeeded = (detectTag.pose.y - Y_TARGET) / WHEEL_CIRCUMFERENCE;
                int encoderTarget = (int) (rotationsNeeded * TICKS_PER_REVOLUTION);

                motorFrontLeft.setTargetPosition(encoderTarget);
                motorFrontRight.setTargetPosition(encoderTarget);
                motorBackLeft.setTargetPosition(encoderTarget);
                motorBackRight.setTargetPosition(encoderTarget);
            }
        }

        while (opModeIsActive()) {
            motorSlider.setPower(gamepad2.right_stick_y);

            if (gamepad2.right_trigger > 0.1 && servoPos < (1 - SERVO_INCREMENT)) {
                servoPos += SERVO_INCREMENT;
            } else if (gamepad2.left_trigger > 0.1 && servoPos > SERVO_INCREMENT) {
                servoPos -= SERVO_INCREMENT;
            }

            servoClaw.setPosition(servoPos);
        }
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection) {
        Orientation rot = Orientation.getOrientation(detection.pose.R, AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);

        telemetry.addLine(String.format("\nDetected tag ID: %d", detection.id));

        telemetry.addLine(String.format("Translation X: %.2f metres", detection.pose.x));
        telemetry.addLine(String.format("Translation Y: %.2f metres", detection.pose.y));
        telemetry.addLine(String.format("Translation Z: %.2f metres", detection.pose.z));

        telemetry.addLine(String.format("Rotation X: %.2f degrees", rot.secondAngle));
        telemetry.addLine(String.format("Rotation Y: %.2f degrees", rot.firstAngle));
        telemetry.addLine(String.format("Rotation Z: %.2f degrees", rot.thirdAngle));
    }

    void resetEncoders() {
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}