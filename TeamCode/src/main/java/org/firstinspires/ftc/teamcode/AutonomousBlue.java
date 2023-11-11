package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Size;

import java.util.Arrays;
import java.util.List;
import java.util.stream.IntStream;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Autonomous Blue", group="Autonomous Programs")
public class AutonomousBlue extends LinearOpMode {
    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    DcMotor motorIntake;

    DcMotorEx motorSliderLeft;
    DcMotorEx motorSliderRight;

    DcMotorEx motorFrontLeft;
    DcMotorEx motorFrontRight;
    DcMotorEx motorBackLeft;
    DcMotorEx motorBackRight;

    Servo servoClaw;
    Servo servoPitch;
    Servo servoBaseLeft;
    Servo servoBaseRight;

    int motorBLPos, motorBRPos, motorFLPos, motorFRPos;

    double clawPos = 0.22, pitchPos = 0.55;
    double baseLeftPos = 1.0, baseRightPos = 1.0 - baseLeftPos;

    final double WHEEL_CIRCUMFERENCE = Math.PI * 0.096;
    final double TICKS_PER_REVOLUTION = 537.7;

    int[] detectTags = {1, 2, 3};

    boolean parked;

    @Override
    public void runOpMode() throws InterruptedException {
        // region Initialize Motors and Servos
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motorFrontRight");

        motorSliderLeft = hardwareMap.get(DcMotorEx.class, "motorLeftSlider");
        motorSliderRight = hardwareMap.get(DcMotorEx.class, "motorRightSlider");

        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorSliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorSliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorSliderLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorSliderRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorIntake = hardwareMap.dcMotor.get("motorIntake");

        servoClaw = hardwareMap.servo.get("servoClaw");
        servoPitch = hardwareMap.servo.get("servoPitch");
        servoBaseLeft = hardwareMap.servo.get("servoBaseLeft");
        servoBaseRight = hardwareMap.servo.get("servoBaseRight");
        // endregion

        initDetection();

        while (!isStopRequested()) {
            if (opModeInInit()) {
                telemetry.addLine("Preview Camera Stream by pressing the three dots in the top right then press 'Camera Stream'");
                telemetry.addLine("\n----------------------------------------");
            } else {
                if (!parked) {
                    grabPixel.start();

                    strafe(0.61, 0);
                    driveToPos.start();
                    driveToPos.join();
                }

                while (!parked) {
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();

                    for (AprilTagDetection detection : currentDetections) {
                        if (IntStream.of(detectTags).anyMatch(x -> x == detection.id)) {
                            drive(detection.ftcPose.y, (3 * 0.336 / Math.cos(Math.PI/6)) - 0.1);
                            driveToPos.start();

                            motorSliderLeft.setTargetPosition(-620);
                            motorSliderRight.setTargetPosition(620);
                            moveSliders.start();

                            placePixel.start();
                            placePixel.join();

                            drive(-0.305, 0);
                            driveToPos.start();

                            zeroArm.start();

                            motorSliderLeft.setTargetPosition(0);
                            motorSliderRight.setTargetPosition(0);
                            moveSliders.start();

                            driveToPos.join();

                            drive(detection.ftcPose.y, 0.05);
                            driveToPos.start();
                            driveToPos.join();

                            parked = true;
                            break;
                        }
                    }
                }
            }

            if (visionPortal.getProcessorEnabled(aprilTag)) {
                telemetry.addLine("\nDpad Left to disable AprilTag");
                telemetryAprilTag();
            } else {
                telemetry.addLine("Dpad Right to enable AprilTag");
            }

            telemetry.addLine("\n----------------------------------------");

            if (visionPortal.getProcessorEnabled(tfod)) {
                telemetry.addLine("\nDpad Down to disable TFOD");
                telemetryTfod();
            } else {
                telemetry.addLine("Dpad Up to enable TFOD");
            }

            telemetry.addLine("\n----------------------------------------");

            telemetry.addData("\nMotor BL Position", motorBackLeft.getCurrentPosition());
            telemetry.addData("Motor BR Position", motorBackRight.getCurrentPosition());
            telemetry.addData("Motor FL Position", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Motor FR Position", motorFrontRight.getCurrentPosition());

            telemetry.addData("\nMotor BL Target Position", motorBackLeft.getTargetPosition());
            telemetry.addData("Motor BR Target Position", motorBackRight.getTargetPosition());
            telemetry.addData("Motor FL Target Position", motorFrontLeft.getTargetPosition());
            telemetry.addData("Motor FR Target Position", motorFrontRight.getTargetPosition());

            telemetry.update();

            if (gamepad1.dpad_left) {
                visionPortal.setProcessorEnabled(aprilTag, false);
            } else if (gamepad1.dpad_right) {
                visionPortal.setProcessorEnabled(aprilTag, true);
            }

            if (gamepad1.dpad_down) {
                visionPortal.setProcessorEnabled(tfod, false);
            } else if (gamepad1.dpad_up) {
                visionPortal.setProcessorEnabled(tfod, true);
            }
        }
    }

    Thread grabPixel = new Thread(() -> {
        clawPos = 0.11;
        pitchPos = 0.878;
        servoClaw.setPosition(clawPos);
        servoPitch.setPosition(pitchPos);

        sleep(500);

        baseLeftPos = 0;
        baseRightPos = 1;

        servoBaseLeft.setPosition(baseLeftPos);
        servoBaseRight.setPosition(baseRightPos);

        sleep(1000);

        clawPos = 0.22;
        servoClaw.setPosition(clawPos);

        sleep(500);

        baseLeftPos = 0.5;
        baseRightPos = 1.0 - baseLeftPos;

        servoBaseLeft.setPosition(baseLeftPos);
        servoBaseRight.setPosition(baseRightPos);
    });

    Thread placePixel = new Thread(() -> {
        pitchPos = 0.555;
        servoPitch.setPosition(pitchPos);

        sleep(500);

        baseLeftPos = 0.565;
        baseRightPos = 1.0 - baseLeftPos;

        servoBaseLeft.setPosition(baseLeftPos);
        servoBaseRight.setPosition(baseRightPos);

        sleep(500);

        pitchPos = 0.345;
        servoPitch.setPosition(pitchPos);

        sleep(500);

        clawPos = 0;
        servoClaw.setPosition(clawPos);
    });

    Thread zeroArm = new Thread(() -> {
        pitchPos = 0.555;
        servoPitch.setPosition(pitchPos);

        sleep(500);

        baseLeftPos = 0.4;
        baseRightPos = 1.0 - baseLeftPos;

        servoBaseLeft.setPosition(baseLeftPos);
        servoBaseRight.setPosition(baseRightPos);

        sleep(500);

        pitchPos = 0.878;
        servoPitch.setPosition(pitchPos);
    });

    Thread moveSliders = new Thread(() -> {
        motorSliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorSliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorSliderLeft.setPower(1);
        motorSliderRight.setPower(1);

        while (true) {
            if (motorSliderLeft.getTargetPosition() == motorSliderLeft.getCurrentPosition()) {
                motorSliderLeft.setPower(0);
                motorSliderRight.setPower(0);
                break;
            }
        }
    });

    Thread driveToPos = new Thread(() -> {
        motorBackLeft.setTargetPosition(motorBLPos);
        motorBackRight.setTargetPosition(motorBRPos);
        motorFrontLeft.setTargetPosition(motorFLPos);
        motorFrontRight.setTargetPosition(motorFRPos);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackLeft.setPower(1);
        motorBackRight.setPower(1);
        motorFrontLeft.setPower(1);
        motorFrontRight.setPower(1);

        while (true) {
            if (motorBackRight.getTargetPosition() <= motorBackLeft.getCurrentPosition() && motorFrontLeft.getTargetPosition() <= motorFrontLeft.getCurrentPosition()) {
                motorBackLeft.setPower(0);
                motorBackRight.setPower(0);
                motorFrontLeft.setPower(0);
                motorFrontRight.setPower(0);

                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                break;
            }
        }
    });

    private void drive(double distance, double distanceTarget) {
        motorBLPos = motorBRPos = motorFLPos = motorFRPos = calculatePos(distance, distanceTarget);
    }

    private void strafe(double distance, double distanceTarget) {
        motorBRPos = motorFLPos = calculatePos(distance, distanceTarget);
        motorBLPos = motorFRPos = -calculatePos(distance, distanceTarget);
    }

//    private void rotate(double degrees) {
//        motorBLPos = motorFRPos = calculatePos(degrees / 360, 0);
//        motorBRPos = motorFLPos = -calculatePos(degrees / 360, 0);
//    }

    private int calculatePos(double distance, double distanceTarget) {
        return (int) ((distance - distanceTarget) / WHEEL_CIRCUMFERENCE * TICKS_PER_REVOLUTION);
    }

    private void initDetection() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .setLensIntrinsics(1445.262036, 1458.020517, 624.518936, 335.987846)
                .build();

        tfod = new TfodProcessor.Builder().build();
        tfod.setMinResultConfidence(0.75f);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Front Camera"))
                .setCameraResolution(new Size(1280, 720))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(false)
                .addProcessors(tfod, aprilTag)
                .build();
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("\n# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (m)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (m, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        telemetry.addLine("\nKey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("\n# Objects Detected:", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("\nImage:", "%s (%.0f %% Confidence)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position:", "%.0f / %.0f", x, y);
            telemetry.addData("- Size:", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
    }
}