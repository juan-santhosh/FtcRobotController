package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Size;

import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp
public class AutonomousBase extends LinearOpMode {
    DcMotorEx motorFrontLeft;
    DcMotorEx motorFrontRight;
    DcMotorEx motorBackLeft;
    DcMotorEx motorBackRight;

    private static final boolean USE_WEBCAM = true;

    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;


    @Override
    public void runOpMode() throws InterruptedException {
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

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        initTfod();
        initAprilTag();

        waitForStart();

        while (opModeIsActive()) {
            telemetryTfod();
            telemetryAprilTag();
            
            telemetry.update();
            
            if (gamepad1.back) {
                visionPortal.stopStreaming();
            } else if (gamepad1.start) {
                visionPortal.resumeStreaming();
            }

            sleep(20);
        }

        visionPortal.close();
    }

    private void initTfod() {
        tfod = new TfodProcessor.Builder().build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Front Camera"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(new Size(1280, 720));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.setAutoStopLiveView(false);
        builder.addProcessor(tfod);

        visionPortal = builder.build();
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
                .setLensIntrinsics(1445.262036, 1458.020517, 624.518936, 335.987846)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Front Camera"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.setCameraResolution(new Size(1280, 720));
        builder.enableLiveView(true);
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.setAutoStopLiveView(false);
        builder.addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    private void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected:", currentRecognitions.size());

        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            double y = (recognition.getTop() + recognition.getBottom()) / 2;

            telemetry.addData("\nImage:", "%s (%.0f %% Confidence)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position:", "%.0f / %.0f", x, y);
            telemetry.addData("- Size:", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

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
}