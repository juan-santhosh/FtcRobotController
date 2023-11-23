package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.io.File;
import java.util.Locale;

import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import org.tensorflow.lite.*;


@TeleOp(name = "Camera Frame Capture", group = "Utility")
public class ModelTester extends LinearOpMode {
    boolean lastX;
    long capReqTime;

    @Override
    public void runOpMode() {
        VisionPortal portal;
        InterpreterApi model = InterpreterApi.create(new File("models\\propPosDetectorM4.tflite"), new InterpreterApi.Options());
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "FrontalCamera"))
                .setCameraResolution(new Size(192, 72))
                .build();

        while (!isStopRequested()) {
            boolean x = gamepad1.x;

            if (x && !lastX) {
                portal.saveNextFrameRaw("inputImage.png");
                Mat inputImage = Imgcodecs.imread("inputImage.png");
                Mat inputImageGrayscale = new Mat();
                Imgproc.cvtColor(inputImage, inputImageGrayscale, Imgproc.COLOR_RGB2GRAY);



                capReqTime = System.currentTimeMillis();
            }

            lastX = x;

            telemetry.addLine("-------- Camera Capture Utility --------");
            telemetry.addLine(String.format(Locale.UK, " > Resolution: %dx%d", 1280, 720));
            telemetry.addLine(" > Press X to capture a frame");
            telemetry.addData(" > Camera Status", portal.getCameraState());

            if (capReqTime != 0) {
                telemetry.addLine("\nCaptured Frame.");
            }

            if (capReqTime != 0 && System.currentTimeMillis() - capReqTime > 1000) {
                capReqTime = 0;
            }

            telemetry.update();
        }
    }
}
