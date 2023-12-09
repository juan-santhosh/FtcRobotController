package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import java.util.Locale;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.Mat;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;


@TeleOp(name = "Model Tester", group = "Utility")
public class ModelTester extends LinearOpMode {
    boolean lastX;
    long capReqTime;
    double left_prob, right_prob, center_prob;

    @Override
    public void runOpMode() {

        // this op mode is for the red prop.
        // with slight modifications, this can be used for the blue prop as well.
        VisionPortal portal;
        Net model = Dnn.readNet("propPosDetector.onnx");
        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "FrontalCamera"))
                .setCameraResolution(new Size(256, 144))
                .build();

        while (!isStopRequested()) {
            boolean x = gamepad1.x;

            if (x && !lastX) {
                portal.saveNextFrameRaw("inputImage.png");
                Mat inputImage = Imgcodecs.imread("inputImage.png");
                Mat extractedChannel = new Mat();
                // coi (below) should be 2 if red and 0 if blue
                Core.extractChannel(inputImage, extractedChannel, 2);
                Mat fullBlack = new Mat();
                Core.multiply(Mat.ones(144, 256, CvType.CV_8UC1), new Scalar(255), fullBlack);
                Core.subtract(fullBlack, extractedChannel, extractedChannel);
                for (int i = 0; i < 144; i++) {
                    for (int j = 0; j < 256; j++) {
                        // for the blue prop, one might want to replace 30 (below) with some larger number
                        if (extractedChannel.get(i, j)[0] > 30) {
                            extractedChannel.put(i, j, 255);
                        }
                    }
                }
                Mat inputFloatImage = new Mat(144, 256, CvType.CV_32FC1);
                extractedChannel.convertTo(inputFloatImage, CvType.CV_32FC1);
                Mat inputBlob = Dnn.blobFromImage(extractedChannel, 1.0, new org.opencv.core.Size(144, 256));

                model.setInput(inputBlob);
                Mat y = model.forward();
                left_prob = y.get(0, 0)[0];
                right_prob = y.get(0, 0)[0];
                center_prob = y.get(0, 0)[0];

                capReqTime = System.currentTimeMillis();
            }

            lastX = x;

            telemetry.addLine("-------- Camera Capture Utility --------");
            telemetry.addLine(String.format(Locale.UK, " > Resolution: %dx%d", 1280, 720));
            telemetry.addLine(" > Press X to capture a frame");
            telemetry.addData(" > Camera Status", portal.getCameraState());

            if (capReqTime != 0) {
                telemetry.addLine("\nCaptured Frame.\nLeft Probability: " + left_prob +
                        "\nRight Probability: " + right_prob + "\nCentre Probability: " + center_prob);
                if (left_prob > center_prob && left_prob > right_prob) {
                    telemetry.addLine("\nConclusion: Left");
                } else if (right_prob > center_prob && right_prob > left_prob) {
                    telemetry.addLine("\nConclusion: Right");
                } else if (center_prob > right_prob && center_prob > left_prob) {
                    telemetry.addLine("\nConclusion: Centre");
                }
            }

            if (capReqTime != 0 && System.currentTimeMillis() - capReqTime > 1000) {
                capReqTime = 0;
            }

            telemetry.update();
        }
    }
}
