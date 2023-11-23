package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;
import java.util.Random;

@TeleOp(name="Camera Frame Capture", group="Utility")
public class WebcamSnapshot extends LinearOpMode {
    boolean lastX;
    int frameCount;
    long capReqTime;

    @Override
    public void runOpMode() {
        VisionPortal portal;

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "FrontalCamera"))
                .setCameraResolution(new Size(1280, 720))
                .build();
        Random rand = new Random();

        while (!isStopRequested()) {
            boolean x = gamepad1.x;

            if (x && !lastX) {
                portal.saveNextFrameRaw("CameraFrameCapture" + rand.nextInt(10000000));
                capReqTime = System.currentTimeMillis();
            }

            lastX = x;

            telemetry.addLine("-------- Camera Capture Utility --------");
            telemetry.addLine(String.format(Locale.UK, " > Resolution: %dx%d", 1280, 720));
            telemetry.addLine(" > Press X to capture a frame");
            telemetry.addData(" > Camera Status", portal.getCameraState());

            if (capReqTime != 0) telemetry.addLine("\nCaptured Frame.");
            if (capReqTime != 0 && System.currentTimeMillis() - capReqTime > 1000) capReqTime = 0;

            telemetry.update();
        }
    }
}
