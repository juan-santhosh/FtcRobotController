package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Distance Sensor Testing", group="Utility")
public class DistanceSensorTest extends LinearOpMode {
    DistanceSensor distanceSensorLeft;
    DistanceSensor distanceSensorRight;

    @Override
    public void runOpMode() {
        distanceSensorLeft = hardwareMap.get(DistanceSensor.class, "Intake Sensor Left");
        distanceSensorRight = hardwareMap.get(DistanceSensor.class, "Intake Sensor Right");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Distance Sensor Left", distanceSensorLeft.getDistance(DistanceUnit.CM));
            telemetry.addData("Distance Sensor Right", distanceSensorRight.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
