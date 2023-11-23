package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

//Pitch value should INCREASE as the robot is tipped UP at the front. (Rotation about X)
//Roll value should INCREASE as the robot is tipped UP at the left side. (Rotation about Y)
//Yaw value should INCREASE as the robot is rotated Counter Clockwise. (Rotation about Z)

@TeleOp(name="IMU Orientation", group="Utility")
public class IMUOrientation extends LinearOpMode {
    static RevHubOrientationOnRobot.LogoFacingDirection[] logoFacingDirections = RevHubOrientationOnRobot.LogoFacingDirection.values();
    static RevHubOrientationOnRobot.UsbFacingDirection[] usbFacingDirections = RevHubOrientationOnRobot.UsbFacingDirection.values();

    static int LAST_DIRECTION = logoFacingDirections.length - 1;
    static float TRIGGER_THRESHOLD = 0.2f;

    IMU imu;

    int logoFacingDirectionPosition;
    int usbFacingDirectionPosition;
    boolean orientationIsValid = true;

    @Override public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        logoFacingDirectionPosition = 0; // Up
        usbFacingDirectionPosition = 2; // Forward

        updateOrientation();

        boolean justChangedLogoDirection = false;
        boolean justChangedUsbDirection = false;

        while (!isStopRequested()) {
            if (gamepad1.start) {
                telemetry.addData("Yaw", "Resetting\n");
                imu.resetYaw();
            } else {
                telemetry.addData("Yaw", "Press Start on Gamepad to reset.\n");
            }

            if (gamepad1.left_bumper || gamepad1.right_bumper) {
                if (!justChangedLogoDirection) {
                    justChangedLogoDirection = true;

                    if (gamepad1.left_bumper) {
                        logoFacingDirectionPosition--;
                        if (logoFacingDirectionPosition < 0) {
                            logoFacingDirectionPosition = LAST_DIRECTION;
                        }
                    } else {
                        logoFacingDirectionPosition++;
                        if (logoFacingDirectionPosition > LAST_DIRECTION) {
                            logoFacingDirectionPosition = 0;
                        }
                    }

                    updateOrientation();
                }
            } else {
                justChangedLogoDirection = false;
            }

            if (gamepad1.left_trigger > TRIGGER_THRESHOLD || gamepad1.right_trigger > TRIGGER_THRESHOLD) {
                if (!justChangedUsbDirection) {
                    justChangedUsbDirection = true;

                    if (gamepad1.left_trigger > TRIGGER_THRESHOLD) {
                        usbFacingDirectionPosition--;
                        if (usbFacingDirectionPosition < 0) {
                            usbFacingDirectionPosition = LAST_DIRECTION;
                        }
                    } else {
                        usbFacingDirectionPosition++;
                        if (usbFacingDirectionPosition > LAST_DIRECTION) {
                            usbFacingDirectionPosition = 0;
                        }
                    }

                    updateOrientation();
                }
            } else {
                justChangedUsbDirection = false;
            }

            telemetry.addData("Logo Direction (cycle with bumpers)", logoFacingDirections[logoFacingDirectionPosition]);
            telemetry.addData("USB Direction (cycle with triggers)", usbFacingDirections[usbFacingDirectionPosition] + "\n");

            if (orientationIsValid) {
                YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
                AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
                telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
                telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
                telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
                telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
                telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
            } else {
                telemetry.addData("Error", "Selected orientation on robot is invalid");
            }

            telemetry.update();
        }
    }

    void updateOrientation() {
        RevHubOrientationOnRobot.LogoFacingDirection logo = logoFacingDirections[logoFacingDirectionPosition];
        RevHubOrientationOnRobot.UsbFacingDirection usb = usbFacingDirections[usbFacingDirectionPosition];

        try {
            RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            orientationIsValid = true;
        } catch (IllegalArgumentException e) {
            orientationIsValid = false;
        }
    }
}
