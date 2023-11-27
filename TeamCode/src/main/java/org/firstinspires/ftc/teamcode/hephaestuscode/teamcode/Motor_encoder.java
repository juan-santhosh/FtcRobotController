import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Motor_encoder extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {
        MotorEx rightspool = new MotorEx(hardwareMap, "rightspool");
        MotorEx leftspool = new MotorEx(hardwareMap, "leftspool");
        rightspool.setInverted(true);
        telemetry.addData("spoolpos",rightspool.getCurrentPosition());
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            rightspool.set(gamepad2.left_stick_y);
            leftspool.set(gamepad2.left_stick_y);
            telemetry.addData("spoolpos",rightspool.getCurrentPosition());
        }
    }
}