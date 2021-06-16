package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Main Program", group="Main")
public class MainProgram extends LinearOpMode {
    private BNO055IMU InitImuSensor () {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        BNO055IMU configuredImu = hardwareMap.get(BNO055IMU.class, "imu");
        configuredImu.initialize(parameters);
        return configuredImu;
    }

    private DcMotor InitDcMotor (String motorName, DcMotorSimple.Direction direction, DcMotor.RunMode runMode) {
        DcMotor motor = hardwareMap.get(DcMotor.class, motorName);
        motor.setDirection(direction);
        motor.setMode(runMode);
        return motor;
    }

    DcMotor leftMotor, rightMotor;

    @Override public void runOpMode() {
        // Init hardware stuff
        BNO055IMU imu = InitImuSensor();
        leftMotor = InitDcMotor("left_drive", DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor = InitDcMotor("right_drive", DcMotorSimple.Direction.REVERSE, DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait until we're told to go
        waitForStart();

        // Loop and update the dashboard
        while (opModeIsActive()) {
            // Get user input
            double drive = gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;

            double robotSpeed = gamepad1.right_trigger != 0 ? 1 : 0.5 ;

            double  leftPower    = (drive - turn) * robotSpeed,
                    rightPower   = (drive + turn) * robotSpeed;
            
            leftMotor.setPower(leftPower);
            rightMotor.setPower(rightPower);

            ShowTeletryData(imu);
        }
    }

    public void ShowTeletryData (BNO055IMU imu) {
        telemetry.addLine()
                .addData("Left Trigger", gamepad1.left_trigger)
                .addData("Left Trigger", gamepad1.right_trigger);

        // Update data on user's screen
        telemetry.update();
    }
}