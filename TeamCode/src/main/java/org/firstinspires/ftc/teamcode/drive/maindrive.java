package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@TeleOp(name = "shoot + color sensor", group = "2024-2025 Test OP")
public class maindrive extends LinearOpMode {


    private DcMotor GT;
    private DcMotorEx SL, SR;
    private DcMotor FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor;


    private Servo servo_L, servo_R;

    private boolean shooter_status = false;
    private int shooter_speed_status = 0;
    private int target_speed = 0;

    private ColorSensor colorSensor_L, colorSensor_R;
    private Servo light_L, light_R;

    public static int high_speed = 0;
    public static int low_speed = 0;


    private IMU imu;


    private final int GREEN_DIFF = 15;
    private final int PURPLE_DIFF = 20;

    @Override
    public void runOpMode() throws InterruptedException {


        SL = hardwareMap.get(DcMotorEx.class, "SL");
        SR = hardwareMap.get(DcMotorEx.class, "SR");
        GT = hardwareMap.dcMotor.get("GT");

        SL.setDirection(DcMotorSimple.Direction.REVERSE);
        SR.setDirection(DcMotorSimple.Direction.FORWARD);
        GT.setDirection(DcMotorSimple.Direction.REVERSE);

        SL.setPower(0);
        SR.setPower(0);
        GT.setPower(0);

        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        GT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeftMotor = hardwareMap.dcMotor.get("FL");
        FrontRightMotor = hardwareMap.dcMotor.get("FR");
        BackLeftMotor = hardwareMap.dcMotor.get("BL");
        BackRightMotor = hardwareMap.dcMotor.get("BR");

        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //BackRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);


        servo_L = hardwareMap.servo.get("servo_L");
        servo_R = hardwareMap.servo.get("servo_R");
        servo_L.setDirection(Servo.Direction.REVERSE);


        colorSensor_L = hardwareMap.get(ColorSensor.class, "colorSensor_L");
        colorSensor_R = hardwareMap.get(ColorSensor.class, "colorSensor_R");

        light_L = hardwareMap.servo.get("light_L");
        light_R = hardwareMap.servo.get("light_R");

        try { colorSensor_L.enableLed(true); } catch (Exception ignored) {}
        try { colorSensor_R.enableLed(true); } catch (Exception ignored) {}


        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        SL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        SL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        com.qualcomm.robotcore.hardware.PIDFCoefficients flywheel_pidfCoeffiients
                = new com.qualcomm.robotcore.hardware
                .PIDFCoefficients(700, 100, 400, 0);

        SL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, flywheel_pidfCoeffiients);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        boolean shooterOn = false;
        double power = 0.78;
        double step = 0.02;

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;
            double slow = 1 - (0.8 * gamepad1.right_trigger);

            if (gamepad1.options) imu.resetYaw();

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            FrontLeftMotor.setPower((rotY + rotX - rx) / denominator * slow);
            BackLeftMotor.setPower((rotY - rotX - rx) / denominator * slow);
            FrontRightMotor.setPower((rotY - rotX + rx) / denominator * slow);
            BackRightMotor.setPower((rotY + rotX + rx) / denominator * slow);


            if (rising_edge(currentGamepad1.dpad_up, previousGamepad1.dpad_up))
                power = Math.min(1.0, power + step);
            if (rising_edge(currentGamepad1.dpad_down, previousGamepad1.dpad_down))
                power = Math.max(0.0, power - step);

//            if (rising_edge(currentGamepad1.x, previousGamepad1.x))
//                shooterOn = true;
//            if (rising_edge(currentGamepad1.y, previousGamepad1.y))
//                shooterOn = false;
//
//            if (shooterOn) {
//                SL.setPower(power);
//                SR.setPower(power);
//            } else {
//                SL.setPower(0);
//                SR.setPower(0);
//            }

            if (rising_edge(currentGamepad1.a, previousGamepad1.a))
                GT.setPower(0.6);
            if (rising_edge(currentGamepad1.b, previousGamepad1.b))
                GT.setPower(0);


            double rightBumperPosition = gamepad1.right_bumper ? 0.9 : 0.5;
            double leftBumperPosition = gamepad1.left_bumper ? 0.9 : 0.5;

            servo_R.setPosition(rightBumperPosition);
            servo_L.setPosition(leftBumperPosition);

            if (!shooter_status) {
                if (gamepad1.xWasPressed()) shooter_status = true;
                SL.setVelocity(target_speed);
                SR.setPower(SL.getPower());
            }

            if (shooter_status) {
                if (gamepad1.yWasPressed()) shooter_status = false;
                SL.setVelocity(0);
                SR.setPower(0);
            }

            switch (shooter_speed_status) {
                case 0:
                    target_speed = low_speed;
                    if (gamepad1.dpadUpWasPressed()) shooter_speed_status = 1;
                    break;

                case 1:
                    target_speed = high_speed;
                    if (gamepad1.dpadDownWasPressed()) shooter_speed_status = 0;
                    break;
            }


            int red_L = colorSensor_L.red();
            int green_L = colorSensor_L.green();
            int blue_L = colorSensor_L.blue();
            String detectedColor_L = detectColor(red_L, green_L, blue_L);
            setServo(light_L, detectedColor_L);

            int red_R = colorSensor_R.red();
            int green_R = colorSensor_R.green();
            int blue_R = colorSensor_R.blue();
            String detectedColor_R = detectColor(red_R, green_R, blue_R);
            setServo(light_R, detectedColor_R);


            telemetry.addData("Shooter Power", "%.2f", power);
            telemetry.addData("Shooter On", shooterOn);
            telemetry.addData("GT Power", GT.getPower());
            telemetry.addData("servo_L pos", servo_L.getPosition());
            telemetry.addData("Heading (deg)", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Left Sensor", detectedColor_L + " [R:%d G:%d B:%d]", red_L, green_L, blue_L);
            telemetry.addData("Right Sensor", detectedColor_R + " [R:%d G:%d B:%d]", red_R, green_R, blue_R);
            telemetry.update();

            sleep(50);
        }


        SL.setPower(0);
        SR.setPower(0);
        GT.setPower(0);
    }


    private String detectColor(int red, int green, int blue) {
        if (isGreen(red, green, blue)) return "GREEN";
        else if (isPurple(red, green, blue)) return "PURPLE";
        else return "UNKNOWN";
    }

    private void setServo(Servo servo, String detectedColor) {
        switch (detectedColor) {
            case "GREEN":
                servo.setPosition(0.5);
                break;
            case "PURPLE":
                servo.setPosition(0.722);
                break;
            default:
                servo.setPosition(0);
                break;
        }
    }

    private boolean isGreen(int red, int green, int blue) {
        return (green > red && green > blue) &&
                (green - red > GREEN_DIFF) &&
                (green - blue > GREEN_DIFF) &&
                (green - red > 100);
    }

    private boolean isPurple(int red, int green, int blue) {
        return (blue > red && blue > green) &&
                ((blue - red > PURPLE_DIFF) || (blue - green > PURPLE_DIFF));
    }

    private boolean rising_edge(boolean current, boolean previous) {
        return current && !previous;
    }
}
