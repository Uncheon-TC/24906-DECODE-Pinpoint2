package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "decode 23020", group = "2024-2025 Test OP")
public class maindrive extends LinearOpMode {

    private DcMotor FrontLeftMotor, FrontRightMotor, BackLeftMotor, BackRightMotor;
    private DcMotor eat, SL, SR;
    private Servo servo_S;
    private IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {


        FrontLeftMotor  = hardwareMap.dcMotor.get("FL");
        FrontRightMotor = hardwareMap.dcMotor.get("FR");
        BackLeftMotor   = hardwareMap.dcMotor.get("BL");
        BackRightMotor  = hardwareMap.dcMotor.get("BR");

        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        eat = hardwareMap.dcMotor.get("eat");
        SL  = hardwareMap.dcMotor.get("SL");
        SR  = hardwareMap.dcMotor.get("SR");

        eat.setDirection(DcMotorSimple.Direction.REVERSE);
        SL.setDirection(DcMotorSimple.Direction.FORWARD);
        SR.setDirection(DcMotorSimple.Direction.REVERSE);

        eat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        eat.setPower(0);
        SL.setPower(0);
        SR.setPower(0);


        servo_S = hardwareMap.servo.get("servo_S");
        servo_S.setPosition(0.5); // 기본 위치


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        waitForStart();

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double slow = 1 - (0.8 * gamepad1.right_trigger);

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1;

            double denominator = Math.max(
                    Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1
            );

            FrontLeftMotor.setPower((rotY + rotX - rx) / denominator * slow);
            BackLeftMotor.setPower((rotY - rotX - rx) / denominator * slow);
            FrontRightMotor.setPower((rotY - rotX + rx) / denominator * slow);
            BackRightMotor.setPower((rotY + rotX + rx) / denominator * slow);


            servo_S.setPosition(gamepad1.left_bumper ? 0.3 : 0.5);


            if (rising_edge(currentGamepad1.a, previousGamepad1.a)) {
                eat.setPower(0.8);
            }

            if (rising_edge(currentGamepad1.b, previousGamepad1.b)) {
                eat.setPower(0);
            }

            if (rising_edge(currentGamepad1.x, previousGamepad1.x)) {
                SL.setPower(0.8);
                SR.setPower(0.8);
            }

            if (rising_edge(currentGamepad1.y, previousGamepad1.y)) {
                SL.setPower(0);
                SR.setPower(0);
            }

            telemetry.addData("eat Power", eat.getPower());
            telemetry.addData("SL Power", SL.getPower());
            telemetry.addData("SR Power", SR.getPower());
            telemetry.addData("Servo_S Pos", servo_S.getPosition());
            telemetry.addData("Heading (deg)",
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            sleep(50);
        }

        eat.setPower(0);
        SL.setPower(0);
        SR.setPower(0);
    }

    private boolean rising_edge(boolean current, boolean previous) {
        return current && !previous;
    }
}
