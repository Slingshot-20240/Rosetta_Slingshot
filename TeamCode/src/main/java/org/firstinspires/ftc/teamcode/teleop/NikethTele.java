package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.NextFTC.autonomous.PoseStorage;
import org.firstinspires.ftc.teamcode.misc.PIDFControllerEx;
import org.firstinspires.ftc.teamcode.teleop.gamepad.GamepadMapping;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Config
@TeleOp
public class NikethTele extends OpMode {

    private GamepadMapping controls;
    private FSM fsm;
    private Robot robot;
    private Follower follower;
    private TelemetryManager telemetryM;

    public static double tolerance = 0.01;

    // Vision tuning
    public static double visionTurn_kP = 0.04;
    public static double visionMinTurnPower = 0.07;
    public static double visionMiniTolerance = 0.01;

    public static double kP = 0.05;

    // Bee tuning
    private double targetAngle = 0;
    public static double turnP = 0.013;
    public static double turnI = 0;
    public static double turnD = 0.0001;
    public static double turnF = 0;
    private PIDFControllerEx turnController = new PIDFControllerEx(turnP, turnI, turnD, turnF);

    @Override
    public void init() {

        controls = new GamepadMapping(gamepad1, gamepad2);
        robot = new Robot(hardwareMap, controls);
        fsm = new FSM(hardwareMap, controls, robot);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(PoseStorage.startingPose);
        //follower.setStartingPose(new Pose(126.2, 119, Math.toRadians(36)));
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


    }

    @Override
    public void start() {
        //robot.hardwareSoftReset();
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {

        fsm.update();
        follower.update();
        telemetryM.update();
        telemetry.update();

//        double visionBearing = Math.toRadians(Robot.cam.getATangle());
//        double visionError = Math.abs(visionBearing);
        Pose pose = follower.getPose();
        double heading = pose.getHeading();


        boolean controllerBusy =
                Math.abs(gamepad1.left_stick_x) > 0.05 ||
                        Math.abs(gamepad1.left_stick_y) > 0.05 ||
                        Math.abs(gamepad1.right_stick_x) > 0.05;


    //Drive Controls ----------------------------------------------------------------
        double forward = -Math.pow(gamepad1.left_stick_y, 5);
        double strafe  = -Math.pow(gamepad1.left_stick_x, 5);
        double rotate = 0;

//        //TODO - try holding and also pressing
//        if (gamepad1.aWasPressed()) {
//            follower.turn(Math.toRadians(visionBearing));
//
//        } else if (controllerBusy) {
//            if (gamepad1.right_bumper) {
//                follower.setTeleOpDrive(forward, strafe, rotate, true);
//            } else {
//                follower.setTeleOpDrive(forward, 0, rotate, true);
//            }
//
//        }


//    //Auto Align
//        if (gamepad1.right_trigger > 0.1) {
//            rotate = visionBearing * kP;
//        } else {
//            rotate = -Math.pow(gamepad1.right_stick_x, 5);
//        }
//        rotate = Math.max(-1.0, Math.min(1.0, rotate));

        //------------- error calculation -------------\\
        // Vision error
        double visionBearing = Math.toRadians(Robot.cam.getATangle());
        double visionHeadingError = angleWrap(visionBearing);
        boolean visionTurnFinished =
                Math.abs(visionHeadingError) < tolerance;
        //------------- rotate logic -------------\\

        double error;
        double kP;
        double minPower;
        double miniTolerance;

        error = visionHeadingError;
        kP = visionTurn_kP;
        minPower = visionMinTurnPower;
        miniTolerance = visionMiniTolerance;

        if (gamepad1.a) {
            forward = 0;
            strafe = 0;

            rotate = error * kP;

            if (Math.abs(rotate) < minPower && Math.abs(error) > miniTolerance) {
                rotate = Math.signum(rotate) * minPower;
            }
        } else {
            rotate = -gamepad1.right_stick_x * 0.55;
        }

        // bee rotate logic

        //Tank-Mecanum Override
        if (gamepad1.left_trigger > 0.1) {
            follower.setTeleOpDrive(forward, strafe, rotate, true);
        } else {
            follower.setTeleOpDrive(forward, 0, rotate, true);
        }

//        //Rumble Settings
//        if (Math.abs(visionHeadingError) < 1) {
//            gamepad1.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
//        } else {
//            gamepad1.stopRumble();
//        }

        // Telemetry
        telemetry.addData("Pose", pose.toString());
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
    }

    @Override
    public void stop() {
        PoseStorage.startingPose = follower.getPose();
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public double lockHeading(double targetAngle, double currentHeading) {
        double wrappedTarget = angleWrap(targetAngle);
        double error = angleWrap(targetAngle - currentHeading);
        double pid = turnController.calculate(error, false);
        double power = pid + turnF;
        return -power;
    }
}
