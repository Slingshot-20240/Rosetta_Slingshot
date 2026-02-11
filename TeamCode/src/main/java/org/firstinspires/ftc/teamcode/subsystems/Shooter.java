package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Robot;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import org.firstinspires.ftc.teamcode.subsystems.Logi;

@Configurable
public class Shooter implements Subsystem {

    // ------------------ INSTANCES ------------------ //

    public static final Shooter INSTANCE = new Shooter();

    private Shooter() { }

    public MotorEx outtake1, outtake2;

    public MotorGroup shooter;

    private final ControlSystem shooterController = ControlSystem.builder()
            .velPid(3.5, 0, 0.002)
            .basicFF(0.001)
            .build();

    private boolean enabled = false;

    // ------------------ AUTONOMOUS COMMANDS ------------------ //

    public Command closeSide() {
        return new RunToVelocity(shooterController, -1210).requires(shooter);
    }
    public Command farSide() {
        return new RunToVelocity(shooterController, -1450).requires(shooter);
    }
    public Command idle() {
        return new RunToVelocity(shooterController, 0).requires(shooter);
    }

    public Command setShooterVel(double shooterVel) {
        return new RunToVelocity(shooterController,shooterVel).requires(shooter);
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
        shooter.setPower(0);
    }

    @Override
    public void initialize() {
        outtake1 = new MotorEx("outtakeTop");
        outtake2 = new MotorEx("outtakeBot");
        outtake2.reverse();
        shooter = new MotorGroup(outtake1, outtake2);

        disable();
    }

    @Override
    public void periodic() {
        if (!enabled) {
            shooter.setPower(0);
            return;
        }
        shooter.setPower(shooterController.calculate(shooter.getState()));
    }

    // ------------------ TELEOP COMMANDS ------------------ //

    public Command instanceSetVelo(double velocity) {
        return new ParallelGroup(
                new InstantCommand(() -> outtake1.getMotor().setVelocity(velocity)),
                new InstantCommand(() -> outtake2.getMotor().setVelocity(velocity))
        );
    }

    public Command instanceShootBack() {
        return new ParallelGroup(
                new InstantCommand(() -> outtake1.getMotor().setVelocity(-1500)),
                new InstantCommand(() -> outtake2.getMotor().setVelocity(-1500))
        );
    }

    public Command instanceShootFront() {
        return new ParallelGroup(
                new InstantCommand(() -> outtake1.getMotor().setVelocity(-1000)),
                new InstantCommand(() -> outtake2.getMotor().setVelocity(-1000))
        );
    }

    // ------------------ MATH ------------------ //

    private static final double g = 9.79284;
    // y distance to goal - to the middle of the shooter
    private static final double H = 0.360837;

    private static double shootVel;

    // calculates target velocity with GIVEN distance in INCHES from the goal
    public static double calculateShooterMPS(double d) {
        double R = d * 0.0254;
        shootVel = Math.sqrt(H * g + g * Math.sqrt(Math.pow(R, 2) + Math.pow(H, 2)));
        return shootVel;
    }

    // calculates target velocity with CURRENT distance away from the goal
    public double calculateShooterMPS() {
        return calculateShooterRPM(Logi.INSTANCE.getTargetArtifactTravelDistanceX());
    }

    // calculates target velocity in TICKS PER SECOND instead of meters per second
    public double calculateShooterRPM(double d) {
        return convertMPSToRPM(calculateShooterMPS(d));
    }

    // converts the target velocity from meters per second to rpm for DcMotor
    public static double convertMPSToRPM(double mpsVel) {
        double c = -2.28611;
        double a = 12.79622;
        double b = 0.000790302;
        double lnArgument = Math.abs(1.0 - ((mpsVel - c) / a));
        return -Math.log(lnArgument) / b; // Math.log is natural logarithm
    }

    // HOOD ANGLE CALCULATIONS
    // ---------------------------------
    private static double hoodAngle;

    // returns the target angle in RADIANS depending on GIVEN distance in INCHES from the april tag
    public static double calculateHoodAngle(double d) {
        double R = d * 0.0254;
        hoodAngle = Math.atan(Math.pow(calculateShooterMPS(d), 2) / (g * R));
        return hoodAngle;
    }

    // returns the target angle in RADIANS depending on CURRENT distance from the april tag
    public double calculateHoodAngle() {
        return calculateHoodAngle(Logi.INSTANCE.getTargetArtifactTravelDistanceX());
    }

    // returns the target angle in HOOD POS (0-1) instead of radians
    public double calculateHoodPos(double d) {
        return convertTargetAngleToHoodPos(calculateHoodAngle(d));
    }

    // converts the target angle from calculateHoodAngle() to a servo position from 0-1
    public static double convertTargetAngleToHoodPos(double targetAngle) {
        double m = 42.8718;
        double b = 37.09643;
        return (Math.toDegrees(targetAngle) - b) / m;
    }
}
