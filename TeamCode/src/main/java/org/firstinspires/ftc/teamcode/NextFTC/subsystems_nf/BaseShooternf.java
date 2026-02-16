package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class BaseShooternf implements Subsystem {
    public static final BaseShooternf INSTANCE = new BaseShooternf();
    private BaseShooternf() { }

    public MotorEx outtake1, outtake2;
    public MotorGroup shooter;

    private final ControlSystem shooterController = ControlSystem.builder()
            .velPid(3.5, 0, 0.002)
            .basicFF(0.001)
            .build();


    private boolean enabled = false;


    public Command closeSide() {
        return new RunToVelocity(shooterController, -1210).requires(shooter);
    }

    public Command setShooterVel(double shooterVel) {
        return new RunToVelocity(shooterController, shooterVel).requires(shooter);
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
        outtake1 = new MotorEx("outtakeL");
        outtake2 = new MotorEx("outtakeR");
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
}
