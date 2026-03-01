package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.Controllable;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.VoltageCompensatingMotor;

@Configurable
public class Shooternf implements Subsystem {
    public static final Shooternf INSTANCE = new Shooternf();
    private Shooternf() { }

    private final MotorEx outtake1 = new MotorEx("outtakeL").reversed();
    private final MotorEx outtake2 = new MotorEx("outtakeR");
    private final Controllable outtakeL = new VoltageCompensatingMotor(outtake1, 0.01, 13);
    private final Controllable outtakeR = new VoltageCompensatingMotor(outtake2, 0.01, 13);
    public MotorGroup shooter = new MotorGroup(outtakeL, outtakeR);
;

    private final ControlSystem shooterController = ControlSystem.builder()
            .velPid(3.5, 0, 0.002)
            .basicFF(0.001)
            .build();

    private boolean enabled = false;


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
