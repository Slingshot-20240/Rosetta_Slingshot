package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Intake implements Subsystem {

    // ------------------ INSTANCES ------------------ //

    public static final Intake INSTANCE = new Intake();

    private Intake() { }

    public MotorEx intake;

    public ServoEx intakeLift;

    // ------------------ AUTONOMOUS COMMANDS ------------------ //

    // --- Intake --- //
    public Command in() {
        return new SetPower(intake, 1.0);
    }
    public Command idle() {
        return new SetPower(intake, 0);
    }
    public Command out() {
        return new SetPower(intake, -1.0);
    }

    // --- Intake Lift --- //
    public Command down() {
        return new SetPosition(intakeLift, 0.4).requires(this);
    }
    public Command up() {
        return new SetPosition(intakeLift, 0.2).requires(this);
    }

    @Override
    public void initialize() {
        intake = new MotorEx("intake");
        intakeLift = new ServoEx("iL",-0.1);
    }

    @Override
    public void periodic() {}

    // ------------------ TELEOP COMMANDS ------------------ //

    public Command instanceIntakeOn() {
        return new InstantCommand(() -> intake.getMotor().setPower(1));
    }

    public Command instanceIntakeOff() {
        return new InstantCommand(() -> intake.getMotor().setPower(0));
    }

    public Command instanceIntakeReverse() {
        return new InstantCommand(() -> intake.getMotor().setPower(-1));
    }

    public Command instanceLiftDown() {
        return new InstantCommand(() -> intakeLift.getServo().setPosition(1));
    }

    public Command instanceLiftUp() {
        return new InstantCommand(() -> intakeLift.getServo().setPosition(0));
    }
}
