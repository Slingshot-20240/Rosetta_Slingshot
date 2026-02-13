package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Intakenf implements Subsystem {

    // ------------------ INSTANCES ------------------ //

    public static final Intakenf INSTANCE = new Intakenf();

    private Intakenf() { }

    public MotorEx intake;

    public ServoEx intakeLift;


// --- Intake --- //
    public Command in() {
        return new InstantCommand(() -> intake.getMotor().setPower(1.0));
    }
    public Command idle() {
        return new InstantCommand(() -> intake.getMotor().setPower(0));
    }
    public Command out() {
        return new InstantCommand(() -> intake.getMotor().setPower(-1.0));
    }

// --- Intake Lift --- //
    public Command down() {
        return new InstantCommand(() -> intakeLift.getServo().setPosition(0.4)).requires(this);
    }
    public Command up() {
        return new InstantCommand(() -> intakeLift.getServo().setPosition(0.2)).requires(this);
    }


    public Command intakeState() {
        return new SequentialGroup(
                in(),
                down()
        );
    }
    public Command goToScoreState() {
        return new SequentialGroup(
                in(),
                down()
        );
    }

    @Override
    public void initialize() {
        intake = new MotorEx("intake");
        intakeLift = new ServoEx("iL",-0.1);
    }

    @Override
    public void periodic() {}


}
