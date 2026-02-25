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

    public static final Intakenf INSTANCE = new Intakenf();

    private Intakenf() { }

    public MotorEx intake;


    public Command in() {
        return new InstantCommand(() -> intake.getMotor().setPower(1.0));
    }
    public Command idle() {
        return new InstantCommand(() -> intake.getMotor().setPower(0));
    }
    public Command out() {
        return new InstantCommand(() -> intake.getMotor().setPower(-1.0));
    }


    @Override
    public void initialize() {
        intake = new MotorEx("intake");
    }

    @Override
    public void periodic() {}


}
