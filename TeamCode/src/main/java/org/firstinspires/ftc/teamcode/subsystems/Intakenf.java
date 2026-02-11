package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Intakenf implements Subsystem {
    public static final Intakenf INSTANCE = new Intakenf();
    private Intakenf() { }

    public MotorEx intake;
    public ServoEx intakeLift;


    //Intake Commands
    public Command in() {
        return new SetPower(intake, 1.0);
    }
    public Command idle() {
        return new SetPower(intake, 0);
    }
    public Command out() {
        return new SetPower(intake, -1.0);
    }

    //Intake Lift Commands
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
}
