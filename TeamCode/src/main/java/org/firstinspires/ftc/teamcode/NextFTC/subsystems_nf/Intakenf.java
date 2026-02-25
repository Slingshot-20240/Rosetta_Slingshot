package org.firstinspires.ftc.teamcode.NextFTC.subsystems_nf;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

public class Intakenf implements Subsystem {

    public static final Intakenf INSTANCE = new Intakenf();

    private Intakenf() { }

    public MotorEx intake1;
    public MotorEx intake2;
    public MotorGroup intake = new MotorGroup(intake1, intake2);



    public Command in() {
        return new SetPower(intake, 1.0);
    }
    public Command idle() {
        return new SetPower(intake, 0);
    }
    public Command out() {
        return new SetPower(intake, -1.0);
    }



    @Override
    public void initialize() {
        intake1 = new MotorEx("intake1");
        intake2 = new MotorEx("intake2");
        intake1.reverse();
        intake = new MotorGroup(intake1, intake2);
    }

    @Override
    public void periodic() {}
    
}
