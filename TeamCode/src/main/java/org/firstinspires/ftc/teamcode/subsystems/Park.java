package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Park {
    public Servo parkServo1, parkServo2;

    public Park(HardwareMap hwMap) {
        parkServo1 = hwMap.get(Servo.class, "p1");
        parkServo2 = hwMap.get(Servo.class, "p2");
    }

    public void tilt() {
        parkServo1.setPosition(0.23);
        parkServo2.setPosition(0.23);
    }

    public void untilt() {
        parkServo1.setPosition(0.5);
        parkServo2.setPosition(0.5);
    }

}
