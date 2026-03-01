package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Park {
    public Servo parkServo1;
    public Servo parkServo2;

    public Park(HardwareMap hwMap) {
        parkServo1 = hwMap.get(Servo.class, "park1");
        parkServo2 = hwMap.get(Servo.class, "park2");
    }

    public void tilt() {
        parkServo1.setPosition(0.23);
        parkServo2.setPosition(0.23);
    }

    public void unTilt() {
        parkServo1.setPosition(0.5);
        parkServo2.setPosition(0.5);
    }

}
