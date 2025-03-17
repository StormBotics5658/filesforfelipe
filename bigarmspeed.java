package frc.robot.commands;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class bigarmspeed{
    public static void main(String[] args) {
        try (SparkFlex motor = new SparkFlex(19, MotorType.kBrushless);) {
            motor.set(0.30);
        }

    }
}
