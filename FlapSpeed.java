package frc.robot.commands;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class FlapSpeed {
    public static void main(String[] args) {
        try (SparkFlex motor = new SparkFlex(16, MotorType.kBrushless);) {
            motor.set(1);
        }

    }
}
