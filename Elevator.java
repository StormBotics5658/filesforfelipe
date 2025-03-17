package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    private final SparkFlex elevatorleader = new SparkFlex(17, MotorType.kBrushless);
    private final SparkFlex elevatorfollower = new SparkFlex(18, MotorType.kBrushless);

    public Elevator() {
        SparkFlexConfig motorConfig = new SparkFlexConfig();
        motorConfig.idleMode(IdleMode.kBrake);
        elevatorleader.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorConfig.follow(elevatorleader, true);
        elevatorfollower.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void runelevator(double speed) {
        elevatorleader.set(speed);
    }
}