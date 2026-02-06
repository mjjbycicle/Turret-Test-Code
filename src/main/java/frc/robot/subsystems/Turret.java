package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    public final SparkFlex leftShooter, rightShooter;

    public Turret() {
        leftShooter = new SparkFlex(10, MotorType.kBrushless);
        rightShooter = new SparkFlex(11, MotorType.kBrushless);
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(40)
            .idleMode(IdleMode.kCoast)
            .follow(11, true);
        leftShooter.configure(
            config,
            ResetMode.kResetSafeParameters, 
            PersistMode.kNoPersistParameters
        );
    }

    public Command runVoltage(DoubleSupplier volts) {
        return this.run(
            () -> {
                rightShooter.setVoltage(volts.getAsDouble());
            }
        ).finallyDo(
            () -> rightShooter.setVoltage(0)
        );
    }
}
