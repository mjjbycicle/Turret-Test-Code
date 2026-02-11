package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.compound.Diff_PositionVoltage_Position;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.TunableNumber;

public class Turret extends SubsystemBase {
    public final SparkFlex leftShooter, rightShooter;
    private final TalonFX hoodMotor;
    private final CANcoder hoodEncoder;

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
        hoodMotor = new TalonFX(TurretConstants.hoodMotorID);
        hoodMotor.setNeutralMode(NeutralModeValue.Brake);
        var hoodMotorConfig = new TalonFXConfiguration()
            .withSlot0(
                new Slot0Configs()
                    .withKP(TurretConstants.hoodKp.getAsDouble())   
                    .withKI(TurretConstants.hoodKi.getAsDouble())   
                    .withKD(TurretConstants.hoodKd.getAsDouble())   
            ).withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
            );
        hoodMotor.getConfigurator().apply(hoodMotorConfig);
        hoodEncoder = new CANcoder(TurretConstants.hoodEncoderID);
        hoodEncoder.getConfigurator().apply(
            new CANcoderConfiguration()
                .withMagnetSensor(
                    new MagnetSensorConfigs()
                        .withMagnetOffset(TurretConstants.hoodEncoderOffset)
                )     
        );
        hoodMotor.getConfigurator().apply(
            new FeedbackConfigs()
                .withFeedbackRemoteSensorID(hoodEncoder.getDeviceID())
                .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        );
    }

    public void runVoltage(double volts) {
        rightShooter.setVoltage(volts);
    }

    public Command runVoltageCommand(DoubleSupplier volts) {
        return this.run(
            () -> runVoltage(volts.getAsDouble())
        ).finallyDo(
            () -> rightShooter.setVoltage(0)
        );
    }

    public void periodic() {
        boolean refresh = false;
        for (TunableNumber t : TurretConstants.tunableNumbers) {
            if (t.hasChanged()) {
                refresh = true;
                break;
            }
        }
        if (refresh) {
            hoodMotor.getConfigurator().apply(
                new Slot0Configs()
                    .withKP(TurretConstants.hoodKp.getAsDouble())   
                    .withKI(TurretConstants.hoodKi.getAsDouble())   
                    .withKD(TurretConstants.hoodKd.getAsDouble())   
            );
        }
        SmartDashboard.putNumber("Turret/Hood Voltage", hoodMotor.getMotorVoltage().getValueAsDouble());
    }

    public void runHood(double hoodAngle) {
        double hoodAngleClamped = MathUtil.clamp(hoodAngle, TurretConstants.minHoodAngle, TurretConstants.maxHoodAngle);
        hoodMotor.setControl(new PositionVoltage(Rotations.of(hoodAngleClamped)));
    }

    public Command runHoodCommand(DoubleSupplier hoodAngle) {
        return this.run(
            () -> runHood(hoodAngle.getAsDouble())
        ).finallyDo(
            () -> hoodMotor.stopMotor()
        );
    }

    public Command runHoodAndShooterCommand(DoubleSupplier volts, DoubleSupplier hoodAngle) {
        return this.run(
            () -> {
                runHood(hoodAngle.getAsDouble());
                runVoltage(volts.getAsDouble());
            }
        ).finallyDo(
            () -> {
                rightShooter.setVoltage(0);
                hoodMotor.stopMotor();
            }
        );
    }
}
