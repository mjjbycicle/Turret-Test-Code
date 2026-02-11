package frc.robot.subsystems;

import lib.TunableNumber;

public class TurretConstants {
    public static final TunableNumber hoodKp = new TunableNumber("Turret/Hood KP");
    public static final TunableNumber hoodKi = new TunableNumber("Turret/Hood Ki");
    public static final TunableNumber hoodKd = new TunableNumber("Turret/Hood Kd");
    public static final int hoodMotorID = 20;
    public static final int hoodEncoderID = 21;
    public static final double minHoodAngle = 0.018, maxHoodAngle = 0.92;
    public static final double hoodEncoderOffset = 0.1;

    static {
        hoodKp.setDefault(0);
        hoodKi.setDefault(0);
        hoodKd.setDefault(0);
    }

    public static final TunableNumber[] tunableNumbers = new TunableNumber[]{hoodKp, hoodKi, hoodKd};
}
