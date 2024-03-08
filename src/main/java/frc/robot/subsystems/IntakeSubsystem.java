package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    // Intake Motor
    private final CANSparkMax m_intake = new CANSparkMax(6, MotorType.kBrushless);
    // Feeder Motor
    private final CANSparkMax m_feeder = new CANSparkMax(7, MotorType.kBrushless);

    // Vars
    private final double INTAKE_SPEED = 0.8;
    private final double REV_INTAKE_SPEED = -0.8;

    private final double FEED_SPEED = 1;
    private final double REV_FEED_SPEED = -1;

    public void startIntake() {
        m_intake.set(INTAKE_SPEED);
    }

    public void reverseIntake() {
        m_intake.set(REV_INTAKE_SPEED);
    }

    public void stopIntake() {
        m_intake.set(0);
    }

    public void runFeeder() {
        m_feeder.set(FEED_SPEED);
    }

    public void reverseFeeder() {
        m_feeder.set(REV_FEED_SPEED);
    }

    public void stopFeeder() {
        m_feeder.set(0);
    }
}
