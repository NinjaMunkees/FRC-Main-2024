package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    // Main Arm
    private final TalonFX m_Forearm = new TalonFX(9);

    // Grippers
    private final CANSparkMax m_Top_Grippy = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax m_Bottom_Grippy = new CANSparkMax(4, MotorType.kBrushless);

}
