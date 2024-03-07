// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    // Subsystems
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();

    final CommandXboxController driverXbox = new CommandXboxController(0);

    // Create Commands

    // Intake

    // Shoot
    Command shoot = Commands.sequence(
            Commands.runOnce(shooter::spoolShooter, shooter),
            Commands.waitUntil(shooter::isAtSpeed),
            Commands.runOnce(intake::startIntake, intake),
            Commands.runOnce(intake::runFeeder, intake),
            Commands.waitSeconds(2).withTimeout(5)
                    .andThen(
                            () -> {
                                intake.stopFeeder();
                                intake.stopIntake();
                                shooter.stopShooter();
                            }));

    public RobotContainer() {
        configureBindings();

        AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND),
                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND),
                () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                        OperatorConstants.RIGHT_X_DEADBAND),
                driverXbox.getHID()::getYButtonPressed,
                driverXbox.getHID()::getAButtonPressed,
                driverXbox.getHID()::getXButtonPressed,
                driverXbox.getHID()::getBButtonPressed);
        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the desired angle NOT angular rotation
        Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
                () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> -driverXbox.getRightX(),
                () -> -driverXbox.getRightY());

        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // right stick controls the angular velocity of the robot
        Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
                () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> -driverXbox.getRightX());

        Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
                () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> -driverXbox.getRawAxis(2));

        drivebase.setDefaultCommand(
                !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);
    }

    private void configureBindings() {
        driverXbox.y().onTrue((Commands.runOnce(drivebase::zeroGyro)));
        driverXbox.rightBumper().onTrue(shoot).debounce(3);
        driverXbox.x().onTrue(Commands.runOnce(intake::startIntake, intake))
                .onFalse(Commands.runOnce(intake::stopIntake, intake));

        driverXbox.b().whileTrue(
                Commands.deferredProxy(() -> drivebase.driveToPose(
                        new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
