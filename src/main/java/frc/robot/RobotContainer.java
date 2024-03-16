// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    // Subsystems
    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final ArmSubsystem arm = new ArmSubsystem();
    private final ClimbSubsystem climber = new ClimbSubsystem();

    //private final SendableChooser<Command> autoPicker;

    final CommandXboxController driverXbox = new CommandXboxController(0);
    final CommandJoystick m_stick = new CommandJoystick(0);

    // Create Commands

    // Intake

    // Shoot
    Command shoot = Commands.sequence(
            Commands.runOnce(shooter::spoolShooter, shooter),
            Commands.waitSeconds(1).andThen(
                () -> {
                        intake.startIntake();
                        intake.runFeeder();
                }
            ).withTimeout(2),
            Commands.waitSeconds(2)
                    .andThen(
                            () -> {
                                intake.stopFeeder();
                                intake.stopIntake();
                                shooter.stopShooter();
                            }).withTimeout(2));



        // Auto Shoot maybe?
        Command Auto = Commands.sequence(
                // Rotate 90 and move a little away
                //Commands.deferredProxy(() -> drivebase.driveToPose(new Pose2d(0, 0, new Rotation2d(70) ))),
                Commands.waitSeconds(2).andThen(
                        Commands.sequence(
                Commands.runOnce(shooter::spoolShooter, shooter),
                Commands.waitSeconds(1).andThen(
                        () -> {
                                intake.startIntake();
                                intake.runFeeder();
                        }
                ).withTimeout(2),
                Commands.waitSeconds(1)
                    .andThen(
                            () -> {
                                intake.stopFeeder();
                                intake.stopIntake();
                                shooter.stopShooter();
                            }).withTimeout(5)
                ).andThen(
                        Commands.print("Auto Done"),
                        Commands.deferredProxy(() -> drivebase.driveToPose(new Pose2d(2, 0, new Rotation2d(0) )))
                )
                )
                );

    public RobotContainer() {

        //autoPicker = AutoBuilder.buildAutoChooser();

        //SmartDashboard.putData("Which Auto Shall we do", autoPicker);

        configureBindings();

        // AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
        //         () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
        //                 OperatorConstants.LEFT_Y_DEADBAND),
        //         () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
        //                 OperatorConstants.LEFT_X_DEADBAND),
        //         () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
        //                 OperatorConstants.RIGHT_X_DEADBAND),
        //         driverXbox.getHID()::getYButtonPressed,
        //         driverXbox.getHID()::getAButtonPressed,
        //         driverXbox.getHID()::getXButtonPressed,
        //         driverXbox.getHID()::getBButtonPressed);
        // Applies deadbands and inverts controls because joysticks
        // are back-right positive while robot
        // controls are front-left positive
        // left stick controls translation
        // // right stick controls the desired angle NOT angular rotation
        // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        //         () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        //         () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        //         () -> -driverXbox.getRightX(),
        //         () -> -driverXbox.getRightY());

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
        //arm.requestBrake();
        //driverXbox.y().onTrue(Auto).debounce(5);
        driverXbox.rightBumper().onTrue(shoot).debounce(8);
        driverXbox.x().onTrue(Commands.runOnce(intake::startIntake, intake))
                .onFalse(Commands.runOnce(intake::stopIntake, intake));

        driverXbox.povUp().onTrue(Commands.runOnce(arm::moveArmUp, arm)).onFalse(Commands.runOnce(arm::stopArm, arm));
        driverXbox.povDown().onTrue(Commands.runOnce(arm::moveArmDown, arm)).onFalse(Commands.runOnce(arm::stopArm, arm));
        driverXbox.leftTrigger().onTrue(Commands.runOnce(arm::ejectNote, arm)).onFalse(Commands.runOnce(arm::stopGrippers, arm));
        driverXbox.leftBumper().onTrue(Commands.runOnce(arm::intakeNote, arm)).onFalse(Commands.runOnce(arm::stopGrippers, arm));
        driverXbox.povLeft().onTrue(Commands.runOnce(arm::rotateNote, arm)).onFalse(Commands.runOnce(arm::stopGrippers, arm));
        driverXbox.povRight().onTrue(Commands.runOnce(arm::rotateNoteOpposite, arm)).onFalse(Commands.runOnce(arm::stopGrippers, arm));

        driverXbox.y().onTrue(Commands.runOnce(climber::beginClimb, climber)).onFalse(Commands.runOnce(climber::stopClimb, climber));
        driverXbox.a().onTrue(Commands.runOnce(climber::releaseClimb, climber)).onFalse(Commands.runOnce(climber::stopClimb, climber));

        driverXbox.b().onTrue(Commands.sequence(
                Commands.runOnce(intake::reverseFeeder, intake),
                Commands.runOnce(intake::reverseIntake, intake)
        )).onFalse(Commands.sequence(
                Commands.runOnce(intake::stopFeeder, intake),
                Commands.runOnce(intake::stopIntake, intake)
        ));
    }

    public Command getAutonomousCommand() {
        // rotate 45 degrees drive 4 feet shoot
        
        return Auto;
    }
}
