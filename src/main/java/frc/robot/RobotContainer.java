// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.IndexerCMD;
import frc.robot.Commands.IntakeArmCMD;
import frc.robot.Commands.IntakeCMD;
import frc.robot.Commands.LauncherCMD;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final double TurtleModifier = .3;

     /* Declare subsystems. */
    private final Telemetry logger = new Telemetry(MaxSpeed);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Indexer indexer = new Indexer();
    private final Intake intake = new Intake();
    private final Launcher launcher = new Launcher();

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private PathPlannerPath LauncherEnvelope;


    /* Declare human interface devices. */
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final GenericHID operatorPanel = new GenericHID(1);


    public RobotContainer() {
        // Register any commands you might want to use in paths.
        NamedCommands.registerCommand("Indexer into launcher", new IndexerCMD(indexer, 0.5, 0.5));
        NamedCommands.registerCommand("Intake into hopper", new IntakeCMD(intake, false));
        NamedCommands.registerCommand("Shoot", new LauncherCMD(launcher, 0.7));
        NamedCommands.registerCommand("Bring intake up", new IntakeArmCMD(intake, true));
        NamedCommands.registerCommand("Bring intake down", new IntakeArmCMD(intake, false));
        NamedCommands.registerCommand("Unload", new ParallelCommandGroup(new IndexerCMD(indexer, -0.5, -0.5), new IntakeCMD(intake, true)));
        
        // Init autochooser and send data to SmartDashboard.
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));


        // Turtle Mode (slow) 
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * TurtleModifier) // Drive forward with negative Y (forward).
        .withVelocityY(-joystick.getLeftX() * MaxSpeed * TurtleModifier) // Drive left with negative X (left).
        .withRotationalRate(-joystick.getRightX() * MaxAngularRate * TurtleModifier) // Drive counterclockwise with negative X (left).
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Indexer into launcher.
        new JoystickButton(operatorPanel, 0).whileTrue(new IndexerCMD(indexer, 0.5, 0.5));
        // Intake into hopper.
        new JoystickButton(operatorPanel, 0).whileTrue(new IntakeCMD(intake, false));
        // Shoot.
        new JoystickButton(operatorPanel, 0).whileTrue(new LauncherCMD(launcher, 0.7));
        // Bring intake up.
        new JoystickButton(operatorPanel, 0).onTrue(new IntakeArmCMD(intake, true));
        // Bring intake down.
        new JoystickButton(operatorPanel, 0).onTrue(new IntakeArmCMD(intake, false));
        // Unload.
        new JoystickButton(operatorPanel, 0).whileTrue(new ParallelCommandGroup(new IndexerCMD(indexer, -0.5, -0.5), new IntakeCMD(intake, true)));

        // PLAN B - Annihilate the youth.
        try{
            // Load the path to launcher envelope using its name in the GUI.
            LauncherEnvelope = PathPlannerPath.fromPathFile("LauncherEnvelope");

        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        }

        // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
        PathConstraints constraints = new PathConstraints(
            3.0, 4.0,
            Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        new JoystickButton(operatorPanel, 0).whileTrue(AutoBuilder.pathfindThenFollowPath(
            LauncherEnvelope,
            constraints));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Return pathplanner auto from AutoChooser.
        return autoChooser.getSelected();
    }
}
