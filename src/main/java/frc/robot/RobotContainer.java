// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.LimelightHelpers;

//hello
public class RobotContainer {

    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final CommandXboxController driverController, operatorController;
    private final SendableChooser<Command> autoChooser;
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    //private final Climber climber = new Climber();


    public RobotContainer() {
        driverController = new CommandXboxController(0);
        operatorController = new CommandXboxController(1);
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }


    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
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

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on x button press
        joystick.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        
        drivetrain.registerTelemetry(logger::telemeterize);
    
    operatorController.rightTrigger().whileTrue(
    new RunCommand(
        () -> {
            
            int[] allowedTags = {25, 26, 21, 24,27,18,19,20}; // target specific tags
            
            int currentTagID = (int) LimelightHelpers.getFiducialID("limelight");
            double rotationRate = 0;
            
            // Check if the current tag is in our allowed list
            boolean isAllowedTag = false;
            for (int allowedTag : allowedTags) {
                if (currentTagID == allowedTag) {
                    isAllowedTag = true;
                    break;
                }
            }
            
            // Only apply Limelight rotation if we're seeing an allowed tag
            if (isAllowedTag && LimelightHelpers.getTV("limelight")) {
                rotationRate = LimelightHelpers.getTX("limelight") * -0.06;
            }
            
            drivetrain.setControl(
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(rotationRate)
            );
        },
        drivetrain
    )
);
            operatorController.leftTrigger().whileTrue(
    new RunCommand(
        () -> {
            
            int[] allowedTags = {32,31}; // target specific tags
            
            int currentTagID = (int) LimelightHelpers.getFiducialID("limelight");
            double rotationRate = 0;
            
            // Check if the current tag is in our allowed list
            boolean isAllowedTag = false;
            for (int allowedTag : allowedTags) {
                if (currentTagID == allowedTag) {
                    isAllowedTag = true;
                    break;
                }
            }
            
            // Only apply Limelight rotation if we're seeing an allowed tag
            if (isAllowedTag && LimelightHelpers.getTV("limelight")) {
                rotationRate = LimelightHelpers.getTX("limelight") * -0.065;
            }
            
            drivetrain.setControl(
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                    .withRotationalRate(rotationRate)
            );
        },
        drivetrain
    )
);


// moves climber up
//joystick.rightBumper()
//      .whileTrue(climber.Run());
    
    //  moves climber DOWN
  //  joystick.leftBumper()
    //  .whileTrue(climber.Reverse());
   }


public Command getAutonomousCommand() {
    return autoChooser.getSelected();
}


}

//End code