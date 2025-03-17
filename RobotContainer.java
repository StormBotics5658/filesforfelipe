// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.arm_jog;
import frc.robot.commands.arm_jog2;
import frc.robot.subsystems.arm;
import frc.robot.subsystems.arm2;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.stream.Stream;

import frc.robot.commands.FlapMove;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Flap;
import swervelib.SwerveInputStream;
//import com.pathplanner.lib.auto.NamedCommands;
//import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{ 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // Joysticks
private final CommandXboxController xboxOperator = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  public final arm m_arm = new arm();
  public final arm2 m_arm2 = new arm2();
  private final Flap m_Flap = new Flap();
  private final Elevator m_elevator = new Elevator();
  private final Compressor m_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
  public final FlapMove flapMove = new FlapMove(xboxOperator,m_Flap);
  public final SendableChooser<Command> autoChooser;

  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1, // was -1
                                                                () -> driverXbox.getLeftX() * -1) // was -1
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1 )
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(false);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(false);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY() * -1,
                                                                        () -> -driverXbox.getLeftX() * -1)
                                                                    .withControllerRotationAxis(() -> -driverXbox.getRightY() * -1)
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(false);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy();
  //                                                                             .withControllerHeadingAxis(() ->
  //                                                                                                            Math.sin(
  //                                                                                                                driverXbox.getRawAxis(
  //                                                                                                                    2
  //                                                                                                                 ) *
  //                                                                                                                Math.PI) *
  //                                                                                                            (Math.PI *
  //                                                                                                             2),
  //                                                                                                        () ->
  //                                                                                                            Math.cos(
  //                                                                                                                driverXbox.getRawAxis(
  //                                                                                                                    2) *
  //                                                                                                                Math.PI) *
  //                                                                                                            (Math.PI *
  //                                                                                                             2))
  //                                                                             .headingWhile(true)
  //                                                                             .translationHeadingOffset(true)
  //                                                                             .translationHeadingOffset(Rotation2d.fromDegrees(
  //                                                                                 0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();
     // Configure default commands
        // _BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SUBSYSTEM_DEFAULT_COMMAND
    m_arm.setDefaultCommand(new arm_jog(() -> xboxOperator.getLeftY()* -.2, m_arm));
  m_arm2.setDefaultCommand(new arm_jog2(() -> xboxOperator.getRightY()*-.2, m_arm2));
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

   boolean isCompetition = true;

   autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
     (stream) -> isCompetition
     ? stream.filter(auto -> auto.getName().startsWith("5658"))
     : stream);
   SmartDashboard.putData("Auto Choser", autoChooser);
    autoChooser.addOption("drive_back", drivebase.driveCommand(()-> -.1, ()->0, ()-> 0));
    m_compressor.enableDigital();
    //m_compressor.getAnalogVoltage();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
  //   xboxOperator.button(1, FlapMove).whileTrue(Flap, FlapMove);
//      driverXbox.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      // driverXbox.leftBumper().whileTrue(f lapMove.setinverted);
      driverXbox.rightBumper().whileTrue(flapMove);
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      xboxOperator.leftBumper().whileTrue(m_Flap.run(()->m_Flap.my_MotorJog(-0.1)).finallyDo(()->m_Flap.my_MotorJog(0)));
      xboxOperator.rightBumper().whileTrue(m_Flap.run(()->m_Flap.my_MotorJog(0.1)).finallyDo(()->m_Flap.my_MotorJog(0)));
      xboxOperator.rightTrigger().whileTrue(m_elevator.run(()->m_elevator.runelevator(0.15)).finallyDo(()->m_elevator.runelevator(0)));
      xboxOperator.leftTrigger().whileTrue(m_elevator.run(()->m_elevator.runelevator(-0.15)).finallyDo(()->m_elevator.runelevator(0)));
  
      
    } 
    {
    


  
    }

  
     

  

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
  //return drivebase.getAutonomousCommand("New Auto");
    return autoChooser.getSelected();
  // return Commands.none();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
