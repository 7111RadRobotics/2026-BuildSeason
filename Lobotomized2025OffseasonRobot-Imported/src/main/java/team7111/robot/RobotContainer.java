package team7111.robot;

import team7111.lib.pathfinding.Waypoint;
import team7111.lib.pathfinding.WaypointConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import team7111.lib.pathfinding.Path;
import team7111.robot.Constants.ControllerConstants;
import team7111.robot.Constants.SwerveConstants;
//import team7111.robot.subsystems.BarrelSubsystem;
//import team7111.robot.subsystems.IntakeSubsystem;
import team7111.robot.subsystems.PathSubsystem;
//import team7111.robot.subsystems.ShooterSubsystem;
import team7111.robot.subsystems.SuperStructure;
import team7111.robot.subsystems.SwerveSubsystem;
import team7111.robot.subsystems.VisionSubsystem;
import team7111.robot.subsystems.SuperStructure.ControlState;
import team7111.robot.subsystems.SwerveSubsystem.SwerveState;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final CommandXboxController driverController = new CommandXboxController(ControllerConstants.driverControllerID);
    public final CommandXboxController operatorController = new CommandXboxController(ControllerConstants.operatorControllerID);
    public final SwerveSubsystem swerve;
    public final VisionSubsystem vision;
    public final PathSubsystem paths;
    //public final IntakeSubsystem intake;
    //public final BarrelSubsystem barrel;
    //public final ShooterSubsystem shooter;
    public final SuperStructure superStructure;
    public SendableChooser<Command> autoChooser;

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        swerve = new SwerveSubsystem();
        vision = new VisionSubsystem();
        paths = new PathSubsystem(null);
        //intake = new IntakeSubsystem();
        //barrel = new BarrelSubsystem();
        //shooter = new ShooterSubsystem();
        superStructure = new SuperStructure(vision, swerve, paths, ControllerConstants.operatorControllerID);

        autoChooser = new SendableChooser<>();

        Waypoint[] waypoints = new Waypoint[]{
            new Waypoint(new Pose2d(7.217, 4.199, Rotation2d.fromDegrees(180.0)), new WaypointConstraints(10, 0, 0.25), new WaypointConstraints(360, 0, 10)),
        };

        Path path = new Path(waypoints);

        autoChooser.addOption("Path_TEST", swerve.setPathCommand(path).andThen(swerve.setSwerveStateCommand(SwerveState.initializePath)));

        SmartDashboard.putData("autoChooser", autoChooser);

        // Configure button bindings
        configureButtonBindings();
    }

    public Command getAutonomousCommand() {
        Command auto = Commands.print("autochooser null");
        if(autoChooser != null)
            auto = autoChooser.getSelected();
        return auto;
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        swerve.setJoysickInputs(
            () -> -ControllerConstants.xDriveLimiter.calculate((Math.pow(driverController.getLeftX(), 1) / SwerveConstants.sensitivity)), 
            () -> ControllerConstants.yDriveLimiter.calculate((Math.pow(driverController.getLeftY(), 1) / SwerveConstants.sensitivity)),  
            () -> ControllerConstants.rotationLimiter.calculate((Math.pow(driverController.getRightX(), 3) / SwerveConstants.sensitivity)));

        swerve.setDriveFieldRelative(true);

        driverController.start().onTrue(swerve.zeroGyroCommand());
    }
}
