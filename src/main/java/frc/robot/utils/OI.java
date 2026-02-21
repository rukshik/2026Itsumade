package frc.robot.utils;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlignToHub;
import frc.robot.commands.AlignToTower;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimelightFrontMiddle;
import frc.robot.utils.Constants.DriveConstants;

public class OI {
    private static OI instance;
    private PS4Controller controller;

    public static OI getInstance() {
        if (instance == null)
            instance = new OI();
        return instance;
    }
    
    public OI() {
        controller = new PS4Controller(0);
        configurate();
    }

    public void configurate() {
        Trigger PSButton = new JoystickButton(controller, PS4Controller.Button.kPS.value);
        PSButton.onTrue(new InstantCommand(() -> Drivetrain.getInstance().resetGyro()));    
        
        // Trigger circleButton = new JoystickButton(controller, PS4Controller.Button.kCircle.value);
        // circleButton.whileTrue(new AlignToHub());

        Trigger squareButton = new JoystickButton(controller, PS4Controller.Button.kSquare.value);
        squareButton.whileTrue(new AlignToTower());

        Trigger xButton = new JoystickButton(controller, PS4Controller.Button.kCross.value);
        xButton.onTrue(new InstantCommand(() -> {
            Optional<Pose2d> pose = LimelightFrontMiddle.getInstance().getEstimatedPoseMT2();
            if(pose.isPresent())
                Drivetrain.getInstance().resetTranslation(pose.get().getTranslation());
        }));

        // Trigger muteButton = new JoystickButton(controller, 15);
    }
    
    public double getForward() {
        // return -controller.getLeftY();
        double val = -controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
        return Math.abs(val) < 0.1 ? 0 : val;
    }
    
    public double getStrafe() {
        double val = -controller.getRawAxis(PS4Controller.Axis.kLeftX.value);
        return Math.abs(val) < 0.1 ? 0 : val;
    }
    
    public Translation2d getSwerveTranslation() {
        return new Translation2d(
            getForward() * DriveConstants.kMaxFloorSpeed,
            getStrafe() * DriveConstants.kMaxFloorSpeed
        );
    }
    
    public double getRotation() {
        double leftRotation = controller.getRawAxis(PS4Controller.Axis.kR2.value);
        double rightRotation = controller.getRawAxis(PS4Controller.Axis.kL2.value);
        double val = (rightRotation - leftRotation) / 2.0;
        return Math.abs(val) < 0.1 ? 0 : val;
    }

    public double getDPadPOV() {
        return controller.getPOV();
    }
}
