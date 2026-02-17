package frc.robot.commands;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightFrontMiddle;
import frc.robot.utils.Constants.HubAlignConstants;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Retro;

public class AlignToHub extends Command {
    private Drivetrain drivetrain;
    private PIDController lateralPIDController, rotationalPIDController, depthPIDController;
    private double rotationalP, rotationalI, rotationalD, rotationalFF,
            lateralP, lateralI, lateralD, lateralFF,
            depthP, depthI, depthD, depthFF;
    private double rotationalLowerP, lateralLowerP, depthLowerP;
    private double rotationalErrorThreshold, lateralErrorThreshold, depthErrorThreshold,
            rotationalLowerPThreshold, lateralLowerPThreshold, depthLowerPThreshold;
    private Limelight frontMiddleLimelight;
    private double desiredAngle, desiredY, desiredX;
    private double rotationalError, lateralError, depthError;
    private double lateralMaxSpeed, depthMaxSpeed;
    private double rotation, lateral, depth;

    public AlignToHub() {
        drivetrain = Drivetrain.getInstance();
        frontMiddleLimelight = LimelightFrontMiddle.getInstance();

        rotationalP = HubAlignConstants.kRotationalP;
        rotationalI = HubAlignConstants.kRotationalI;
        rotationalD = HubAlignConstants.kRotationalD;
        rotationalFF = HubAlignConstants.kRotationalFF;
        rotationalLowerP = HubAlignConstants.kRotationalLowerP;
        rotationalErrorThreshold = HubAlignConstants.kRotationalErrorThreshold;
        rotationalLowerPThreshold = HubAlignConstants.kRotationLowerPThreshold;

        // SmartDashboard.putNumber("rotationalP", 0.05);
        // SmartDashboard.putNumber("rotationalI", 0);
        // SmartDashboard.putNumber("rotationalD", 0);
        // SmartDashboard.putNumber("rotationalFF", 0);
        // SmartDashboard.putNumber("rotationalLowerP", 0.02);
        // SmartDashboard.putNumber("rotationalErrorThreshold", 1);
        // SmartDashboard.putNumber("rotationalLowerPThreshold", 2);
        SmartDashboard.putNumber("rotationalError", 0);

        lateralP = HubAlignConstants.kLateralP;
        lateralI = HubAlignConstants.kLateralI;
        lateralD = HubAlignConstants.kLateralD;
        lateralFF = HubAlignConstants.kLateralFF;
        lateralLowerP = HubAlignConstants.kLateralLowerP;
        lateralErrorThreshold = HubAlignConstants.kLateralErrorThreshold;
        lateralLowerPThreshold = HubAlignConstants.kLateralLowerPThreshold;
        lateralMaxSpeed = HubAlignConstants.kLateralMaxSpeed;

        // SmartDashboard.putNumber("lateralP", 1.344);
        // SmartDashboard.putNumber("lateralI", 0);
        // SmartDashboard.putNumber("lateralD", 0);
        // SmartDashboard.putNumber("lateralFF", 0);
        // SmartDashboard.putNumber("lateralLowerP", 0.896);
        // SmartDashboard.putNumber("lateralErrorThreshold", 0.01);
        // SmartDashboard.putNumber("lateralLowerPThreshold", 0.05);
        // SmartDashboard.putNumber("lateralMaxSpeed", 2);
        SmartDashboard.putNumber("lateralError", 0);
        SmartDashboard.putNumber("desiredY", 0);
        
        depthP = HubAlignConstants.kDepthP;
        depthI = HubAlignConstants.kDepthI;
        depthD = HubAlignConstants.kDepthD;
        depthFF = HubAlignConstants.kDepthFF;
        depthLowerP = HubAlignConstants.kDepthLowerP;
        depthErrorThreshold = HubAlignConstants.kDepthErrorThreshold;
        depthLowerPThreshold = HubAlignConstants.kDepthLowerPThreshold;
        depthMaxSpeed = HubAlignConstants.kDepthMaxSpeed;

        // SmartDashboard.putNumber("depthP", 0.85);
        // SmartDashboard.putNumber("depthI", 0);
        // SmartDashboard.putNumber("depthD", 0);
        // SmartDashboard.putNumber("depthFF", 0);
        // SmartDashboard.putNumber("depthLowerP", 0.4);
        // SmartDashboard.putNumber("depthErrorThreshold", 0.01);
        // SmartDashboard.putNumber("depthLowerPThreshold", 0.05);
        // SmartDashboard.putNumber("depthMaxSpeed", 2);
        SmartDashboard.putNumber("depthError", 0);
        SmartDashboard.putNumber("desiredX", 0);

        SmartDashboard.putNumber("rotational", 0);
        SmartDashboard.putNumber("lateral", 0);
        SmartDashboard.putNumber("depth", 0);

        SmartDashboard.putNumber("Target April Tag ID", 20);
        
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Pose2d tagPose = Limelight.getAprilTagPose((int)SmartDashboard.getNumber("Target April Tag ID", 20));
        
        desiredAngle = 180;
        // rotationalP = SmartDashboard.getNumber("rotationalP", 0);
        // rotationalI = SmartDashboard.getNumber("rotationalI", 0);
        // rotationalD = SmartDashboard.getNumber("rotationalD", 0);
        // rotationalFF = SmartDashboard.getNumber("rotationalFF", 0);
        // rotationalLowerP = SmartDashboard.getNumber("rotationalLowerP", 0);
        // rotationalErrorThreshold = SmartDashboard.getNumber("rotationalErrorThreshold", 0);
        // rotationalLowerPThreshold = SmartDashboard.getNumber("rotationalLowerPThreshold", 0);
        rotationalPIDController = new PIDController(rotationalP, rotationalI, rotationalD);
        rotationalPIDController.enableContinuousInput(-180,180);
        
        desiredY = tagPose.getY();
        SmartDashboard.putNumber("desiredY", desiredY);
        // lateralP = SmartDashboard.getNumber("lateralP", 0);
        // lateralI = SmartDashboard.getNumber("lateralI", 0);
        // lateralD = SmartDashboard.getNumber("lateralD", 0);
        // lateralFF = SmartDashboard.getNumber("lateralFF", 0);
        // lateralLowerP = SmartDashboard.getNumber("lateralLowerP", 0);
        // lateralErrorThreshold = SmartDashboard.getNumber("lateralErrorThreshold", 0);
        // lateralLowerPThreshold = SmartDashboard.getNumber("lateralLowerPThreshold", 0);
        // lateralMaxSpeed = SmartDashboard.getNumber("lateralMaxSpeed", 0);
        lateralPIDController = new PIDController(lateralP, lateralI, lateralD);    
        
        desiredX = tagPose.getX() + Math.signum(tagPose.getX());
        SmartDashboard.putNumber("desiredX", desiredX);
        // depthP = SmartDashboard.getNumber("depthP", 0);
        // depthI = SmartDashboard.getNumber("depthI", 0);
        // depthD = SmartDashboard.getNumber("depthD", 0);
        // depthFF = SmartDashboard.getNumber("depthFF", 0);
        // depthLowerP = SmartDashboard.getNumber("depthLowerP", 0);
        // depthErrorThreshold = SmartDashboard.getNumber("depthErrorThreshold", 0);
        // depthLowerPThreshold = SmartDashboard.getNumber("depthLowerPThreshold", 0);
        // depthMaxSpeed = SmartDashboard.getNumber("depthMaxSpeed", 0);
        depthPIDController = new PIDController(depthP, depthI, depthD);    
    }

    @Override
    public void execute() {
        rotationalError = drivetrain.getHeadingBlue() - desiredAngle;
        SmartDashboard.putNumber("rotationalError", rotationalError);

        if (Math.abs(rotationalError) < rotationalLowerPThreshold) 
            rotationalPIDController.setP(rotationalLowerP);
        else
            rotationalPIDController.setP(rotationalP);

        if (Math.abs(rotationalError) > rotationalErrorThreshold) {
            rotation = rotationalPIDController.calculate(rotationalError) + Math.signum(rotationalError) * rotationalFF;
            SmartDashboard.putNumber("rotational", rotation);
        } else {
            rotation = 0;
        }        

        // Optional<Pose2d> optionalPose = frontMiddleLimelight.getEstimatedPoseMT2();
        // if (optionalPose.get().getX() == 0 && optionalPose.get().getY() == 0) 
        //     optionalPose = Optional.of(drivetrain.getPose());
        // Pose2d estimatedPose = optionalPose.get();

        lateralError = drivetrain.getPose().getY() - desiredY;
        SmartDashboard.putNumber("lateralError", lateralError);
        
        if (Math.abs(lateralError) < lateralLowerPThreshold)
            lateralPIDController.setP(lateralLowerP);
        else
            lateralPIDController.setP(lateralP);

        if (Math.abs(lateralError) > lateralErrorThreshold) {
            lateral = lateralPIDController.calculate(lateralError) - Math.signum(lateralError) * lateralFF;
            SmartDashboard.putNumber("lateral", lateral);
        } else {
            lateral = 0;
        }

        depthError = drivetrain.getPose().getX() - desiredX;
        SmartDashboard.putNumber("depthError", depthError);
        
        if (Math.abs(depthError) < depthLowerPThreshold)
            depthPIDController.setP(depthLowerP);
        else
            depthPIDController.setP(depthP);

        if (Math.abs(depthError) > depthErrorThreshold) {
            depth = depthPIDController.calculate(depthError) - Math.signum(depthError) * depthFF;
            SmartDashboard.putNumber("depth", depth);
        } else {
            depth = 0;
        }

        double lateralSpeed = Math.signum(lateral) * Math.min(lateralMaxSpeed, Math.abs(lateral));
        double depthSpeed = Math.signum(depth) * Math.min(depthMaxSpeed, Math.abs(depth));

        drivetrain.drive(new Translation2d(depthSpeed, lateralSpeed), rotation, true, null);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}