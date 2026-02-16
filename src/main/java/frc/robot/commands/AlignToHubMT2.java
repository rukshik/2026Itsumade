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

public class AlignToHubMT2 extends Command {
    private Drivetrain drivetrain;
    private PIDController lateralPIDController, depthPIDController,
            rotationalPIDController;
    private double lateralP, lateralI, lateralD, lateralFF,
            depthP, depthI, depthD, depthFF,
            rotationalP, rotationalI, rotationalD, rotationalFF;
    private double rotationalLowerP, lateralLowerP; // lower P if error is small, since degrees is larger unit
    private double lateralErrorThreshold, depthErrorThreshold,
            rotationalErrorThreshold, // determines when error is small enough
            rotationalLowerPThreshold, lateralLowerPThreshold; // determines which rotationalP to use
    private Limelight frontMiddleLimelight;
    private double desiredAngle, desiredY;
    private Translation2d desiredPose;
    private double rotationalError, lateralError, depthError;
    private double lateral, rotation;
    private Field2d estimatedFieldPoseMT2;

    public AlignToHubMT2() {
        drivetrain = Drivetrain.getInstance();
        frontMiddleLimelight = LimelightFrontMiddle.getInstance();

        // rotationalP = HubAlignConstants.kRotationalP;
        // rotationalI = HubAlignConstants.kRotationalI;
        // rotationalD = HubAlignConstants.kRotationalD;
        // rotationalFF = HubAlignConstants.kRotationalFF;
        // rotationalLowerP = HubAlignConstants.kRotationalLowerP;
        // rotationalErrorThreshold = HubAlignConstants.kRotationalErrorThreshold;
        // rotationalLowerPThreshold = HubAlignConstants.kRotationLowerPThreshold;

        SmartDashboard.putNumber("rotationalP", 0.05);
        SmartDashboard.putNumber("rotationalI", 0);
        SmartDashboard.putNumber("rotationalD", 0);
        SmartDashboard.putNumber("rotationalFF", 0);
        SmartDashboard.putNumber("rotationalLowerP", 0.02);
        SmartDashboard.putNumber("rotationalErrorThreshold", 1);
        SmartDashboard.putNumber("rotationalLowerPThreshold", 2);
        SmartDashboard.putNumber("rotationalError", 0);
        // rotationalPIDController.enableContinuousInput(-180, 180);

        // lateralP = HubAlignConstants.kLateralP;
        // lateralI = HubAlignConstants.kLateralI;
        // lateralD = HubAlignConstants.kLateralD;
        // lateralFF = HubAlignConstants.kLateralFF;
        // lateralErrorThreshold = HubAlignConstants.kLateralErrorThreshold;
        // lateralLowerP = HubAlignConstants.kLateralLowerP;
        // lateralLowerPThreshold = HubAlignConstants.kLateralLowerPThreshold;
        estimatedFieldPoseMT2 = new Field2d();

        SmartDashboard.putNumber("lateralP", 0.4);
        SmartDashboard.putNumber("lateralI", 0);
        SmartDashboard.putNumber("lateralD", 0);
        SmartDashboard.putNumber("lateralFF", 0);
        SmartDashboard.putNumber("lateralErrorThreshold", 0.01);
        SmartDashboard.putNumber("lateralError", 0);
        SmartDashboard.putNumber("lateralLowerPThreshold", 0.05);
        SmartDashboard.putNumber("lateralLowerP", 0.2);
        SmartDashboard.putNumber("drivetrain estimated pose x", drivetrain.getPose().getX());
        SmartDashboard.putNumber("drivetrain estimated pose y", drivetrain.getPose().getY());
        SmartDashboard.putData("estimatedPoseMT2", estimatedFieldPoseMT2);
        SmartDashboard.putNumber("desiredX", 0);

        // depthP = HubAlignConstants.kDepthP;
        // depthI = HubAlignConstants.kDepthI;
        // depthD = HubAlignConstants.kDepthD;
        // depthFF = HubAlignConstants.kDepthFF;
        // depthErrorThreshold = HubAlignConstants.kDepthErrorThreshold;
        // depthPIDController = new PIDController(depthP, depthI, depthD);

        SmartDashboard.putNumber("lateral", 0);
        SmartDashboard.putNumber("rotational", 0);
        
        addRequirements(drivetrain);
        SmartDashboard.putNumber("Target April Tag ID", 20);
    }

    @Override
    public void initialize() {

        Pose2d tagPose = Limelight.getAprilTagPose((int)SmartDashboard.getNumber("Target April Tag ID", 20));
        
        desiredAngle = 0;
        rotationalP = SmartDashboard.getNumber("rotationalP", 0);
        rotationalI = SmartDashboard.getNumber("rotationalI", 0);
        rotationalD = SmartDashboard.getNumber("rotationalD", 0);
        rotationalFF = SmartDashboard.getNumber("rotationalFF", 0);
        rotationalLowerP = SmartDashboard.getNumber("rotationalLowerP", 0);
        rotationalLowerPThreshold = SmartDashboard.getNumber("rotationalLowerPThreshold", 0);
        rotationalErrorThreshold = SmartDashboard.getNumber("rotationalErrorThreshold", 0);
        rotationalPIDController = new PIDController(rotationalP, rotationalI, rotationalD);
        
        desiredY = tagPose.getY();
        SmartDashboard.putNumber("desiredY", desiredY);
        lateralP = SmartDashboard.getNumber("lateralP", 0);
        lateralI = SmartDashboard.getNumber("lateralI", 0);
        lateralD = SmartDashboard.getNumber("lateralD", 0);
        lateralFF = SmartDashboard.getNumber("lateralFF", 0);
        lateralLowerP = SmartDashboard.getNumber("lateralLowerP", 0);
        lateralLowerPThreshold = SmartDashboard.getNumber("lateralLowerPThreshold", 0);
        lateralErrorThreshold = SmartDashboard.getNumber("lateralErrorThreshold", 0);
        lateralPIDController = new PIDController(lateralP, lateralI, lateralD);

        // drivetrain = Drivetrain.getInstance();
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

        // TODO: implement global odometry model when limelight not visible
        Optional<Pose2d> optionalPose = frontMiddleLimelight.getEstimatedPoseMT2();
        if (optionalPose.isEmpty()) 
            optionalPose = Optional.of(drivetrain.getPose());
        
        Pose2d estimatedPose = optionalPose.get();
        // Pose2d estimatedPose = estimatedPoseOptional.isEmpty() ? drivetrain.getPose() : estimatedPoseOptional.get();
        estimatedFieldPoseMT2.setRobotPose(estimatedPose);
        SmartDashboard.putData("estimatedFieldPoseMT2", estimatedFieldPoseMT2);
        SmartDashboard.putNumber("epx", estimatedPose.getX());
        SmartDashboard.putNumber("epy", estimatedPose.getY());

        lateralError = estimatedPose.getY() - desiredY;
        SmartDashboard.putNumber("lateralError", lateralError);
        
         
        // if (Math.abs(lateralError) < lateralLowerPThreshold)
        //     lateralPIDController.setP(lateralLowerP);
        // else
        //     lateralPIDController.setP(lateralP);

        // if (Math.abs(lateralError) > lateralErrorThreshold) {
        //     lateral = lateralPIDController.calculate(-lateralError) - Math.signum(lateralError) * lateralFF;
        //     SmartDashboard.putNumber("lateral", lateral);
        // } else {
        //     lateral = 0;
        // }

        

        drivetrain.drive(new Translation2d(0, lateral), rotation, true, null);
        // SmartDashboard.putData("estimatedPoseMT2",estimatedFieldPoseMT2);

        // SmartDashboard.putNumber("drivetrain estimated pose x", drivetrain.getPose().getX());
        // SmartDashboard.putNumber("drivetrain estimated pose y", drivetrain.getPose().getY());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}