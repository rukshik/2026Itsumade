package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class AlignToTower extends Command {
    private Drivetrain drivetrain;
    // private static boolean isBlue;
    private PIDController rotationalController, lateralController, depthController;
    private Field2d field;

    // pid
    private double rotationalP, rotationalI, rotationalD, rotationalFF,
            lateralP, lateralI, lateralD, lateralFF,
            depthP, depthI, depthD, depthFF;
    private double rotationalLowerP, lateralLowerP, depthLowerP;
    private double rotationalErrorThreshold, lateralErrorThreshold, depthErrorThreshold,
            rotationalLowerPThreshold, lateralLowerPThreshold, depthLowerPThreshold;

    private double rotationalError, lateralError, depthError;
    private double rotationalMaxSpeed, lateralMaxSpeed, depthMaxSpeed;
    private double rotation, lateral, depth;

    private double desiredY, desiredX;
    private double desiredAngle;

    public AlignToTower() {
        drivetrain = Drivetrain.getInstance();
        field = new Field2d();

        SmartDashboard.putNumber("shiftFromCenterTagToTowerSide", 0);

        SmartDashboard.putNumber("rotationalP", 0.05);
        SmartDashboard.putNumber("rotationalI", 0);
        SmartDashboard.putNumber("rotationalD", 0);
        SmartDashboard.putNumber("rotationalFF", 0);
        SmartDashboard.putNumber("rotationalLowerP", 0.02);
        SmartDashboard.putNumber("rotationalErrorThreshold", 1);
        SmartDashboard.putNumber("rotationalLowerPThreshold", 2);
        SmartDashboard.putNumber("rotationalError", 0);
        SmartDashboard.putNumber("desiredAngle", 0);

        SmartDashboard.putNumber("lateralP", 1.344);
        SmartDashboard.putNumber("lateralI", 0);
        SmartDashboard.putNumber("lateralD", 0);
        SmartDashboard.putNumber("lateralFF", 0);
        SmartDashboard.putNumber("lateralLowerP", 0.896);
        SmartDashboard.putNumber("lateralErrorThreshold", 0.01);
        SmartDashboard.putNumber("lateralLowerPThreshold", 0.05);
        SmartDashboard.putNumber("lateralMaxSpeed", 2);
        SmartDashboard.putNumber("lateralError", 0);
        SmartDashboard.putNumber("desiredY", 0);

        SmartDashboard.putNumber("depthP", 0.85);
        SmartDashboard.putNumber("depthI", 0);
        SmartDashboard.putNumber("depthD", 0);
        SmartDashboard.putNumber("depthFF", 0);
        SmartDashboard.putNumber("depthLowerP", 0.4);
        SmartDashboard.putNumber("depthErrorThreshold", 0.01);
        SmartDashboard.putNumber("depthLowerPThreshold", 0.05);
        SmartDashboard.putNumber("depthMaxSpeed", 2);
        SmartDashboard.putNumber("depthError", 0);
        SmartDashboard.putNumber("desiredX", 0);

        SmartDashboard.putNumber("rotational", 0);
        SmartDashboard.putNumber("lateral", 0);
        SmartDashboard.putNumber("depth", 0);

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        // isBlue = (DriverStation.getAlliance().isEmpty()
        //         || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue);

        Pose2d centerTagPose = Limelight.getAprilTagPose(31);

        rotationalP = SmartDashboard.getNumber("rotationalP", 0);
        rotationalI = SmartDashboard.getNumber("rotationalI", 0);
        rotationalD = SmartDashboard.getNumber("rotationalD", 0);
        rotationalFF = SmartDashboard.getNumber("rotationalFF", 0);
        rotationalLowerP = SmartDashboard.getNumber("rotationalLowerP", 0);
        rotationalErrorThreshold = SmartDashboard.getNumber("rotationalErrorThreshold", 0);
        rotationalLowerPThreshold = SmartDashboard.getNumber("rotationalLowerPThreshold", 0);

        rotationalController = new PIDController(rotationalP, rotationalI, rotationalD);
        lateralController = new PIDController(lateralP, lateralI, lateralD);
        depthController = new PIDController(depthP, depthI, depthD);
        
        rotationalController.enableContinuousInput(-180, 180);

        double xOffsetTowerCenterToSide = 0.25;
        double yOffsetTowerCenterToSide = SmartDashboard.getNumber("shiftFromCenterTagToTowerSide", 0); 
        boolean climbLeft = drivetrain.getPose().getY() > centerTagPose.getY();

        // original WORKING version
        if (climbLeft) {
            desiredY = centerTagPose.getY() + SmartDashboard.getNumber("shiftFromCenterTagToTowerSide", 0);
            desiredX = centerTagPose.getX() + 0.25;
            desiredAngle = -90;
        } else {
            desiredY = centerTagPose.getY() - SmartDashboard.getNumber("shiftFromCenterTagToTowerSide", 0);
            desiredX = centerTagPose.getX() + 0.25;
            desiredAngle = 90;
        }
        
        // editted version
        // desiredX = centerTagPose.getX() + xOffsetTowerCenterToSide;
        // desiredY = centerTagPose.getY() + (climbLeft ? yOffsetTowerCenterToSide : -yOffsetTowerCenterToSide);
        // desiredAngle = climbLeft ? -90 : 90;

        SmartDashboard.putNumber("desiredAngle", desiredAngle);
        SmartDashboard.putNumber("desiredY", desiredY);
        SmartDashboard.putNumber("desiredX", desiredX);
    }

    @Override
    public void execute() {

        rotationalError = drivetrain.getHeadingBlue() - desiredAngle;
        lateralError = drivetrain.getPose().getX() - desiredX;
        depthError = drivetrain.getPose().getY() - desiredY;

        SmartDashboard.putNumber("rotationalError", rotationalError);
        SmartDashboard.putNumber("lateralError", lateralError);
        SmartDashboard.putNumber("depthError", depthError);

        // potential new logic
        // if (Math.abs(rotationalError) < rotationalErrorThreshold) 
        //     rotationalController.setP(0);
        // else if (Math.abs(rotationalError) < rotationalLowerPThreshold) 
        //     rotationalController.setP(rotationalLowerP);
        // else 
        //     rotationalController.setP(rotationalP);
        // 
        // rotation = rotationalController.calculate(rotationalError) + Math.signum(rotationalError) * rotationalFF;

        // if (Math.abs(lateralError) < lateralErrorThreshold) 
        //     lateralController.setP(0);
        // else if (Math.abs(lateralError) < lateralLowerPThreshold) 
        //     lateralController.setP(lateralLowerP);
        // else 
        //     lateralController.setP(lateralP);
        
        // lateral = lateralController.calculate(lateralError) + Math.signum(lateralError) * lateralFF;
        
        // if (Math.abs(depthError) < depthErrorThreshold) 
        //     depthController.setP(0);
        // else if (Math.abs(depthError) < depthLowerPThreshold) 
        //     depthController.setP(depthLowerP);
        // else 
        //     depthController.setP(depthP);
        
        // depth = depthController.calculate(depthError) + Math.signum(depthError) * depthFF;

        // SmartDashboard.putNumber("rotational", rotation);
        // SmartDashboard.putNumber("lateral", lateral);
        // SmartDashboard.putNumber("depth", depth);

        // drivetrain.drive(new Translation2d(depth, lateral), rotation, true, null);

        if (Math.abs(rotationalError) < rotationalLowerPThreshold) 
            rotationalController.setP(rotationalLowerP);
        else
            rotationalController.setP(rotationalP);

        if (Math.abs(rotationalError) > rotationalErrorThreshold) {
            rotation = rotationalController.calculate(rotationalError) + Math.signum(rotationalError) * rotationalFF;
            SmartDashboard.putNumber("rotational", rotation);
        } else {
            rotation = 0;
        } 
        
        if (Math.abs(lateralError) < lateralLowerPThreshold)
            lateralController.setP(lateralLowerP);
        else
            lateralController.setP(lateralP);

        if (Math.abs(lateralError) > lateralErrorThreshold) {
            lateral = lateralController.calculate(lateralError) - Math.signum(lateralError) * lateralFF;
            SmartDashboard.putNumber("lateral", lateral);
        } else {
            lateral = 0;
        }
        
        if (Math.abs(depthError) < depthLowerPThreshold)
            depthController.setP(depthLowerP);
        else
            depthController.setP(depthP);

        if (Math.abs(depthError) > depthErrorThreshold) {
            depth = depthController.calculate(depthError) - Math.signum(depthError) * depthFF;
            SmartDashboard.putNumber("depth", depth);
        } else {
            depth = 0;
        }

        lateral = Math.signum(lateral) * Math.min(lateralMaxSpeed, Math.abs(lateral));
        depth = Math.signum(depth) * Math.min(depthMaxSpeed, Math.abs(depth));

        drivetrain.drive(new Translation2d(lateral, depth), rotation, true, null);

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
