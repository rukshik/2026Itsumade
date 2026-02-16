
package frc.robot.subsystems;

public class LimelightFrontMiddle extends Limelight {
    private static LimelightFrontMiddle llFrontMiddle;

    private LimelightFrontMiddle() {
        super("limelight-shoot", true);
    }

    public static LimelightFrontMiddle getInstance() {
        if (llFrontMiddle == null)
            llFrontMiddle = new LimelightFrontMiddle();
        return llFrontMiddle;
    }

    @Override
    public void periodic() {
        super.periodic();
    }
}
