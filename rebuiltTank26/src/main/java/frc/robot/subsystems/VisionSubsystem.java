package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveSubsystem;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera;

    public VisionSubsystem() {
        camera = new PhotonCamera("photonvision"); // must match PhotonVision UI name
    }

    public boolean hasTarget() {
        return camera.getLatestResult().hasTargets();
    }

    public PhotonTrackedTarget getBestTarget() {
        if (!hasTarget()) return null;
        return camera.getLatestResult().getBestTarget();
    }

    public double getYaw() {
        PhotonTrackedTarget target = getBestTarget();
        return (target == null) ? 0.0 : target.getYaw();
    }

    public Command holdDistance(DriveSubsystem drive) {
        return run(() -> {
            if (!hasTarget()) {
                drive.stop();
                return;
            }

        PhotonTrackedTarget target = getBestTarget();

        double yaw = target.getYaw();
        double area = target.getArea();

        // tuning constants
        double turnKP = 0.02;
        double forwardKP = 0.1;

        double desiredDistance = 10; // target size when at desired distance

        double turn = yaw * turnKP;
        double forward = (desiredDistance - area) * forwardKP;

        drive.driveArcade(forward, turn);
        
        });
    }

    // Command to turn robot toward the target

    public Command turnToTarget(DriveSubsystem drive) {
        return run(() -> {
            if (!hasTarget()) {
                drive.stop();
                return;
            }

            double yaw = getYaw();

            // simple pid turn
            double kP = 0.02;
            double turn = yaw * kP;

            drive.driveArcade(0, turn);
        });
    }
    public Command fireProjectileCommand(double range, double heightTarget){
        double gravity = 9.81;
        double wheelCircumfrence = 0.11999999;
        double heightLeBron = 0.4953;
        double thota = 80; // this is in degrees to be converted to radians later
        range +=  0.4318; //this accounts for the distance from the robots projectile storage to the fron to f the robot
        double cosSquared = Math.cos(Math.toRadians(thota))*Math.cos(Math.toRadians(thota));
        double constantThing = (25*60/26*Math.PI*wheelCircumfrence) * (Math.sqrt(gravity/2*cosSquared));
        double targetRpm = constantThing*(range/
        (Math.sqrt(range*
        Math.tan(Math.toRadians(range))
        -(heightTarget - heightLeBron))))*1.4; // the 1.4 is because i assume its a closed system so to account for losses
        return run(() -> {
            fuelMotor.setRefernce(targetRpm, ControlType.kVelocity);
        });
    }
}


