package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShooterController {

    private static final HashMap<Double, ShooterParams> SHOOTER_MAP = new HashMap<>();

    // Example LUT: distance (m) → {RPS, time of flight (s)}
    private static final InterpolatingTreeMap<Double, ShooterParams> SHOOTER_INTERP_MAP = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            (startParams, endParams, t) -> new ShooterParams( // interpolador del ShooterParams
                MathUtil.interpolate(startParams.rps, endParams.rps, t),
                MathUtil.interpolate(startParams.timeOfFlight, endParams.timeOfFlight, t)
            )
    );

    static {
        SHOOTER_MAP.put(2d, new ShooterParams(47.0, 0.42));
        SHOOTER_MAP.put(2.5, new ShooterParams(48.0, 0.51));
        SHOOTER_MAP.put(3d, new ShooterParams(50.0, 0.58));
        SHOOTER_MAP.put(3.5, new ShooterParams(54.0, 0.65));
        SHOOTER_MAP.put(4d, new ShooterParams(58.0, 0.71));
        SHOOTER_MAP.put(4.5, new ShooterParams(61.0, 0.78));
        SHOOTER_MAP.put(5d, new ShooterParams(63, 0.84));
        SHOOTER_MAP.put(5.5, new ShooterParams(65, 0.91));

        for (Entry<Double, ShooterParams> entry : SHOOTER_MAP.entrySet()) {
            SHOOTER_INTERP_MAP.put(entry.getKey(), entry.getValue());
        }
    }

    public static ShooterResult calculate(
            Translation2d robotPosition,
            Translation2d robotVelocity,
            Translation2d goalPosition,
            double latencyCompensation) {
        // 1. Project future position
        Translation2d futurePos = robotPosition.plus(
                robotVelocity.times(latencyCompensation));

        // 2. Get target vector
        Translation2d toGoal = goalPosition.minus(futurePos);
        double distance = toGoal.getNorm();
        Translation2d targetDirection = toGoal.div(distance);

        // 3. Look up baseline velocity from table
        ShooterParams baseline = SHOOTER_MAP.get(distance);
        double baselineVelocity = distance / baseline.timeOfFlight;

        // 4. Build target velocity vector
        Translation2d targetVelocity = targetDirection.times(baselineVelocity);

        // 5. THE MAGIC: subtract robot velocity
        Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

        // 6. Extract results
        Rotation2d turretAngle = shotVelocity.getAngle();
        double requiredVelocity = shotVelocity.getNorm();

        // 7. Use table in reverse: velocity → effective distance → RPS
        double effectiveDistance = velocityToEffectiveDistance(requiredVelocity);
        double requiredRps = SHOOTER_MAP.get(effectiveDistance).rps;

        return new ShooterResult(turretAngle, requiredRps);
    }

    public static double getHorizontalVelocity(double distance) {
        ShooterParams params = SHOOTER_MAP.get(distance);
        return distance / params.timeOfFlight;
    }

    public static double velocityToEffectiveDistance(double velocity) {
        // Binary search or iterate through table to find distance
        // where (distance / ToF) = velocity
        // Most InterpolatingTreeMap implementations support inverse lookup
        // or you can build a reverse map: velocity → distance

        for (Map.Entry<Double, ShooterParams> entry : SHOOTER_MAP.entrySet()) {
            double dist = entry.getKey();
            double vel = dist / entry.getValue().timeOfFlight;
            if (vel >= velocity) {
                return dist; // Interpolate for better accuracy
            }
        }

        // find max
        double maxDist = 0;
        for (double dist : SHOOTER_MAP.keySet()) {
            if (dist > maxDist) {
                maxDist = dist;
            }
        }

        return maxDist; // If velocity is higher than any in the table, return max distance
    }

    public static double calculateAdjustedRps(double requiredVelocity) {
        double effectiveDistance = velocityToEffectiveDistance(requiredVelocity);
        return SHOOTER_INTERP_MAP.get(effectiveDistance).rps;
    }

    public static record ShooterParams(double rps, double timeOfFlight) {}

    public static record ShooterResult(Rotation2d turretAngle, double requiredRps) {}
}
