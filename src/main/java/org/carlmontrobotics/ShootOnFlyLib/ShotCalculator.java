package org.carlmontrobotics.ShootOnFlyLib;

import static org.carlmontrobotics.Constants.ShootOnFlyc.*;

public class ShotCalculator {

    public static class ShotResult {
        public double requiredRPM;
        public boolean impossible;
        public boolean nearImpossible;
        public boolean descendingAtGoal;
        public boolean clearanceOK;
        public double[] clearanceMargins; // meters above each obstacle
    }

    public static ShotResult calculateShot(
            double dx, double dy, double dz,          // displacement to goal (m)
            double vxRobot, double vyRobot,           // robot velocity field-relative (m/s)
            double[] obstacleDistances,               // horizontal distance to each obstacle
            double[] obstacleHeights                 // required height at each obstacle
    ) {

        ShotResult result = new ShotResult();

        double x = Math.hypot(dx, dy);
        double cos = Math.cos(launchAngleRad);
        double tan = Math.tan(launchAngleRad);

        double margin = x * tan - dz;

        // 1. impossible check
        if (margin <= 0.0) {
            result.impossible = true;
            result.requiredRPM = Double.POSITIVE_INFINITY;
            return result;
        }

        // 2. field-relative projectile speed
        double vField = Math.sqrt(
            (g * x * x) /
            (2.0 * cos * cos * margin)
        );

        // 3. compensate robot motion
        double dirX = dx / x;
        double dirY = dy / x;

        double vxField = vField * dirX;
        double vyField = vField * dirY;

        double vxExit = vxField - vxRobot;
        double vyExit = vyField - vyRobot;

        double exitSpeed = Math.hypot(vxExit, vyExit);

        // 4. convert to RPM
        double wheelRadPerSec = exitSpeed / wheelRadiusMeters;
        result.requiredRPM = wheelRadPerSec * 60.0 / (2.0 * Math.PI);

        // 5. near-impossible check
        result.nearImpossible = margin < 0.20;

        // 6. descending check
        double vzFinal =
            vField * Math.sin(launchAngleRad)
            - g * x / (vField * cos);

        result.descendingAtGoal = vzFinal < 0.0;

        // 7. clearance checks
        if (obstacleDistances != null && obstacleHeights != null) {

            int n = obstacleDistances.length;
            result.clearanceMargins = new double[n];
            result.clearanceOK = true;

            for (int i = 0; i < n; i++) {

                double s = obstacleDistances[i];

                double z = launchHeightMeters
                    + s * Math.tan(launchAngleRad)
                    - (g * s * s) / (2.0 * vField * vField * cos * cos);

                double marginClear = z - obstacleHeights[i];
                result.clearanceMargins[i] = marginClear;

                if (marginClear < clearanceSafety) {
                    result.clearanceOK = false;
                }
            }
        } else {
            result.clearanceOK = true;
            result.clearanceMargins = new double[0];
        }

        return result;
    }
}
