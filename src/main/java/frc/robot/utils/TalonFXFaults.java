package frc.robot.utils;

public record TalonFXFaults(
        boolean bootDuringEnable,
        boolean bridgeBrownout,
        boolean deviceTemp,
        boolean forwardHardLimit,
        boolean forwardSoftLimit,
        boolean fusedSensorOutOfSync,
        boolean hardware,
        boolean missingDifferentialFX,
        boolean missingHardLimitRemote,
        boolean missingSoftLimitRemote,
        boolean overSupplyV,
        boolean procTemp,
        boolean remoteSensorDataInvalid,
        boolean remoteSensorPosOverflow,
        boolean remoteSensorReset,
        boolean reverseHardLimit,
        boolean reverseSoftLimit,
        boolean staticBrakeDisabled,
        boolean statorCurrLimit,
        boolean supplyCurrLimit,
        boolean undervoltage,
        boolean unlicensedFeatureInUse,
        boolean unstableSupplyV,
        boolean usingFusedCANCoderWhileUnlicensed) {

    public boolean hasCriticalFaults() {
        return bootDuringEnable
                || bridgeBrownout
                || deviceTemp
                || fusedSensorOutOfSync
                || hardware
                || missingDifferentialFX
                || missingHardLimitRemote
                || missingSoftLimitRemote
                || overSupplyV
                || procTemp
                || remoteSensorDataInvalid
                || remoteSensorPosOverflow
                || remoteSensorReset
                || supplyCurrLimit
                || undervoltage
                || unlicensedFeatureInUse
                || unstableSupplyV
                || usingFusedCANCoderWhileUnlicensed;
    }
}
