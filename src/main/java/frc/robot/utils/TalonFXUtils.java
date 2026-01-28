package frc.robot.utils;

import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXUtils {

    private TalonFXUtils() {}

    public static TalonFXFaults getAllActiveFaults(TalonFX motor) {
        if (motor == null) {
            Logger.reportWarning("Cannot get faults from a null TalonFX", true);
            return new TalonFXFaults(
                    false, false, false, false, false, false, false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false, false, false);
        }

        return new TalonFXFaults(
                motor.getFault_BootDuringEnable().getValue(),
                motor.getFault_BridgeBrownout().getValue(),
                motor.getFault_DeviceTemp().getValue(),
                motor.getFault_ForwardHardLimit().getValue(),
                motor.getFault_ForwardSoftLimit().getValue(),
                motor.getFault_FusedSensorOutOfSync().getValue(),
                motor.getFault_Hardware().getValue(),
                motor.getFault_MissingDifferentialFX().getValue(),
                motor.getFault_MissingHardLimitRemote().getValue(),
                motor.getFault_MissingSoftLimitRemote().getValue(),
                motor.getFault_OverSupplyV().getValue(),
                motor.getFault_ProcTemp().getValue(),
                motor.getFault_RemoteSensorDataInvalid().getValue(),
                motor.getFault_RemoteSensorPosOverflow().getValue(),
                motor.getFault_RemoteSensorReset().getValue(),
                motor.getFault_ReverseHardLimit().getValue(),
                motor.getFault_ReverseSoftLimit().getValue(),
                motor.getFault_StaticBrakeDisabled().getValue(),
                motor.getFault_StatorCurrLimit().getValue(),
                motor.getFault_SupplyCurrLimit().getValue(),
                motor.getFault_Undervoltage().getValue(),
                motor.getFault_UnlicensedFeatureInUse().getValue(),
                motor.getFault_UnstableSupplyV().getValue(),
                motor.getFault_UsingFusedCANcoderWhileUnlicensed().getValue());
    }

    public static TalonFXFaults getAllStickyFaults(TalonFX motor) {
        if (motor == null) {
            Logger.reportWarning("Cannot get sticky faults from a null TalonFX", true);
            return new TalonFXFaults(
                    false, false, false, false, false, false, false, false, false, false, false, false, false, false,
                    false, false, false, false, false, false, false, false, false, false);
        }

        return new TalonFXFaults(
                motor.getStickyFault_BootDuringEnable().getValue(),
                motor.getStickyFault_BridgeBrownout().getValue(),
                motor.getStickyFault_DeviceTemp().getValue(),
                motor.getStickyFault_ForwardHardLimit().getValue(),
                motor.getStickyFault_ForwardSoftLimit().getValue(),
                motor.getStickyFault_FusedSensorOutOfSync().getValue(),
                motor.getStickyFault_Hardware().getValue(),
                motor.getStickyFault_MissingDifferentialFX().getValue(),
                motor.getStickyFault_MissingHardLimitRemote().getValue(),
                motor.getStickyFault_MissingSoftLimitRemote().getValue(),
                motor.getStickyFault_OverSupplyV().getValue(),
                motor.getStickyFault_ProcTemp().getValue(),
                motor.getStickyFault_RemoteSensorDataInvalid().getValue(),
                motor.getStickyFault_RemoteSensorPosOverflow().getValue(),
                motor.getStickyFault_RemoteSensorReset().getValue(),
                motor.getStickyFault_ReverseHardLimit().getValue(),
                motor.getStickyFault_ReverseSoftLimit().getValue(),
                motor.getStickyFault_StaticBrakeDisabled().getValue(),
                motor.getStickyFault_StatorCurrLimit().getValue(),
                motor.getStickyFault_SupplyCurrLimit().getValue(),
                motor.getStickyFault_Undervoltage().getValue(),
                motor.getStickyFault_UnlicensedFeatureInUse().getValue(),
                motor.getStickyFault_UnstableSupplyV().getValue(),
                motor.getStickyFault_UsingFusedCANcoderWhileUnlicensed().getValue());
    }
}
